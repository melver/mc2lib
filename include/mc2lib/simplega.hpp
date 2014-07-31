/*
 * Copyright (c) 2014, Marco Elver
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of the software nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MC2LIB_SIMPLEGA_HPP_
#define MC2LIB_SIMPLEGA_HPP_

#include <algorithm>
#include <cassert>
#include <list>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace mc2lib {
namespace simplega {

/*
 * Simple Genome interface.
 *
 * Can be used as a base-class for a genome implementation, but this is
 * optional, as long as the interface is implemented.
 *
 * Note that this class is virtual, as it is conceivable that we could have a
 * heterogeneous collection of genomes, all based off of the same baseclass,
 * where a specialized crossover_mutate operator yields children of different
 * classes.
 */
template <class T>
class Genome {
  public:
    Genome()
    {}

    Genome(const std::vector<T>& g)
        : genome_(g)
    {}

    virtual ~Genome()
    {}

    const std::vector<T>& get() const
    { return genome_; }

    std::vector<T>* getptr()
    { return &genome_; }

    /*
     * Ranks genomes from best fitness to worst fitness.
     * Assumes higher fitness means better.
     */
    virtual bool operator<(const Genome& rhs) const
    {
        return fitness() > rhs.fitness();
    }

    virtual void mutate(float rate) = 0;
    virtual float fitness() const = 0;

    /*
     * As this is also used for the roulette selection, the assumption is that
     * higher fitness means better.
     */
    virtual operator float() const
    {
        return fitness();
    }

    virtual operator std::string() const
    {
        std::ostringstream oss;
        oss << "[";
        bool first = true;
        for (const auto& v : genome_) {
            oss << (first ? "" : ", ") << v;
            first = false;
        }
        oss << "]";
        return oss.str();
    }

  protected:
    std::vector<T> genome_;
};

/*
 * Default crossover_mutate operator using cut&splice.
 *
 * Assumes that GenomeT implements get() and uses a vector-like structure to
 * represent the genome.
 */
template <class URNG, class GenomeT, class C>
void crossover_mutate(URNG& urng, const GenomeT& mate1, const GenomeT& mate2,
                      float mutation_rate, C *container)
{
    assert(!mate1.get().empty());
    assert(!mate2.get().empty());

    std::uniform_int_distribution<size_t> dist1(0, mate1.get().size() - 1);
    std::uniform_int_distribution<size_t> dist2(0, mate2.get().size() - 1);

    const size_t cut1 = dist1(urng);
    const size_t cut2 = dist2(urng);

    // a[0:cut_a] + b[cut_b:]
    auto cut_and_splice = [](const GenomeT& a, const GenomeT& b,
                             size_t cut_a, size_t cut_b) {
        auto result = a.get();
        auto ita = result.begin();
        std::advance(ita, cut_a);
        result.erase(ita, result.end());
        auto itb = b.get().begin();
        std::advance(itb, cut_b);
        result.insert(result.end(), itb, b.get().end());
        return GenomeT(result);
    };

    // child1 = mate1[0:cut1] + mate2[cut2:]
    GenomeT child1 = cut_and_splice(mate1, mate2, cut1, cut2);
    if (child1.get().size()) {
        child1.mutate(mutation_rate);
        container->push_back(child1);
    }

    // child2 = mate2[0:cut2] + mate1[cut1:]
    GenomeT child2 = cut_and_splice(mate2, mate1, cut2, cut1);
    if (child2.get().size()) {
        child2.mutate(mutation_rate);
        container->push_back(child2);
    }
}

/*
 * GenePool
 *
 * Helps evolve a population, and provides various primitives for implementing
 * selection operators.
 */
template <class GenomeT>
class GenePool {
  public:
    /*
     * Use a list for the population pool, as this permits O(1) removal of
     * elements.
     */
    typedef std::list<GenomeT> Population;
    typedef std::vector<GenomeT*> Selection;

    explicit GenePool(size_t target_population_size = 80, float mutation_rate = 0.02f)
        : target_population_size_(target_population_size), mutation_rate_(mutation_rate),
          generation_(0)
    {
        // mutation rate is a percentage
        if (mutation_rate_ > 1.0f)
            mutation_rate_ = 1.0f;
        else if (mutation_rate_ < 0.0f)
            mutation_rate_ = 0.0f;

        // Initialize with defaults (e.g. random)
        population_.resize(target_population_size_);
    }

    explicit GenePool(Population population, float mutation_rate = 0.02f)
        : target_population_size_(population.size()), mutation_rate_(mutation_rate),
          generation_(0), population_(population)
    {}

    explicit GenePool(Selection selection, float mutation_rate = 0.02f)
        : target_population_size_(selection.size()), mutation_rate_(mutation_rate),
          generation_(0)
    {
        for (const auto& gptr : selection) {
            population_.push_back(*gptr);
        }
    }

    operator std::string() const
    {
        std::ostringstream oss;
        oss << "{ ";

        bool first = true;
        for (const auto& genome : population_) {
            oss << (first ? "" : ", ") << static_cast<std::string>(genome);
            first = false;
        }

        oss << " }";
        return oss.str();
    }

    const std::list<GenomeT>& get() const
    { return population_; }

    float mutation_rate() const
    { return mutation_rate_; }

    void set_mutation_rate(float mutation_rate)
    {
        mutation_rate_ = mutation_rate;
    }

    size_t target_population_size() const
    { return target_population_size_; }

    size_t population_size() const
    { return population_.size(); }

    size_t generation() const
    { return generation_; }

    float avg_fitness() const
    {
        float result = 0.0f;
        for (const auto& g : population_) {
            result += g.fitness();
        }
        return result / population_.size();
    }

    float worst_fitness() const
    {
        return std::max_element(population_.begin(), population_.end())
                               ->fitness();
    }

    float best_fitness() const
    {
        return std::min_element(population_.begin(), population_.end())
                               ->fitness();
    }

    /*
     * Sorts the population based on fitness.
     */
    void sort()
    {
        population_.sort();
    }

    GenomeT select_best() const
    {
        return *std::min_element(population_.begin(), population_.end());
    }

    /*
     * Return the entire population.
     */
    Selection select_all()
    {
        Selection result;
        result.reserve(population_.size());
        for (GenomeT& g : population_) {
            result.push_back(&g);
        }
        return result;
    }

    /*
     * Return random subset of population, using distribution dist to select.
     */
    template <class URNG, class DIST>
    Selection select_dist(URNG& urng, DIST& dist, size_t count)
    {
        assert(population_.size() >= count);

        std::unordered_set<size_t> used;
        Selection result;
        result.reserve(count);

        while (result.size() < count) {
            size_t idx = dist(urng);
            if (used.find(idx) != used.end())
                continue;

            auto it = population_.begin();
            std::advance(it, idx);
            result.push_back(&(*it));
            used.insert(idx);
        }

        return result;
    }

    /*
     * Return random subset of population, where a higher fitness means an
     * individual is more likely to be selected.
     */
    template <class URNG>
    Selection select_roulette(URNG& urng, size_t count)
    {
        std::discrete_distribution<size_t> dist(population_.begin(), population_.end());
        return select_dist(urng, dist, count);
    }

    /*
     * Return a random subset of the population, where each individual has the
     * same probability of being selected.
     */
    template <class URNG>
    Selection select_uniform(URNG& urng, size_t count)
    {
        std::uniform_int_distribution<size_t> dist(0, population_.size() - 1);
        return select_dist(urng, dist, count);
    }

    /*
     * Sorts a selection based on fitness.
     */
    void selection_sort(Selection *v)
    {
        std::sort(v->begin(), v->end(),
                  [](const GenomeT* lhs, const GenomeT* rhs)
                  { return (*lhs) < (*rhs); });
    }

    /*
     * Takes a selection and mates the initial selection[:mates]
     * individuals.
     *
     * The elements in selection also determine which individuals are to be
     * removed from the population; selection[elite:] are removed from
     * population.
     */
    template <class URNG>
    void nextgen(URNG& urng, Selection *selection, size_t mates,
                 size_t elite = 0, bool sort_selection = true)
    {
        assert(selection->size() >= mates);
        assert(selection->size() >= elite);

        if (sort_selection) {
            selection_sort(selection);
        }

        size_t replace = selection->size() - elite;
        assert(replace > 0);

        const size_t target_population_size = target_population_size_ + replace;

        // Add offspring
        for (size_t i = 0; i < mates; ++i) {
            const auto mate1 = (*selection)[i];

            // j = i: avoid mating 2 individuals twice
            for (size_t j = i; j < mates; ++j) {
                if (i == j) continue;
                const auto mate2 = (*selection)[j];

                crossover_mutate(urng,
                                 *mate1, *mate2,
                                 mutation_rate_,
                                 &population_);

                if (population_.size() >= target_population_size)
                    goto target_reached;
            }
        }
target_reached:

        // Remove non-elite
        size_t selection_start_idx = elite;
        population_.remove_if([&](const GenomeT& val) {
            if (!replace) return false;
            for (size_t i = selection_start_idx; i < selection->size(); ++i) {
                if (&val == (*selection)[i]) {
                    --replace;
                    return true;
                }
            }
            return false;
        });

        assert(population_.size() >= target_population_size_);
        // The population might be larger than the target, if crossover
        // generates more than one offspring.

        ++generation_;
    }

  protected:
    size_t target_population_size_;
    float mutation_rate_;
    size_t generation_;
    Population population_;
};

} /* namespace simplega */
} /* namespace mc2lib */

#endif /* SIMPLEGA_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
