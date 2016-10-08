/*
 * Copyright (c) 2014-2016, Marco Elver
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
#include <cstddef>
#include <list>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mc2lib {

/**
 * @namespace mc2lib::simplega
 * @brief Simple Genetic Algorithm library.
 */
namespace simplega {

/**
 * @namespace mc2lib::simplega::evolve
 * @brief Example CrossoverMutateFunc implementations.
 */
namespace evolve {

/**
 * Cut & splice. If template parameter one_point is set, turns this into
 * one-point crossover.
 *
 * Assumes that GenomeT implements get() and uses a vector-like structure to
 * represent the genome.
 */
template <class URNG, class GenomeT, class C, bool one_point = false,
          bool theone = false>
inline void CutSpliceMutate(URNG& urng, const GenomeT& mate1,
                            const GenomeT& mate2, float mutation_rate,
                            C* container) {
  assert(!mate1.get().empty());
  assert(!mate2.get().empty());

  std::uniform_int_distribution<std::size_t> dist1(0, mate1.get().size() - 1);
  std::uniform_int_distribution<std::size_t> dist2(0, mate2.get().size() - 1);

  const std::size_t cut1 = dist1(urng);
  const std::size_t cut2 = one_point ? cut1 : dist2(urng);

  // a[0:cut_a] + b[cut_b:]
  auto cut_and_splice = [](const GenomeT& a, const GenomeT& b,
                           std::size_t cut_a, std::size_t cut_b) {
    auto result = a.get();
    auto ita = result.begin();
    std::advance(ita, cut_a);
    result.erase(ita, result.end());
    auto itb = b.get().begin();
    std::advance(itb, cut_b);
    result.insert(result.end(), itb, b.get().end());
    return GenomeT(a, b, result);
  };

  // child1 = mate1[0:cut1] + mate2[cut2:]
  GenomeT child1 = cut_and_splice(mate1, mate2, cut1, cut2);
  if (child1.get().size()) {
    child1.Mutate(mutation_rate);
    container->push_back(std::move(child1));
  }

  if (!theone) {
    // child2 = mate2[0:cut2] + mate1[cut1:]
    GenomeT child2 = cut_and_splice(mate2, mate1, cut2, cut1);
    if (child2.get().size()) {
      child2.Mutate(mutation_rate);
      container->push_back(std::move(child2));
    }
  }
}

}  // namespace evolve

/**
 * @brief Simple Genome interface.
 *
 * Use as a baseclass for genome implementations, but this is optional, as long
 * as the interface is implemented (see GenePool).
 *
 * Note that this class is virtual, as it is possible that we could have a
 * heterogeneous collection of genomes, all based off of the same baseclass,
 * e.g., where a specialized crossover_mutate operator yields children of
 * different classes.
 *
 * @tparam T The type of genes in this Genome.
 */
template <class T>
class Genome {
 public:
  typedef std::vector<T> Container;

  /**
   * @brief Default constructor.
   *
   * Yields an empty genome.
   */
  Genome() {}

  /**
   * @brief Converting constructor.
   *
   * @param g A raw vector of type T genes forming this new Genome.
   */
  explicit Genome(Container g) : genome_(std::move(g)) {}

  virtual ~Genome() {}

  /**
   * @brief Read-only genome accessor.
   *
   * @return Const reference to vector of genes.
   */
  const Container& get() const { return genome_; }

  /**
   * @brief Modifiable genome accessor.
   *
   * @return Pointer to vector of genes.
   */
  Container* get_ptr() { return &genome_; }

  /**
   * @brief Less than comparison operator.
   *
   * Use to rank genomes from best fitness to worst fitness. Assumes higher
   * fitness means better.
   *
   * @return True if this instance (lhs) is fitter than rhs, false otherwise.
   */
  virtual bool operator<(const Genome& rhs) const {
    return Fitness() > rhs.Fitness();
  }

  /**
   * @brief Mutate this Genome.
   *
   * @param rate Mutation rate.
   */
  virtual void Mutate(float rate) = 0;

  /**
   * @brief Fitness accessor.
   *
   * @return Current fitness.
   */
  virtual float Fitness() const = 0;

  /**
   * @brief Converting operator to float.
   *
   * As this is also used for the roulette selection, the assumption is that
   * higher fitness means better.
   */
  virtual operator float() const { return Fitness(); }

  /**
   * @brief Converting operator to std::string.
   */
  virtual operator std::string() const {
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
  /**
   * @brief Raw genome of individual genes of T.
   */
  Container genome_;
};

/**
 * @brief Helper to manages and evolve a populates.
 *
 * Helps manage and evolve a population, and provides various primitives for
 * implementing selection operators.
 */
template <class GenomeT>
class GenePool {
 public:
  /**
   * Using a list for the population pool, as this permits O(1) removal of
   * elements.
   */
  typedef std::list<GenomeT> Population;
  typedef std::vector<GenomeT*> Selection;

  explicit GenePool(std::size_t target_population_size = 80,
                    float mutation_rate = 0.02f)
      : target_population_size_(target_population_size),
        mutation_rate_(mutation_rate),
        steps_(0) {
    // mutation rate is a percentage
    if (mutation_rate_ > 1.0f) {
      mutation_rate_ = 1.0f;
    } else if (mutation_rate_ < 0.0f) {
      mutation_rate_ = 0.0f;
    }

    // Initialize with defaults (e.g. random)
    population_.resize(target_population_size_);
  }

  explicit GenePool(Population population, float mutation_rate = 0.02f)
      : target_population_size_(population.size()),
        mutation_rate_(mutation_rate),
        steps_(0),
        population_(std::move(population)) {}

  explicit GenePool(Selection selection, float mutation_rate = 0.02f)
      : target_population_size_(selection.size()),
        mutation_rate_(mutation_rate),
        steps_(0) {
    for (const auto& gptr : selection) {
      population_.push_back(*gptr);
    }
  }

  operator std::string() const {
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

  const Population& get() const { return population_; }

  Population* get_ptr() { return &population_; }

  float mutation_rate() const { return mutation_rate_; }

  void set_mutation_rate(float mutation_rate) {
    mutation_rate_ = mutation_rate;
  }

  std::size_t target_population_size() const { return target_population_size_; }

  std::size_t population_size() const { return population_.size(); }

  std::size_t steps() const { return steps_; }

  float AverageFitness() const {
    float result = 0.0f;
    for (const auto& g : population_) {
      result += g.Fitness();
    }
    return result / population_.size();
  }

  float WorstFitness() const {
    return std::max_element(population_.begin(), population_.end())->Fitness();
  }

  float BestFitness() const {
    return std::min_element(population_.begin(), population_.end())->Fitness();
  }

  /**
   * Sorts (in-place) the population based on fitness.
   */
  void Sort() { population_.Sort(); }

  const GenomeT& SelectBest() const {
    return *std::min_element(population_.begin(), population_.end());
  }

  /**
   * @return Entire population as Selection.
   */
  Selection SelectAll() {
    Selection result;
    result.reserve(population_.size());
    for (GenomeT& g : population_) {
      result.push_back(&g);
    }
    return result;
  }

  /**
   * @return Random subset of population, using distribution dist to select.
   */
  template <class URNG, class DIST>
  Selection SelectDist(URNG& urng, DIST& dist, std::size_t count) {
    assert(population_.size() >= count);

    std::unordered_set<std::size_t> used;
    Selection result;
    result.reserve(count);

    while (result.size() < count) {
      std::size_t idx = dist(urng);
      if (used.find(idx) != used.end()) {
        continue;
      }

      auto it = population_.begin();
      std::advance(it, idx);
      result.push_back(&(*it));
      used.insert(idx);
    }

    return result;
  }

  /**
   * @return Random subset of population, where a higher fitness means an
   *         individual is more likely to be selected.
   */
  template <class URNG>
  Selection SelectRoulette(URNG& urng, std::size_t count) {
    std::discrete_distribution<std::size_t> dist(population_.begin(),
                                                 population_.end());
    return SelectDist(urng, dist, count);
  }

  /**
   * @return Random subset of the population, where each individual has the
   *         same probability of being selected.
   */
  template <class URNG>
  Selection SelectUniform(URNG& urng, std::size_t count) {
    std::uniform_int_distribution<std::size_t> dist(0, population_.size() - 1);
    return SelectDist(urng, dist, count);
  }

  /**
   * Sorts (in-place) a Selection based on fitness.
   */
  void SelectionSort(Selection* v) {
    std::sort(v->begin(), v->end(), [](const GenomeT* lhs, const GenomeT* rhs) {
      return (*lhs) < (*rhs);
    });
  }

  /**
   * Takes a selection and mates the initial selection[:mates] individuals.
   *
   * The elements in selection also determine which individuals are to be
   * removed from the population; selection[keep:] are removed from population
   * (can e.g. be used for elitism).
   */
  template <class URNG, class CrossoverMutateFunc>
  void Step(URNG& urng, CrossoverMutateFunc crossover_mutate,
            const Selection& selection, std::size_t mates, std::size_t keep = 0,
            std::size_t mate1_stride = 1, std::size_t mate2_stride = 1) {
    assert(selection.size() >= mates);
    assert(selection.size() >= keep);

    std::size_t replace = selection.size() - keep;
    assert(replace > 0);

    const std::size_t target_population_size =
        target_population_size_ + replace;

    // Add offspring
    for (std::size_t i = 0; i < mates; i += mate1_stride) {
      const auto mate1 = selection[i];

      // j = i: avoid mating 2 individuals twice
      for (std::size_t j = i + 1; j < mates; j += mate2_stride) {
        const auto mate2 = selection[j];

        crossover_mutate(urng, *mate1, *mate2, mutation_rate_, &population_);

        if (population_.size() >= target_population_size) {
          goto target_reached;
        }
      }
    }
  target_reached:

    // Remove selection[keep:]
    auto selection_start = selection.begin();
    std::advance(selection_start, keep);

    for (auto it = population_.begin();
         it != population_.end() && replace > 0;) {
      const GenomeT* val = &(*it);
      auto match = std::find(selection_start, selection.end(), val);

      if (match != selection.end()) {
        if (match == selection_start) {
          ++selection_start;
        }

        it = population_.erase(it);
        --replace;
      } else {
        ++it;
      }
    }

    assert(population_.size() >= target_population_size_);
    // The population might be larger than the target, if crossover
    // generates more than one offspring.

    ++steps_;
  }

 protected:
  std::size_t target_population_size_;
  float mutation_rate_;
  std::size_t steps_;
  Population population_;
};

}  // namespace simplega
}  // namespace mc2lib

#endif /* SIMPLEGA_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
