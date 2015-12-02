#include "mc2lib/simplega.hpp"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace mc2lib::simplega;

BOOST_AUTO_TEST_SUITE(simplega)

std::default_random_engine* generator_ptr = nullptr;

class GenomeAdd : public Genome<float> {
 public:
  GenomeAdd() {
    genome_.resize(5);
    MutateImpl(1.0f);
  }

  GenomeAdd(const GenomeAdd& parent1, const GenomeAdd& parent2,
            const std::vector<float>& g)
      : Genome<float>(g) {}

  void Mutate(float rate) override { MutateImpl(rate); };

  float Fitness() const override {
    float total = 0.0;
    for (const auto& f : genome_) {
      total += f;
    }

    // restrict size
    if (genome_.size() > 10) {
      return 999.0f;
    }

    // want to get sum closest to 24.
    return (24 - total) * (24 - total);
  }

  bool operator<(const Genome& rhs) const override {
    return Fitness() < rhs.Fitness();
  }

  operator float() const override { return 1000.f - Fitness(); }

 private:
  void MutateImpl(float rate) {
    std::uniform_int_distribution<std::size_t> dist_idx(0, genome_.size() - 1);
    std::uniform_real_distribution<float> dist_mut(-2.0f, 2.0f);
    std::unordered_set<std::size_t> used;
    std::size_t selection_count =
        static_cast<std::size_t>(static_cast<float>(genome_.size()) * rate);

    while (selection_count) {
      auto idx = dist_idx(*generator_ptr);
      if (used.find(idx) != used.end()) {
        continue;
      }

      genome_[idx] += dist_mut(*generator_ptr);

      used.insert(idx);
      --selection_count;
    }
  }
};

BOOST_AUTO_TEST_CASE(SimpleGAAdd24) {
  std::default_random_engine generator(1234);
  generator_ptr = &generator;

  GenePool<GenomeAdd> pool(25,     // population_size
                           0.3f);  // mutation_rate

  std::size_t tournament_size = 10;
  std::size_t tournament_winners = 5;
  std::size_t elite = tournament_winners;

  for (int i = 0; i < 50; ++i) {
    auto tournament_population = pool.SelectUniform(generator, tournament_size);
    pool.SelectionSort(&tournament_population);
    pool.Step(generator,
              evolve::CutSpliceMutate<std::default_random_engine, GenomeAdd,
                                      GenePool<GenomeAdd>::Population>,
              tournament_population, tournament_winners, elite);
    BOOST_CHECK(pool.population_size() <= pool.target_population_size() + 1);
  }

  // This mainly checks that the discrete_distribution implementation works
  // as expected.
  BOOST_CHECK(
      GenePool<GenomeAdd>(pool.SelectRoulette(generator, tournament_size))
          .AverageFitness() <
      GenePool<GenomeAdd>(pool.SelectUniform(generator, tournament_size))
          .AverageFitness());

  BOOST_CHECK(pool.BestFitness() < pool.WorstFitness());

  auto gene = pool.SelectBest();
  float sum = 0.0f;
  for (const auto& f : gene.Get()) {
    sum += f;
  }

  BOOST_CHECK(sum >= 23.1f && sum <= 24.9);
}

BOOST_AUTO_TEST_SUITE_END()
