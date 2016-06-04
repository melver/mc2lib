// This code is licensed under the BSD 3-Clause license. See the LICENSE file
// in the project root for license terms.

#include "mc2lib/codegen/ops/strong.hpp"
#include "mc2lib/codegen/rit.hpp"
#include "mc2lib/mcversi.hpp"

#include <gtest/gtest.h>

using namespace mc2lib;

// This can only check syntax and type correctness: most literals are
// arbitrary.
TEST(McVerSi, CrossoverMutate) {
  std::default_random_engine urng;
  codegen::strong::RandomFactory factory(0, 2, 0, 10);

  typedef codegen::RandInstTest<std::default_random_engine,
                                codegen::strong::RandomFactory> RandInstTest;

  simplega::GenePool<RandInstTest>::Population initial_population;

  for (size_t i = 0; i < 10; ++i) {
    initial_population.emplace_back(urng, &factory, 20);
  }

  simplega::GenePool<RandInstTest> pool(initial_population, 0.1f);

  auto selection = pool.SelectUniform(urng, 3);

  // Same values as in McVerSi paper.
  mcversi::CrossoverMutate<std::default_random_engine, RandInstTest,
                           codegen::strong::MemOperation>
      crossover_mutate(0.2, 0.05);

  pool.Step(urng, crossover_mutate, selection, selection.size());
}
