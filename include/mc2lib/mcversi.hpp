/*
 * Copyright (c) 2015-2016, Marco Elver
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

#ifndef MC2LIB_MCVERSI_HPP_
#define MC2LIB_MCVERSI_HPP_

#include <cassert>
#include <cstddef>
#include <random>

#include "simplega.hpp"

namespace mc2lib {

/**
 * @namespace mc2lib::mcversi
 * @brief Implementations of algorithms from McVerSi paper.
 */
namespace mcversi {

/**
 * Crossover and mutation (Algorithm 1) from McVerSi paper.
 */
template <class URNG, class RandInstTest, class MemOperation>
class CrossoverMutate {
 public:
  explicit CrossoverMutate(double P_USEL, double P_BFA)
      : P_USEL_(P_USEL), P_BFA_(P_BFA) {}

  void operator()(
      URNG& urng, const RandInstTest& test1, const RandInstTest& test2,
      float mutation_rate,
      typename simplega::GenePool<RandInstTest>::Population* container) {
    assert(test1.get().size() == test2.get().size());

    // Probability with which we unconditionally select a particular gene. We
    // don't always just want to pick fitaddr operations, as surrounding
    // operations may contribute to coverage or other timing effects.
    std::bernoulli_distribution mem_unconditional_select(P_USEL_);

    const double faf1 = FitaddrFraction(test1);
    const double faf2 = FitaddrFraction(test2);

    // Same rate as selecting a memory-operation.
    std::bernoulli_distribution nonmem_select1(faf1 + P_USEL_ -
                                               (faf1 * P_USEL_));
    std::bernoulli_distribution nonmem_select2(faf2 + P_USEL_ -
                                               (faf2 * P_USEL_));

    // Tiebreaker: If both are valid, pick one consistently and do not break up
    // a good genome's dominant genes as they may interact in an imporant way.
    const bool prefer_test2 = std::bernoulli_distribution(0.5)(urng);

    // In case both operations are invalid, probability with which we make a
    // new operation with the address-set restricted to fitaddrs.
    std::bernoulli_distribution new_from_fitaddrs(P_BFA_);
    const auto all_fitaddrs = test1.fitaddrs() | test2.fitaddrs();

    auto child = test1.get();
    std::size_t mutations = 0;

    for (std::size_t i = 0; i < child.size(); ++i) {
      bool select1 = false;
      bool select2 = false;

      auto mem_op1 = dynamic_cast<MemOperation*>(test1.get()[i].get());
      auto mem_op2 = dynamic_cast<MemOperation*>(test2.get()[i].get());

      // Decide validity of genes
      if (mem_op1 != nullptr) {
        select1 = test1.fitaddrs().Contains(mem_op1->addr()) ||
                  mem_unconditional_select(urng);
      } else {
        select1 = nonmem_select1(urng);
      }

      if (mem_op2 != nullptr) {
        select2 = test2.fitaddrs().Contains(mem_op2->addr()) ||
                  mem_unconditional_select(urng);
      } else {
        select2 = nonmem_select2(urng);
      }

      // Pick gene
      if (select1 && select2) {
        if (prefer_test2) {
          child[i] = test2.get()[i];
        }
      } else if (!select1 && select2) {
        child[i] = test2.get()[i];
      } else if (!select1 && !select2) {
        ++mutations;

        if (new_from_fitaddrs(urng)) {
          // Make new random operation, but only select from set of
          // fitaddrs.
          child[i] = test1.MakeRandom(all_fitaddrs);
        } else {
          // By deciding to discard fit addresses, we control where the
          // tests mutate, so that in the initial phases the good
          // operations do not get mutated away.
          //
          child[i] = test1.MakeRandom();
        }
      } else {
        assert(select1);
        // child[i] is valid, don't do anything.
      }
    }

    auto result = RandInstTest(test1, test2, child);

    float cur_mutation =
        static_cast<float>(mutations) / static_cast<float>(child.size());
    if (cur_mutation < mutation_rate) {
      // cur_mutation is the fraction of newly generated instructions, i.e.
      // replaced non-dominant genes. We must mutate full mutation_rate
      // again, as newly generated instructions are considered for mutation
      // as well (redundant).
      result.Mutate(mutation_rate);
    } else {
      // No mutation neccessary.
    }

    container->push_back(std::move(result));
  }

 private:
  double P_USEL_;
  double P_BFA_;

  double FitaddrFraction(const RandInstTest& rit) {
    std::size_t mem_ops = 0;
    std::size_t fitaddr_ops = 0;

    for (const auto& op : rit.get()) {
      auto mem_op = dynamic_cast<MemOperation*>(op.get());
      if (mem_op != nullptr) {
        ++mem_ops;
        if (rit.fitaddrs().Contains(mem_op->addr())) {
          ++fitaddr_ops;
        }
      }
    }

    return static_cast<double>(fitaddr_ops) / static_cast<double>(mem_ops);
  }
};

}  // namespace mcversi
}  // namespace mc2lib

#endif /* MCVERSI_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
