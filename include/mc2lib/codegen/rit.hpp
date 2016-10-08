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

#ifndef MC2LIB_CODEGEN_RIT_HPP_
#define MC2LIB_CODEGEN_RIT_HPP_

#include <functional>
#include <random>
#include <vector>

#include "../sets.hpp"
#include "../simplega.hpp"
#include "../types.hpp"

namespace mc2lib {
namespace codegen {

template <class URNG, class OperationFactory>
class RandInstTest
    : public simplega::Genome<typename OperationFactory::ResultType::Ptr> {
 public:
  typedef typename OperationFactory::ResultType Operation;
  typedef sets::Set<sets::Types<types::Addr, std::hash<types::Addr>>> AddrSet;

  explicit RandInstTest(URNG& urng, const OperationFactory* factory,
                        std::size_t len)
      : urng_(urng), factory_(factory), fitness_(0.0f) {
    this->genome_.resize(len);

    for (auto& op_ptr : this->genome_) {
      op_ptr = (*factory)(urng);
    }
  }

  explicit RandInstTest(const RandInstTest& parent1,
                        const RandInstTest& parent2,
                        std::vector<typename Operation::Ptr> g)
      : simplega::Genome<typename Operation::Ptr>(std::move(g)),
        urng_(parent1.urng_),
        factory_(parent1.factory_),
        fitness_(0.0f) {}

  void Mutate(float rate) override {
    std::uniform_int_distribution<std::size_t> dist_idx(
        0, this->genome_.size() - 1);
    std::unordered_set<std::size_t> used;
    std::size_t selection_count = static_cast<std::size_t>(
        static_cast<float>(this->genome_.size()) * rate);

    while (selection_count) {
      auto idx = dist_idx(urng_);
      if (used.find(idx) != used.end()) {
        continue;
      }

      this->genome_[idx] = MakeRandom();

      used.insert(idx);
      --selection_count;
    }
  }

  float Fitness() const override { return fitness_; }

  void set_fitness(float fitness) { fitness_ = fitness; }

  const AddrSet& fitaddrs() const { return fitaddrs_; }

  AddrSet* fitaddrsptr() { return &fitaddrs_; }

  typename Operation::Ptr MakeRandom() const { return (*factory_)(urng_); }

  typename Operation::Ptr MakeRandom(const AddrSet& subset_addrs,
                                     std::size_t max_tries = 1000) const {
    return (*factory_)(urng_,
                       [&subset_addrs](types::Addr addr) {
                         return subset_addrs.Contains(addr);
                       },
                       max_tries);
  }

  typename Operation::Threads threads() {
    return ExtractThreads(this->get_ptr());
  }

 private:
  URNG& urng_;
  const OperationFactory* factory_;

  float fitness_;
  AddrSet fitaddrs_;
};

}  // namespace codegen
}  // namespace mc2lib

#endif /* MC2LIB_CODEGEN_RIT_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
