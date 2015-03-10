/*
 * Copyright (c) 2014-2015, Marco Elver
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

#include "../simplega.hpp"
#include "compiler.hpp"

#include <random>
#include <unordered_set>

namespace mc2lib {
namespace codegen {

template <class URNG, class OperationFactory>
class RandInstTest : public simplega::Genome<OperationPtr> {
  public:
    explicit RandInstTest(URNG& urng, const OperationFactory *factory, std::size_t len)
        : fitness_(0.0f), urng_(urng), factory_(factory)
    {
        genome_.resize(len);

        for (auto& op_ptr : genome_) {
            op_ptr = (*factory)(urng);
        }
    }

    explicit RandInstTest(const RandInstTest& parent1, const RandInstTest& parent2,
                          const std::vector<OperationPtr>& g)
        : simplega::Genome<OperationPtr>(g)
        , fitness_(0.0f), urng_(parent1.urng_), factory_(parent1.factory_)
    {}

    void mutate(float rate)
    {
        std::uniform_int_distribution<std::size_t> dist_idx(0, genome_.size() - 1);
        std::unordered_set<std::size_t> used;
        std::size_t selection_count = (std::size_t)((float)genome_.size() * rate);

        while (selection_count) {
            auto idx = dist_idx(urng_);
            if (used.find(idx) != used.end())
                continue;

            genome_[idx] = make_random();

            used.insert(idx);
            --selection_count;
        }
    }

    float fitness() const
    { return fitness_; }

    void set_fitness(float fitness)
    { fitness_ = fitness; }

    OperationPtr make_random() const
    {
        return (*factory_)(urng_);
    }

    Threads threads()
    {
        return threads_extract(getptr());
    }

  private:
    float fitness_;
    URNG& urng_;
    const OperationFactory *factory_;
};

} /* namespace codegen */
} /* namespace mc2lib */

#endif /* MC2LIB_CODEGEN_RIT_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
