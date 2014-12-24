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

#ifndef MC2LIB_CODEGEN_OPS_GENERIC_HPP_
#define MC2LIB_CODEGEN_OPS_GENERIC_HPP_

#include <random>

#include "../compiler.hpp"

namespace mc2lib {
namespace codegen {
namespace ops {

class Return : public Operation {
  public:
    explicit Return(mc::Iiid::Pid pid = -1)
        : Operation(pid)
    {}

    void reset() {}

    std::size_t emit_X86_64(AssemblerState *asms,
                            mc::model14::Arch_TSO *arch, InstPtr start,
                            void *code, std::size_t len);

    const mc::Event* insert_po(AssemblerState *asms, mc::model14::ExecWitness *ew,
                               const mc::Event *before) const
    { return nullptr; }

    bool insert_from(AssemblerState *asms, mc::model14::ExecWitness *ew, InstPtr ip,
                     const WriteID *from_id, std::size_t size) const
    { return true; }
};

class Read : public Operation {
  public:
    explicit Read(mc::Event::Addr addr, mc::Iiid::Pid pid = -1)
        : Operation(pid), addr_(addr), event_(nullptr)
    {}

    void reset()
    {
        event_ = nullptr;
    }

    std::size_t emit_X86_64(AssemblerState *asms,
                            mc::model14::Arch_TSO *arch, InstPtr start,
                            void *code, std::size_t len);

    const mc::Event* insert_po(AssemblerState *asms, mc::model14::ExecWitness *ew,
                               const mc::Event *before) const
    {
        assert(event_ != nullptr);

        if (before != nullptr) {
            ew->po.insert(*before, *event_);
        }

        return event_;
    }

    bool insert_from(AssemblerState *asms, mc::model14::ExecWitness *ew, InstPtr ip,
                     const WriteID *from_id, std::size_t size) const
    {
        assert(event_ != nullptr);
        assert(size == 1);

        const mc::Event *from = asms->get_write<1>(event_, addr_, from_id)[0];
        insert_from(ew, from, event_);
        return true;
    }

  protected:
    virtual void insert_from(mc::model14::ExecWitness *ew,
                             const mc::Event *e1, const mc::Event *e2) const
    {
        ew->rf.insert(*e1, *e2);
    }

    mc::Event::Addr addr_;
    const mc::Event *event_;
};

class Write : public Read {
  public:
    explicit Write(mc::Event::Addr addr, mc::Iiid::Pid pid = -1)
        : Read(addr, pid)
    {}

    std::size_t emit_X86_64(AssemblerState *asms,
                            mc::model14::Arch_TSO *arch, InstPtr start,
                            void *code, std::size_t len);

  protected:
    void insert_from(mc::model14::ExecWitness *ew,
                             const mc::Event *e1, const mc::Event *e2) const
    {
        ew->co.insert(*e1, *e2);
    }
};

struct RandomFactory {
    static constexpr std::size_t OPERATIONS = 2;

    explicit RandomFactory(mc::Iiid::Pid min_pid, mc::Iiid::Pid max_pid,
                           mc::Event::Addr min_addr, mc::Event::Addr max_addr)
        : min_pid_(min_pid), max_pid_(max_pid),
          min_addr_(min_addr), max_addr_(max_addr)
    {}

    template <class URNG>
    Operation* operator ()(URNG& urng) const
    {
        std::uniform_int_distribution<std::size_t> dist_idx(0, OPERATIONS - 1);
        std::uniform_int_distribution<mc::Event::Addr> dist_addr(min_addr_, max_addr_);
        std::uniform_int_distribution<mc::Iiid::Pid> dist_pid(min_pid_, max_pid_);

        const auto pid = dist_pid(urng);
        const auto addr = dist_addr(urng);

        switch (dist_idx(urng)) {
            case 0: return new Read(addr, pid);
            case 1: return new Write(addr, pid);
        }

        assert(false);
        return nullptr;
    }

  private:
    mc::Iiid::Pid min_pid_;
    mc::Iiid::Pid max_pid_;
    mc::Event::Addr min_addr_;
    mc::Event::Addr max_addr_;
};

} /* namespace ops */
} /* namespace codegen */
} /* namespace mc2lib */

#endif /* MC2LIB_CODEGEN_OPS_GENERIC_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
