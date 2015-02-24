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

#ifndef MC2LIB_CODEGEN_OPS_GENERIC_HPP_
#define MC2LIB_CODEGEN_OPS_GENERIC_HPP_

#include "../compiler.hpp"

#include <algorithm>
#include <random>

namespace mc2lib {
namespace codegen {

/**
 * @namespace mc2lib::codegen::ops
 * @brief Implementations of Operations.
 */
namespace ops {

class Return : public Operation {
  public:
    explicit Return(types::Pid pid = -1)
        : Operation(pid)
    {}

    OperationPtr clone() const
    {
        return std::make_shared<Return>(*this);
    }

    void reset() {}

    bool enable_emit(AssemblerState *asms)
    { return true; }

    void insert_po(const Operation *before, AssemblerState *asms)
    {}

    std::size_t emit_X86_64(types::InstPtr start, AssemblerState *asms,
                            void *code, std::size_t len);

    const mc::Event* last_event(const mc::Event *next_event,
                                AssemblerState *asms) const
    { return nullptr; }

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms)
    { return true; }
};

class Read : public MemOperation {
  public:
    explicit Read(types::Addr addr, types::Pid pid = -1)
        : MemOperation(pid), addr_(addr), event_(nullptr), from_(nullptr)
    {}

    OperationPtr clone() const
    {
        return std::make_shared<Read>(*this);
    }

    void reset()
    {
        event_ = nullptr;
        from_ = nullptr;
    }

    bool enable_emit(AssemblerState *asms)
    { return !asms->exhausted(); }

    void insert_po(const Operation *before, AssemblerState *asms)
    {
        event_ = asms->make_read<1>(pid(), mc::Event::Read, addr_)[0];

        if (before != nullptr) {
            auto event_before = before->last_event(event_, asms);
            assert(event_before != nullptr);
            asms->ew()->po.insert(*event_before, *event_);
        }
    }

    std::size_t emit_X86_64(types::InstPtr start, AssemblerState *asms,
                            void *code, std::size_t len);

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms)
    {
        assert(event_ != nullptr);
        assert(ip == at_);
        assert(addr == addr_);
        assert(size == 1);

        const mc::Event *from = asms->get_write<1>({event_}, addr_, from_id)[0];

        if (from_ != nullptr) {
            // If from_ == from, we still need to continue to try to erase and
            // insert, in case the from-relation has been cleared.

            erase_from_helper(from_, event_, asms->ew());
        }

        from_ = from;
        insert_from_helper(from_, event_, asms->ew());

        return true;
    }

    const mc::Event* last_event(const mc::Event *next_event,
                                AssemblerState *asms) const
    { return event_; }

    types::Addr addr() const
    { return addr_; }

  protected:
    virtual void insert_from_helper(const mc::Event *e1, const mc::Event *e2,
                                    mc::model14::ExecWitness *ew)
    {
        ew->rf.insert(*e1, *e2, true);
    }

    virtual void erase_from_helper(const mc::Event *e1, const mc::Event *e2,
                                   mc::model14::ExecWitness *ew)
    {
        ew->rf.erase(*e1, *e2);
    }

    types::Addr addr_;
    const mc::Event *event_;
    const mc::Event *from_;
    types::InstPtr at_;
};

class Write : public Read {
  public:
    explicit Write(types::Addr addr, types::Pid pid = -1)
        : Read(addr, pid), write_id_(0)
    {}

    OperationPtr clone() const
    {
        return std::make_shared<Write>(*this);
    }

    void reset()
    {
        event_ = nullptr;
        from_ = nullptr;
        write_id_ = 0;
    }

    void insert_po(const Operation *before, AssemblerState *asms)
    {
        event_ = asms->make_write<1>(pid(), mc::Event::Write, addr_, &write_id_)[0];

        if (before != nullptr) {
            auto event_before = before->last_event(event_, asms);
            assert(event_before != nullptr);
            asms->ew()->po.insert(*event_before, *event_);
        }
    }

    std::size_t emit_X86_64(types::InstPtr start, AssemblerState *asms,
                            void *code, std::size_t len);

  protected:
    virtual void insert_from_helper(const mc::Event *e1, const mc::Event *e2,
                                    mc::model14::ExecWitness *ew)
    {
        ew->co.insert(*e1, *e2);
    }

    virtual void erase_from_helper(const mc::Event *e1, const mc::Event *e2,
                                   mc::model14::ExecWitness *ew)
    {
        ew->co.erase(*e1, *e2);
    }

    types::WriteID write_id_;
};

class ReadModifyWrite : public MemOperation {
  public:
    explicit ReadModifyWrite(types::Addr addr, types::Pid pid = -1)
        : MemOperation(pid), addr_(addr), last_part_(-1)
    {}

    OperationPtr clone() const
    {
        return std::make_shared<ReadModifyWrite>(*this);
    }

    void reset()
    {
        last_part_ = -1;
        event_r_ = nullptr;
        event_w_ = nullptr;
        from_ = nullptr;
        write_id_ = 0;
    }

    bool enable_emit(AssemblerState *asms)
    { return !asms->exhausted(); }

    void insert_po(const Operation *before, AssemblerState *asms)
    {
        event_r_ = asms->make_read<1>(pid(), mc::Event::Read, addr_)[0];
        event_w_ = asms->make_write<1>(pid(), mc::Event::Write, addr_, &write_id_)[0];

        if (before != nullptr) {
            auto event_before = before->last_event(event_r_, asms);
            assert(event_before != nullptr);

            asms->ew()->po.insert(*event_before, *event_r_);

            // Implied fence before atomic
            if (dynamic_cast<mc::model14::Arch_TSO*>(asms->arch()) != nullptr) {
                auto arch_tso = dynamic_cast<mc::model14::Arch_TSO*>(asms->arch());
                arch_tso->mfence.insert(*event_before, *event_r_);
            }

        }

        asms->ew()->po.insert(*event_r_, *event_w_);
    }

    std::size_t emit_X86_64(types::InstPtr start, AssemblerState *asms,
                            void *code, std::size_t len);

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms)
    {
        assert(event_r_ != nullptr);
        assert(event_w_ != nullptr);
        assert(ip == at_);
        assert(addr == addr_);
        assert(size == 1);

        // This also alerts us if the read would be seeing the write's data.
        auto from = asms->get_write<1>({event_w_}, addr_, from_id)[0];

        auto part_event = event_r_;
        auto obs_rel = &asms->ew()->rf;

        // TODO: clean this up! Do we need ability to update squashed?

        if (last_part_ == -1 || part <= last_part_) {
            // First part: read

            if (from_ != nullptr) {
                // Restart
                asms->ew()->rf.erase(*from_, *event_r_);
                asms->ew()->co.erase(*from_, *event_w_);
            }
        } else {
            // Second part: write
            assert(part > last_part_);

            // Assert atomicity; pointer comparison is fine, as get_write
            // returns unique instances for each Event.
            assert(from == from_ && "Not atomic!");

            part_event = event_w_;
            obs_rel = &asms->ew()->co;
        }

        obs_rel->insert(*from, *part_event, true);

        from_ = from;
        last_part_ = part;
        return true;
    }

    const mc::Event* last_event(const mc::Event *next_event,
                                AssemblerState *asms) const
    {
        // Implied fence after atomic
        if (dynamic_cast<mc::model14::Arch_TSO*>(asms->arch()) != nullptr) {
            auto arch_tso = dynamic_cast<mc::model14::Arch_TSO*>(asms->arch());
            arch_tso->mfence.insert(*event_w_, *next_event);
        }

        return event_w_;
    }

    types::Addr addr() const
    { return addr_; }

  protected:
    types::Addr addr_;
    int last_part_;
    const mc::Event *event_r_;
    const mc::Event *event_w_;
    const mc::Event *from_;
    types::WriteID write_id_;
    types::InstPtr at_;
};

/**
 * RandomFactory.
 */
struct RandomFactory {
    explicit RandomFactory(types::Pid min_pid, types::Pid max_pid,
                           types::Addr min_addr, types::Addr max_addr)
        : min_pid_(min_pid), max_pid_(max_pid),
          min_addr_(min_addr), max_addr_(max_addr)
    {}

    void reset(types::Pid min_pid, types::Pid max_pid,
               types::Addr min_addr, types::Addr max_addr)
    {
        min_pid_ = min_pid;
        max_pid_ = max_pid;
        min_addr_ = min_addr;
        max_addr_ = max_addr;
    }

    template <class URNG>
    OperationPtr operator ()(URNG& urng) const
    {
        std::uniform_int_distribution<types::Addr> dist_addr(min_addr_,
                                    max_addr_ - AssemblerState::MAX_INST_SIZE);
        std::uniform_int_distribution<types::Pid> dist_pid(min_pid_, max_pid_);

        const auto pid = dist_pid(urng);
        const auto addr = dist_addr(urng);

        std::uniform_int_distribution<std::size_t> dist_choice(0, 100);
        const auto choice = dist_choice(urng);

        if (choice <= 50) // 50%
            return std::make_shared<Read>(addr, pid);
        else if (choice <= 99) // 49%
            return std::make_shared<Write>(addr, pid);
        else if (choice <= 100) // 1%
            return std::make_shared<ReadModifyWrite>(addr, pid);

        assert(false);
        return nullptr;
    }

    types::Pid min_pid() const
    { return min_pid_; }

    types::Pid max_pid() const
    { return max_pid_; }

    types::Addr min_addr() const
    { return min_addr_; }

    types::Addr max_addr() const
    { return max_addr_; }

  private:
    types::Pid min_pid_;
    types::Pid max_pid_;
    types::Addr min_addr_;
    types::Addr max_addr_;
};

} /* namespace ops */
} /* namespace codegen */
} /* namespace mc2lib */

#endif /* MC2LIB_CODEGEN_OPS_GENERIC_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
