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

#ifndef MC2LIB_CODEGEN_OPS_STRONG_HPP_
#define MC2LIB_CODEGEN_OPS_STRONG_HPP_

#include "../../config.hpp"
#include "../compiler.hpp"

#include <algorithm>
#include <random>

namespace mc2lib {
namespace codegen {

/**
 * @namespace mc2lib::codegen::strong
 * @brief Implementations of Operations for strong memory consistency models.
 */
namespace strong {

struct Backend {
    virtual ~Backend() {}

    virtual std::size_t Return(void *code, std::size_t len) const = 0;

    virtual std::size_t Delay(std::size_t length, void *code, std::size_t len) const = 0;

    virtual std::size_t Read(types::Addr addr, types::InstPtr start, void *code,
                             std::size_t len, types::InstPtr *at) const = 0;

    virtual std::size_t ReadAddrDp(types::Addr addr, types::InstPtr start,
                                   void *code, std::size_t len, types::InstPtr *at) const = 0;

    virtual std::size_t Write(types::Addr addr, types::WriteID write_id,
                              types::InstPtr start, void *code, std::size_t len,
                              types::InstPtr *at) const = 0;

    virtual std::size_t ReadModifyWrite(types::Addr addr, types::WriteID write_id,
                                        types::InstPtr start, void *code, std::size_t len,
                                        types::InstPtr *at) const = 0;

    virtual std::size_t CacheFlush(types::Addr addr, void *code, std::size_t len) const = 0;
};

typedef Op<Backend> Operation;
typedef MemOp<Backend> MemOperation;
typedef NullOp<Backend> NullOperation;

class Return : public Operation
{
  public:
    explicit Return(types::Pid pid = -1)
        : Operation(pid)
    {}

    Operation::Ptr clone() const override
    {
        return std::make_shared<Return>(*this);
    }

    void reset() override {}

    bool enable_emit(AssemblerState *asms) override
    { return true; }

    void insert_po(Operation::SeqConstIt before, AssemblerState *asms) override
    {}

    std::size_t emit(const Backend *backend, types::InstPtr start,
                     AssemblerState *asms, void *code, std::size_t len) override
    {
        return backend->Return(code, len);
    }

    const mc::Event* last_event(const mc::Event *next_event,
                                AssemblerState *asms) const override
    { return nullptr; }

    const mc::Event* first_event(const mc::Event *prev_event,
                                 AssemblerState *asms) const override
    { return nullptr; }

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms) override
    { return true; }
};

class Delay : public Operation
{
  public:
    explicit Delay(std::size_t length, types::Pid pid = -1)
        : Operation(pid), length_(length), before_(nullptr)
    {}

    Operation::Ptr clone() const override
    {
        return std::make_shared<Delay>(*this);
    }

    void reset() override
    {
        before_ = nullptr;
    }

    bool enable_emit(AssemblerState *asms) override
    { return true; }

    void insert_po(Operation::SeqConstIt before, AssemblerState *asms) override
    {
        before_ = *before;
    }

    std::size_t emit(const Backend *backend, types::InstPtr start,
                     AssemblerState *asms, void *code, std::size_t len) override
    {
        return backend->Delay(length_, code, len);
    }

    const mc::Event* last_event(const mc::Event *next_event,
                                AssemblerState *asms) const override
    {
        // Forward
        if (before_ != nullptr) {
            return before_->last_event(next_event, asms);
        }

        return nullptr;
    }

    const mc::Event* first_event(const mc::Event *prev_event,
                                 AssemblerState *asms) const override
    {
        // Forward
        if (before_ != nullptr) {
            return before_->first_event(prev_event, asms);
        }

        return nullptr;
    }

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms) override
    {
        assert(false);
        return false;
    }

  protected:
    std::size_t length_;
    const Operation *before_;
};

class Read : public MemOperation
{
  public:
    explicit Read(types::Addr addr, types::Pid pid = -1)
        : MemOperation(pid), addr_(addr), event_(nullptr), from_(nullptr)
    {}

    Operation::Ptr clone() const override
    {
        return std::make_shared<Read>(*this);
    }

    void reset() override
    {
        event_ = nullptr;
        from_ = nullptr;
    }

    bool enable_emit(AssemblerState *asms) override
    { return !asms->exhausted(); }

    void insert_po(Operation::SeqConstIt before, AssemblerState *asms) override
    {
        event_ = asms->make_read(pid(), mc::Event::Read, addr_)[0];

        if (*before != nullptr) {
            auto event_before = (*before)->last_event(event_, asms);
            if (event_before != nullptr) {
                asms->ew()->po.insert(*event_before, *event_);
            }
        }
    }

    std::size_t emit(const Backend *backend, types::InstPtr start,
                     AssemblerState *asms, void *code, std::size_t len) override
    {
        return backend->Read(addr_, start, code, len, &at_);
    }

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms) override
    {
        assert(event_ != nullptr);
        assert(ip == at_);
        assert(addr == addr_);
        assert(size == sizeof(types::WriteID));

        const mc::Event *from = asms->get_write(make_eventptrs(event_), addr_, from_id)[0];

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
                                AssemblerState *asms) const override
    { return event_; }

    const mc::Event* first_event(const mc::Event *prev_event,
                                AssemblerState *asms) const override
    { return event_; }

    types::Addr addr() const override
    { return addr_; }

  protected:
    virtual void insert_from_helper(const mc::Event *e1, const mc::Event *e2,
                                    mc::cats::ExecWitness *ew)
    {
        ew->rf.insert(*e1, *e2, true);
    }

    virtual void erase_from_helper(const mc::Event *e1, const mc::Event *e2,
                                   mc::cats::ExecWitness *ew)
    {
        ew->rf.erase(*e1, *e2);
    }

    types::Addr addr_;
    const mc::Event *event_;
    const mc::Event *from_;
    types::InstPtr at_;
};

class ReadAddrDp : public Read
{
  public:
    explicit ReadAddrDp(types::Addr addr, types::Pid pid = -1)
        : Read(addr, pid)
    {}

    Operation::Ptr clone() const override
    {
        return std::make_shared<ReadAddrDp>(*this);
    }

    // TODO: insert_po: if we start supporting an Arch which does not order
    // Read->Read, add a dependency-hb between this and the last Read -- this
    // assumes all Reads are reading into the same register, and this read
    // computes the address with this one register. NOTE: before can be used to
    // traverse operations backwards before "before".

    std::size_t emit(const Backend *backend, types::InstPtr start,
                     AssemblerState *asms, void *code, std::size_t len) override
    {
        return backend->ReadAddrDp(addr_, start, code, len, &at_);
    }
};

class Write : public Read
{
  public:
    explicit Write(types::Addr addr, types::Pid pid = -1)
        : Read(addr, pid), write_id_(0)
    {}

    Operation::Ptr clone() const override
    {
        return std::make_shared<Write>(*this);
    }

    void reset() override
    {
        event_ = nullptr;
        from_ = nullptr;
        write_id_ = 0;
    }

    void insert_po(Operation::SeqConstIt before, AssemblerState *asms) override
    {
        event_ = asms->make_write(pid(), mc::Event::Write, addr_, &write_id_)[0];

        if (*before != nullptr) {
            auto event_before = (*before)->last_event(event_, asms);
            if (event_before != nullptr) {
                asms->ew()->po.insert(*event_before, *event_);
            }
        }
    }

    std::size_t emit(const Backend *backend, types::InstPtr start,
                     AssemblerState *asms, void *code, std::size_t len) override
    {
        return backend->Write(addr_, write_id_, start, code, len, &at_);
    }

  protected:
    void insert_from_helper(const mc::Event *e1, const mc::Event *e2,
                            mc::cats::ExecWitness *ew) override
    {
        ew->co.insert(*e1, *e2);
    }

    void erase_from_helper(const mc::Event *e1, const mc::Event *e2,
                           mc::cats::ExecWitness *ew) override
    {
        ew->co.erase(*e1, *e2);
    }

    types::WriteID write_id_;
};

class ReadModifyWrite : public MemOperation
{
  public:
    explicit ReadModifyWrite(types::Addr addr, types::Pid pid = -1)
        : MemOperation(pid), addr_(addr), last_part_(-1)
    {}

    Operation::Ptr clone() const override
    {
        return std::make_shared<ReadModifyWrite>(*this);
    }

    void reset() override
    {
        last_part_ = -1;
        event_r_ = nullptr;
        event_w_ = nullptr;
        from_ = nullptr;
        write_id_ = 0;
    }

    bool enable_emit(AssemblerState *asms) override
    { return !asms->exhausted(); }

    void insert_po(Operation::SeqConstIt before, AssemblerState *asms) override
    {
        event_r_ = asms->make_read(pid(), mc::Event::Read, addr_)[0];
        event_w_ = asms->make_write(pid(), mc::Event::Write, addr_, &write_id_)[0];

        if (*before != nullptr) {
            auto event_before = (*before)->last_event(event_r_, asms);
            if (event_before != nullptr) {
                asms->ew()->po.insert(*event_before, *event_r_);

                if (dynamic_cast<mc::cats::Arch_TSO*>(asms->arch()) != nullptr) {
                    // Implied fence before atomic
                    auto arch_tso = dynamic_cast<mc::cats::Arch_TSO*>(asms->arch());
                    arch_tso->mfence.insert(*event_before, *event_r_);
                }
            }
        }

        asms->ew()->po.insert(*event_r_, *event_w_);
    }

    std::size_t emit(const Backend *backend, types::InstPtr start,
                     AssemblerState *asms, void *code, std::size_t len) override
    {
        return backend->ReadModifyWrite(addr_, write_id_, start, code, len, &at_);
    }

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms) override
    {
        assert(event_r_ != nullptr);
        assert(event_w_ != nullptr);
        assert(ip == at_);
        assert(addr == addr_);
        assert(size == sizeof(types::WriteID));

        // This also alerts us if the read would be seeing the write's data.
        const mc::Event *from = asms->get_write(make_eventptrs(event_w_), addr_, from_id)[0];

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
                                AssemblerState *asms) const override
    {
        if (dynamic_cast<mc::cats::Arch_TSO*>(asms->arch()) != nullptr) {
            // Implied fence after atomic
            auto arch_tso = dynamic_cast<mc::cats::Arch_TSO*>(asms->arch());
            arch_tso->mfence.insert(*event_w_, *next_event);
        }

        return event_w_;
    }

    const mc::Event* first_event(const mc::Event *prev_event,
                                 AssemblerState *asms) const override
    { return event_r_; }

    types::Addr addr() const override
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

class CacheFlush : public MemOperation
{
  public:
    explicit CacheFlush(types::Addr addr, types::Pid pid = -1)
        : MemOperation(pid), addr_(addr), before_(nullptr)
    {}

    Operation::Ptr clone() const override
    {
        return std::make_shared<CacheFlush>(*this);
    }

    void reset() override
    {
        before_ = nullptr;
    }

    bool enable_emit(AssemblerState *asms) override
    { return true; }

    void insert_po(Operation::SeqConstIt before, AssemblerState *asms) override
    {
        before_ = *before;
    }

    std::size_t emit(const Backend *backend, types::InstPtr start,
                     AssemblerState *asms, void *code, std::size_t len) override
    {
        return backend->CacheFlush(addr_, code, len);
    }

    const mc::Event* last_event(const mc::Event *next_event,
                                AssemblerState *asms) const override
    {
        // Forward
        if (before_ != nullptr) {
            return before_->last_event(next_event, asms);
        }

        return nullptr;
    }

    const mc::Event* first_event(const mc::Event *prev_event,
                                 AssemblerState *asms) const override
    {
        // Forward
        if (before_ != nullptr) {
            return before_->first_event(prev_event, asms);
        }

        return nullptr;
    }

    bool update_from(types::InstPtr ip, int part, types::Addr addr,
                     const types::WriteID *from_id, std::size_t size,
                     AssemblerState *asms) override
    {
        return true;
    }

    types::Addr addr() const override
    { return addr_; }

  protected:
    types::Addr addr_;
    const Operation *before_;
};

class ReadSequence : public NullOperation
{
  public:
    explicit ReadSequence(types::Addr min_addr, types::Addr max_addr, types::Pid pid = -1)
        : NullOperation(pid), min_addr_(min_addr), max_addr_(max_addr)
    {
        while (min_addr <= max_addr) {
            sequence_.emplace_back(std::make_shared<Read>(min_addr, pid));
            min_addr += 64;
        }
    }

    void advance_seq(Operation::SeqItStack *it_stack) const override
    {
        ++(it_stack->back().first);
        it_stack->emplace_back(sequence_.begin(), sequence_.end());
    }

    Operation::Ptr clone() const override
    {
        // Don't just copy, need deep clone
        return std::make_shared<ReadSequence>(min_addr_, max_addr_, pid());
    }

    void reset() override
    {
        for (const auto& op : sequence_) {
            op->reset();
        }
    }

  protected:
    types::Addr min_addr_;
    types::Addr max_addr_;
    Operation::Seq sequence_;
};

/**
 * RandomFactory.
 */
struct RandomFactory {
    typedef Operation ResultType;

    explicit RandomFactory(types::Pid min_pid, types::Pid max_pid,
                           types::Addr min_addr, types::Addr max_addr,
                           std::size_t stride = sizeof(types::WriteID),
                           std::size_t max_sequence = 50,
                           bool extended = false)
        : min_pid_(min_pid), max_pid_(max_pid),
          min_addr_(min_addr), max_addr_(max_addr),
          stride_(stride), max_sequence_(max_sequence),
          extended_(extended)
    {
        assert(stride_ >= sizeof(types::WriteID));
        assert(stride_ % sizeof(types::WriteID) == 0);
    }

    void reset(types::Pid min_pid, types::Pid max_pid,
               types::Addr min_addr, types::Addr max_addr,
               std::size_t stride = sizeof(types::WriteID))
    {
        min_pid_ = min_pid;
        max_pid_ = max_pid;
        min_addr_ = min_addr;
        max_addr_ = max_addr;
        stride_ = stride;
    }

    template <class URNG, class AddrFilterFunc>
    Operation::Ptr operator ()(URNG& urng, AddrFilterFunc addr_filter_func,
                               std::size_t max_fails = 0) const
    {
        // Choice distribution
        const std::size_t max_choice = (extended_ ? 1001 : 1000) - 1;
        std::uniform_int_distribution<std::size_t> dist_choice(0, max_choice);

        // Attributes of the operation
        std::uniform_int_distribution<types::Pid> dist_pid(min_pid_, max_pid_);
        std::uniform_int_distribution<types::Addr> dist_addr(min_addr_,
                                    max_addr_ - AssemblerState::MAX_OP_SIZE);
        std::uniform_int_distribution<std::size_t> dist_sequence(1, max_sequence_);

        // select op
        const auto choice = dist_choice(urng);

        // pid
        const auto pid = dist_pid(urng);

        // addr (lazy)
        auto addr = [&]() {
            types::Addr result = 0;

            for (std::size_t tries = 0; tries < max_fails + 1; ++tries) {
                result = dist_addr(urng);
                result -= result % stride_;

                if (addr_filter_func(result)) {
                    return result;
                }
            }

            return result;
        };

        // sequence (lazy)
        auto sequence = [&dist_sequence, &urng]() {
            return dist_sequence(urng);
        };

        if (choice < 500) { // 50%
            return std::make_shared<Read>(addr(), pid);
        } else if (choice < 550) { // 5%
            return std::make_shared<ReadAddrDp>(addr(), pid);
        } else if (choice < 970) { // 42%
            return std::make_shared<Write>(addr(), pid);
        } else if (choice < 980) { // 1%
            return std::make_shared<ReadModifyWrite>(addr(), pid);
        } else if (choice < 990) { // 1%
            return std::make_shared<CacheFlush>(addr(), pid);
        } else if (choice < 1000) { // 1%
            return std::make_shared<Delay>(sequence(), pid);
        } else if (extended_) {
            // REAL_PERCENTAGE_OF_100 = PERC * (1000 / MAX_CHOICE)

            if (choice < 1001) { // 0.1%
                auto min_a = addr();
                //TODO: do not hard-code stride
                auto max_a = min_a + sequence() * 64;
                if (max_a > max_addr()) {
                    max_a = max_addr();
                }

                return std::make_shared<ReadSequence>(min_a, max_a, pid);
            }
        }

        // should never get here
        assert(false);
        return nullptr;
    }

    template <class URNG>
    Operation::Ptr operator ()(URNG& urng) const
    {
        return (*this)(urng, [](types::Addr addr) { return true; });
    }

    types::Pid min_pid() const
    { return min_pid_; }

    types::Pid max_pid() const
    { return max_pid_; }

    types::Addr min_addr() const
    { return min_addr_; }

    types::Addr max_addr() const
    { return max_addr_; }

    std::size_t stride() const
    { return stride_; }

    std::size_t max_sequence() const
    { return max_sequence_; }

  private:
    types::Pid min_pid_;
    types::Pid max_pid_;
    types::Addr min_addr_;
    types::Addr max_addr_;
    std::size_t stride_;
    std::size_t max_sequence_;
    bool extended_;
};

} /* namespace strong */
} /* namespace codegen */
} /* namespace mc2lib */

#endif /* MC2LIB_CODEGEN_OPS_STRONG_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
