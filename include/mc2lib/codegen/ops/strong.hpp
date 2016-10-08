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

#ifndef MC2LIB_CODEGEN_OPS_STRONG_HPP_
#define MC2LIB_CODEGEN_OPS_STRONG_HPP_

#include <algorithm>
#include <random>
#include <sstream>
#include <stdexcept>

#include "../cats.hpp"
#include "../compiler.hpp"

namespace mc2lib {
namespace codegen {

/**
 * @namespace mc2lib::codegen::strong
 * @brief Implementations of Operations for strong memory consistency models.
 */
namespace strong {

struct Backend {
  virtual ~Backend() {}

  virtual void Reset() {}

  virtual std::size_t Return(void *code, std::size_t len) const = 0;

  virtual std::size_t Delay(std::size_t length, void *code,
                            std::size_t len) const = 0;

  virtual std::size_t Read(types::Addr addr, types::InstPtr start, void *code,
                           std::size_t len, types::InstPtr *at) const = 0;

  virtual std::size_t ReadAddrDp(types::Addr addr, types::InstPtr start,
                                 void *code, std::size_t len,
                                 types::InstPtr *at) const = 0;

  virtual std::size_t Write(types::Addr addr, types::WriteID write_id,
                            types::InstPtr start, void *code, std::size_t len,
                            types::InstPtr *at) const = 0;

  virtual std::size_t ReadModifyWrite(types::Addr addr, types::WriteID write_id,
                                      types::InstPtr start, void *code,
                                      std::size_t len,
                                      types::InstPtr *at) const = 0;

  virtual std::size_t CacheFlush(types::Addr addr, void *code,
                                 std::size_t len) const = 0;
};

typedef Op<Backend, EvtStateCats> Operation;
typedef MemOp<Backend, EvtStateCats> MemOperation;
typedef NullOp<Backend, EvtStateCats> NullOperation;

class Return : public Operation {
 public:
  explicit Return(types::Pid pid = -1) : Operation(pid) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<Return>(*this);
  }

  void Reset() override {}

  bool EnableEmit(EvtStateCats *evts) override { return true; }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {}

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->Return(code, len);
  }

  const mc::Event *LastEvent(const mc::Event *next_event,
                             EvtStateCats *evts) const override {
    return nullptr;
  }

  const mc::Event *FirstEvent(const mc::Event *prev_event,
                              EvtStateCats *evts) const override {
    return nullptr;
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size,
                 EvtStateCats *evts) override {
    return true;
  }
};

class Delay : public Operation {
 public:
  explicit Delay(std::size_t length, types::Pid pid = -1)
      : Operation(pid), length_(length), before_(nullptr) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<Delay>(*this);
  }

  void Reset() override { before_ = nullptr; }

  bool EnableEmit(EvtStateCats *evts) override { return true; }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {
    before_ = *before;
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->Delay(length_, code, len);
  }

  const mc::Event *LastEvent(const mc::Event *next_event,
                             EvtStateCats *evts) const override {
    // Forward
    if (before_ != nullptr) {
      return before_->LastEvent(next_event, evts);
    }

    return nullptr;
  }

  const mc::Event *FirstEvent(const mc::Event *prev_event,
                              EvtStateCats *evts) const override {
    // Forward
    if (before_ != nullptr) {
      return before_->FirstEvent(prev_event, evts);
    }

    return nullptr;
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size,
                 EvtStateCats *evts) override {
    throw std::logic_error("Unexpected UpdateObs");
    return false;
  }

 protected:
  std::size_t length_;
  const Operation *before_;
};

class Read : public MemOperation {
 public:
  explicit Read(types::Addr addr, types::Pid pid = -1)
      : MemOperation(pid), addr_(addr), event_(nullptr), from_(nullptr) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<Read>(*this);
  }

  void Reset() override {
    event_ = nullptr;
    from_ = nullptr;
  }

  bool EnableEmit(EvtStateCats *evts) override { return !evts->Exhausted(); }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {
    event_ = evts->MakeRead(pid(), mc::Event::kRead, addr_)[0];

    if (*before != nullptr) {
      auto event_before = (*before)->LastEvent(event_, evts);
      if (event_before != nullptr) {
        evts->ew()->po.Insert(*event_before, *event_);
      }
    }
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->Read(addr_, start, code, len, &at_);
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size,
                 EvtStateCats *evts) override {
    assert(event_ != nullptr);
    assert(ip == at_);
    assert(addr == addr_);
    assert(size == sizeof(types::WriteID));

    const mc::Event *from =
        evts->GetWrite(MakeEventPtrs(event_), addr_, from_id)[0];

    if (from_ != nullptr) {
      // If from_ == from, we still need to continue to try to erase and
      // insert, in case the from-relation has been cleared.

      EraseObsHelper(from_, event_, evts->ew());
    }

    from_ = from;
    InsertObsHelper(from_, event_, evts->ew());

    return true;
  }

  const mc::Event *LastEvent(const mc::Event *next_event,
                             EvtStateCats *evts) const override {
    return event_;
  }

  const mc::Event *FirstEvent(const mc::Event *prev_event,
                              EvtStateCats *evts) const override {
    return event_;
  }

  types::Addr addr() const override { return addr_; }

 protected:
  virtual void InsertObsHelper(const mc::Event *e1, const mc::Event *e2,
                               mc::cats::ExecWitness *ew) {
    ew->rf.Insert(*e1, *e2, true);
  }

  virtual void EraseObsHelper(const mc::Event *e1, const mc::Event *e2,
                              mc::cats::ExecWitness *ew) {
    ew->rf.Erase(*e1, *e2);
  }

  types::Addr addr_;
  const mc::Event *event_;
  const mc::Event *from_;
  types::InstPtr at_;
};

class ReadAddrDp : public Read {
 public:
  explicit ReadAddrDp(types::Addr addr, types::Pid pid = -1)
      : Read(addr, pid) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<ReadAddrDp>(*this);
  }

  // TODO(melver): InsertPo: if we start supporting an Arch which does not
  // order Read->Read, add a dependency-hb between this and the last Read --
  // this assumes all Reads are reading into the same register, and this read
  // computes the address with this one register.
  // NOTE: before can be used to traverse operations backwards before "before".

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->ReadAddrDp(addr_, start, code, len, &at_);
  }
};

class Write : public Read {
 public:
  explicit Write(types::Addr addr, types::Pid pid = -1)
      : Read(addr, pid), write_id_(0) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<Write>(*this);
  }

  void Reset() override {
    event_ = nullptr;
    from_ = nullptr;
    write_id_ = 0;
  }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {
    event_ = evts->MakeWrite(pid(), mc::Event::kWrite, addr_, &write_id_)[0];

    if (*before != nullptr) {
      auto event_before = (*before)->LastEvent(event_, evts);
      if (event_before != nullptr) {
        evts->ew()->po.Insert(*event_before, *event_);
      }
    }
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->Write(addr_, write_id_, start, code, len, &at_);
  }

 protected:
  void InsertObsHelper(const mc::Event *e1, const mc::Event *e2,
                       mc::cats::ExecWitness *ew) override {
    ew->co.Insert(*e1, *e2);
  }

  void EraseObsHelper(const mc::Event *e1, const mc::Event *e2,
                      mc::cats::ExecWitness *ew) override {
    ew->co.Erase(*e1, *e2);
  }

  types::WriteID write_id_;
};

class ReadModifyWrite : public MemOperation {
 public:
  explicit ReadModifyWrite(types::Addr addr, types::Pid pid = -1)
      : MemOperation(pid), addr_(addr), last_part_(-1) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<ReadModifyWrite>(*this);
  }

  void Reset() override {
    last_part_ = -1;
    event_r_ = nullptr;
    event_w_ = nullptr;
    from_ = nullptr;
    write_id_ = 0;
  }

  bool EnableEmit(EvtStateCats *evts) override { return !evts->Exhausted(); }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {
    event_r_ = evts->MakeRead(pid(), mc::Event::kRead, addr_)[0];
    event_w_ = evts->MakeWrite(pid(), mc::Event::kWrite, addr_, &write_id_)[0];

    if (*before != nullptr) {
      auto event_before = (*before)->LastEvent(event_r_, evts);
      if (event_before != nullptr) {
        evts->ew()->po.Insert(*event_before, *event_r_);

        if (dynamic_cast<mc::cats::Arch_TSO *>(evts->arch()) != nullptr) {
          // Implied fence before atomic
          auto arch_tso = dynamic_cast<mc::cats::Arch_TSO *>(evts->arch());
          arch_tso->mfence.Insert(*event_before, *event_r_);
        }
      }
    }

    evts->ew()->po.Insert(*event_r_, *event_w_);
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->ReadModifyWrite(addr_, write_id_, start, code, len, &at_);
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size,
                 EvtStateCats *evts) override {
    assert(event_r_ != nullptr);
    assert(event_w_ != nullptr);
    assert(ip == at_);
    assert(addr == addr_);
    assert(size == sizeof(types::WriteID));

    // This also alerts us if the read would be seeing the write's data.
    const mc::Event *from =
        evts->GetWrite(MakeEventPtrs(event_w_), addr_, from_id)[0];

    auto part_event = event_r_;
    auto obs_rel = &evts->ew()->rf;

    // TODO(melver): clean this up! Do we need ability to update squashed?

    if (last_part_ == -1 || part <= last_part_) {
      // First part: read

      if (from_ != nullptr) {
        // Restart
        evts->ew()->rf.Erase(*from_, *event_r_);
        evts->ew()->co.Erase(*from_, *event_w_);
      }
    } else {
      // Second part: write
      assert(part > last_part_);

      // Check atomicity.
      if (*from != *from_) {
        std::ostringstream oss;
        oss << "RMW NOT ATOMIC: expected <" << static_cast<std::string>(*from_)
            << ">, but overwriting <" << static_cast<std::string>(*from)
            << ">!";
        throw mc::Error(oss.str());
      }

      part_event = event_w_;
      obs_rel = &evts->ew()->co;
    }

    obs_rel->Insert(*from, *part_event, true);

    from_ = from;
    last_part_ = part;
    return true;
  }

  const mc::Event *LastEvent(const mc::Event *next_event,
                             EvtStateCats *evts) const override {
    if (dynamic_cast<mc::cats::Arch_TSO *>(evts->arch()) != nullptr) {
      // Implied fence after atomic
      auto arch_tso = dynamic_cast<mc::cats::Arch_TSO *>(evts->arch());
      arch_tso->mfence.Insert(*event_w_, *next_event);
    }

    return event_w_;
  }

  const mc::Event *FirstEvent(const mc::Event *prev_event,
                              EvtStateCats *evts) const override {
    return event_r_;
  }

  types::Addr addr() const override { return addr_; }

 protected:
  types::Addr addr_;
  int last_part_;
  const mc::Event *event_r_;
  const mc::Event *event_w_;
  const mc::Event *from_;
  types::WriteID write_id_;
  types::InstPtr at_;
};

class CacheFlush : public MemOperation {
 public:
  explicit CacheFlush(types::Addr addr, types::Pid pid = -1)
      : MemOperation(pid), addr_(addr), before_(nullptr) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<CacheFlush>(*this);
  }

  void Reset() override { before_ = nullptr; }

  bool EnableEmit(EvtStateCats *evts) override { return true; }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {
    before_ = *before;
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->CacheFlush(addr_, code, len);
  }

  const mc::Event *LastEvent(const mc::Event *next_event,
                             EvtStateCats *evts) const override {
    // Forward
    if (before_ != nullptr) {
      return before_->LastEvent(next_event, evts);
    }

    return nullptr;
  }

  const mc::Event *FirstEvent(const mc::Event *prev_event,
                              EvtStateCats *evts) const override {
    // Forward
    if (before_ != nullptr) {
      return before_->FirstEvent(prev_event, evts);
    }

    return nullptr;
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size,
                 EvtStateCats *evts) override {
    return true;
  }

  types::Addr addr() const override { return addr_; }

 protected:
  types::Addr addr_;
  const Operation *before_;
};

class ReadSequence : public NullOperation {
 public:
  explicit ReadSequence(types::Addr min_addr, types::Addr max_addr,
                        types::Pid pid = -1)
      : NullOperation(pid), min_addr_(min_addr), max_addr_(max_addr) {
    while (min_addr <= max_addr) {
      sequence_.emplace_back(std::make_shared<Read>(min_addr, pid));
      min_addr += 64;
    }
  }

  void AdvanceThread(Operation::ThreadItStack *it_stack) const override {
    ++(it_stack->back().first);
    it_stack->emplace_back(sequence_.begin(), sequence_.end());
  }

  Operation::Ptr Clone() const override {
    // Don't just copy, need deep clone
    return std::make_shared<ReadSequence>(min_addr_, max_addr_, pid());
  }

  void Reset() override {
    for (const auto &op : sequence_) {
      op->Reset();
    }
  }

 protected:
  types::Addr min_addr_;
  types::Addr max_addr_;
  Operation::Thread sequence_;
};

/**
 * RandomFactory.
 */
struct RandomFactory {
  typedef Operation ResultType;

  explicit RandomFactory(types::Pid min_pid, types::Pid max_pid,
                         types::Addr min_addr, types::Addr max_addr,
                         std::size_t stride = sizeof(types::WriteID),
                         std::size_t max_sequence = 50, bool extended = false)
      : min_pid_(min_pid),
        max_pid_(max_pid),
        min_addr_(min_addr),
        max_addr_(max_addr),
        stride_(stride),
        max_sequence_(max_sequence),
        extended_(extended) {
    assert(this->stride() >= sizeof(types::WriteID));
    assert(this->stride() % sizeof(types::WriteID) == 0);
  }

  void Reset(types::Pid min_pid, types::Pid max_pid, types::Addr min_addr,
             types::Addr max_addr,
             std::size_t stride = sizeof(types::WriteID)) {
    min_pid_ = min_pid;
    max_pid_ = max_pid;
    min_addr_ = min_addr;
    max_addr_ = max_addr;
    stride_ = stride;
  }

  template <class URNG, class AddrFilterFunc>
  Operation::Ptr operator()(URNG &urng, AddrFilterFunc addr_filter_func,
                            std::size_t max_fails = 0) const {
    // Choice distribution
    const std::size_t max_choice = (extended_ ? 1005 : 1000) - 1;
    std::uniform_int_distribution<std::size_t> dist_choice(0, max_choice);

    // Pid distribution
    std::uniform_int_distribution<types::Pid> dist_pid(min_pid_, max_pid_);

    // Addr distribution
    auto chunk_min_addr = min_addr_;
    auto chunk_max_addr = max_addr_;

    if (ChunkSize() > 1 && HoleSize() > 1) {
      std::size_t chunk_cnt =
          for_each_AddrRange([](types::Addr a, types::Addr b) {});
      std::size_t select_chunk =
          std::uniform_int_distribution<std::size_t>(0, chunk_cnt - 1)(urng);

      chunk_min_addr = min_addr_ + (select_chunk * HoleSize());
      chunk_max_addr = chunk_min_addr + ChunkSize() - 1;

      assert(chunk_min_addr >= min_addr_);
      assert(chunk_max_addr <= max_addr_);
    }

    std::uniform_int_distribution<types::Addr> dist_addr(
        chunk_min_addr, chunk_max_addr - EvtStateCats::kMaxOpSize);

    // Sequence distribution
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
        result -= result % stride();
        if (result < chunk_min_addr) result += stride();
        assert(result >= chunk_min_addr);
        assert(result <= chunk_max_addr - EvtStateCats::kMaxOpSize);

        if (addr_filter_func(result)) {
          return result;
        }
      }

      return result;
    };

    // sequence (lazy)
    auto sequence = [&dist_sequence, &urng]() { return dist_sequence(urng); };

    if (choice < 500) {  // 50%
      return std::make_shared<Read>(addr(), pid);
    } else if (choice < 550) {  // 5%
      return std::make_shared<ReadAddrDp>(addr(), pid);
    } else if (choice < 970) {  // 42%
      return std::make_shared<Write>(addr(), pid);
    } else if (choice < 980) {  // 1%
      return std::make_shared<ReadModifyWrite>(addr(), pid);
    } else if (choice < 990) {  // 1%
      return std::make_shared<CacheFlush>(addr(), pid);
    } else if (choice < 1000) {  // 1%
      return std::make_shared<Delay>(sequence(), pid);
    } else if (extended_) {
      // REAL_PERCENTAGE_OF_100 = PERC * (1000 / MAX_CHOICE)

      if (choice < 1005) {  // 0.5%
        auto min_a = addr();
        // TODO(melver): do not hard-code stride
        auto max_a = min_a + sequence() * 64;
        if (max_a > max_addr()) {
          max_a = max_addr();
        }

        return std::make_shared<ReadSequence>(min_a, max_a, pid);
      }
    }

    // should never get here
    throw std::logic_error("Not exhaustive");
    return nullptr;
  }

  template <class URNG>
  Operation::Ptr operator()(URNG &urng) const {
    return (*this)(urng, [](types::Addr addr) { return true; });
  }

  types::Pid min_pid() const { return min_pid_; }

  types::Pid max_pid() const { return max_pid_; }

  types::Addr min_addr() const { return min_addr_; }

  types::Addr max_addr() const { return max_addr_; }

  std::size_t stride() const { return stride_ & ((1ULL << 16) - 1); }

  std::size_t ChunkSize() const {
    return 1ULL << ((stride_ & (0xffULL << 24)) >> 24);
  }

  std::size_t HoleSize() const {
    return 1ULL << ((stride_ & (0xffULL << 16)) >> 16);
  }

  template <class Func>
  std::size_t for_each_AddrRange(Func func) const {
    if (ChunkSize() > 1 && HoleSize() > 1) {
      assert(HoleSize() >= ChunkSize());
      assert(ChunkSize() <= (max_addr_ - min_addr_ + 1));

      std::size_t chunk_cnt = 0;

      for (;; ++chunk_cnt) {
        types::Addr min = min_addr_ + (chunk_cnt * HoleSize());
        types::Addr max = min + ChunkSize() - 1;
        if (max > max_addr_) break;

        func(min, max);
      }

      return chunk_cnt;
    }

    func(min_addr_, max_addr_);
    return 1;
  }

  std::size_t max_sequence() const { return max_sequence_; }

  void set_max_sequence(std::size_t val) { max_sequence_ = val; }

  bool extended() const { return extended_; }

  void set_extended(bool val) { extended_ = val; }

 private:
  types::Pid min_pid_;
  types::Pid max_pid_;
  types::Addr min_addr_;
  types::Addr max_addr_;
  std::size_t stride_;
  std::size_t max_sequence_;
  bool extended_;
};

}  // namespace strong
}  // namespace codegen
}  // namespace mc2lib

#endif /* MC2LIB_CODEGEN_OPS_STRONG_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
