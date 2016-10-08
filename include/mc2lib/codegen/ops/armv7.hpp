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

#ifndef MC2LIB_CODEGEN_OPS_ARMv7_HPP_
#define MC2LIB_CODEGEN_OPS_ARMv7_HPP_

#include <algorithm>
#include <random>
#include <stdexcept>

#include "../cats.hpp"
#include "../compiler.hpp"

namespace mc2lib {
namespace codegen {

/**
 * @namespace mc2lib::codegen::armv7
 * @brief Implementations of Operations for ARMv7 (incomplete).
 *
 * The current operations do not exercise all aspects of the MCM.
 */
namespace armv7 {

#define ASM_PRELUDE char *cnext__ = static_cast<char *>(code);
#define ASM_LEN (static_cast<std::size_t>(cnext__ - static_cast<char *>(code)))
#define ASM_AT (start + ASM_LEN)

#define ASM16(v)                                       \
  do {                                                 \
    assert(ASM_LEN + 2 <= len);                        \
    *reinterpret_cast<std::uint16_t *>(cnext__) = (v); \
    cnext__ += 2;                                      \
  } while (0)

#define ASM_PROLOGUE return ASM_LEN;

// Thumb
class Backend {
 public:
  void Reset() {}

  // Currently supports single byte operations only; to test for single-copy
  // atomicity, implement multi-byte operations support.
  static_assert(sizeof(types::WriteID) == 1, "Unsupported read/write size!");

  enum Reg {
    r0 = 0,
    r1,
    r2,
    r3,
    r4,

    // reserved
    r5__,
    r6__,
    r7__
  };

  std::size_t Return(void *code, std::size_t len) const {
    ASM_PRELUDE;
    ASM16(0x4770);  // bx lr
    ASM_PROLOGUE;
  }

  std::size_t Delay(std::size_t length, void *code, std::size_t len) const {
    ASM_PRELUDE;
    for (std::size_t i = 0; i < length; ++i) {
      ASM16(0xbf00);  // nop
    }
    ASM_PROLOGUE;
  }

  std::size_t DMB_ST(void *code, std::size_t len) const {
    ASM_PRELUDE;
    // dmb st
    ASM16(0xf3bf);
    ASM16(0x8f5e);
    ASM_PROLOGUE;
  }

  std::size_t Read(types::Addr addr, Reg out, types::InstPtr start, void *code,
                   std::size_t len, types::InstPtr *at) const {
    ASM_PRELUDE;

    Helper h(cnext__, code, len);
    h.MovImm32(r6__, addr);

    // ldrb out, [r6, #0]
    *at = ASM_AT;
    ASM16(0x7830 | out);

    ASM_PROLOGUE;
  }

  std::size_t ReadAddrDp(types::Addr addr, Reg out, Reg dp,
                         types::InstPtr start, void *code, std::size_t len,
                         types::InstPtr *at) const {
    ASM_PRELUDE;

    Helper h(cnext__, code, len);
    h.MovImm32(r6__, addr);

    // eor dp, dp
    ASM16(0x4040 | (dp << 3) | dp);

    // ldrb out, [r6, dp]
    *at = ASM_AT;
    ASM16(0x5c30 | (dp << 6) | out);

    ASM_PROLOGUE;
  }

  std::size_t Write(types::Addr addr, types::WriteID write_id,
                    types::InstPtr start, void *code, std::size_t len,
                    types::InstPtr *at) const {
    ASM_PRELUDE;

    Helper h(cnext__, code, len);
    h.MovImm32(r6__, addr);

    // movs r7, #write_id
    ASM16(0x2700 | write_id);

    // strb r7, [r6, #0]
    *at = ASM_AT;
    ASM16(0x7037);

    ASM_PROLOGUE;
  }

 protected:
  class Helper {
   public:
    Helper(char *&cnext, void *&code, std::size_t len)
        : cnext__(cnext), code(code), len(len) {}

    void MovImm32(Reg reg, std::uint32_t imm32) {
      // movw reg, #(imm32 & 0xffff)
      std::uint16_t imm32_w = imm32 & 0xffff;
      ASM16(0xf240
            // [10:10]
            | ((imm32_w & 0x0800) >> 1)
            // [3:0]
            | ((imm32_w & 0xf000) >> 12));
      ASM16(  // [14:12]
          ((imm32_w & 0x0700) << 4)
          // [11:8]
          | (reg << 8)
          // [7:0]
          | (imm32_w & 0x00ff));

      // movt reg, #((imm32 & 0xffff0000) >> 16)
      std::uint16_t imm32_t = (imm32 & 0xffff0000) >> 16;
      ASM16(0xf2c0
            // [10:10]
            | ((imm32_t & 0x0800) >> 1)
            // [3:0]
            | ((imm32_t & 0xf000) >> 12));
      ASM16(  // [14:12]
          ((imm32_t & 0x0700) << 4)
          // [11:8]
          | (reg << 8)
          // [7:0]
          | (imm32_t & 0x00ff));
    }

   protected:
    char *&cnext__;
    void *&code;
    const std::size_t len;
  };
};

#undef ASM_PRELUDE
#undef ASM_LEN
#undef ASM_AT
#undef ASM16
#undef ASM_PROLOGUE

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
  explicit Read(types::Addr addr, Backend::Reg out, types::Pid pid = -1)
      : MemOperation(pid),
        addr_(addr),
        out_(out),
        event_(nullptr),
        from_(nullptr) {}

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
    return backend->Read(addr_, out(), start, code, len, &at_);
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

      evts->ew()->rf.Erase(*from_, *event_);
    }

    from_ = from;
    evts->ew()->rf.Insert(*from_, *event_, true);

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

  Backend::Reg out() const { return out_; }

 protected:
  types::Addr addr_;
  Backend::Reg out_;
  const mc::Event *event_;
  const mc::Event *from_;
  types::InstPtr at_;
};

class ReadAddrDp : public Read {
 public:
  explicit ReadAddrDp(types::Addr addr, Backend::Reg reg, Backend::Reg dp,
                      types::Pid pid = -1)
      : Read(addr, reg, pid), dp_(dp) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<ReadAddrDp>(*this);
  }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {
    event_ = evts->MakeRead(pid(), mc::Event::kRead | mc::Event::kRegInAddr,
                            addr_)[0];

    if (*before != nullptr) {
      auto event_before = (*before)->LastEvent(event_, evts);
      if (event_before != nullptr) {
        evts->ew()->po.Insert(*event_before, *event_);

        // Find read dependency.
        auto arch = dynamic_cast<mc::cats::Arch_ARMv7 *>(evts->arch());
        if (arch != nullptr) {
          do {
            auto potential_dp_read = dynamic_cast<const Read *>(*before);
            if (potential_dp_read != nullptr) {
              if (potential_dp_read->out() == dp_) {
                auto event_dp = potential_dp_read->LastEvent(event_, evts);
                arch->dd_reg.Insert(*event_dp, *event_);
                break;
              }
            }
          } while (*(--before) != nullptr);
        }
      }
    }
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->ReadAddrDp(addr_, out(), dp_, start, code, len, &at_);
  }

 protected:
  Backend::Reg dp_;
};

class Write : public MemOperation {
 public:
  explicit Write(types::Addr addr, types::Pid pid = -1)
      : MemOperation(pid), addr_(addr), write_id_(0) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<Write>(*this);
  }

  void Reset() override {
    event_ = nullptr;
    from_ = nullptr;
    write_id_ = 0;
  }

  bool EnableEmit(EvtStateCats *evts) override { return !evts->Exhausted(); }

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

      evts->ew()->co.Erase(*from_, *event_);
    }

    from_ = from;
    evts->ew()->co.Insert(*from_, *event_, true);

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
  types::Addr addr_;
  types::WriteID write_id_;
  const mc::Event *event_;
  const mc::Event *from_;
  types::InstPtr at_;
};

class DMB_ST : public Operation {
 public:
  explicit DMB_ST(types::Pid pid = -1)
      : Operation(pid), before_(nullptr), first_write_before_(nullptr) {}

  Operation::Ptr Clone() const override {
    return std::make_shared<DMB_ST>(*this);
  }

  void Reset() override {
    before_ = nullptr;
    first_write_before_ = nullptr;
  }

  bool EnableEmit(EvtStateCats *evts) override { return true; }

  void InsertPo(Operation::ThreadConstIt before, EvtStateCats *evts) override {
    before_ = *before;

    while (*before != nullptr) {
      auto potential_write = dynamic_cast<const Write *>(*before);
      if (potential_write != nullptr) {
        first_write_before_ = potential_write;
        break;
      }
      --before;
    }
  }

  void RegisterCallback(Operation::CallbackStack *callback_stack) override {
    callback_stack->push_back([this](Operation *after, types::InstPtr start,
                                     Backend *backend, EvtStateCats *evts,
                                     void *code, std::size_t len) {
      if (first_write_before_ != nullptr) {
        auto potential_write = dynamic_cast<const Write *>(after);
        if (potential_write != nullptr) {
          auto arch = dynamic_cast<mc::cats::Arch_ARMv7 *>(evts->arch());
          if (arch != nullptr) {
            auto event_before = first_write_before_->LastEvent(nullptr, evts);
            auto event_after = potential_write->FirstEvent(nullptr, evts);
            assert(event_before != nullptr);
            assert(event_after != nullptr);

            arch->dmb_st.Insert(*event_before, *event_after);
            // cats::ARMv7 takes care of transitivity.
          }
          first_write_before_ = nullptr;
        }
      }
      return 0;
    });
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtStateCats *evts,
                   void *code, std::size_t len) override {
    return backend->DMB_ST(code, len);
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
  const Operation *before_;
  const Operation *first_write_before_;
};

/**
 * RandomFactory.
 */
struct RandomFactory {
  typedef Operation ResultType;

  explicit RandomFactory(types::Pid min_pid, types::Pid max_pid,
                         types::Addr min_addr, types::Addr max_addr,
                         std::size_t stride = sizeof(types::WriteID),
                         std::size_t max_sequence = 50)
      : min_pid_(min_pid),
        max_pid_(max_pid),
        min_addr_(min_addr),
        max_addr_(max_addr),
        stride_(stride),
        max_sequence_(max_sequence) {
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
    std::uniform_int_distribution<std::size_t> dist_choice(0, 1000 - 1);

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

    // Register distribution
    std::uniform_int_distribution<int> dist_reg(Backend::r0, Backend::r4);

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

    // sequence (lazy)
    auto reg = [&dist_reg, &urng]() {
      return static_cast<Backend::Reg>(dist_reg(urng));
    };

    if (choice < 320) {  // 32%
      return std::make_shared<Read>(addr(), reg(), pid);
    } else if (choice < 560) {  // 24%
      return std::make_shared<ReadAddrDp>(addr(), reg(), reg(), pid);
    } else if (choice < 980) {  // 42%
      return std::make_shared<Write>(addr(), pid);
    } else if (choice < 990) {  // 1%
      return std::make_shared<DMB_ST>(pid);
    } else if (choice < 1000) {  // 1%
      return std::make_shared<Delay>(sequence(), pid);
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

 private:
  types::Pid min_pid_;
  types::Pid max_pid_;
  types::Addr min_addr_;
  types::Addr max_addr_;
  std::size_t stride_;
  std::size_t max_sequence_;
};

}  // namespace armv7
}  // namespace codegen
}  // namespace mc2lib

#endif /* MC2LIB_CODEGEN_OPS_ARMv7_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
