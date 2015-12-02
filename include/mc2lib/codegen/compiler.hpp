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

#ifndef MC2LIB_CODEGEN_COMPILER_HPP_
#define MC2LIB_CODEGEN_COMPILER_HPP_

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../memconsistency/cats.hpp"
#include "../types.hpp"

namespace mc2lib {

/**
 * @namespace mc2lib::codegen
 * @brief Code generation for memory consistency verification.
 */
namespace codegen {

namespace mc = memconsistency;

class AssemblerState;

#if defined(__GNUC__) && (__GNUC__ == 4 && (__GNUC_MINOR__ == 6))
template <std::size_t max_size_bytes>
struct EventPtrs : public std::array<const mc::Event *,
                                     max_size_bytes / sizeof(types::WriteID)> {
  template <class... Ts>
  EventPtrs(Ts... en)
      : std::array<const mc::Event *, max_size_bytes / sizeof(types::WriteID)>(
            {{en...}}) {}
};
#else
// Only works for GCC > 4.6
template <std::size_t max_size_bytes>
using EventPtrs =
    std::array<const mc::Event *, max_size_bytes / sizeof(types::WriteID)>;
#endif

template <class... Ts>
inline auto MakeEventPtrs(const mc::Event *e1, Ts... en)
    -> EventPtrs<(1 + sizeof...(Ts)) * sizeof(types::WriteID)> {
  EventPtrs<(1 + sizeof...(Ts)) *sizeof(types::WriteID)> es = {e1, en...};
  return es;
}

/**
 * Baseclass for Operation implementations.
 */
template <class Backend>
class Op {
 public:
  // Types for sequences and threads
  typedef std::shared_ptr<Op> Ptr;
  typedef std::vector<Ptr> Seq;
  typedef typename Seq::const_iterator SeqIt;
  typedef std::unordered_map<types::Pid, Seq> Threads;
  typedef std::vector<std::pair<SeqIt, SeqIt>> SeqItStack;

  // Read-only sequence types: new Ops can access previous Ops.
  typedef std::vector<const Op *> SeqConst;
  typedef typename SeqConst::const_iterator SeqConstIt;

  // Callback type: optionally, previous Ops get called back with new Ops.
  // E.g. for lazily constructing control flow graph with random branches.
  typedef std::function<std::size_t(Op *, const Backend *, types::InstPtr,
                                    AssemblerState *, void *, std::size_t)>
      Callback;
  typedef std::vector<Callback> CallbackStack;

  template <class T>
  friend Threads ExtractThreads(T *container) {
    Threads result;
    std::unordered_set<Op *> used;

    for (auto &op : (*container)) {
      assert(op != nullptr);

      if (used.insert(op.get()).second) {
        // Using same instance of Op multiple times is not permitted.
        op = op->Clone();
      }

      result[op->pid()].emplace_back(op);
    }

    return result;
  }

  friend std::size_t threads_size(const Threads &threads) {
    std::size_t result = 0;

    for (const auto &thread : threads) {
      result += thread.second.size();
    }

    return result;
  }

 public:
  explicit Op(types::Pid pid) : pid_(pid) {}

  virtual ~Op() {}

  virtual void AdvanceSeq(SeqItStack *it_stack) const {
    ++(it_stack->back().first);
  }

  /**
   * Clone the instance.
   */
  virtual Ptr Clone() const = 0;

  /**
   * Provide Reset, as emit functions may modify the state of an Op to store
   * information to map instructions to events.
   */
  virtual void Reset() = 0;

  /**
   * Prepares the operation for emit; common emit code.
   *
   * @param[in,out] asms Pointer to AssemblerState instance of calling Compiler.
   *
   * @return true if can emit; false otherwise.
   */
  virtual bool EnableEmit(AssemblerState *asms) = 0;

  /**
   * Generate static program-order relation.
   *
   * @param before Pointer to last Op; nullptr if none exists.
   * @param[in,out] asms Pointer to AssemblerState instance maintained by
   *                     Compiler.
   *
   * @return Last event in program-order generated by this operation.
   */
  virtual void InsertPo(SeqConstIt before, AssemblerState *asms) = 0;

  /**
   * Optionally register callback.
   *
   * @param[out] callback_stack Pointer to callback_stack with which to
   *                            register the callback.
   */
  virtual void RegisterCallback(CallbackStack *callback_stack) {}

  /**
   * Emit machine code.
   *
   * @param backend Architecture backend.
   * @param start Instruction pointer to first instruction when executing.
   * @param[in,out] asms Pointer to AssemblerState instance of calling Compiler.
   * @param[out] code Pointer to memory to be copied into.
   * @param len Maximum lenth of code.
   *
   * @return Size of emitted code.
   */
  virtual std::size_t Emit(const Backend *backend, types::InstPtr start,
                           AssemblerState *asms, void *code,
                           std::size_t len) = 0;

  /**
   * Accessor for last event generated. Also used to insert additional
   * ordering based on passed next_event (e.g. fences).
   *
   * @param next_event Event after last in program order; nullptr if none
   *                   exists.
   * @param[in,out] asms Pointer to AssemblerState instance maintained by
   *                     Compiler.
   *
   * @return Last event in program-order; nullptr if none exists.
   */
  virtual const mc::Event *LastEvent(const mc::Event *next_event,
                                     AssemblerState *asms) const = 0;

  /**
   * Accessor for first event generated.
   *
   * @param prev_event Event before first in program order; nullptr if none
   *                   exists.
   * @param[in,out] asms Pointer to AssemblerState instance maintained by
   *                     Compiler.
   *
   * @return First event in program-order; nullptr if none exists.
   */
  virtual const mc::Event *FirstEvent(const mc::Event *prev_event,
                                      AssemblerState *asms) const = 0;

  /**
   * Updates dynamic observation for instruction's memory operation.
   *
   * @param ip Instruction pointer of instruction for which a value was
   *           observed.
   * @param part Which part of an instruction; e.g., if an instruction
   *             generates multiple memory events, part can be used to denote
   *             which.
   * @param addr Address for observed operation.
   * @param from_id Pointer to observed memory (WriteIDs).
   * @param size Total size of observed memory operations in from_id;
   *             implementation should assert expected size.
   * @param[in,out] asms Pointer to AssemblerState instance maintained by
   *                     Compiler.
   *
   * @return Success or not.
   */
  virtual bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                         const types::WriteID *from_id, std::size_t size,
                         AssemblerState *asms) = 0;

  types::Pid pid() const { return pid_; }

  void set_pid(types::Pid pid) { pid_ = pid; }

 private:
  types::Pid pid_;
};

template <class Backend>
class MemOp : public Op<Backend> {
 public:
  explicit MemOp(types::Pid pid) : Op<Backend>(pid) {}

  virtual types::Addr addr() const = 0;
};

template <class Backend>
class NullOp : public Op<Backend> {
 public:
  explicit NullOp(types::Pid pid) : Op<Backend>(pid) {}

  bool EnableEmit(AssemblerState *asms) override { return false; }

  void InsertPo(typename Op<Backend>::SeqConstIt before,
                AssemblerState *asms) override {
    throw std::logic_error("Not supported!");
  }

  std::size_t Emit(const Backend *backend, types::InstPtr start,
                   AssemblerState *asms, void *code, std::size_t len) override {
    throw std::logic_error("Not supported!");
    return 0;
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size,
                 AssemblerState *asms) override {
    throw std::logic_error("Not supported!");
    return false;
  }

  const mc::Event *LastEvent(const mc::Event *next_event,
                             AssemblerState *asms) const override {
    throw std::logic_error("Not supported!");
    return nullptr;
  }

  const mc::Event *FirstEvent(const mc::Event *prev_event,
                              AssemblerState *asms) const override {
    throw std::logic_error("Not supported!");
    return nullptr;
  }
};

class AssemblerState {
 public:
  // 1 Op can at most emit 2 write Events
  static constexpr std::size_t MAX_OP_SIZE = sizeof(types::WriteID) * 2;
  static constexpr std::size_t MAX_OP_EVTS =
      MAX_OP_SIZE / sizeof(types::WriteID);

  static constexpr types::Poi MIN_OTHER = static_cast<types::Poi>(1)
                                          << (sizeof(types::Poi) * 8 - 1);
  static constexpr types::Poi MAX_OTHER =
      std::numeric_limits<types::Poi>::max() - (MAX_OP_EVTS - 1);

  static constexpr types::WriteID INIT_WRITE =
      std::numeric_limits<types::WriteID>::min();
  static constexpr types::WriteID MIN_WRITE = INIT_WRITE + 1;
  static constexpr types::WriteID MAX_WRITE =
      (std::numeric_limits<types::WriteID>::max() < MIN_OTHER
           ? std::numeric_limits<types::WriteID>::max()
           : MIN_OTHER - 1) -
      (MAX_OP_EVTS - 1);

  static_assert(MIN_OTHER > MAX_WRITE, "Invalid read/write ID limits!");

  explicit AssemblerState(mc::cats::ExecWitness *ew,
                          mc::cats::Architecture *arch)
      : ew_(ew), arch_(arch) {}

  void Reset() {
    last_write_id_ = MIN_WRITE - 1;
    last_other_id = MIN_OTHER - 1;

    writes_.clear();
    ew_->Clear();
    arch_->Clear();
  }

  bool Exhausted() const {
    return last_write_id_ >= MAX_WRITE || last_other_id >= MAX_OTHER;
  }

  template <std::size_t max_size_bytes, class Func>
  EventPtrs<max_size_bytes> MakeEvent(types::Pid pid, mc::Event::Type type,
                                      types::Addr addr, std::size_t size,
                                      Func mkevt) {
    static_assert(max_size_bytes <= MAX_OP_SIZE, "Invalid size!");
    static_assert(sizeof(types::WriteID) <= max_size_bytes, "Invalid size!");
    static_assert(max_size_bytes % sizeof(types::WriteID) == 0,
                  "Invalid size!");
    assert(size <= max_size_bytes);
    assert(sizeof(types::WriteID) <= size);
    assert(size % sizeof(types::WriteID) == 0);

    assert(!Exhausted());

    EventPtrs<max_size_bytes> result;

    for (std::size_t i = 0; i < size / sizeof(types::WriteID); ++i) {
      result[i] = mkevt(i * sizeof(types::WriteID));
    }

    return result;
  }

  mc::Event MakeOther(types::Pid pid, mc::Event::Type type, types::Addr addr) {
    return mc::Event(type, addr, mc::Iiid(pid, ++last_other_id));
  }

  template <std::size_t max_size_bytes = sizeof(types::WriteID)>
  EventPtrs<max_size_bytes> MakeRead(types::Pid pid, mc::Event::Type type,
                                     types::Addr addr,
                                     std::size_t size = max_size_bytes) {
    ++last_other_id;
    return MakeEvent<max_size_bytes>(
        pid, type, addr, size, [&](types::Addr offset) {
          const mc::Event event =
              mc::Event(type, addr + offset, mc::Iiid(pid, last_other_id));

          return &ew_->events.Insert(event, true);
        });
  }

  template <std::size_t max_size_bytes = sizeof(types::WriteID)>
  EventPtrs<max_size_bytes> MakeWrite(types::Pid pid, mc::Event::Type type,
                                      types::Addr addr, types::WriteID *data,
                                      std::size_t size = max_size_bytes) {
    ++last_write_id_;
    return MakeEvent<max_size_bytes>(
        pid, type, addr, size, [&](types::Addr offset) {
          const types::WriteID write_id = last_write_id_;

          const mc::Event event =
              mc::Event(type, addr + offset, mc::Iiid(pid, write_id));

          *(data + offset) = write_id;
          return (writes_[write_id] = &ew_->events.Insert(event, true));
        });
  }

  template <std::size_t max_size_bytes = sizeof(types::WriteID)>
  EventPtrs<max_size_bytes> GetWrite(const EventPtrs<max_size_bytes> &after,
                                     types::Addr addr,
                                     const types::WriteID *from_id,
                                     std::size_t size = max_size_bytes) {
    static_assert(max_size_bytes <= MAX_OP_SIZE, "Invalid size!");
    static_assert(sizeof(types::WriteID) <= max_size_bytes, "Invalid size!");
    static_assert(max_size_bytes % sizeof(types::WriteID) == 0,
                  "Invalid size!");
    assert(size <= max_size_bytes);
    assert(sizeof(types::WriteID) <= size);
    assert(size % sizeof(types::WriteID) == 0);

    EventPtrs<max_size_bytes> result;
    result.fill(nullptr);  // init

    for (std::size_t i = 0; i < size / sizeof(types::WriteID); ++i) {
      WriteID_EventPtr::const_iterator write;

      const bool valid = from_id[i] != INIT_WRITE &&
                         (write = writes_.find(from_id[i])) != writes_.end() &&
                         write->second->addr == addr &&
                         write->second->iiid != after[i]->iiid;
      if (valid) {
        result[i] = write->second;
      } else {
        if (from_id[i] != INIT_WRITE) {
          // While the checker works even if memory is not 0'ed out
          // completely, as the chances of reading a write-id from a
          // previous test that has already been used in this test is
          // low and doesn't necessarily cause a false positive, it is
          // recommended that memory is 0'ed out for every new test.
          //
          // This does also provides limited checking for single-copy
          // atomicity violations where sizeof(WriteID) > 1.

          std::ostringstream oss;
          oss << __func__ << ": Invalid write!"
              << " A=" << std::hex << addr << " S=" << size;

          if (write != writes_.end()) {
            oss << ((write->second->addr != addr) ? " (addr mismatch)" : "")
                << ((write->second->iiid == after[i]->iiid) ? " (same iiid)"
                                                            : "");
          }

          throw std::logic_error(oss.str());
        }

        auto initial = mc::Event(mc::Event::kWrite, addr, mc::Iiid(-1, addr));
        result[i] = &ew_->events.Insert(initial);
      }

      addr += sizeof(types::WriteID);
    }

    return result;
  }

  mc::cats::ExecWitness *ew() { return ew_; }

  mc::cats::Architecture *arch() { return arch_; }

 private:
  typedef std::unordered_map<types::WriteID, const mc::Event *>
      WriteID_EventPtr;

  mc::cats::ExecWitness *ew_;
  mc::cats::Architecture *arch_;

  WriteID_EventPtr writes_;

  types::WriteID last_write_id_;
  types::Poi last_other_id;
};

template <class Operation, class Backend>
class Compiler {
 public:
  typedef typename Operation::Threads OperationThreads;
  typedef typename Operation::SeqIt OperationSeqIt;
  typedef typename Operation::SeqItStack OperationSeqItStack;
  typedef typename Operation::SeqConst OperationSeqConst;
  typedef typename Operation::Callback OperationCallback;
  typedef typename Operation::CallbackStack OperationCallbackStack;

  explicit Compiler(mc::cats::Architecture *arch, mc::cats::ExecWitness *ew,
                    const OperationThreads *threads = nullptr)
      : asms_(ew, arch) {
    Reset(threads);
  }

  void Reset(const OperationThreads *threads = nullptr) {
    threads_ = threads;

    if (threads_ != nullptr) {
      // Must ensure all Operation instances have been reset.
      for (const auto &thread : (*threads_)) {
        for (const auto &op : thread.second) {
          op->Reset();
        }
      }
    }

    asms_.Reset();
    ip_to_op_.clear();
  }

  const OperationThreads *threads() { return threads_; }

  const AssemblerState *asms() const { return &asms_; }

  std::size_t Emit(types::InstPtr base, Operation *op, void *code,
                   std::size_t len, OperationSeqConst *ops,
                   OperationCallbackStack *callback_stack) {
    // Prepare op for emit.
    if (!op->EnableEmit(&asms_)) {
      return 0;
    }

    // Generate program-order.
    if (ops != nullptr) {
      assert(!ops->empty());
      op->InsertPo(--ops->end(), &asms_);
      ops->push_back(op);
    } else {
      OperationSeqConst invalid{nullptr};
      op->InsertPo(invalid.begin(), &asms_);
    }

    std::size_t ctrl_len = 0;

    if (callback_stack != nullptr) {
      // Call all registered callbacks
      for (auto &callback : (*callback_stack)) {
        // Pass in current base, e.g. to allow late resolving of branch
        // targets; allows inserting code for flow control.
        const std::size_t s = callback(op, &backend_, base, &asms_, code, len);

        assert(s < len);
        base += s;
        code = static_cast<char *>(code) + s;
        len -= s;
        ctrl_len += s;
      }

      // Register callback
      op->RegisterCallback(callback_stack);
    }

    // Must be called *after* InsertPo and callback!
    const std::size_t op_len = op->Emit(&backend_, base, &asms_, code, len);
    assert(op_len != 0);

    // Base IP must be unique!
    assert(IpToOp(base) == nullptr);
    // Insert IP to Operation mapping.
    ip_to_op_[base] = std::make_pair(base + op_len, op);

    return op_len + ctrl_len;
  }

  std::size_t Emit(types::Pid pid, types::InstPtr base, void *code,
                   std::size_t len) {
    assert(threads_ != nullptr);

    auto thread = threads_->find(pid);

    if (thread == threads_->end()) {
      return 0;
    }

    std::size_t emit_len = 0;

    // Maintain const sequence of *emitted* ops; nullptr denotes beginning
    // of sequence (in absence of ops.begin()).
    //
    // This will be a flattened sequence of ops (evaluated recursive ops).
    OperationSeqConst ops{nullptr};
    ops.reserve(thread->second.size() + 1);

    // Callback function list
    OperationCallbackStack callback_stack;

    // Enable recursive, nested sequences.
    OperationSeqItStack it_stack;
    it_stack.emplace_back(thread->second.begin(), thread->second.end());

    while (!it_stack.empty()) {
      auto &it = it_stack.back().first;
      auto &end = it_stack.back().second;

      if (it == end) {
        it_stack.pop_back();
        continue;
      }

      const auto &op = *it;

      // Generate code and architecture-specific ordering relations.
      const std::size_t op_len = Emit(base + emit_len, op.get(), code,
                                      len - emit_len, &ops, &callback_stack);

      emit_len += op_len;
      assert(emit_len <= len);
      code = static_cast<char *>(code) + op_len;

      op->AdvanceSeq(&it_stack);
    }

    // Notify ops of completion
    for (auto &callback : callback_stack) {
      const std::size_t s = callback(nullptr, &backend_, base + emit_len,
                                     &asms_, code, len - emit_len);

      emit_len += s;
      assert(emit_len <= len);
      code = static_cast<char *>(code) + s;
    }

    return emit_len;
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size) {
    auto op = IpToOp(ip);

    if (op == nullptr) {
      return false;
    }

    return op->UpdateObs(ip, part, addr, from_id, size, &asms_);
  }

  Operation *IpToOp(types::InstPtr ip) {
    if (ip_to_op_.empty()) {
      // Can be legally empty if no code has yet been emitted, i.e. right
      // after host system startup. By not faulting here, the host can
      // still use ip_to_op to check if an instruction needs to be
      // treated specially: before any code has been emitted, no
      // instructions will be treated specially.
      return nullptr;
    }

    auto e = ip_to_op_.upper_bound(ip);
    if (e != ip_to_op_.begin()) {
      e--;
    }

    if (!(e->first <= ip && ip < e->second.first)) {
      return nullptr;
    }

    return e->second.second;
  }

 private:
  typedef std::map<types::InstPtr, std::pair<types::InstPtr, Operation *>>
      InstPtr_Op;

  AssemblerState asms_;
  Backend backend_;
  const OperationThreads *threads_;

  // Each processor executes unique code, hence IP must be unique. Only stores
  // the start IP of Op-sequence.
  InstPtr_Op ip_to_op_;
};

}  // namespace codegen
}  // namespace mc2lib

#endif /* MC2LIB_CODEGEN_COMPILER_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
