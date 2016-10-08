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

#ifndef MC2LIB_CODEGEN_COMPILER_HPP_
#define MC2LIB_CODEGEN_COMPILER_HPP_

#include <array>
#include <cassert>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../memconsistency/eventsets.hpp"
#include "../types.hpp"

namespace mc2lib {

/**
 * @namespace mc2lib::codegen
 * @brief Code generation for memory consistency verification.
 */
namespace codegen {

namespace mc = memconsistency;

template <std::size_t max_size_bytes>
using EventPtrs =
    std::array<const mc::Event *, max_size_bytes / sizeof(types::WriteID)>;

template <class... Ts>
inline auto MakeEventPtrs(const mc::Event *e1, Ts... en)
    -> EventPtrs<(1 + sizeof...(Ts)) * sizeof(types::WriteID)> {
  EventPtrs<(1 + sizeof...(Ts)) * sizeof(types::WriteID)> es{{e1, en...}};
  return es;
}

/**
 * Baseclass for Operation implementations.
 */
template <class Backend, class EvtStateT>
class Op {
 public:
  typedef EvtStateT EvtState;

  // Types
  typedef std::shared_ptr<Op> Ptr;
  typedef std::vector<Ptr> Thread;
  typedef typename Thread::const_iterator ThreadIt;
  typedef std::unordered_map<types::Pid, Thread> Threads;
  typedef std::vector<std::pair<ThreadIt, ThreadIt>> ThreadItStack;

  // Read-only thread types: new Ops can access previous Ops.
  typedef std::vector<const Op *> ThreadConst;
  typedef typename ThreadConst::const_iterator ThreadConstIt;

  // Callback type: optionally, previous Ops get called back with new Ops.
  // E.g. for lazily constructing control flow graph with random branches.
  typedef std::function<std::size_t(Op *, types::InstPtr, Backend *, EvtState *,
                                    void *, std::size_t)>
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

  virtual void AdvanceThread(ThreadItStack *it_stack) const {
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
   * @param[in,out] evts Pointer to EvtState instance of calling Compiler.
   *
   * @return true if can emit; false otherwise.
   */
  virtual bool EnableEmit(EvtState *evts) = 0;

  /**
   * Generate static program-order relation.
   *
   * @param before Pointer to last Op; nullptr if none exists.
   * @param[in,out] evts Pointer to EvtState instance maintained by
   *                     Compiler.
   */
  virtual void InsertPo(ThreadConstIt before, EvtState *evts) = 0;

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
   * @param start Instruction pointer to first instruction when executing.
   * @param[in,out] backend Architecture backend.
   * @param[in,out] evts Pointer to EvtState instance of calling Compiler.
   * @param[out] code Pointer to memory to be copied into.
   * @param len Maximum lenth of code.
   *
   * @return Size of emitted code.
   */
  virtual std::size_t Emit(types::InstPtr start, Backend *backend,
                           EvtState *evts, void *code, std::size_t len) = 0;

  /**
   * Accessor for last event generated. Also used to insert additional
   * ordering based on passed next_event (e.g. fences).
   *
   * @param next_event Event after last in program order; nullptr if none
   *                   exists.
   * @param[in,out] evts Pointer to EvtState instance maintained by
   *                     Compiler.
   *
   * @return Last event in program-order; nullptr if none exists.
   */
  virtual const mc::Event *LastEvent(const mc::Event *next_event,
                                     EvtState *evts) const = 0;

  /**
   * Accessor for first event generated.
   *
   * @param prev_event Event before first in program order; nullptr if none
   *                   exists.
   * @param[in,out] evts Pointer to EvtState instance maintained by
   *                     Compiler.
   *
   * @return First event in program-order; nullptr if none exists.
   */
  virtual const mc::Event *FirstEvent(const mc::Event *prev_event,
                                      EvtState *evts) const = 0;

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
   * @param[in,out] evts Pointer to EvtState instance maintained by
   *                     Compiler.
   *
   * @return Success or not.
   */
  virtual bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                         const types::WriteID *from_id, std::size_t size,
                         EvtState *evts) = 0;

  types::Pid pid() const { return pid_; }

  void set_pid(types::Pid pid) { pid_ = pid; }

 private:
  types::Pid pid_;
};

template <class Backend, class EvtState>
class MemOp : public Op<Backend, EvtState> {
 public:
  explicit MemOp(types::Pid pid) : Op<Backend, EvtState>(pid) {}

  virtual types::Addr addr() const = 0;
};

template <class Backend, class EvtState>
class NullOp : public Op<Backend, EvtState> {
 public:
  explicit NullOp(types::Pid pid) : Op<Backend, EvtState>(pid) {}

  bool EnableEmit(EvtState *evts) override { return false; }

  void InsertPo(typename Op<Backend, EvtState>::ThreadConstIt before,
                EvtState *evts) override {
    throw std::logic_error("Not supported");
  }

  std::size_t Emit(types::InstPtr start, Backend *backend, EvtState *evts,
                   void *code, std::size_t len) override {
    throw std::logic_error("Not supported");
    return 0;
  }

  bool UpdateObs(types::InstPtr ip, int part, types::Addr addr,
                 const types::WriteID *from_id, std::size_t size,
                 EvtState *evts) override {
    throw std::logic_error("Not supported");
    return false;
  }

  const mc::Event *LastEvent(const mc::Event *next_event,
                             EvtState *evts) const override {
    throw std::logic_error("Not supported");
    return nullptr;
  }

  const mc::Event *FirstEvent(const mc::Event *prev_event,
                              EvtState *evts) const override {
    throw std::logic_error("Not supported");
    return nullptr;
  }
};

/**
 * @brief Top level class used to manage code generation (compiler).
 */
template <class Operation, class Backend>
class Compiler {
 public:
  typedef typename Operation::EvtState EvtState;
  typedef typename Operation::Threads Threads;
  typedef typename Operation::ThreadIt ThreadIt;
  typedef typename Operation::ThreadItStack ThreadItStack;
  typedef typename Operation::ThreadConst ThreadConst;
  typedef typename Operation::Callback Callback;
  typedef typename Operation::CallbackStack CallbackStack;

  /**
   * Implies a reset of state in evts.
   *
   * @param evts Pointer to instance of EvtState as required by Operation. The
   *             choice of unique_ptr here is deliberate, in that the class
   *             that Operation requires may be a virtual base class, and
   *             Compiler takes ownership.
   */
  explicit Compiler(std::unique_ptr<EvtState> evts) : evts_(std::move(evts)) {
    Reset();
  }

  /**
   * Implies a reset of state in evts and threads (the Ops contained).
   *
   * @param evts Pointer to instance of EvtState as required by Operation. The
   *             choice of unique_ptr here is deliberate, in that the class
   *             that Operation requires may be a virtual base class.
   *             Takes ownership.
   * @param[in,out] threads Threads container. The Ops pointed to by Threads may
   *                        be modified.
   */
  explicit Compiler(std::unique_ptr<EvtState> evts, Threads &&threads)
      : evts_(std::move(evts)) {
    Reset(std::move(threads));
  }

  void Reset() {
    evts_->Reset();
    backend_.Reset();
    ip_to_op_.clear();
  }

  void Reset(Threads &&threads) {
    threads_ = std::move(threads);

    // Must ensure all Operation instances have been reset.
    for (const auto &thread : threads_) {
      for (const auto &op : thread.second) {
        op->Reset();
      }
    }

    Reset();
  }

  const Threads &threads() { return threads_; }

  const EvtState *evts() const { return evts_.get(); }

  EvtState *evts() { return evts_.get(); }

  std::size_t Emit(types::InstPtr base, Operation *op, void *code,
                   std::size_t len, ThreadConst *thread_const_ops,
                   CallbackStack *callback_stack) {
    // Prepare op for emit.
    if (!op->EnableEmit(evts_.get())) {
      return 0;
    }

    // Generate program-order.
    if (thread_const_ops != nullptr) {
      assert(!thread_const_ops->empty());
      op->InsertPo(--thread_const_ops->end(), evts_.get());
      thread_const_ops->push_back(op);
    } else {
      ThreadConst invalid{nullptr};
      op->InsertPo(invalid.begin(), evts_.get());
    }

    std::size_t ctrl_len = 0;

    if (callback_stack != nullptr) {
      // Call all registered callbacks
      for (auto &callback : (*callback_stack)) {
        // Pass in current base, e.g. to allow late resolving of branch
        // targets; allows inserting code for flow control.
        const std::size_t s =
            callback(op, base, &backend_, evts_.get(), code, len);

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
    const std::size_t op_len =
        op->Emit(base, &backend_, evts_.get(), code, len);
    assert(op_len != 0);

    // Base IP must be unique!
    assert(IpToOp(base) == nullptr);
    // Insert IP to Operation mapping.
    ip_to_op_[base] = std::make_pair(base + op_len, op);

    return op_len + ctrl_len;
  }

  std::size_t Emit(types::Pid pid, types::InstPtr base, void *code,
                   std::size_t len) {
    auto thread = threads_.find(pid);

    if (thread == threads_.end()) {
      return 0;
    }

    std::size_t emit_len = 0;

    // Maintain const sequence of *emitted* ops; nullptr denotes beginning
    // of sequence (in absence of thread_const_ops.begin()).
    //
    // This will be a flattened sequence of ops (evaluated recursive ops).
    ThreadConst thread_const_ops{nullptr};
    thread_const_ops.reserve(thread->second.size() + 1);

    // Callback function list
    CallbackStack callback_stack;

    // Enable recursive, nested sequences.
    ThreadItStack it_stack;
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
      const std::size_t op_len =
          Emit(base + emit_len, op.get(), code, len - emit_len,
               &thread_const_ops, &callback_stack);

      emit_len += op_len;
      assert(emit_len <= len);
      code = static_cast<char *>(code) + op_len;

      op->AdvanceThread(&it_stack);
    }

    // Notify ops of completion
    for (auto &callback : callback_stack) {
      const std::size_t s = callback(nullptr, base + emit_len, &backend_,
                                     evts_.get(), code, len - emit_len);

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

    return op->UpdateObs(ip, part, addr, from_id, size, evts_.get());
  }

  Operation *IpToOp(types::InstPtr ip) const {
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

  std::unique_ptr<EvtState> evts_;
  Backend backend_;
  Threads threads_;

  // Each processor executes unique code, hence IP must be unique. Only stores
  // the start IP of Op's code.
  InstPtr_Op ip_to_op_;
};

}  // namespace codegen
}  // namespace mc2lib

#endif /* MC2LIB_CODEGEN_COMPILER_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
