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

#ifndef MC2LIB_CODEGEN_CATS_HPP_
#define MC2LIB_CODEGEN_CATS_HPP_

#include <cassert>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "../memconsistency/cats.hpp"
#include "compiler.hpp"

namespace mc2lib {

namespace codegen {

class AsmStateCats {
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

  explicit AsmStateCats(mc::cats::ExecWitness *ew, mc::cats::Architecture *arch)
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

  const mc::cats::ExecWitness *ew() const { return ew_; }

  mc::cats::Architecture *arch() { return arch_; }

  const mc::cats::Architecture *arch() const { return arch_; }

 private:
  typedef std::unordered_map<types::WriteID, const mc::Event *>
      WriteID_EventPtr;

  mc::cats::ExecWitness *ew_;
  mc::cats::Architecture *arch_;

  WriteID_EventPtr writes_;

  types::WriteID last_write_id_;
  types::Poi last_other_id;
};

}  // namespace codegen
}  // namespace mc2lib

#endif /* MC2LIB_CODEGEN_CATS_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
