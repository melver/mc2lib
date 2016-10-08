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

#ifndef MC2LIB_MEMCONSISTENCY_MODEL12_HPP_
#define MC2LIB_MEMCONSISTENCY_MODEL12_HPP_

#include <memory>

#include "eventsets.hpp"

namespace mc2lib {
namespace memconsistency {

/**
 * @namespace mc2lib::memconsistency::model12
 * @brief Memory consistency model framework based on 2012 FMSD paper.
 *
 * This memory consistency model framework is based upon [1].
 *
 * [1] <a href="http://dx.doi.org/10.1007/s10703-011-0135-z">
 *      J. Alglave, L. Maranget, S. Sarkar, and P. Sewell. "Fences in weak
 *      memory models", 2012.</a>
*/
namespace model12 {

class ExecWitness {
 public:
  template <class FilterFunc>
  EventRel fr(FilterFunc filter_func) const {
    EventRel er;

    for (const auto& rf_tuples : rf.get()) {
      const auto ws_reach = ws.Reachable(rf_tuples.first);
      for (const auto& ws_w : ws_reach.get()) {
        for (const auto& rf_r : rf_tuples.second.get()) {
          if (filter_func(std::make_pair(rf_tuples.first, rf_r),
                          std::make_pair(rf_tuples.first, ws_w))) {
            er.Insert(rf_r, ws_w);
          }
        }
      }
    }

    return er;
  }

  EventRel fr() const {
    return fr([](const EventRel::Tuple& t1, const EventRel::Tuple& t2) {
      return true;
    });
  }

  EventRel fri() const {
    return fr([](const EventRel::Tuple& t1, const EventRel::Tuple& t2) {
      return t1.second.iiid.pid == t2.second.iiid.pid;
    });
  }

  EventRel fre() const {
    return fr([](const EventRel::Tuple& t1, const EventRel::Tuple& t2) {
      return t1.second.iiid.pid != t2.second.iiid.pid;
    });
  }

  EventRel rfi() const {
    return rf.Filter([](const Event& e1, const Event& e2) {
      return e1.iiid.pid == e2.iiid.pid;
    });
  }

  EventRel rfe() const {
    return rf.Filter([](const Event& e1, const Event& e2) {
      return e1.iiid.pid != e2.iiid.pid;
    });
  }

  EventRel wsi() const {
    return ws.Filter([](const Event& e1, const Event& e2) {
      return e1.iiid.pid == e2.iiid.pid;
    });
  }

  EventRel wse() const {
    return ws.Filter([](const Event& e1, const Event& e2) {
      return e1.iiid.pid != e2.iiid.pid;
    });
  }

  EventRel com() const { return rf | ws | fr(); }

  EventRel po_loc() const {
    return po.Filter(
        [](const Event& e1, const Event& e2) { return e1.addr == e2.addr; });
  }

  void Clear() {
    events.Clear();
    po.Clear();
    dp.Clear();
    rf.Clear();
    ws.Clear();
  }

 public:
  EventSet events;
  EventRel po;
  EventRel dp;
  EventRel rf;
  EventRel ws;
};

class Checker;

class Architecture {
 public:
  virtual ~Architecture() {}

  virtual void Clear() {}

  /**
   * Creates a checker compatible with this Architecture.
   */
  virtual std::unique_ptr<Checker> MakeChecker(
      const Architecture* arch, const ExecWitness* exec) const = 0;

  virtual EventRel ppo(const ExecWitness& ew) const = 0;
  virtual EventRel grf(const ExecWitness& ew) const = 0;
  virtual EventRel ab(const ExecWitness& ew) const = 0;

  virtual EventRel ghb(const ExecWitness& ew) const {
    return ew.ws | ew.fr() | ppo(ew) | grf(ew) | ab(ew);
  }

  /**
   * Should return the mask of all types that are classed as read.
   */
  virtual Event::Type EventTypeRead() const = 0;

  /**
   * Should return the mask of all types that are classed as write.
   */
  virtual Event::Type EventTypeWrite() const = 0;
};

class Checker {
 public:
  Checker(const Architecture* arch, const ExecWitness* exec)
      : arch_(arch), exec_(exec) {}

  virtual ~Checker() {}

  virtual void wf_rf() const {
    EventSet reads;

    for (const auto& tuples : exec_->rf.get()) {
      if (!tuples.first.AnyType(arch_->EventTypeWrite())) {
        throw Error("WF_RF_NOT_FROM_WRITE");
      }

      for (const auto& e : tuples.second.get()) {
        if (!e.AnyType(arch_->EventTypeRead()) || tuples.first.addr != e.addr) {
          throw Error("WF_RF_NOT_SAME_LOC");
        }

        // For every read, there exists only 1 source!
        if (reads.Contains(e)) {
          throw Error("WF_RF_MULTI_SOURCE");
        }
        reads.Insert(e);
      }
    }
  }

  virtual void wf_ws() const {
    std::unordered_set<types::Addr> addrs;

    // Assert writes ordered captured in ws are to the same location.
    for (const auto& tuples : exec_->ws.get()) {
      addrs.insert(tuples.first.addr);

      for (const auto& e : tuples.second.get()) {
        if (tuples.first.addr != e.addr) {
          throw Error("WF_WS_NOT_SAME_LOC");
        }
      }
    }

    auto writes = exec_->events.Filter(
        [&](const Event& e) { return e.AnyType(arch_->EventTypeWrite()); });
    if (!exec_->ws.StrictPartialOrder(writes)) {
      throw Error("WF_WS_NOT_STRICT_PARTIAL_ORDER");
    }

    for (const auto& addr : addrs) {
      auto same_addr_writes =
          writes.Filter([&](const Event& e) { return e.addr == addr; });
      if (!exec_->ws.ConnexOn(same_addr_writes)) {
        throw Error("WF_WS_NOT_CONNEX");
      }
    }
  }

  virtual void wf() const {
    wf_rf();
    wf_ws();
  }

  virtual bool uniproc(EventRel::Path* cyclic = nullptr) const {
    return (exec_->com() | exec_->po_loc()).Acyclic(cyclic);
  }

  virtual bool thin(EventRel::Path* cyclic = nullptr) const {
    return (exec_->rf | exec_->dp).Acyclic(cyclic);
  }

  virtual bool check_exec(EventRel::Path* cyclic = nullptr) const {
    return arch_->ghb(*exec_).Acyclic(cyclic);
  }

  virtual void valid_exec(EventRel::Path* cyclic = nullptr) const {
    wf();

    if (!uniproc(cyclic)) {
      throw Error("UNIPROC");
    }

    if (!thin(cyclic)) {
      throw Error("THIN");
    }

    if (!check_exec(cyclic)) {
      throw Error("CHECK_EXEC");
    }
  }

 protected:
  const Architecture* arch_;
  const ExecWitness* exec_;
};

/*
=============================
Some common memory models.
=============================
*/

class Arch_SC : public Architecture {
 public:
  std::unique_ptr<Checker> MakeChecker(const Architecture* arch,
                                       const ExecWitness* exec) const override {
    return std::unique_ptr<Checker>(new Checker(arch, exec));
  }

  EventRel ppo(const ExecWitness& ew) const override {
    assert(ew.po.Transitive());
    return ew.po.Eval();
  }

  EventRel grf(const ExecWitness& ew) const override { return ew.rf; }

  EventRel ab(const ExecWitness& ew) const override { return EventRel(); }

  Event::Type EventTypeRead() const override { return Event::kRead; }

  Event::Type EventTypeWrite() const override { return Event::kWrite; }
};

class Arch_TSO : public Architecture {
 public:
  void Clear() override { mfence.Clear(); }

  std::unique_ptr<Checker> MakeChecker(const Architecture* arch,
                                       const ExecWitness* exec) const override {
    return std::unique_ptr<Checker>(new Checker(arch, exec));
  }

  EventRel ppo(const ExecWitness& ew) const override {
    assert(ew.po.Transitive());
    return ew.po.Filter([](const Event& e1, const Event& e2) {
      return !e1.AllType(Event::kWrite) || !e2.AllType(Event::kRead);
    });
  }

  EventRel grf(const ExecWitness& ew) const override { return ew.rfe(); }

  EventRel ab(const ExecWitness& ew) const override {
    if (mfence.empty()) {
      return mfence;
    }

    // Filter postar by only those events which are possibly relevent.
    const auto postar =
        ew.po
            .Filter([&](const Event& e1, const Event& e2) {
              // Only include those where first event is write or second
              // is a read, all other are included in po regardless.
              return e1.AllType(Event::kWrite) || e2.AllType(Event::kRead);
            })
            .set_props(EventRel::kReflexiveClosure);

    return EventRelSeq({postar, mfence, postar}).EvalClear();
  }

  Event::Type EventTypeRead() const override { return Event::kRead; }

  Event::Type EventTypeWrite() const override { return Event::kWrite; }

 public:
  EventRel mfence;
};

}  // namespace model12
}  // namespace memconsistency
}  // namespace mc2lib

#endif /* MEMCONSISTENCY_MODEL12_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
