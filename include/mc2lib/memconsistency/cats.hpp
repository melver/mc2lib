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

#ifndef MC2LIB_MEMCONSISTENCY_CATS_HPP_
#define MC2LIB_MEMCONSISTENCY_CATS_HPP_

#include <memory>

#include "eventsets.hpp"

namespace mc2lib {
namespace memconsistency {

/**
 * @namespace mc2lib::memconsistency::cats
 * @brief Memory consistency model framework based on "Herding cats".
 *
 * This memory consistency model framework is based upon [1], and [2].
 *
 * [1] <a href="http://dx.doi.org/10.1145/2627752">
 *      J. Alglave, L. Maranget, M.  Tautschnig, "Herding cats: Modelling,
 *      Simulation, Testing, and Data Mining for Weak Memory", 2014.</a>
 *
 * [2] <a href="http://dx.doi.org/10.1007/s10703-011-0135-z">
 *      J. Alglave, L. Maranget, S. Sarkar, and P. Sewell. "Fences in weak
 *      memory models", 2012.</a>
*/
namespace cats {

class ExecWitness {
 public:
  template <class FilterFunc>
  EventRel fr(FilterFunc filter_func) const {
    EventRel er;

    // Use of get() is justified, as we do not expect (according to wf_rf), the
    // rf-relation to have any additional properties.
    for (const auto& rf_tuples : rf.get()) {
      const auto co_reach = co.Reachable(rf_tuples.first);
      for (const auto& co_w : co_reach.get()) {
        for (const auto& rf_r : rf_tuples.second.get()) {
          if (filter_func(std::make_pair(rf_tuples.first, rf_r),
                          std::make_pair(rf_tuples.first, co_w))) {
            er.Insert(rf_r, co_w);
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

  EventRel coi() const {
    return co.Filter([](const Event& e1, const Event& e2) {
      return e1.iiid.pid == e2.iiid.pid;
    });
  }

  EventRel coe() const {
    return co.Filter([](const Event& e1, const Event& e2) {
      return e1.iiid.pid != e2.iiid.pid;
    });
  }

  EventRel com() const { return rf | co | fr(); }

  EventRel po_loc() const {
    return po.Filter(
        [](const Event& e1, const Event& e2) { return e1.addr == e2.addr; });
  }

  void Clear() {
    events.Clear();
    po.Clear();
    co.Clear();
    rf.Clear();
  }

 public:
  EventSet events;
  EventRel po;
  EventRel co;
  EventRel rf;
};

class Checker;

class Architecture {
 public:
  Architecture() : proxy_(this) {}

  virtual ~Architecture() { assert(proxy_ == this); }

  virtual void Clear() {}

  /**
   * Creates a checker compatible with this Architecture.
   */
  virtual std::unique_ptr<Checker> MakeChecker(
      const Architecture* arch, const ExecWitness* exec) const = 0;

  virtual EventRel ppo(const ExecWitness& ew) const = 0;
  virtual EventRel fences(const ExecWitness& ew) const = 0;
  virtual EventRel prop(const ExecWitness& ew) const = 0;

  virtual EventRel hb(const ExecWitness& ew) const {
    return ew.rfe() | proxy_->ppo(ew) | proxy_->fences(ew);
  }

  /**
   * Should return the mask of all types that are classed as read.
   */
  virtual Event::Type EventTypeRead() const = 0;

  /**
   * Should return the mask of all types that are classed as write.
   */
  virtual Event::Type EventTypeWrite() const = 0;

  void set_proxy(const Architecture* proxy) {
    assert(proxy != nullptr);
    proxy_ = proxy;
  }

 protected:
  const Architecture* proxy_;
};

template <class ConcreteArch>
class ArchProxy : public Architecture {
 public:
  explicit ArchProxy(ConcreteArch* arch)
      : arch_(arch),
        memoized_ppo_(false),
        memoized_fences_(false),
        memoized_prop_(false),
        memoized_hb_(false) {
    arch_->set_proxy(this);
  }

  ~ArchProxy() override { arch_->set_proxy(arch_); }

  void Clear() override {
    arch_->Clear();
    memoized_ppo_ = false;
    memoized_fences_ = false;
    memoized_prop_ = false;
    memoized_hb_ = false;
  }

  std::unique_ptr<Checker> MakeChecker(const Architecture* arch,
                                       const ExecWitness* exec) const override {
    return arch_->MakeChecker(arch, exec);
  }

  std::unique_ptr<Checker> MakeChecker(const ExecWitness* exec) const {
    return MakeChecker(this, exec);
  }

  void Memoize(const ExecWitness& ew) {
    // fences and ppo are likely used by hb and prop
    fences_ = arch_->fences(ew);
    memoized_fences_ = true;

    ppo_ = arch_->ppo(ew);
    memoized_ppo_ = true;

    hb_ = arch_->hb(ew);
    memoized_hb_ = true;

    prop_ = arch_->prop(ew);
    memoized_prop_ = true;
  }

  EventRel ppo(const ExecWitness& ew) const override {
    return memoized_ppo_ ? ppo_ : arch_->ppo(ew);
  }

  EventRel fences(const ExecWitness& ew) const override {
    return memoized_fences_ ? fences_ : arch_->fences(ew);
  }

  EventRel prop(const ExecWitness& ew) const override {
    return memoized_prop_ ? prop_ : arch_->prop(ew);
  }

  EventRel hb(const ExecWitness& ew) const override {
    return memoized_hb_ ? hb_ : arch_->hb(ew);
  }

  Event::Type EventTypeRead() const override { return arch_->EventTypeRead(); }

  Event::Type EventTypeWrite() const override {
    return arch_->EventTypeWrite();
  }

 protected:
  ConcreteArch* arch_;

  bool memoized_ppo_;
  bool memoized_fences_;
  bool memoized_prop_;
  bool memoized_hb_;

  EventRel ppo_;
  EventRel fences_;
  EventRel prop_;
  EventRel hb_;
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

  virtual void wf_co() const {
    std::unordered_set<types::Addr> addrs;

    // Assert writes ordered captured in ws are to the same location.
    for (const auto& tuples : exec_->co.get()) {
      addrs.insert(tuples.first.addr);

      for (const auto& e : tuples.second.get()) {
        if (tuples.first.addr != e.addr) {
          throw Error("WF_CO_NOT_SAME_LOC");
        }
      }
    }

    auto writes = exec_->events.Filter(
        [&](const Event& e) { return e.AnyType(arch_->EventTypeWrite()); });
    if (!exec_->co.StrictPartialOrder(writes)) {
      throw Error("WF_CO_NOT_STRICT_PARTIAL_ORDER");
    }

    for (const auto& addr : addrs) {
      auto same_addr_writes =
          writes.Filter([&](const Event& e) { return e.addr == addr; });
      if (!exec_->co.ConnexOn(same_addr_writes)) {
        throw Error("WF_CO_NOT_CONNEX");
      }
    }
  }

  virtual void wf() const {
    wf_rf();
    wf_co();
  }

  virtual bool sc_per_location(EventRel::Path* cyclic = nullptr) const {
    return (exec_->com() | exec_->po_loc()).Acyclic(cyclic);
  }

  virtual bool no_thin_air(EventRel::Path* cyclic = nullptr) const {
    return arch_->hb(*exec_).Acyclic(cyclic);
  }

  virtual bool observation(EventRel::Path* cyclic = nullptr) const {
    const EventRel prop = arch_->prop(*exec_);
    const EventRel hbstar =
        arch_->hb(*exec_).set_props(EventRel::kReflexiveTransitiveClosure);

    // Not eval'ing hbstar causes performance to degrade substantially, as
    // EventRelSeq recomputes reachability from nodes from prop to hbstar
    // several times!
    bool r = EventRelSeq({exec_->fre(), prop, hbstar.Eval()}).Irreflexive();

    if (!r && cyclic != nullptr) {
      // However, here we want hbstar unevald, as otherwise the graph is
      // too collapsed.
      EventRelSeq({exec_->fre(), prop, hbstar}).Irreflexive(cyclic);
    }

    return r;
  }

  virtual bool propagation(EventRel::Path* cyclic = nullptr) const {
    return (exec_->co | arch_->prop(*exec_)).Acyclic(cyclic);
  }

  virtual void valid_exec(EventRel::Path* cyclic = nullptr) const {
    wf();

    if (!sc_per_location(cyclic)) {
      throw Error("SC_PER_LOCATION");
    }

    if (!no_thin_air(cyclic)) {
      throw Error("NO_THIN_AIR");
    }

    if (!observation(cyclic)) {
      throw Error("OBSERVATION");
    }

    if (!propagation(cyclic)) {
      throw Error("PROPAGATION");
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

  EventRel fences(const ExecWitness& ew) const override { return EventRel(); }

  EventRel prop(const ExecWitness& ew) const override {
    return proxy_->ppo(ew) | proxy_->fences(ew) | ew.rf | ew.fr();
  }

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

  EventRel fences(const ExecWitness& ew) const override {
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

  EventRel prop(const ExecWitness& ew) const override {
    return proxy_->ppo(ew) | proxy_->fences(ew) | ew.rfe() | ew.fr();
  }

  Event::Type EventTypeRead() const override { return Event::kRead; }

  Event::Type EventTypeWrite() const override { return Event::kWrite; }

 public:
  EventRel mfence;
};

/**
 * ARMv7 as defined in [1]
 */
class Arch_ARMv7 : public Architecture {
 public:
  Arch_ARMv7() { dd_reg.set_props(EventRel::kTransitiveClosure); }

  void Clear() override {
    dd_reg.Clear();
    dsb.Clear();
    dmb.Clear();
    dsb_st.Clear();
    dmb_st.Clear();
    isb.Clear();
  }

  std::unique_ptr<Checker> MakeChecker(const Architecture* arch,
                                       const ExecWitness* exec) const override {
    return std::unique_ptr<Checker>(new Checker(arch, exec));
  }

  EventRel ppo(const ExecWitness& ew) const override {
    assert(ew.po.Transitive());
    assert(dd_reg.SubsetEq(ew.po));

    // 1. Obtain dependencies
    //
    EventRel addr, data, ctrl_part;
    dd_reg.for_each(
        [&addr, &data, &ctrl_part, this](const Event& e1, const Event& e2) {
          if (!e1.AnyType(EventTypeRead())) {
            return;
          }

          if (e2.AnyType(Event::kMemoryOperation)) {
            if (e2.AllType(Event::kRegInAddr)) {
              addr.Insert(e1, e2);
            }

            if (e2.AllType(Event::kRegInData)) {
              data.Insert(e1, e2);
            }
          }

          if (e2.AllType(Event::kBranch)) {
            ctrl_part.Insert(e1, e2);
          }
        });

    EventRel ctrl = EventRelSeq({ctrl_part, ew.po}).EvalClear();
    EventRel ctrl_cfence = EventRelSeq({ctrl_part, isb}).EvalClear();

    // 2. Compute helper relations
    //
    const auto po_loc = ew.po_loc();
    const auto rfe = ew.rfe();
    EventRel dd = addr | data;
    EventRel rdw = po_loc & EventRelSeq({ew.fre(), rfe}).EvalClear();
    EventRel detour = po_loc & EventRelSeq({ew.coe(), rfe}).EvalClear();
    EventRel addrpo = EventRelSeq({addr, ew.po}).EvalClear();

    // 3. Compute ppo
    //
    // Init
    EventRel ci = ctrl_cfence | detour;
    EventRel ii = dd | ew.rfi() | rdw;
    EventRel cc = dd | ctrl | addrpo | po_loc;
    EventRel ic;

    std::size_t total_size = ci.size() + ii.size() + cc.size() + ic.size();
    std::size_t prev_total_size;

    // Fix-point computation
    do {
      prev_total_size = total_size;

      ci |=
          EventRelSeq({ci, ii}).EvalClear() | EventRelSeq({cc, ci}).EvalClear();

      ii |= ci | EventRelSeq({ic, ci}).EvalClear() |
            EventRelSeq({ii, ii}).EvalClear();

      cc |= ci | EventRelSeq({ci, ic}).EvalClear() |
            EventRelSeq({cc, cc}).EvalClear();

      ic |= ii | cc | EventRelSeq({ic, cc}).EvalClear() |
            EventRelSeq({ii, ic}).EvalClear();

      total_size = ci.size() + ii.size() + cc.size() + ic.size();
      assert(prev_total_size <= total_size);
    } while (total_size != prev_total_size);

    EventRel result = ic.Filter([this](const Event& e1, const Event& e2) {
      return e1.AnyType(EventTypeRead()) && e2.AnyType(EventTypeWrite());
    });
    result |= ii.Filter([this](const Event& e1, const Event& e2) {
      return e1.AnyType(EventTypeRead()) && e2.AnyType(EventTypeRead());
    });

    return result;
  }

  // Ensure fences is transitive
  EventRel fences(const ExecWitness& ew) const override {
    const auto postar = ew.po.Eval().set_props(EventRel::kReflexiveClosure);
    const auto postar_WW = postar.Filter([&](const Event& e1, const Event& e2) {
      return e1.AllType(Event::kWrite) && e2.AllType(Event::kWrite);
    });

    auto ff = EventRelSeq({postar, (dmb | dsb), postar}).EvalClear();
    ff |= EventRelSeq({postar_WW, (dmb_st | dsb_st), postar_WW}).EvalClear();
    return ff;
  }

  EventRel prop(const ExecWitness& ew) const override {
    EventRel hbstar = proxy_->hb(ew)
                          .set_props(EventRel::kReflexiveTransitiveClosure)
                          .EvalInplace();
    EventRel A_cumul = EventRelSeq({ew.rfe(), proxy_->fences(ew)}).EvalClear();
    EventRel propbase =
        EventRelSeq({(proxy_->fences(ew) | A_cumul), hbstar}).EvalClear();

    EventRel comstar = ew.com().set_props(EventRel::kReflexiveClosure);

    EventRel result = propbase.Filter([this](const Event& e1, const Event& e2) {
      return e1.AnyType(EventTypeWrite()) && e2.AnyType(EventTypeWrite());
    });

    propbase.set_props(EventRel::kReflexiveTransitiveClosure).EvalInplace();
    result |=
        EventRelSeq({comstar, propbase /*star*/, proxy_->fences(ew), hbstar})
            .EvalClear();
    return result;
  }

  Event::Type EventTypeRead() const override { return Event::kRead; }

  Event::Type EventTypeWrite() const override { return Event::kWrite; }

 public:
  EventRel dd_reg;
  EventRel dsb;
  EventRel dmb;
  EventRel dsb_st;
  EventRel dmb_st;
  EventRel isb;
};

}  // namespace cats
}  // namespace memconsistency
}  // namespace mc2lib

#endif /* MEMCONSISTENCY_CATS_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
