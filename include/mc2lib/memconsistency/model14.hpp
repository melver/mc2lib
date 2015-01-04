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

/*
 * Based on the axiomatic framework for defining memory consistency models in
 *
 * J. Alglave, L. Maranget, M. Tautschnig, "Herding cats", 2014.
 *
 */

#ifndef MC2LIB_MEMCONSISTENCY_MODEL14_HPP_
#define MC2LIB_MEMCONSISTENCY_MODEL14_HPP_

#include "eventsets.hpp"

namespace mc2lib {
namespace memconsistency {
namespace model14 {

class ExecWitness;

class Architecture {
  public:
    virtual ~Architecture()
    {}

    virtual void clear()
    {}

    virtual EventRel ppo(const ExecWitness&) const = 0;
    virtual EventRel fences(const ExecWitness&) const = 0;
    virtual EventRel prop(const ExecWitness&) const = 0;

    /*
     * Should return the mask of all types that are classed as read.
     */
    virtual Event::TypeMask eventTypeRead() const = 0;

    /*
     * Should return the mask of all types that are classed as write.
     */
    virtual Event::TypeMask eventTypeWrite() const = 0;

    /*
     * How we deal with address to cache-line/tag conversion.
     *
     * This permits having a relations with events that are to different
     * addresses, but to the same cache-line! The well-formedness checks for
     * rf and co require this. This is an extension.
     */
    virtual types::Addr addrToLine(types::Addr) const = 0;
};

class ExecWitness {
  public:

    /*
     * Use of raw() in rf is justified, as we do not expect (according to
     * wf_rf), the rf-relation to have any additional properties.
     */
    template <class FilterFunc>
    EventRel fr(FilterFunc filterFunc) const
    {
        EventRel er;
        for (const auto& rf_tuples : rf.raw()) {
            const auto co_reach = co.reachable(rf_tuples.first);
            for (const auto& co_w : co_reach.get()) {
                for (const auto& rf_r : rf_tuples.second.get()) {
                    if (filterFunc(std::make_pair(rf_tuples.first, rf_r),
                                   std::make_pair(rf_tuples.first, co_w))) {
                        er.insert(rf_r, co_w);
                    }
                }
            }
        }
        return er;
    }

    EventRel fr() const
    {
        return fr([](const EventRel::Tuple& t1,
                     const EventRel::Tuple& t2)
                  { return true; });
    }

    EventRel fri() const
    {
        return fr([](const EventRel::Tuple& t1,
                     const EventRel::Tuple& t2)
                  { return t1.second.iiid.pid == t2.second.iiid.pid; });
    }

    EventRel fre() const
    {
        return fr([](const EventRel::Tuple& t1,
                     const EventRel::Tuple& t2)
                  { return t1.second.iiid.pid != t2.second.iiid.pid; });
    }

    EventRel rfi() const
    {
        return rf.filter([](const EventRel::Tuple& t)
                         { return t.first.iiid.pid == t.second.iiid.pid; });
    }

    EventRel rfe() const
    {
        return rf.filter([](const EventRel::Tuple& t)
                         { return t.first.iiid.pid != t.second.iiid.pid; });
    }

    EventRel coi() const
    {
        return co.filter([](const EventRel::Tuple& t)
                         { return t.first.iiid.pid == t.second.iiid.pid; });
    }

    EventRel coe() const
    {
        return co.filter([](const EventRel::Tuple& t)
                         { return t.first.iiid.pid != t.second.iiid.pid; });
    }

    EventRel com() const
    {
        return rf + co + fr();
    }

    EventRel po_loc() const
    {
        return po.filter([](const EventRel::Tuple& t)
                         { return t.first.addr == t.second.addr; });
    }

    EventRel hb(const Architecture& arch) const
    {
        return rfe() + arch.ppo(*this) + arch.fences(*this);
    }

    void clear()
    {
        events.clear();
        po.clear();
        co.clear();
        rf.clear();
    }

  public:
    EventSet events;
    EventRel po;
    EventRel co;
    EventRel rf;

};

class Checker {
  public:
    enum class Error {
        OK,
        WF_RF_NOT_FROM_WRITE,
        WF_RF_NOT_SAME_LOC,
        WF_RF_MULTI_SOURCE,
        WF_CO_NOT_SAME_LOC,
        WF_CO_NOT_STRICT_PARTIAL_ORDER,
        WF_CO_NOT_CONNEX
    };

    Checker(const Architecture *arch, const ExecWitness *exec)
        : arch_(arch), exec_(exec)
    {}

    virtual ~Checker()
    {}

    virtual Error wf_rf() const
    {
        EventSet reads;

        for (const auto& tuples : exec_->rf.raw()) {
            if (!tuples.first.any_type(arch_->eventTypeWrite())) {
                return Error::WF_RF_NOT_FROM_WRITE;
            }

            for (const auto& e : tuples.second.get()) {
                if (   !e.any_type(arch_->eventTypeRead())
                    || arch_->addrToLine(tuples.first.addr) != arch_->addrToLine(e.addr))
                {
                    return Error::WF_RF_NOT_SAME_LOC;
                }

                // For every read, there exists only 1 source!
                if (reads.contains(e)) {
                    return Error::WF_RF_MULTI_SOURCE;
                }
                reads += e;
            }
        }

        return Error::OK;
    }

    virtual Error wf_co() const
    {
        std::unordered_set<types::Addr> addrs;

        // Assert writes ordered captured in ws are to the same location.
        for (const auto& tuples : exec_->co.raw()) {
            addrs.insert(tuples.first.addr);

            for (const auto& e : tuples.second.get()) {
                if (arch_->addrToLine(tuples.first.addr) != arch_->addrToLine(e.addr)) {
                    return Error::WF_CO_NOT_SAME_LOC;
                }
            }
        }

        auto writes = exec_->events.filter([&](const Event& e) {
                    return e.any_type(arch_->eventTypeWrite());
                });
        if (!exec_->co.strict_partial_order(writes)) {
            return Error::WF_CO_NOT_STRICT_PARTIAL_ORDER;
        }

        for (const auto& addr : addrs) {
            auto same_addr_writes = writes.filter([&](const Event& e) {
                        return e.addr == addr;
                    });
            if (!exec_->co.connex_on(same_addr_writes)) {
                return Error::WF_CO_NOT_CONNEX;
            }
        }

        return Error::OK;
    }

    virtual bool wf() const
    {
        return wf_rf() == Error::OK && wf_co() == Error::OK;
    }

    virtual bool sc_per_location(EventRel::Path *cyclic = nullptr) const
    {
        return (exec_->com() + exec_->po_loc()).acyclic(cyclic);
    }

    virtual bool no_thin_air(EventRel::Path *cyclic = nullptr) const
    {
        return exec_->hb(*arch_).acyclic(cyclic);
    }

    virtual bool observation(EventRel::Path *cyclic = nullptr) const
    {
        const EventRel prop = arch_->prop(*exec_);
        const EventRel hbstar = exec_->hb(*arch_).set_props(
                EventRel::ReflexiveTransitiveClosure);

        // Not eval'ing hbstar causes performance to degrade substantially, as
        // EventRelSeq recomputes reachability from nodes from prop to hbstar
        // several times!
        return EventRelSeq({exec_->fre(), prop, hbstar.eval()}).irreflexive(cyclic);
    }

    virtual bool propagation(EventRel::Path *cyclic = nullptr) const
    {
        return (exec_->co + arch_->prop(*exec_)).acyclic(cyclic);
    }

    virtual bool valid_exec(EventRel::Path *cyclic = nullptr) const
    {
        return wf()
            && sc_per_location(cyclic)
            && no_thin_air(cyclic)
            && observation(cyclic)
            && propagation(cyclic);
    }

  protected:
    const Architecture *arch_;
    const ExecWitness *exec_;
};

/*
 * Some common memory models.
 */

class Arch_SC : public Architecture {
  public:
    EventRel ppo(const ExecWitness& ew) const
    {
        assert(ew.po.transitive());
        return ew.po.eval();
    }

    EventRel fences(const ExecWitness& ew) const
    {
        return EventRel();
    }

    EventRel prop(const ExecWitness& ew) const
    {
        return ppo(ew) + fences(ew) + ew.rf + ew.fr();
    }

    Event::TypeMask eventTypeRead() const
    {
        return Event::Read;
    }

    Event::TypeMask eventTypeWrite() const
    {
        return Event::Write;
    }

    types::Addr addrToLine(types::Addr a) const
    {
        return a;
    }
};

class Arch_TSO : public Architecture {
  public:
    void clear()
    {
        fences_.clear();
    }

    EventRel ppo(const ExecWitness& ew) const
    {
        assert(ew.po.transitive());
        return ew.po.filter([](const EventRel::Tuple& t)
                            { return !t.first.all_type(Event::Write)
                                  || !t.second.all_type(Event::Read); });
    }

    EventRel fences(const ExecWitness& ew) const
    {
        return fences_;
    }

    EventRel prop(const ExecWitness& ew) const
    {
        return ppo(ew) + fences(ew) + ew.rfe() + ew.fr();
    }

    Event::TypeMask eventTypeRead() const
    {
        return Event::Read;
    }

    Event::TypeMask eventTypeWrite() const
    {
        return Event::Write;
    }

    types::Addr addrToLine(types::Addr a) const
    {
        return a;
    }

    EventRel fences_;
};

} /* namespace model14 */
} /* namespace memconsistency */
} /* namespace mc2lib */

#endif /* MEMCONSISTENCY_MODEL14_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
