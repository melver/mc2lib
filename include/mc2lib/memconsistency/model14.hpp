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

#ifndef MC2LIB_MEMCONSISTENCY_MODEL14_HPP_
#define MC2LIB_MEMCONSISTENCY_MODEL14_HPP_

#include "eventsets.hpp"

#include <exception>

namespace mc2lib {
namespace memconsistency {

/**
 * @namespace mc2lib::memconsistency::model14
 * @brief Memory consistency model framework based on 2014 TOPLAS paper.
 *
 * This memory consistency model framework is based upon [1], and [2].
 *
 * References:
 *
 * [1] J. Alglave, L. Maranget, M. Tautschnig, "Herding cats", 2014.
 *
 * [2] J. Alglave, L. Maranget, S. Sarkar, and P. Sewell. "Fences in weak memory
 *      models", 2012.
*/
namespace model14 {

class ExecWitness;

class Architecture {
  public:
    virtual ~Architecture()
    {}

    virtual void clear()
    {}

    virtual EventRel ppo(const ExecWitness& ew) const = 0;
    virtual EventRel fences(const ExecWitness& ew) const = 0;
    virtual EventRel prop(const ExecWitness& ew) const = 0;

    /*
     * Should return the mask of all types that are classed as read.
     */
    virtual Event::TypeMask eventTypeRead() const = 0;

    /*
     * Should return the mask of all types that are classed as write.
     */
    virtual Event::TypeMask eventTypeWrite() const = 0;
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
        return rf.filter([](const Event& e1, const Event& e2)
                         { return e1.iiid.pid == e2.iiid.pid; });
    }

    EventRel rfe() const
    {
        return rf.filter([](const Event& e1, const Event& e2)
                         { return e1.iiid.pid != e2.iiid.pid; });
    }

    EventRel coi() const
    {
        return co.filter([](const Event& e1, const Event& e2)
                         { return e1.iiid.pid == e2.iiid.pid; });
    }

    EventRel coe() const
    {
        return co.filter([](const Event& e1, const Event& e2)
                         { return e1.iiid.pid != e2.iiid.pid; });
    }

    EventRel com() const
    {
        return rf + co + fr();
    }

    EventRel po_loc() const
    {
        return po.filter([](const Event& e1, const Event& e2)
                         { return e1.addr == e2.addr; });
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
    class Error : public std::exception {
      public:
        explicit Error(const char *w)
            : what_(w)
        {}

        const char* what() const noexcept
        { return what_; }

      private:
        const char *what_;
    };

    Checker(const Architecture *arch, const ExecWitness *exec)
        : arch_(arch), exec_(exec)
    {}

    virtual ~Checker()
    {}

    virtual void wf_rf() const
    {
        EventSet reads;

        for (const auto& tuples : exec_->rf.raw()) {
            if (!tuples.first.any_type(arch_->eventTypeWrite())) {
                throw Error("WF_RF_NOT_FROM_WRITE");
            }

            for (const auto& e : tuples.second.get()) {
                if (   !e.any_type(arch_->eventTypeRead())
                    || tuples.first.addr != e.addr)
                {
                    throw Error("WF_RF_NOT_SAME_LOC");
                }

                // For every read, there exists only 1 source!
                if (reads.contains(e)) {
                    throw Error("WF_RF_MULTI_SOURCE");
                }
                reads += e;
            }
        }
    }

    virtual void wf_co() const
    {
        std::unordered_set<types::Addr> addrs;

        // Assert writes ordered captured in ws are to the same location.
        for (const auto& tuples : exec_->co.raw()) {
            addrs.insert(tuples.first.addr);

            for (const auto& e : tuples.second.get()) {
                if (tuples.first.addr != e.addr) {
                    throw Error("WF_CO_NOT_SAME_LOC");
                }
            }
        }

        auto writes = exec_->events.filter([&](const Event& e) {
                    return e.any_type(arch_->eventTypeWrite());
                });
        if (!exec_->co.strict_partial_order(writes)) {
            throw Error("WF_CO_NOT_STRICT_PARTIAL_ORDER");
        }

        for (const auto& addr : addrs) {
            auto same_addr_writes = writes.filter([&](const Event& e) {
                        return e.addr == addr;
                    });
            if (!exec_->co.connex_on(same_addr_writes)) {
                throw Error("WF_CO_NOT_CONNEX");
            }
        }
    }

    virtual void wf() const
    {
        wf_rf();
        wf_co();
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
        bool r = EventRelSeq({exec_->fre(), prop, hbstar.eval()}).irreflexive();

        if (!r && cyclic != nullptr) {
            EventRelSeq({exec_->fre(), prop, hbstar}).irreflexive(cyclic);
        }

        return r;
    }

    virtual bool propagation(EventRel::Path *cyclic = nullptr) const
    {
        return (exec_->co + arch_->prop(*exec_)).acyclic(cyclic);
    }

    virtual void valid_exec(EventRel::Path *cyclic = nullptr) const
    {
        wf();

        if (!sc_per_location(cyclic))
            throw Error("SC_PER_LOCATION");

        if (!no_thin_air(cyclic))
            throw Error("NO_THIN_AIR");

        if (!observation(cyclic))
            throw Error("OBSERVATION");

        if (!propagation(cyclic))
            throw Error("PROPAGATION");
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
};

class Arch_TSO : public Architecture {
  public:
    void clear()
    {
        mfence.clear();
    }

    EventRel ppo(const ExecWitness& ew) const
    {
        assert(ew.po.transitive());
        return ew.po.filter([](const Event& e1, const Event& e2)
                            { return !e1.all_type(Event::Write)
                                  || !e2.all_type(Event::Read); });
    }

    EventRel fences(const ExecWitness& ew) const
    {
        if (mfence.empty()) {
            return mfence;
        }

        // Filter postar by only those events which are possibly relevent.
        const auto postar = ew.po.filter(
                [&](const Event& e1, const Event& e2) {
                    // Only include those where first event is write or second
                    // is a read, all other are included in po regardless.
                    return e1.all_type(Event::Write) || e2.all_type(Event::Read);
                }).set_props(EventRel::ReflexiveClosure);

        return EventRelSeq({postar, mfence, postar}).eval_inplace();
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

  public:
    EventRel mfence;
};

class ArchProxy : public Architecture {
  public:
    explicit ArchProxy(Architecture *arch)
       : arch_(arch), memoized_(false)
    {}

    void clear()
    {
        arch_->clear();
        memoized_ = false;
    }

    void memoize(const ExecWitness& ew)
    {
        ppo_ = arch_->ppo(ew);
        fences_ = arch_->fences(ew);
        prop_ = arch_->prop(ew);

        memoized_ = true;
    }

    EventRel ppo(const ExecWitness& ew) const
    {
        assert(memoized_);
        return ppo_;
    }

    EventRel fences(const ExecWitness& ew) const
    {
        assert(memoized_);
        return fences_;
    }

    EventRel prop(const ExecWitness& ew) const
    {
        assert(memoized_);
        return prop_;
    }

    Event::TypeMask eventTypeRead() const
    {
        return arch_->eventTypeRead();
    }

    Event::TypeMask eventTypeWrite() const
    {
        return arch_->eventTypeWrite();
    }

  protected:
   Architecture *arch_;
   bool memoized_;

   EventRel ppo_;
   EventRel fences_;
   EventRel prop_;
};

} /* namespace model14 */
} /* namespace memconsistency */
} /* namespace mc2lib */

#endif /* MEMCONSISTENCY_MODEL14_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
