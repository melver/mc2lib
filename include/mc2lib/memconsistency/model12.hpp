/*
 * Copyright (c) 2014, Marco Elver
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
 * J. Alglave, L. Maranget, S. Sarkar, and P. Sewell. "Fences in weak memory
 *      models", 2012.
 *
 */

#ifndef MC2LIB_MEMCONSISTENCY_MODEL12_HPP_
#define MC2LIB_MEMCONSISTENCY_MODEL12_HPP_

#include "eventsets.hpp"

namespace mc2lib {
namespace memconsistency {
namespace model12 {

class ExecWitness;

class Architecture {
  public:
    virtual ~Architecture()
    {}

    virtual EventRel ppo(const ExecWitness&) const = 0;
    virtual EventRel grf(const ExecWitness&) const = 0;
    virtual EventRel ab(const ExecWitness&)  const = 0;

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
     * rf and ws require this. This is an extension.
     */
    virtual Addr addrToLine(Addr) const = 0;
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
            const auto ws_reach = ws.reachable(rf_tuples.first);
            for (const auto& ws_w : ws_reach.get()) {
                for (const auto& rf_r : rf_tuples.second.get()) {
                    if (filterFunc(std::make_pair(rf_tuples.first, rf_r),
                                   std::make_pair(rf_tuples.first, ws_w))) {
                        er.insert(rf_r, ws_w);
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

    EventRel wsi() const
    {
        return ws.filter([](const EventRel::Tuple& t)
                         { return t.first.iiid.pid == t.second.iiid.pid; });
    }

    EventRel wse() const
    {
        return ws.filter([](const EventRel::Tuple& t)
                         { return t.first.iiid.pid != t.second.iiid.pid; });
    }

    EventRel com() const
    {
        return rf + ws + fr();
    }

    EventRel po_loc() const
    {
        return po.filter([](const EventRel::Tuple& t)
                         { return t.first.addr == t.second.addr; });
    }

    EventRel ghb(const Architecture& arch) const
    {
        return ws + fr() + arch.ppo(*this) + arch.grf(*this) + arch.ab(*this);
    }

    void clear()
    {
        events.clear();
        po.clear();
        dp.clear();
        rf.clear();
        ws.clear();
    }

  public:
    EventSet events;
    EventRel po;
    EventRel dp;
    EventRel rf;
    EventRel ws;

};

class Checker {
  public:
    Checker(const Architecture *arch, const ExecWitness *exec)
        : arch_(arch), exec_(exec)
    {}

    virtual ~Checker()
    {}

    virtual bool wf_rf()
    {
        EventSet reads;

        for (const auto& tuples : exec_->rf.raw()) {
            if (!tuples.first.any_type(arch_->eventTypeWrite())) {
                assert(false);
                return false;
            }

            for (const auto& e : tuples.second.get()) {
                if (   !e.any_type(arch_->eventTypeRead())
                    || arch_->addrToLine(tuples.first.addr) != arch_->addrToLine(e.addr))
                {
                    assert(false);
                    return false;
                }

                // For every read, there exists only 1 source!
                if (reads.contains(e)) {
                    assert(false);
                    return false;
                }
                reads += e;
            }
        }

        return true;
    }

    virtual bool wf_ws()
    {
        // Assert writes ordered captured in ws are to the same location.
        for (const auto& tuples : exec_->ws.raw()) {
            for (const auto& e : tuples.second.get()) {
                if (arch_->addrToLine(tuples.first.addr) != arch_->addrToLine(e.addr)) {
                    assert(false);
                    return false;
                }
            }
        }

        return exec_->ws.strict_total_order(
                exec_->events.filter([&](const Event& e) {
                        return e.any_type(arch_->eventTypeWrite());
                    }));
    }

    virtual bool wf()
    {
        return wf_rf() && wf_ws();
    }

    virtual bool uniproc()
    {
        return (exec_->com() + exec_->po_loc()).acyclic();
    }

    virtual bool thin()
    {
        return (exec_->rf + exec_->dp).acyclic();
    }

    virtual bool check_exec()
    {
        return exec_->ghb(*arch_).acyclic();
    }

    virtual bool valid_exec()
    {
        return wf()
            && uniproc()
            && thin()
            && check_exec();
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
        return ew.po;
    }

    EventRel grf(const ExecWitness& ew) const
    {
        return ew.rf;
    }

    EventRel ab(const ExecWitness& ew) const
    {
        return EventRel();
    }

    Event::TypeMask eventTypeRead() const
    {
        return Event::Read;
    }

    Event::TypeMask eventTypeWrite() const
    {
        return Event::Write;
    }

    Addr addrToLine(Addr a) const
    {
        return a;
    }
};

class Arch_TSO : public Architecture {
  public:
    EventRel ppo(const ExecWitness& ew) const
    {
        return ew.po.filter([](const EventRel::Tuple& t)
                            { return !t.first.all_type(Event::Write)
                                  || !t.second.all_type(Event::Read); });
    }

    EventRel grf(const ExecWitness& ew) const
    {
        return ew.rfe();
    }

    EventRel ab(const ExecWitness& ew) const
    {
        return EventRel();
    }

    Event::TypeMask eventTypeRead() const
    {
        return Event::Read;
    }

    Event::TypeMask eventTypeWrite() const
    {
        return Event::Write;
    }

    Addr addrToLine(Addr a) const
    {
        return a;
    }
};

} /* namespace model12 */
} /* namespace memconsistency */
} /* namespace mc2lib */

#endif /* MEMCONSISTENCY_MODEL12_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
