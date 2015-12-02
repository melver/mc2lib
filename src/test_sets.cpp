#include "mc2lib/memconsistency/eventsets.hpp"
#include "mc2lib/sets.hpp"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace mc2lib;
using namespace mc2lib::memconsistency;

BOOST_AUTO_TEST_SUITE(sets)

static Event base_event;
static const Event& ResetEvt() {
  base_event.iiid.poi = 0;
  return base_event;
}
static const Event& NextEvt() {
  ++base_event.iiid;
  return base_event;
}

BOOST_AUTO_TEST_CASE(SimpleSet) {
  EventSet s = EventSet({NextEvt(), NextEvt(), NextEvt()});
  BOOST_CHECK((s * s).size() == 9);
  BOOST_CHECK(!s.Subset(s));
  BOOST_CHECK(s.SubsetEq(s));
  BOOST_CHECK(s == (s * s).Range());
  BOOST_CHECK((s | s) == s);
}

BOOST_AUTO_TEST_CASE(CycleDetectionUnionNo) {
  Event e1 = ResetEvt();
  Event e2, e3;

  EventRel er1;
  er1.Insert(e3 = e1, e2 = NextEvt());
  er1.Insert(e2, e1 = NextEvt());
  er1.Insert(e3, e1);

  BOOST_CHECK(!er1.props());
  BOOST_CHECK(er1.Acyclic());
  BOOST_CHECK(!er1.props());

  EventRel er2;
  er2.Insert(e1, e2 = NextEvt());
  er2.Insert(e1, e2 = NextEvt());
  er2.Insert(e1, e2 = NextEvt());

  BOOST_CHECK(er1.Transitive());
  BOOST_CHECK(er2.Transitive());
  BOOST_CHECK((er1 | er2).Acyclic());
  er1 |= er2;
  BOOST_CHECK_EQUAL(er1.size(), 6);

  er2.set_props(EventRel::kTransitiveClosure);
  BOOST_CHECK((EventRel() | er2).Acyclic());
}

BOOST_AUTO_TEST_CASE(CycleDetectionYes) {
  Event e1 = ResetEvt();
  Event e2;
  Event e3;

  EventRel er;
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e3 = e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());

  EventRel er_;
  er_.Insert(e1, e2 = NextEvt());
  er_.Insert(e2, e3);
  er_.Insert(e2, NextEvt());
  er_.Insert(e2, NextEvt());
  er_.Insert(e2, NextEvt());
  er_.Insert(e2, NextEvt());
  er |= er_;

  BOOST_CHECK(!er.Acyclic());

  er.set_props(EventRel::kReflexiveTransitiveClosure);
  BOOST_CHECK_EQUAL(er.size(), 43);
}

BOOST_AUTO_TEST_CASE(CycleDetectionYes2) {
  Event e1 = ResetEvt();
  Event e2;
  Event e3;

  EventRel er;
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e3 = e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e1, e3);

  for (int i = 0; i < 30; ++i) {
    // Catch out buggy cycle detection implementations, where they do not
    // consider a visisted node after being visited once before.
    //
    // As unordered maps are used as backing store, by adding more edges
    // that do not contribute to the cycle, we are likely to traverse these
    // first.
    er.Insert(NextEvt(), e3);
  }

  EventRel::Path p;
  BOOST_CHECK(!er.Acyclic(&p));
  BOOST_CHECK(p[1] == p.back());
  BOOST_CHECK_EQUAL(p.size(), 6);

  p.clear();
  er.set_props(EventRel::kTransitiveClosure);
  BOOST_CHECK(er.R(e3, e1, &p));
  BOOST_CHECK(p.front() == e3);
  BOOST_CHECK(p.back() == e1);
  BOOST_CHECK_EQUAL(p.size(), 4);
}

BOOST_AUTO_TEST_CASE(EventRelDiff) {
  Event e1 = ResetEvt();
  Event e2;

  EventRel er;
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());

  EventRel d;
  d.Insert(e1, e2);
  d.Insert(e1, NextEvt());
  er -= d;
  BOOST_CHECK_EQUAL(er.size(), 4);

  d.set_props(EventRel::kReflexiveTransitiveClosure);
  auto evald = d.Eval();
  BOOST_CHECK(d == evald);
  BOOST_CHECK(d.Raw() != evald.Raw());
  BOOST_CHECK_EQUAL(d.size(), 5);
}

BOOST_AUTO_TEST_CASE(EventRelDiffProps) {
  Event e1 = ResetEvt();
  Event e2, start;

  EventRel er;
  er.Insert(start = e1, e2 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());

  EventRel d;
  d.Insert(start, e2);
  d.Insert(e2, e2);
  d.Insert(e2, start);
  er.add_props(EventRel::kTransitiveClosure);
  d -= er;
  BOOST_CHECK_EQUAL(d.size(), 2);

  er.add_props(EventRel::kReflexiveClosure);
  d -= er;
  BOOST_CHECK_EQUAL(d.size(), 1);
}

BOOST_AUTO_TEST_CASE(EventRelIntersect) {
  Event e1 = ResetEvt();
  Event e2;

  EventRel er;
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e1);

  EventRel d;
  d.Insert(e1, e2);
  d.Insert(e1, NextEvt());
  d.Insert(e1, NextEvt());
  BOOST_CHECK_EQUAL((d & er).size(), 1);

  d.set_props(EventRel::kReflexiveClosure);
  d &= er;
  BOOST_CHECK(!d.props());
  BOOST_CHECK_EQUAL(d.size(), 2);

  BOOST_CHECK(d.Domain().Subset(er.Domain()));
}

BOOST_AUTO_TEST_CASE(EventRelInverse) {
  Event e1 = ResetEvt();
  Event e2;

  EventRel er;
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());

  EventRel inv = er.Inverse();
  BOOST_CHECK(er.Domain() == inv.Range());
  BOOST_CHECK(er.Range() == inv.Domain());
  BOOST_CHECK_EQUAL(er.size(), inv.size());

  er.for_each([&inv](const Event& e1, const Event& e2) {
    BOOST_CHECK(inv.R(e2, e1));
    BOOST_CHECK(!inv.R(e1, e2));
  });

  er.set_props(EventRel::kReflexiveTransitiveClosure);
  inv = er.Inverse();
  BOOST_CHECK(er.Domain() == inv.Range());
  BOOST_CHECK(er.Range() == inv.Domain());
  BOOST_CHECK_EQUAL(er.size(), inv.size());
}

BOOST_AUTO_TEST_CASE(EventRelSeqR) {
  Event e1 = ResetEvt();
  Event e2;
  Event start, end;

  EventRelSeq ers;
  EventRel er;
  er.Insert(start = e1, e2 = NextEvt());
  ers += er;

  er = EventRel();
  er.Insert(e2, e1 = NextEvt());
  ers += EventRel(er);

  er = EventRel();
  er.Insert(e1, end = e2 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.add_props(EventRel::kTransitiveClosure);
  ers += er;

  // First unevald
  BOOST_CHECK(ers.R(start, end));
  BOOST_CHECK(ers.R(start, e1));
  BOOST_CHECK(ers.Irreflexive());

  const EventRel evald = ers.Eval();
  auto ers_copy = ers;
  const EventRel evald_inplace = ers_copy.EvalClear();
  BOOST_CHECK(evald == evald_inplace);

  ers.EvalInplace();
  // Should be same result after eval_inplace
  BOOST_CHECK(ers.R(start, end));
  BOOST_CHECK(ers.R(start, e1));
  BOOST_CHECK(ers.Irreflexive());

  // Check evald
  BOOST_CHECK(evald.R(start, end));
  BOOST_CHECK(evald.R(start, e1));
  BOOST_CHECK(evald.Irreflexive());
}

BOOST_AUTO_TEST_CASE(EventRelSeqIrrefl1) {
  Event e1 = ResetEvt();
  Event e2;
  Event start, end;

  EventRelSeq ers;
  EventRel er;
  er.Insert(e1, NextEvt());
  er.Insert(e1, NextEvt());
  er.Insert(e1, NextEvt());
  er.Insert(start = e1, e2 = NextEvt());
  ers += er;

  er = EventRel();
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.add_props(EventRel::kTransitiveClosure);
  ers += er;

  er = EventRel();
  er.Insert(e2, start);
  ers += er;

  BOOST_CHECK(!ers.Irreflexive());

  const EventRel evald = ers.Eval();
  BOOST_CHECK(!evald.Irreflexive());

  EventRel::Path p;
  ers.Irreflexive(&p);
  BOOST_CHECK_EQUAL(p.size(), 5);
}

BOOST_AUTO_TEST_CASE(EventRelSeqIrrefl2) {
  Event e1 = ResetEvt();
  Event e2;
  Event start;

  EventRelSeq ers;
  EventRel er;
  er.Insert(start = e1, e2 = NextEvt());
  ers += er;

  er = EventRel();
  er.Insert(e2, start);
  ers += er;

  er = EventRel();
  er.Insert(start, NextEvt());
  BOOST_CHECK(er.Irreflexive());

  er.add_props(EventRel::kReflexiveClosure);
  BOOST_CHECK(!er.Irreflexive());
  ers += er;

  BOOST_CHECK(!ers.Irreflexive());

  const EventRel evald = ers.Eval();
  BOOST_CHECK(!evald.Irreflexive());
}

BOOST_AUTO_TEST_CASE(EventRelReflexivePath) {
  Event e1 = ResetEvt();
  Event e2;

  EventRelSeq ers;

  EventRel er;
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.set_props(EventRel::kReflexiveClosure);
  ers += er;

  er = EventRel();
  er.Insert(e2, e1 = NextEvt());
  er.set_props(EventRel::kReflexiveClosure);
  ers += er;

  EventRel::Path p;
  BOOST_CHECK(!er.Irreflexive(&p));
  BOOST_CHECK_EQUAL(p.size(), 2);
  BOOST_CHECK(p[0] == p[1]);

  p.clear();
  BOOST_CHECK(!ers.Irreflexive(&p));
  BOOST_CHECK_EQUAL(p.size(), 3);
  BOOST_CHECK(p[0] == p[1]);
  BOOST_CHECK(p[1] == p[2]);
}

BOOST_AUTO_TEST_CASE(EventRelSubset) {
  Event e1 = ResetEvt();
  Event e2;

  EventRel er1;
  er1.Insert(e1, e2 = NextEvt());
  er1.Insert(e2, e1 = NextEvt());
  er1.Insert(e2, e1 = NextEvt());
  er1.set_props(EventRel::kReflexiveTransitiveClosure);

  EventRel er2 = er1.Eval();
  BOOST_CHECK(er2.SubsetEq(er1));
  BOOST_CHECK(er1.SubsetEq(er2));
  BOOST_CHECK(!er2.Subset(er1));

  er2.Insert(e1, e2 = NextEvt());
  er2.Insert(e1, e2 = NextEvt());

  BOOST_CHECK(!er2.Subset(er1));
  BOOST_CHECK(er1.Subset(er2));
}

BOOST_AUTO_TEST_SUITE_END()
