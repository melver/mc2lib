// This code is licensed under the BSD 3-Clause license. See the LICENSE file
// in the project root for license terms.

#include "mc2lib/memconsistency/eventsets.hpp"
#include "mc2lib/sets.hpp"

#include <gtest/gtest.h>

using namespace mc2lib;
using namespace mc2lib::memconsistency;

static Event base_event;
static Event ResetEvt() {
  base_event.iiid.poi = 0;
  return base_event;
}
static Event NextEvt() {
  ++base_event.iiid;
  return base_event;
}

TEST(Sets, SimpleSet) {
  EventSet s = EventSet({NextEvt(), NextEvt(), NextEvt()});
  ASSERT_EQ((s * s).size(), 9);
  ASSERT_FALSE(s.Subset(s));
  ASSERT_TRUE(s.SubsetEq(s));
  ASSERT_TRUE(s == (s * s).Range());
  ASSERT_FALSE((s | s) != s);
}

TEST(Sets, CycleDetectionUnionNo) {
  Event e1 = ResetEvt();
  Event e2, e3;

  EventRel er1;
  er1.Insert(e3 = e1, e2 = NextEvt());
  er1.Insert(e2, e1 = NextEvt());
  er1.Insert(e3, e1);

  ASSERT_FALSE(er1.props());
  ASSERT_TRUE(er1.Acyclic());
  ASSERT_FALSE(er1.props());

  EventRel er2;
  er2.Insert(e1, e2 = NextEvt());
  er2.Insert(e1, e2 = NextEvt());
  er2.Insert(e1, e2 = NextEvt());

  ASSERT_TRUE(er1.Transitive());
  ASSERT_TRUE(er2.Transitive());
  ASSERT_TRUE((er1 | er2).Acyclic());
  er1 |= er2;
  ASSERT_EQ(er1.size(), 6);

  er2.set_props(EventRel::kTransitiveClosure);
  ASSERT_TRUE((EventRel() | er2).Acyclic());
}

TEST(Sets, CycleDetectionYes) {
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

  ASSERT_FALSE(er.Acyclic());

  er.set_props(EventRel::kReflexiveTransitiveClosure);
  ASSERT_EQ(er.size(), 43);
}

TEST(Sets, CycleDetectionYes2) {
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
  ASSERT_FALSE(er.Acyclic(&p));
  ASSERT_TRUE(p[1] == p.back());
  ASSERT_EQ(p.size(), 6);

  p.clear();
  er.set_props(EventRel::kTransitiveClosure);
  ASSERT_TRUE(er.R(e3, e1, &p));
  ASSERT_TRUE(p.front() == e3);
  ASSERT_TRUE(p.back() == e1);
  ASSERT_EQ(p.size(), 4);
}

TEST(Sets, EventRelDiff) {
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
  ASSERT_EQ(er.size(), 4);

  d.set_props(EventRel::kReflexiveTransitiveClosure);
  auto evald = d.Eval();
  ASSERT_TRUE(d == evald);
  ASSERT_TRUE(d.Raw() != evald.Raw());
  ASSERT_EQ(d.size(), 5);
}

TEST(Sets, EventRelDiffProps) {
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
  ASSERT_EQ(d.size(), 2);

  er.add_props(EventRel::kReflexiveClosure);
  d -= er;
  ASSERT_EQ(d.size(), 1);
}

TEST(Sets, EventRelIntersect) {
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
  ASSERT_EQ((d & er).size(), 1);

  d.set_props(EventRel::kReflexiveClosure);
  d &= er;
  ASSERT_FALSE(d.props());
  ASSERT_EQ(d.size(), 2);

  ASSERT_TRUE(d.Domain().Subset(er.Domain()));
}

TEST(Sets, EventRelInverse) {
  Event e1 = ResetEvt();
  Event e2;

  EventRel er;
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e2, e1 = NextEvt());
  er.Insert(e1, e2 = NextEvt());
  er.Insert(e1, e2 = NextEvt());

  EventRel inv = er.Inverse();
  ASSERT_TRUE(er.Domain() == inv.Range());
  ASSERT_TRUE(er.Range() == inv.Domain());
  ASSERT_EQ(er.size(), inv.size());

  er.for_each([&inv](const Event& e1, const Event& e2) {
    ASSERT_TRUE(inv.R(e2, e1));
    ASSERT_FALSE(inv.R(e1, e2));
  });

  er.set_props(EventRel::kReflexiveTransitiveClosure);
  inv = er.Inverse();
  ASSERT_TRUE(er.Domain() == inv.Range());
  ASSERT_TRUE(er.Range() == inv.Domain());
  ASSERT_EQ(er.size(), inv.size());
}

TEST(Sets, EventRelSeqR) {
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
  ASSERT_TRUE(ers.R(start, end));
  ASSERT_TRUE(ers.R(start, e1));
  ASSERT_TRUE(ers.Irreflexive());

  const EventRel evald = ers.Eval();
  auto ers_copy = ers;
  const EventRel evald_inplace = ers_copy.EvalClear();
  ASSERT_TRUE(evald == evald_inplace);
  ASSERT_FALSE(evald != evald_inplace);

  ers.EvalInplace();
  // Should be same result after eval_inplace
  ASSERT_TRUE(ers.R(start, end));
  ASSERT_TRUE(ers.R(start, e1));
  ASSERT_TRUE(ers.Irreflexive());

  // Check evald
  ASSERT_TRUE(evald.R(start, end));
  ASSERT_TRUE(evald.R(start, e1));
  ASSERT_TRUE(evald.Irreflexive());
}

TEST(Sets, EventRelSeqIrrefl1) {
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

  ASSERT_FALSE(ers.Irreflexive());

  const EventRel evald = ers.Eval();
  ASSERT_FALSE(evald.Irreflexive());

  EventRel::Path p;
  ers.Irreflexive(&p);
  ASSERT_EQ(p.size(), 5);
}

TEST(Sets, EventRelSeqIrrefl2) {
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
  ASSERT_TRUE(er.Irreflexive());

  er.add_props(EventRel::kReflexiveClosure);
  ASSERT_FALSE(er.Irreflexive());
  ers += er;

  ASSERT_FALSE(ers.Irreflexive());

  const EventRel evald = ers.Eval();
  ASSERT_FALSE(evald.Irreflexive());
}

TEST(Sets, EventRelReflexivePath) {
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
  ASSERT_FALSE(er.Irreflexive(&p));
  ASSERT_EQ(p.size(), 2);
  ASSERT_TRUE(p[0] == p[1]);

  p.clear();
  ASSERT_FALSE(ers.Irreflexive(&p));
  ASSERT_EQ(p.size(), 3);
  ASSERT_TRUE(p[0] == p[1]);
  ASSERT_TRUE(p[1] == p[2]);
}

TEST(Sets, EventRelSubset) {
  Event e1 = ResetEvt();
  Event e2;

  EventRel er1;
  er1.Insert(e1, e2 = NextEvt());
  er1.Insert(e2, e1 = NextEvt());
  er1.Insert(e2, e1 = NextEvt());
  er1.set_props(EventRel::kReflexiveTransitiveClosure);

  EventRel er2 = er1.Eval();
  ASSERT_TRUE(er2.SubsetEq(er1));
  ASSERT_TRUE(er1.SubsetEq(er2));
  ASSERT_FALSE(er2.Subset(er1));

  er2.Insert(e1, e2 = NextEvt());
  er2.Insert(e1, e2 = NextEvt());

  ASSERT_FALSE(er2.Subset(er1));
  ASSERT_TRUE(er1.Subset(er2));
}
