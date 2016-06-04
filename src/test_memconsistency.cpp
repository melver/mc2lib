// This code is licensed under the BSD 3-Clause license. See the LICENSE file
// in the project root for license terms.

#include "mc2lib/memconsistency/cats.hpp"
#include "mc2lib/memconsistency/model12.hpp"

#include <string>

#include <gtest/gtest.h>

using namespace mc2lib::memconsistency;

TEST(MemConsistency, Model12Empty) {
  model12::ExecWitness ew;
  model12::Arch_SC sc;
  auto c = sc.MakeChecker(&sc, &ew);

  ASSERT_NO_THROW(c->wf());
  ASSERT_TRUE(c->uniproc());
  ASSERT_TRUE(c->thin());
  ASSERT_TRUE(c->check_exec());
  ASSERT_NO_THROW(c->valid_exec());
}

TEST(MemConsistency, CatsEmpty) {
  cats::ExecWitness ew;
  cats::Arch_SC sc;
  auto c = sc.MakeChecker(&sc, &ew);

  ASSERT_NO_THROW(c->wf());
  ASSERT_TRUE(c->sc_per_location());
  ASSERT_TRUE(c->no_thin_air());
  ASSERT_TRUE(c->observation());
  ASSERT_TRUE(c->propagation());
  ASSERT_NO_THROW(c->valid_exec());
}

TEST(MemConsistency, Model12DekkerValidSC) {
  model12::ExecWitness ew;
  model12::Arch_SC sc;
  auto c = sc.MakeChecker(&sc, &ew);

  Event Ix = Event(Event::kWrite, 10, Iiid(-1, 0));
  Event Iy = Event(Event::kWrite, 20, Iiid(-1, 1));

  Event Wx0 = Event(Event::kWrite, 10, Iiid(0, 12));
  Event Wy1 = Event(Event::kWrite, 20, Iiid(1, 33));
  Event Ry0 = Event(Event::kRead, 20, Iiid(0, 55));
  Event Rx1 = Event(Event::kRead, 10, Iiid(1, 22));

  ew.events |= EventSet({Ix, Iy, Wx0, Wy1, Ry0, Rx1});

  ew.po.Insert(Wx0, Ry0);
  ew.po.Insert(Wy1, Rx1);

  ew.ws.Insert(Ix, Wx0);
  ew.ws.Insert(Iy, Wy1);

  ew.rf.Insert(Wx0, Rx1);
  ew.rf.Insert(Wy1, Ry0);

  ASSERT_NO_THROW(c->wf());
  ASSERT_TRUE(c->uniproc());
  ASSERT_TRUE(c->thin());
  ASSERT_TRUE(c->check_exec());
  ASSERT_NO_THROW(c->valid_exec());
}

TEST(MemConsistency, CatsDekkerInvalidSCValidTSO) {
  cats::ExecWitness ew;
  cats::Arch_SC sc;
  cats::Arch_TSO tso;
  auto c_sc = sc.MakeChecker(&sc, &ew);
  auto c_tso = tso.MakeChecker(&tso, &ew);

  Event Ix = Event(Event::kWrite, 10, Iiid(-1, 0));
  Event Iy = Event(Event::kWrite, 20, Iiid(-1, 1));

  Event Wx0 = Event(Event::kWrite, 10, Iiid(0, 12));
  Event Wy1 = Event(Event::kWrite, 20, Iiid(1, 33));
  Event Ry0 = Event(Event::kRead, 20, Iiid(0, 55));
  Event Rx1 = Event(Event::kRead, 10, Iiid(1, 22));

  ew.events |= EventSet({Ix, Iy, Wx0, Wy1, Ry0, Rx1});

  ew.po.Insert(Wx0, Ry0);
  ew.po.Insert(Wy1, Rx1);

  ew.co.Insert(Ix, Wx0);
  ew.co.Insert(Iy, Wy1);

  ew.rf.Insert(Ix, Rx1);
  ew.rf.Insert(Iy, Ry0);

  ASSERT_NO_THROW(c_sc->wf());
  ASSERT_TRUE(c_sc->sc_per_location());
  ASSERT_TRUE(c_sc->no_thin_air());
  ASSERT_TRUE(c_sc->observation());
  ASSERT_FALSE(c_sc->propagation());

  try {
    c_sc->valid_exec();
    FAIL();
  } catch (const Error& e) {
    ASSERT_EQ(std::string("PROPAGATION"), e.what());
  }

  ASSERT_NO_THROW(c_tso->wf());
  ASSERT_TRUE(c_tso->sc_per_location());
  ASSERT_TRUE(c_tso->no_thin_air());
  ASSERT_TRUE(c_tso->observation());
  ASSERT_TRUE(c_tso->propagation());
  ASSERT_NO_THROW(c_tso->valid_exec());
}
