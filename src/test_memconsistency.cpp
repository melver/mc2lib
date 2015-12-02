#include "mc2lib/memconsistency/cats.hpp"
#include "mc2lib/memconsistency/model12.hpp"

#include <string>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace mc2lib::memconsistency;

BOOST_AUTO_TEST_SUITE(memconsistency)

BOOST_AUTO_TEST_CASE(Model12Empty) {
  model12::ExecWitness ew;
  model12::Arch_SC sc;
  auto c = sc.MakeChecker(&sc, &ew);

  BOOST_CHECK_NO_THROW(c->wf());
  BOOST_CHECK(c->uniproc());
  BOOST_CHECK(c->thin());
  BOOST_CHECK(c->check_exec());
  BOOST_CHECK_NO_THROW(c->valid_exec());
}

BOOST_AUTO_TEST_CASE(CatsEmpty) {
  cats::ExecWitness ew;
  cats::Arch_SC sc;
  auto c = sc.MakeChecker(&sc, &ew);

  BOOST_CHECK_NO_THROW(c->wf());
  BOOST_CHECK(c->sc_per_location());
  BOOST_CHECK(c->no_thin_air());
  BOOST_CHECK(c->observation());
  BOOST_CHECK(c->propagation());
  BOOST_CHECK_NO_THROW(c->valid_exec());
}

BOOST_AUTO_TEST_CASE(Model12DekkerValidSC) {
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

  BOOST_CHECK_NO_THROW(c->wf());
  BOOST_CHECK(c->uniproc());
  BOOST_CHECK(c->thin());
  BOOST_CHECK(c->check_exec());
  BOOST_CHECK_NO_THROW(c->valid_exec());
}

BOOST_AUTO_TEST_CASE(CatsDekkerInvalidSCValidTSO) {
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

  BOOST_CHECK_NO_THROW(c_sc->wf());
  BOOST_CHECK(c_sc->sc_per_location());
  BOOST_CHECK(c_sc->no_thin_air());
  BOOST_CHECK(c_sc->observation());
  BOOST_CHECK(!c_sc->propagation());
  BOOST_CHECK_EXCEPTION(c_sc->valid_exec(), Error, [](const Error& e) {
    return std::string("PROPAGATION") == e.what();
  });

  BOOST_CHECK_NO_THROW(c_tso->wf());
  BOOST_CHECK(c_tso->sc_per_location());
  BOOST_CHECK(c_tso->no_thin_air());
  BOOST_CHECK(c_tso->observation());
  BOOST_CHECK(c_tso->propagation());
  BOOST_CHECK_NO_THROW(c_tso->valid_exec());
}

BOOST_AUTO_TEST_SUITE_END()
