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

#if 1
#include <mc2lib/memconsistency/cats.hpp>
#include <mc2lib/memconsistency/model12.hpp>
#include <mc2lib/simplega.hpp>
#include <mc2lib/codegen/ops/x86_64.hpp>
#include <mc2lib/codegen/rit.hpp>
#else
#include "../include/mc2lib/memconsistency/cats.hpp"
#include "../include/mc2lib/memconsistency/model12.hpp"
#include "../include/mc2lib/simplega.hpp"
#include "../include/mc2lib/codegen/ops/x86_64.hpp"
#include "../include/mc2lib/codegen/rit.hpp"
#endif

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Mc2LibTest
#include <boost/test/unit_test.hpp>

#include <iostream>
#include <string>
#include <vector>

using namespace mc2lib::codegen;
using namespace mc2lib::memconsistency;
using namespace mc2lib::simplega;
using namespace mc2lib;

static Event base_event;
static const Event& ResetEvt() {
  base_event.iiid.poi = 0;
  return base_event;
}
static const Event& NextEvt() {
  ++base_event.iiid;
  return base_event;
}

std::default_random_engine* generator_ptr = nullptr;

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

class GenomeAdd : public Genome<float> {
 public:
  GenomeAdd() {
    genome_.resize(5);
    MutateImpl(1.0f);
  }

  GenomeAdd(const GenomeAdd& parent1, const GenomeAdd& parent2,
            const std::vector<float>& g)
      : Genome<float>(g) {}

  void Mutate(float rate) override { MutateImpl(rate); };

  float Fitness() const override {
    float total = 0.0;
    for (const auto& f : genome_) {
      total += f;
    }

    // restrict size
    if (genome_.size() > 10) {
      return 999.0f;
    }

    // want to get sum closest to 24.
    return (24 - total) * (24 - total);
  }

  bool operator<(const Genome& rhs) const override {
    return Fitness() < rhs.Fitness();
  }

  operator float() const override { return 1000.f - Fitness(); }

 private:
  void MutateImpl(float rate) {
    std::uniform_int_distribution<std::size_t> dist_idx(0, genome_.size() - 1);
    std::uniform_real_distribution<float> dist_mut(-2.0f, 2.0f);
    std::unordered_set<std::size_t> used;
    std::size_t selection_count =
        static_cast<std::size_t>(static_cast<float>(genome_.size()) * rate);

    while (selection_count) {
      auto idx = dist_idx(*generator_ptr);
      if (used.find(idx) != used.end()) {
        continue;
      }

      genome_[idx] += dist_mut(*generator_ptr);

      used.insert(idx);
      --selection_count;
    }
  }
};

BOOST_AUTO_TEST_CASE(SimpleGAAdd24) {
  std::default_random_engine generator(1234);
  generator_ptr = &generator;

  GenePool<GenomeAdd> pool(25,     // population_size
                           0.3f);  // mutation_rate

  std::size_t tournament_size = 10;
  std::size_t tournament_winners = 5;
  std::size_t elite = tournament_winners;

  for (int i = 0; i < 50; ++i) {
    auto tournament_population = pool.SelectUniform(generator, tournament_size);
    pool.SelectionSort(&tournament_population);
    pool.Step(generator,
              evolve::CutSpliceMutate<std::default_random_engine, GenomeAdd,
                                      GenePool<GenomeAdd>::Population>,
              tournament_population, tournament_winners, elite);
    BOOST_CHECK(pool.population_size() <= pool.target_population_size() + 1);
  }

  // This mainly checks that the discrete_distribution implementation works
  // as expected.
  BOOST_CHECK(
      GenePool<GenomeAdd>(pool.SelectRoulette(generator, tournament_size))
          .AverageFitness() <
      GenePool<GenomeAdd>(pool.SelectUniform(generator, tournament_size))
          .AverageFitness());

  BOOST_CHECK(pool.BestFitness() < pool.WorstFitness());

  auto gene = pool.SelectBest();
  float sum = 0.0f;
  for (const auto& f : gene.Get()) {
    sum += f;
  }

  BOOST_CHECK(sum >= 23.1f && sum <= 24.9);
}

BOOST_AUTO_TEST_CASE(CodeGen_X86_64) {
  std::default_random_engine urng(1238);

  cats::ExecWitness ew;
  cats::Arch_TSO arch;

  const types::Addr offset = 0x0;
  strong::RandomFactory factory(0, 1, offset + 0xccc0, offset + 0xccca);
  RandInstTest<std::default_random_engine, strong::RandomFactory> rit(
      urng, &factory, 150);

  const auto threads = rit.threads();
  BOOST_CHECK_EQUAL(threads.size(), 2);
  BOOST_CHECK_EQUAL(threads_size(threads), rit.Get().size());

  Compiler<strong::Operation, strong::Backend_X86_64> compiler(&arch, &ew,
                                                               &threads);

  char code[1024];

  std::size_t emit_len = compiler.Emit(0, 0xffff, code, sizeof(code));
  BOOST_CHECK(emit_len != 0);
  BOOST_CHECK(compiler.IpToOp(0xffff - 1) == nullptr);
  BOOST_CHECK(compiler.IpToOp(0xffff + emit_len) == nullptr);
  BOOST_CHECK(compiler.IpToOp(0x1234) == nullptr);
  BOOST_CHECK(compiler.IpToOp(0xffff) != nullptr);
  BOOST_CHECK(compiler.IpToOp(0xffff + emit_len - 1) != nullptr);

  emit_len = compiler.Emit(1, 0, code, sizeof(code));
  BOOST_CHECK(emit_len != 0);

#if 1
  auto checker = arch.MakeChecker(&arch, &ew);
  ew.po.set_props(mc::EventRel::kTransitiveClosure);
  ew.co.set_props(mc::EventRel::kTransitiveClosure);

  types::WriteID wid = 0;
  // This test passing is dependent on the random number generator
  // implementation.
  BOOST_CHECK(compiler.UpdateObs(0x42, 0, 0xccc5, &wid, 1));  // write 0xccc5
  BOOST_CHECK(compiler.UpdateObs(0x62, 0, 0xccc5, &wid, 1));  // read  0xccc5
  BOOST_CHECK(!checker->sc_per_location());

  wid = 0x27;  // check replacement/update works
  BOOST_CHECK(compiler.UpdateObs(0x62, 0, 0xccc5, &wid, 1));  // read  0xccc5
  BOOST_CHECK(checker->sc_per_location());

  // Check atomic works
  wid = 0;
  BOOST_CHECK(compiler.UpdateObs(0xe9, 0, 0xccc1, &wid, 1));
  wid = 0x28;  // restart atomic
  BOOST_CHECK(compiler.UpdateObs(0xe9, 0, 0xccc1, &wid, 1));
  BOOST_CHECK(compiler.UpdateObs(0xe9, 1, 0xccc1, &wid, 1));
#endif

#if 0
    memset(code + emit_len, 0x90, sizeof(code) - emit_len);
    auto f = fopen("/tmp/mc2lib-test.bin", "wb");
    fwrite(code, sizeof(code), 1, f);
    fclose(f);
#endif
}

#if defined(__linux__) && defined(__x86_64__)
#include <sys/mman.h>
BOOST_AUTO_TEST_CASE(CodeGen_X86_64_ExecLinux) {
  cats::ExecWitness ew;
  cats::Arch_TSO arch;

  Compiler<strong::Operation, strong::Backend_X86_64> compiler(&arch, &ew);

  unsigned char test_mem[] = {0x03, 0x14, 0x25, 0x36, 0x47, 0x58, 0x69, 0x7a,
                              0x8b, 0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2};

  strong::Operation::Ptr ops[] = {
      std::make_shared<strong::Read>(
          reinterpret_cast<types::Addr>(&test_mem[0xf])),
      std::make_shared<strong::Return>()};

  const std::size_t MAX_CODE_SIZE = 4096;
  char* code = reinterpret_cast<char*>(mmap(NULL, MAX_CODE_SIZE,
                                            PROT_READ | PROT_WRITE | PROT_EXEC,
                                            MAP_ANONYMOUS | MAP_PRIVATE, 0, 0));
  memset(code, 0x90, MAX_CODE_SIZE);

  std::size_t emit_len = 0;
  for (auto& op : ops) {
    emit_len += compiler.Emit(emit_len, op.get(), code + emit_len,
                              MAX_CODE_SIZE - emit_len, nullptr, nullptr);
  }

  unsigned char (*func)() = reinterpret_cast<unsigned char (*)()>(code);
  unsigned result = func();

  BOOST_CHECK_EQUAL(result, test_mem[0xf]);

#if 0
    auto f = fopen("/tmp/mc2lib-test.bin", "wb");
    fwrite(code, MAX_CODE_SIZE, 1, f);
    fclose(f);
#endif

  munmap(code, MAX_CODE_SIZE);
}
#endif
