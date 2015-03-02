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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Mc2LibTest
#include <boost/test/unit_test.hpp>
#include <iostream>

#if 1
#include <mc2lib/memconsistency.hpp>
#include <mc2lib/simplega.hpp>
#include <mc2lib/codegen/ops.hpp>
#include <mc2lib/codegen/rit.hpp>
#else
#include "../include/mc2lib/memconsistency.hpp"
#include "../include/mc2lib/simplega.hpp"
#include "../include/mc2lib/codegen/ops.hpp"
#include "../include/mc2lib/codegen/rit.hpp"
#endif

using namespace mc2lib::codegen;
using namespace mc2lib::memconsistency;
using namespace mc2lib::simplega;
using namespace mc2lib;

// TODO: add more tests

static Event base_event;
static const Event& resetevt()
{
    base_event.iiid.poi = 0;
    return base_event;
}
static const Event& nextevt()
{
    ++base_event.iiid;
    return base_event;
}

std::default_random_engine *generator_ptr = nullptr;

BOOST_AUTO_TEST_CASE(SimpleSet)
{
    EventSet s = EventSet({nextevt(), nextevt(), nextevt()});
    BOOST_CHECK((s * s).size() == 9);
    BOOST_CHECK(!s.subset(s));
    BOOST_CHECK(s.subseteq(s));
    BOOST_CHECK(s == (s * s).range());
}

BOOST_AUTO_TEST_CASE(CycleDetectionUnionNo)
{
    Event e1 = resetevt();
    Event e2, e3;

    EventRel er1;
    er1.insert(e3=e1, e2 = nextevt());
    er1.insert(e2, e1 = nextevt());
    er1.insert(e3, e1);

    BOOST_CHECK(!er1.props());
    BOOST_CHECK(er1.acyclic());
    BOOST_CHECK(!er1.props());

    EventRel er2;
    er2.insert(e1, e2 = nextevt());
    er2.insert(e1, e2 = nextevt());
    er2.insert(e1, e2 = nextevt());

    BOOST_CHECK(er1.transitive());
    BOOST_CHECK(er2.transitive());
    BOOST_CHECK((er1+er2).acyclic());
    er1 += er2;
    BOOST_CHECK_EQUAL(er1.size(), 6);

    er2.set_props(EventRel::TransitiveClosure);
    BOOST_CHECK((EventRel()+er2).acyclic());
}

BOOST_AUTO_TEST_CASE(CycleDetectionYes)
{
    Event e1 = resetevt();
    Event e2;
    Event e3;

    EventRel er;
    er.insert(e1, e2 = nextevt());
    er.insert(e3=e2, e1 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());

    EventRel er_;
    er_.insert(e1, e2 = nextevt());
    er_.insert(e2, e3);
    er_.insert(e2, nextevt());
    er_.insert(e2, nextevt());
    er_.insert(e2, nextevt());
    er_.insert(e2, nextevt());
    er += er_;

    BOOST_CHECK(!er.acyclic());

    er.set_props(EventRel::ReflexiveTransitiveClosure);
    BOOST_CHECK_EQUAL(er.size(), 43);
}

BOOST_AUTO_TEST_CASE(CycleDetectionYes2)
{
    Event e1 = resetevt();
    Event e2;
    Event e3;

    EventRel er;
    er.insert(e1, e2 = nextevt());
    er.insert(e3=e2, e1 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e2, e1 = nextevt());
    er.insert(e1, e3);

    for (int i=0; i<30; ++i) {
        // Catch out buggy cycle detection implementations, where they do not
        // consider a visisted node after being visited once before.
        //
        // As unordered maps are used as backing store, by adding more edges
        // that do not contribute to the cycle, we are likely to traverse these
        // first.
        er.insert(nextevt(), e3);
    }

    EventRel::Path p;
    BOOST_CHECK(!er.acyclic(&p));
    BOOST_CHECK(p[1] == p.back());
    BOOST_CHECK_EQUAL(p.size(), 6);

    p.clear();
    er.set_props(EventRel::TransitiveClosure);
    BOOST_CHECK(er.R(e3, e1, &p));
    BOOST_CHECK(p.front() == e3);
    BOOST_CHECK(p.back() == e1);
    BOOST_CHECK_EQUAL(p.size(), 4);
}

BOOST_AUTO_TEST_CASE(EventRelDiff)
{
    Event e1 = resetevt();
    Event e2;

    EventRel er;
    er.insert(e1, e2 = nextevt());
    er.insert(e2, e1 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());

    EventRel d;
    d.insert(e1, e2);
    d.insert(e1, nextevt());
    er -= d;
    BOOST_CHECK_EQUAL(er.size(), 4);

    d.set_props(EventRel::ReflexiveTransitiveClosure);
    auto evald = d.eval();
    BOOST_CHECK(d == evald);
    BOOST_CHECK(d.raw() != evald.raw());
    BOOST_CHECK_EQUAL(d.size(), 5);
}

BOOST_AUTO_TEST_CASE(EventRelDiffProps)
{
    Event e1 = resetevt();
    Event e2, start;

    EventRel er;
    er.insert(start=e1, e2 = nextevt());
    er.insert(e2, e1 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());

    EventRel d;
    d.insert(start, e2);
    d.insert(e2, e2);
    d.insert(e2, start);
    er.add_props(EventRel::TransitiveClosure);
    d -= er;
    BOOST_CHECK_EQUAL(d.size(), 2);

    er.add_props(EventRel::ReflexiveClosure);
    d -= er;
    BOOST_CHECK_EQUAL(d.size(), 1);
}

BOOST_AUTO_TEST_CASE(EventRelIntersect)
{
    Event e1 = resetevt();
    Event e2;

    EventRel er;
    er.insert(e1, e2 = nextevt());
    er.insert(e2, e1 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());

    EventRel d;
    d.insert(e1, e2);
    d.insert(e1, nextevt());
    d.insert(e1, nextevt());
    d &= er;
    BOOST_CHECK_EQUAL(d.size(), 1);
    BOOST_CHECK(d.domain().subset(er.domain()));
}

BOOST_AUTO_TEST_CASE(EventRelInverse)
{
    Event e1 = resetevt();
    Event e2;

    EventRel er;
    er.insert(e1, e2 = nextevt());
    er.insert(e2, e1 = nextevt());
    er.insert(e2, e1 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());

    EventRel inv = er.inverse();
    BOOST_CHECK(er.domain() == inv.range());
    BOOST_CHECK(er.range() == inv.domain());
    BOOST_CHECK_EQUAL(er.size(), inv.size());

    er.for_each([&inv](const Event& e1, const Event& e2) {
        BOOST_CHECK(inv.R(e2, e1));
        BOOST_CHECK(!inv.R(e1, e2));
    });

    er.set_props(EventRel::ReflexiveTransitiveClosure);
    inv = er.inverse();
    BOOST_CHECK(er.domain() == inv.range());
    BOOST_CHECK(er.range() == inv.domain());
    BOOST_CHECK_EQUAL(er.size(), inv.size());
}

BOOST_AUTO_TEST_CASE(EventRelSeqR)
{
    Event e1 = resetevt();
    Event e2;
    Event start, end;

    EventRelSeq ers;
    EventRel er;
    er.insert(start = e1, e2 = nextevt());
    ers += er;

    er = EventRel();
    er.insert(e2, e1 = nextevt());
    ers += er;

    er = EventRel();
    er.insert(e1, end = e2 = nextevt());
    er.insert(e2, e1 = nextevt());
    er.add_props(EventRel::TransitiveClosure);
    ers += er;

    // First unevald
    BOOST_CHECK(ers.R(start, end));
    BOOST_CHECK(ers.R(start, e1));
    BOOST_CHECK(ers.irreflexive());

    const EventRel evald = ers.eval();
    const EventRel evald_inplace = ers.eval_inplace();
    BOOST_CHECK(evald == evald_inplace);

    // Should be same result after eval_inplace
    BOOST_CHECK(ers.R(start, end));
    BOOST_CHECK(ers.R(start, e1));
    BOOST_CHECK(ers.irreflexive());

    // Check evald
    BOOST_CHECK(evald.R(start, end));
    BOOST_CHECK(evald.R(start, e1));
    BOOST_CHECK(evald.irreflexive());
}

BOOST_AUTO_TEST_CASE(EventRelSeqIrrefl1)
{
    Event e1 = resetevt();
    Event e2;
    Event start, end;

    EventRelSeq ers;
    EventRel er;
    er.insert(e1, nextevt());
    er.insert(e1, nextevt());
    er.insert(e1, nextevt());
    er.insert(start = e1, e2 = nextevt());
    ers += er;

    er = EventRel();
    er.insert(e2, e1 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.add_props(EventRel::TransitiveClosure);
    ers += er;

    er = EventRel();
    er.insert(e2, start);
    ers += er;

    BOOST_CHECK(!ers.irreflexive());

    const EventRel evald = ers.eval();
    BOOST_CHECK(!evald.irreflexive());

    EventRel::Path p;
    ers.irreflexive(&p);
    BOOST_CHECK_EQUAL(p.size(), 5);
}

BOOST_AUTO_TEST_CASE(EventRelSeqIrrefl2)
{
    Event e1 = resetevt();
    Event e2;
    Event start;

    EventRelSeq ers;
    EventRel er;
    er.insert(start = e1, e2 = nextevt());
    ers += er;

    er = EventRel();
    er.insert(e2, start);
    ers += er;

    er = EventRel();
    er.insert(start, nextevt());
    BOOST_CHECK(er.irreflexive());

    er.add_props(EventRel::ReflexiveClosure);
    BOOST_CHECK(!er.irreflexive());
    ers += er;

    BOOST_CHECK(!ers.irreflexive());

    const EventRel evald = ers.eval();
    BOOST_CHECK(!evald.irreflexive());
}

BOOST_AUTO_TEST_CASE(EventRelReflexivePath)
{
    Event e1 = resetevt();
    Event e2;

    EventRelSeq ers;

    EventRel er;
    er.insert(e1, e2 = nextevt());
    er.insert(e1, e2 = nextevt());
    er.set_props(EventRel::ReflexiveClosure);
    ers += er;

    er = EventRel();
    er.insert(e2, e1 = nextevt());
    er.set_props(EventRel::ReflexiveClosure);
    ers += er;

    EventRel::Path p;
    BOOST_CHECK(!er.irreflexive(&p));
    BOOST_CHECK_EQUAL(p.size(), 2);
    BOOST_CHECK(p[0] == p[1]);

    p.clear();
    BOOST_CHECK(!ers.irreflexive(&p));
    BOOST_CHECK_EQUAL(p.size(), 3);
    BOOST_CHECK(p[0] == p[1]);
    BOOST_CHECK(p[1] == p[2]);
}

BOOST_AUTO_TEST_CASE(Model12Empty)
{
    model12::ExecWitness ew;
    model12::Arch_SC sc;
    model12::Checker c(&sc, &ew);

    BOOST_CHECK_NO_THROW(c.wf());
    BOOST_CHECK(c.uniproc());
    BOOST_CHECK(c.thin());
    BOOST_CHECK(c.check_exec());
    BOOST_CHECK_NO_THROW(c.valid_exec());
}

BOOST_AUTO_TEST_CASE(Model14Empty)
{
    model14::ExecWitness ew;
    model14::Arch_SC sc;
    model14::Checker c(&sc, &ew);

    BOOST_CHECK_NO_THROW(c.wf());
    BOOST_CHECK(c.sc_per_location());
    BOOST_CHECK(c.no_thin_air());
    BOOST_CHECK(c.observation());
    BOOST_CHECK(c.propagation());
    BOOST_CHECK_NO_THROW(c.valid_exec());
}

BOOST_AUTO_TEST_CASE(Model12DekkerValidSC)
{
    model12::ExecWitness ew;
    model12::Arch_SC sc;
    model12::Checker c(&sc, &ew);

    Event Ix = Event(Event::Write, 10, Iiid(-1, 0));
    Event Iy = Event(Event::Write, 20, Iiid(-1, 1));

    Event Wx0 = Event(Event::Write, 10, Iiid(0, 12));
    Event Wy1 = Event(Event::Write, 20, Iiid(1, 33));
    Event Ry0 = Event(Event::Read, 20, Iiid(0, 55));
    Event Rx1 = Event(Event::Read, 10, Iiid(1, 22));

    ew.events += EventSet({Ix, Iy, Wx0, Wy1, Ry0, Rx1});

    ew.po.insert(Wx0, Ry0);
    ew.po.insert(Wy1, Rx1);

    ew.ws.insert(Ix, Wx0);
    ew.ws.insert(Iy, Wy1);

    ew.rf.insert(Wx0, Rx1);
    ew.rf.insert(Wy1, Ry0);

    BOOST_CHECK_NO_THROW(c.wf());
    BOOST_CHECK(c.uniproc());
    BOOST_CHECK(c.thin());
    BOOST_CHECK(c.check_exec());
    BOOST_CHECK_NO_THROW(c.valid_exec());
}

BOOST_AUTO_TEST_CASE(Model14DekkerInvalidSCValidTSO)
{
    model14::ExecWitness ew;
    model14::Arch_SC sc;
    model14::Arch_TSO tso;
    model14::Checker c_sc(&sc, &ew);
    model14::Checker c_tso(&tso, &ew);

    Event Ix = Event(Event::Write, 10, Iiid(-1, 0));
    Event Iy = Event(Event::Write, 20, Iiid(-1, 1));

    Event Wx0 = Event(Event::Write, 10, Iiid(0, 12));
    Event Wy1 = Event(Event::Write, 20, Iiid(1, 33));
    Event Ry0 = Event(Event::Read, 20, Iiid(0, 55));
    Event Rx1 = Event(Event::Read, 10, Iiid(1, 22));

    ew.events += EventSet({Ix, Iy, Wx0, Wy1, Ry0, Rx1});

    ew.po.insert(Wx0, Ry0);
    ew.po.insert(Wy1, Rx1);

    ew.co.insert(Ix, Wx0);
    ew.co.insert(Iy, Wy1);

    ew.rf.insert(Ix, Rx1);
    ew.rf.insert(Iy, Ry0);

    BOOST_CHECK_NO_THROW(c_sc.wf());
    BOOST_CHECK(c_sc.sc_per_location());
    BOOST_CHECK(c_sc.no_thin_air());
    BOOST_CHECK(c_sc.observation());
    BOOST_CHECK(!c_sc.propagation());
    BOOST_CHECK_EXCEPTION(c_sc.valid_exec(), model14::Checker::Error,
            [](const model14::Checker::Error& e) {
                return std::string("PROPAGATION") == e.what();
            });

    BOOST_CHECK_NO_THROW(c_tso.wf());
    BOOST_CHECK(c_tso.sc_per_location());
    BOOST_CHECK(c_tso.no_thin_air());
    BOOST_CHECK(c_tso.observation());
    BOOST_CHECK(c_tso.propagation());
    BOOST_CHECK_NO_THROW(c_tso.valid_exec());
}

class GenomeAdd : public Genome<float> {
  public:
    GenomeAdd()
    {
        genome_.resize(5);
        mutate(1.0f);
    }

    GenomeAdd(const GenomeAdd& parent1, const GenomeAdd& parent2,
              const std::vector<float>& g)
        : Genome<float>(g)
    {}

    void mutate(float rate)
    {
        std::uniform_int_distribution<std::size_t> dist_idx(0, genome_.size() - 1);
        std::uniform_real_distribution<float> dist_mut(-2.0f, 2.0f);
        std::unordered_set<std::size_t> used;
        std::size_t selection_count = (std::size_t)((float)genome_.size() * rate);

        while (selection_count) {
            auto idx = dist_idx(*generator_ptr);
            if (used.find(idx) != used.end())
                continue;

            genome_[idx] += dist_mut(*generator_ptr);

            used.insert(idx);
            --selection_count;
        }
    };

    float fitness() const
    {
        float total = 0.0;
        for (const auto& f : genome_) {
            total += f;
        }

        // restrict size
        if (genome_.size() > 10)
            return 999.0f;

        // want to get sum closest to 24.
        return (24 - total) * (24 - total);
    }

    bool operator<(const GenomeAdd& rhs) const
    {
        return fitness() < rhs.fitness();
    }

    operator float() const
    {
        return 1000.f - fitness();
    }
};

BOOST_AUTO_TEST_CASE(SimpleGAAdd24)
{
    std::default_random_engine generator(1234);
    generator_ptr = &generator;

    GenePool<GenomeAdd> pool(25, // population_size
                             0.3f // mutation_rate
                             );

    std::size_t tournament_size = 10;
    std::size_t tournament_winners = 5;
    std::size_t elite = tournament_winners;

    for (int i = 0; i<50; ++i) {
        auto tournament_population = pool.select_uniform(generator, tournament_size);
        pool.nextgen(generator, &tournament_population, tournament_winners, elite);
        BOOST_CHECK(pool.population_size() <= pool.target_population_size() + 1);
    }

    // This mainly checks that the discrete_distribution implementation works
    // as expected.
    BOOST_CHECK( GenePool<GenomeAdd>(pool.select_roulette(generator, tournament_size)).avg_fitness()
               < GenePool<GenomeAdd>(pool.select_uniform(generator, tournament_size)).avg_fitness());

    BOOST_CHECK( pool.best_fitness() < pool.worst_fitness() );

    auto gene = pool.select_best();
    float sum = 0.0f;
    for (const auto& f : gene.get()) {
        sum += f;
    }

    BOOST_CHECK(sum >= 23.1f && sum <= 24.9);
}

BOOST_AUTO_TEST_CASE(CodeGen_X86_64)
{
    std::default_random_engine urng(1235);

    model14::ExecWitness ew;
    model14::Arch_TSO arch;

    const types::Addr offset = 0x0;
    ops::RandomFactory factory(0, 1, offset + 0xccc0, offset + 0xccca);
    RandInstTest<std::default_random_engine, ops::RandomFactory> rit(urng, &factory, 150);

    const auto threads = rit.threads();
    BOOST_CHECK_EQUAL(threads.size(), 2);
    BOOST_CHECK_EQUAL(threads_size(threads), rit.get().size());

    Compiler<Backend_X86_64> compiler(&arch, &ew,  &threads);

    char code[1024];

    BOOST_CHECK(compiler.emit(0, 0xffff, code, sizeof(code)) != 0);

    std::size_t emit_len = compiler.emit(1, 0, code, sizeof(code));
    BOOST_CHECK(emit_len != 0);

#if 1
    mc::model14::Checker checker(&arch, &ew);
    ew.po.set_props(mc::EventRel::TransitiveClosure);
    ew.co.set_props(mc::EventRel::TransitiveClosure);

    types::WriteID wid = 0;
    // This test passing is dependent on the random number generator
    // implementation.
    BOOST_CHECK(compiler.update_from(0x3a, 0, 0xccc5, &wid, 1)); // write 0xccc5
    BOOST_CHECK(compiler.update_from(0xe6, 0, 0xccc5, &wid, 1)); // read  0xccc5
    BOOST_CHECK(!checker.sc_per_location());

    wid = 0x21; // check replacement/update works
    BOOST_CHECK(compiler.update_from(0xe6, 0, 0xccc5, &wid, 1)); // read  0xccc5
    BOOST_CHECK(checker.sc_per_location());

    // Check atomic works
    wid = 0;
    BOOST_CHECK(compiler.update_from(0x27f, 0, 0xccc7, &wid, 1));
    wid = 0x3c; // restart atomic
    BOOST_CHECK(compiler.update_from(0x27f, 0, 0xccc7, &wid, 1));
    BOOST_CHECK(compiler.update_from(0x27f, 1, 0xccc7, &wid, 1));
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
BOOST_AUTO_TEST_CASE(CodeGen_X86_64_ExecLinux)
{
    model14::ExecWitness ew;
    model14::Arch_TSO arch;

    Compiler<Backend_X86_64> compiler(&arch, &ew);

    unsigned char test_mem[] = {
        0x03, 0x14, 0x25, 0x36, 0x47, 0x58, 0x69, 0x7a, 0x8b, 0x9c,
        0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2
    };

    OperationPtr ops[] = {
        std::make_shared<ops::Read>(reinterpret_cast<types::Addr>(&test_mem[0xf])),
        std::make_shared<ops::Return>()
    };

    const std::size_t MAX_CODE_SIZE = 4096;
    char *code = (char*)mmap(NULL, MAX_CODE_SIZE,
                             PROT_READ | PROT_WRITE | PROT_EXEC,
                             MAP_ANONYMOUS | MAP_PRIVATE,
                             0, 0);
    memset(code, 0x90, MAX_CODE_SIZE);

    std::size_t emit_len = 0;
    for (auto& op : ops) {
        emit_len += compiler.emit(emit_len, op.get(), code + emit_len,
                                  MAX_CODE_SIZE - emit_len, nullptr);
    }

    unsigned char (*func)() = (unsigned char (*)()) code;
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

