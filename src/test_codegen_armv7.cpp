// This code is licensed under the BSD 3-Clause license. See the LICENSE file
// in the project root for license terms.

#include "mc2lib/codegen/ops/armv7.hpp"
#include "mc2lib/codegen/rit.hpp"
#include "mc2lib/memconsistency/cats.hpp"

#include <gtest/gtest.h>

using namespace mc2lib;
using namespace mc2lib::codegen;
using namespace mc2lib::memconsistency;

TEST(CodeGen, ARMv7_Short) {
  std::default_random_engine urng(42);

  cats::ExecWitness ew;
  cats::Arch_ARMv7 arch;

  armv7::RandomFactory factory(0, 1, 0xccc0, 0xccca);
  RandInstTest<std::default_random_engine, armv7::RandomFactory> rit(
      urng, &factory, 200);

  auto threads = [&rit]() {
    auto result = rit.threads();
    EXPECT_EQ(result.size(), 2);
    EXPECT_EQ(threads_size(result), rit.get().size());
    return result;
  };

  Compiler<armv7::Operation, armv7::Backend> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)), threads());

  constexpr std::size_t MAX_CODE_SIZE = 4096 / sizeof(std::uint16_t);
  std::uint16_t code[MAX_CODE_SIZE];

  std::size_t emit_len = compiler.Emit(0, 0xffff, code, sizeof(code));
  ASSERT_NE(emit_len, 0);
  emit_len = compiler.Emit(1, 0, code, sizeof(code));
  ASSERT_NE(emit_len, 0);

#ifdef OUTPUT_BIN_TO_TMP
  std::fill(code + (emit_len / sizeof(std::uint16_t)), code + MAX_CODE_SIZE,
            0xbf00);
  auto f = fopen("/tmp/mc2lib-armv7-test.bin", "wb");
  fwrite(code, sizeof(code), 1, f);
  fclose(f);
#endif
}

// Should be able to handle exhausting write-ids gracefully.
TEST(CodeGen, ARMv7_Exhaust) {
  std::default_random_engine urng(42);

  cats::ExecWitness ew;
  cats::Arch_ARMv7 arch;

  armv7::RandomFactory factory(0, 1, 0xbeef, 0xfeed);
  RandInstTest<std::default_random_engine, armv7::RandomFactory> rit(
      urng, &factory, 1234);

  Compiler<armv7::Operation, armv7::Backend> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)),
      rit.threads());

  constexpr std::size_t MAX_CODE_SIZE = 4096 * 2;
  char code[MAX_CODE_SIZE];

  ASSERT_NE(0, compiler.Emit(0, 0, code, sizeof(code)));
  ASSERT_NE(0, compiler.Emit(1, MAX_CODE_SIZE << 1, code, sizeof(code)));
}

TEST(CodeGen, ARMv7_SC_PER_LOCATION) {
  std::vector<codegen::armv7::Operation::Ptr> threads = {
      // p0
      std::make_shared<armv7::Write>(0xf0, 0),                     // @0x0a
      std::make_shared<armv7::Read>(0xf0, armv7::Backend::r1, 0),  // @0x14
      std::make_shared<armv7::Read>(0xf0, armv7::Backend::r2, 0),  // @0x1e
      std::make_shared<armv7::Read>(0xf1, armv7::Backend::r3, 0),  // @0x28
      std::make_shared<armv7::Read>(0xf1, armv7::Backend::r4, 0),  // @0x32

      // p1
      std::make_shared<armv7::Write>(0xf1, 1),
  };

  cats::ExecWitness ew;
  cats::Arch_ARMv7 arch;
  Compiler<armv7::Operation, armv7::Backend> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)),
      ExtractThreads(&threads));

  char code[128];

  ASSERT_NE(0, compiler.Emit(0, 0, code, sizeof(code)));
  ASSERT_NE(0, compiler.Emit(1, 0xffff, code, sizeof(code)));

  auto checker = arch.MakeChecker(&arch, &ew);
  ew.po.set_props(mc::EventRel::kTransitiveClosure);
  ew.co.set_props(mc::EventRel::kTransitiveClosure);

  types::WriteID wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0x0a, 0, 0xf0, &wid, 1));
  ASSERT_TRUE(compiler.UpdateObs(0x14, 0, 0xf0, &wid, 1));
  ASSERT_FALSE(checker->sc_per_location());

  // Check update works.
  wid = 1;
  ASSERT_TRUE(compiler.UpdateObs(0x14, 0, 0xf0, &wid, 1));
  ASSERT_TRUE(checker->sc_per_location());

  // Read-from external
  wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0xffff + 0x0a, 0, 0xf1, &wid, 1));
  wid = 2;
  ASSERT_TRUE(compiler.UpdateObs(0x28, 0, 0xf1, &wid, 1));
  wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0x32, 0, 0xf1, &wid, 1));
  ASSERT_FALSE(checker->sc_per_location());

  // update
  wid = 2;
  ASSERT_TRUE(compiler.UpdateObs(0x32, 0, 0xf1, &wid, 1));
  ASSERT_TRUE(checker->sc_per_location());

  ASSERT_TRUE(checker->no_thin_air());
  ASSERT_TRUE(checker->observation());
  ASSERT_TRUE(checker->propagation());
}

TEST(CodeGen, ARMv7_OBSERVATION) {
  std::vector<codegen::armv7::Operation::Ptr> threads = {
      // p0
      std::make_shared<armv7::Read>(0xf0, armv7::Backend::r1, 0),  // @0x08
      std::make_shared<armv7::Read>(0xf1, armv7::Backend::r3, 0),  // @0x12
      std::make_shared<armv7::ReadAddrDp>(0xf1, armv7::Backend::r2,
                                          armv7::Backend::r1, 0),  // @0x1e

      // p1
      std::make_shared<armv7::DMB_ST>(1),       // check boundary condition
      std::make_shared<armv7::Write>(0xf1, 1),  // 0x0e
      std::make_shared<armv7::Delay>(1, 1),     //
      std::make_shared<armv7::DMB_ST>(1),       //
      std::make_shared<armv7::Delay>(1, 1),     //
      std::make_shared<armv7::Write>(0xfa, 1),  // 0x20
      std::make_shared<armv7::Write>(0xf0, 1),  // 0x2c
      std::make_shared<armv7::DMB_ST>(1),       // check boundary condition
  };

  cats::ExecWitness ew;
  cats::Arch_ARMv7 arch;
  Compiler<armv7::Operation, armv7::Backend> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)),
      ExtractThreads(&threads));

  char code[128];

  ASSERT_NE(0, compiler.Emit(0, 0, code, sizeof(code)));
  ASSERT_NE(0, compiler.Emit(1, 0xffff, code, sizeof(code)));

  auto checker = arch.MakeChecker(&arch, &ew);
  ew.po.set_props(mc::EventRel::kTransitiveClosure);
  ew.co.set_props(mc::EventRel::kTransitiveClosure);

  types::WriteID wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0xffff + 0x0e, 0, 0xf1, &wid, 1));
  ASSERT_TRUE(compiler.UpdateObs(0xffff + 0x22, 0, 0xfa, &wid, 1));
  ASSERT_TRUE(compiler.UpdateObs(0xffff + 0x2e, 0, 0xf0, &wid, 1));

  wid = 3;
  ASSERT_TRUE(compiler.UpdateObs(0x08, 0, 0xf0, &wid, 1));
  wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0x12, 0, 0xf1, &wid, 1));
  ASSERT_TRUE(compiler.UpdateObs(0x1e, 0, 0xf1, &wid, 1));
  ASSERT_FALSE(checker->observation());

  wid = 1;
  ASSERT_TRUE(compiler.UpdateObs(0x1e, 0, 0xf1, &wid, 1));
  ASSERT_TRUE(checker->observation());

  ASSERT_TRUE(checker->no_thin_air());
  ASSERT_TRUE(checker->sc_per_location());
  ASSERT_TRUE(checker->propagation());
}
