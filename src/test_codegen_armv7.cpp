#include "mc2lib/codegen/ops/armv7.hpp"
#include "mc2lib/codegen/rit.hpp"
#include "mc2lib/memconsistency/cats.hpp"

#include <gtest/gtest.h>

using namespace mc2lib;
using namespace mc2lib::codegen;
using namespace mc2lib::memconsistency;

TEST(CodeGen, ARMv7) {
  std::default_random_engine urng(1238);

  cats::ExecWitness ew;
  cats::Arch_ARMv7 arch;

  const types::Addr offset = 0x0;
  armv7::RandomFactory factory(0, 1, offset + 0xccc0, offset + 0xccc5);
  RandInstTest<std::default_random_engine, armv7::RandomFactory> rit(
      urng, &factory, 150);

  auto threads = [&rit]() {
    auto result = rit.threads();
    EXPECT_EQ(result.size(), 2);
    EXPECT_EQ(threads_size(result), rit.Get().size());
    return result;
  };

  Compiler<armv7::Operation, armv7::Backend> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)), threads());

#define MAX_CODE_SIZE (1024 / sizeof(std::uint16_t))
  std::uint16_t code0[MAX_CODE_SIZE];
  std::uint16_t code1[MAX_CODE_SIZE];

  std::size_t emit_len = compiler.Emit(0, 0xffff, code0, sizeof(code0));
  ASSERT_NE(emit_len, 0);
  emit_len = compiler.Emit(1, 0, code1, sizeof(code1));
  ASSERT_NE(emit_len, 0);

#if 1
  auto checker = arch.MakeChecker(&arch, &ew);
  ew.po.set_props(mc::EventRel::kTransitiveClosure);
  ew.co.set_props(mc::EventRel::kTransitiveClosure);

  types::WriteID wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0x2a, 0, 0xccc3, &wid, 1));  // write 0xccc3
  ASSERT_TRUE(compiler.UpdateObs(0x68, 0, 0xccc3, &wid, 1));  // read  0xccc3
  ASSERT_FALSE(checker->sc_per_location());

  wid = 0x29;  // check replacement/update works
  ASSERT_TRUE(compiler.UpdateObs(0x68, 0, 0xccc3, &wid, 1));  // read  0xccc3
  ASSERT_TRUE(checker->sc_per_location());
#endif

#ifdef OUTPUT_BIN_TO_TMP
  std::fill(code1 + (emit_len / sizeof(std::uint16_t)), code1 + MAX_CODE_SIZE,
            0xbf00);
  auto f = fopen("/tmp/mc2lib-armv7-test.bin", "wb");
  fwrite(code1, sizeof(code1), 1, f);
  fclose(f);
#endif
}
