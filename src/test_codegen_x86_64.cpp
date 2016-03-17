#include "mc2lib/codegen/ops/x86_64.hpp"
#include "mc2lib/codegen/rit.hpp"
#include "mc2lib/memconsistency/cats.hpp"

#include <gtest/gtest.h>

using namespace mc2lib;
using namespace mc2lib::codegen;
using namespace mc2lib::memconsistency;

TEST(CodeGen, X86_64) {
  std::default_random_engine urng(1238);

  cats::ExecWitness ew;
  cats::Arch_TSO arch;

  strong::RandomFactory factory(0, 1, 0xccc0, 0xccca);
  RandInstTest<std::default_random_engine, strong::RandomFactory> rit(
      urng, &factory, 150);

  auto threads = [&rit]() {
    auto result = rit.threads();
    EXPECT_EQ(result.size(), 2);
    EXPECT_EQ(threads_size(result), rit.Get().size());
    return result;
  };

  Compiler<strong::Operation, strong::Backend_X86_64> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)), threads());

  char code[1024];

  std::size_t emit_len = compiler.Emit(0, 0xffff, code, sizeof(code));
  ASSERT_NE(emit_len, 0);
  ASSERT_TRUE(compiler.IpToOp(0xffff - 1) == nullptr);
  ASSERT_TRUE(compiler.IpToOp(0xffff + emit_len) == nullptr);
  ASSERT_TRUE(compiler.IpToOp(0x1234) == nullptr);
  ASSERT_TRUE(compiler.IpToOp(0xffff) != nullptr);
  ASSERT_TRUE(compiler.IpToOp(0xffff + emit_len - 1) != nullptr);

  emit_len = compiler.Emit(1, 0, code, sizeof(code));
  ASSERT_NE(emit_len, 0);

#ifdef OUTPUT_BIN_TO_TMP
  memset(code + emit_len, 0x90, sizeof(code) - emit_len);
  auto f = fopen("/tmp/mc2lib-test1.bin", "wb");
  fwrite(code, sizeof(code), 1, f);
  fclose(f);
#endif
}

TEST(CodeGen, X86_64_SC_PER_LOCATION) {
  std::vector<codegen::strong::Operation::Ptr> threads = {
      // p0
      std::make_shared<strong::Write>(0xf0, 0), // @0x0
      std::make_shared<strong::Read>(0xf0, 0),  // @0x8
      std::make_shared<strong::ReadModifyWrite>(0xf1, 0), // 0x17

      // p1
      std::make_shared<strong::Write>(0xf1, 1), // 0x0
  };

  cats::ExecWitness ew;
  cats::Arch_TSO arch;
  Compiler<strong::Operation, strong::Backend_X86_64> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)),
      ExtractThreads(&threads));

  char* code[128];

  ASSERT_NE(0, compiler.Emit(0, 0, code, sizeof(code)));
  ASSERT_NE(0, compiler.Emit(1, 0xffff, code, sizeof(code)));

  auto checker = arch.MakeChecker(&arch, &ew);
  ew.po.set_props(mc::EventRel::kTransitiveClosure);
  ew.co.set_props(mc::EventRel::kTransitiveClosure);

  types::WriteID wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0x0, 0, 0xf0, &wid, 1));
  ASSERT_TRUE(compiler.UpdateObs(0x8, 0, 0xf0, &wid, 1));
  ASSERT_FALSE(checker->sc_per_location());

  wid = 0x1;  // check replacement/update works
  ASSERT_TRUE(compiler.UpdateObs(0x8, 0, 0xf0, &wid, 1));
  ASSERT_TRUE(checker->sc_per_location());

  // Check atomic works
  wid = 0;
  ASSERT_TRUE(compiler.UpdateObs(0xffff+0x0, 0, 0xf1, &wid, 1));
  ASSERT_TRUE(compiler.UpdateObs(0x17, 0, 0xf1, &wid, 1));

  try {
    wid = 3;
    compiler.UpdateObs(0x17, 1, 0xf1, &wid, 1);
    FAIL();
  } catch (const Error& e) {
    ASSERT_NE(std::string(e.what()).find("NOT ATOMIC"), std::string::npos);
  }

  wid = 3;  // restart atomic
  ASSERT_TRUE(compiler.UpdateObs(0x17, 0, 0xf1, &wid, 1));
  ASSERT_TRUE(compiler.UpdateObs(0x17, 1, 0xf1, &wid, 1));

  ASSERT_TRUE(checker->sc_per_location());
  ASSERT_TRUE(checker->no_thin_air());
  ASSERT_TRUE(checker->observation());
  ASSERT_TRUE(checker->propagation());
}

#if defined(__linux__) && defined(__x86_64__)
#include <sys/mman.h>
TEST(CodeGen, X86_64_ExecLinux) {
  cats::ExecWitness ew;
  cats::Arch_TSO arch;

  Compiler<strong::Operation, strong::Backend_X86_64> compiler(
      std::unique_ptr<EvtStateCats>(new EvtStateCats(&ew, &arch)));

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

  ASSERT_EQ(result, test_mem[0xf]);

#ifdef OUTPUT_BIN_TO_TMP
  auto f = fopen("/tmp/mc2lib-test2.bin", "wb");
  fwrite(code, MAX_CODE_SIZE, 1, f);
  fclose(f);
#endif

  munmap(code, MAX_CODE_SIZE);
}
#endif
