#include "mc2lib/memconsistency/cats.hpp"
#include "mc2lib/codegen/ops/x86_64.hpp"
#include "mc2lib/codegen/rit.hpp"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace mc2lib;
using namespace mc2lib::codegen;
using namespace mc2lib::memconsistency;

BOOST_AUTO_TEST_SUITE(codegen)

BOOST_AUTO_TEST_CASE(CodeGen_X86_64) {
  std::default_random_engine urng(1238);

  cats::ExecWitness ew;
  cats::Arch_TSO arch;

  const types::Addr offset = 0x0;
  strong::RandomFactory factory(0, 1, offset + 0xccc0, offset + 0xccca);
  RandInstTest<std::default_random_engine, strong::RandomFactory> rit(
      urng, &factory, 150);

  auto threads = [&rit]() {
    auto result = rit.threads();
    BOOST_CHECK_EQUAL(result.size(), 2);
    BOOST_CHECK_EQUAL(threads_size(result), rit.Get().size());
    return result;
  };

  Compiler<strong::Operation, strong::Backend_X86_64> compiler(
      std::unique_ptr<AsmStateCats>(new AsmStateCats(&ew, &arch)), threads());

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

#ifdef OUTPUT_BIN_TO_TMP
  memset(code + emit_len, 0x90, sizeof(code) - emit_len);
  auto f = fopen("/tmp/mc2lib-test1.bin", "wb");
  fwrite(code, sizeof(code), 1, f);
  fclose(f);
#endif
}

#if defined(__linux__) && defined(__x86_64__)
#include <sys/mman.h>
BOOST_AUTO_TEST_CASE(CodeGen_X86_64_ExecLinux) {
  cats::ExecWitness ew;
  cats::Arch_TSO arch;

  Compiler<strong::Operation, strong::Backend_X86_64> compiler(
      std::unique_ptr<AsmStateCats>(new AsmStateCats(&ew, &arch)));

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

#ifdef OUTPUT_BIN_TO_TMP
  auto f = fopen("/tmp/mc2lib-test2.bin", "wb");
  fwrite(code, MAX_CODE_SIZE, 1, f);
  fclose(f);
#endif

  munmap(code, MAX_CODE_SIZE);
}
#endif

BOOST_AUTO_TEST_SUITE_END()
