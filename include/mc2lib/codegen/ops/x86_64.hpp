/*
 * Copyright (c) 2014-2016, Marco Elver
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

#ifndef MC2LIB_CODEGEN_OPS_X86_64_HPP_
#define MC2LIB_CODEGEN_OPS_X86_64_HPP_

#include <cstdint>
#include <stdexcept>

#include "strong.hpp"

namespace mc2lib {
namespace codegen {
namespace strong {

struct Backend_X86_64 : Backend {
  std::size_t Return(void *code, std::size_t len) const override;

  std::size_t Delay(std::size_t length, void *code,
                    std::size_t len) const override;

  std::size_t Read(types::Addr addr, types::InstPtr start, void *code,
                   std::size_t len, types::InstPtr *at) const override;

  std::size_t ReadAddrDp(types::Addr addr, types::InstPtr start, void *code,
                         std::size_t len, types::InstPtr *at) const override;

  std::size_t Write(types::Addr addr, types::WriteID write_id,
                    types::InstPtr start, void *code, std::size_t len,
                    types::InstPtr *at) const override;

  std::size_t ReadModifyWrite(types::Addr addr, types::WriteID write_id,
                              types::InstPtr start, void *code, std::size_t len,
                              types::InstPtr *at) const override;

  std::size_t CacheFlush(types::Addr addr, void *code,
                         std::size_t len) const override;
};

inline std::size_t Backend_X86_64::Return(void *code, std::size_t len) const {
  assert(len >= 1);
  // ASM> retq ;
  *static_cast<char *>(code) = 0xc3;
  return 1;
}

inline std::size_t Backend_X86_64::Delay(std::size_t length, void *code,
                                         std::size_t len) const {
  char *cnext = static_cast<char *>(code);

  assert(len >= length);
  for (std::size_t i = 0; i < length; ++i) {
    // ASM> nop ;
    *cnext++ = 0x90;
  }

  assert((cnext - static_cast<char *>(code)) ==
         static_cast<std::ptrdiff_t>(length));
  return length;
}

inline std::size_t Backend_X86_64::Read(types::Addr addr, types::InstPtr start,
                                        void *code, std::size_t len,
                                        types::InstPtr *at) const {
  char *cnext = static_cast<char *>(code);
  std::size_t expected_len = 0;

  if (addr <= static_cast<types::Addr>(0xffffffff)) {
    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @0> movzbl addr, %eax ;
        expected_len = 8;
        assert(len >= expected_len);
        *at = start;

        // @0
        *cnext++ = 0x0f;
        *cnext++ = 0xb6;
        *cnext++ = 0x04;
        *cnext++ = 0x25;
        *reinterpret_cast<std::uint32_t *>(cnext) =
            static_cast<std::uint32_t>(addr);
        cnext += sizeof(std::uint32_t);
        break;

      default:
        throw std::logic_error("Not supported");
    }
  } else {
    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @0> movabs addr, %al ;
        expected_len = 9;
        assert(len >= expected_len);
        *at = start;

        // @0
        *cnext++ = 0xa0;
        break;

      case 2:
        // ASM @0> movabs addr, %ax ;
        expected_len = 10;
        assert(len >= expected_len);
        *at = start;

        // @0
        *cnext++ = 0x66;
        *cnext++ = 0xa1;
        break;

      default:
        throw std::logic_error("Not supported");
    }

    *reinterpret_cast<std::uint64_t *>(cnext) =
        static_cast<std::uint64_t>(addr);
    cnext += sizeof(std::uint64_t);
  }

  assert((cnext - static_cast<char *>(code)) ==
         static_cast<std::ptrdiff_t>(expected_len));
  return expected_len;
}

inline std::size_t Backend_X86_64::ReadAddrDp(types::Addr addr,
                                              types::InstPtr start, void *code,
                                              std::size_t len,
                                              types::InstPtr *at) const {
  char *cnext = static_cast<char *>(code);
  std::size_t expected_len = 0;

  // ASM @0> xor %rax, %rax
  expected_len = 3;
  assert(len >= expected_len);

  // @0
  *cnext++ = 0x48;
  *cnext++ = 0x31;
  *cnext++ = 0xc0;

  if (addr <= static_cast<types::Addr>(0xffffffff)) {
    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @3> movzbl addr(%rax), %eax ;
        expected_len = 10;
        assert(len >= expected_len);
        *at = start + 3;

        // @3
        *cnext++ = 0x0f;
        *cnext++ = 0xb6;
        *cnext++ = 0x80;
        *reinterpret_cast<std::uint32_t *>(cnext) =
            static_cast<std::uint32_t>(addr);
        cnext += sizeof(std::uint32_t);
        break;

      default:
        throw std::logic_error("Not supported");
    }
  } else {
    // ASM @03> movabs addr, %rdx ;
    //     @0d> add %rdx, %rax ;
    expected_len = 19;
    assert(len >= expected_len);
    *at = start + 0x10;

    // @03
    *cnext++ = 0x48;
    *cnext++ = 0xba;
    *reinterpret_cast<std::uint64_t *>(cnext) =
        static_cast<std::uint64_t>(addr);
    cnext += sizeof(std::uint64_t);

    // @0d
    *cnext++ = 0x48;
    *cnext++ = 0x01;
    *cnext++ = 0xd0;

    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @10> movzbl (%rax), %eax ;
        // @10
        *cnext++ = 0x0f;
        *cnext++ = 0xb6;
        *cnext++ = 0x00;
        break;

      case 2:
        // ASM @10> movzwl (%rax), %eax ;
        // @10
        *cnext++ = 0x0f;
        *cnext++ = 0xb7;
        *cnext++ = 0x00;
        break;

      default:
        throw std::logic_error("Not supported");
    }
  }

  assert((cnext - static_cast<char *>(code)) ==
         static_cast<std::ptrdiff_t>(expected_len));
  return expected_len;
}

inline std::size_t Backend_X86_64::Write(types::Addr addr,
                                         types::WriteID write_id,
                                         types::InstPtr start, void *code,
                                         std::size_t len,
                                         types::InstPtr *at) const {
  char *cnext = static_cast<char *>(code);
  std::size_t expected_len = 0;

  assert(write_id != 0);

  if (addr <= static_cast<types::Addr>(0xffffffff)) {
    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @0> movb write_id, addr ;
        expected_len = 8;
        assert(len >= expected_len);
        *at = start;

        // @0
        *cnext++ = 0xc6;
        *cnext++ = 0x04;
        *cnext++ = 0x25;

        *reinterpret_cast<std::uint32_t *>(cnext) =
            static_cast<std::uint32_t>(addr);
        cnext += sizeof(std::uint32_t);

        *reinterpret_cast<types::WriteID *>(cnext) = write_id;
        cnext += sizeof(types::WriteID);
        break;

      default:
        throw std::logic_error("Not supported");
    }
  } else {
    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @0> movabs addr, %rax    ;
        //     @a> movb write_id, (%rax) ;
        expected_len = 13;
        assert(len >= expected_len);
        *at = start + 0xa;

        // @0
        *cnext++ = 0x48;
        *cnext++ = 0xb8;
        *reinterpret_cast<std::uint64_t *>(cnext) =
            static_cast<std::uint64_t>(addr);
        cnext += sizeof(std::uint64_t);

        // @a
        *cnext++ = 0xc6;
        *cnext++ = 0x00;
        *reinterpret_cast<types::WriteID *>(cnext) = write_id;
        cnext += sizeof(types::WriteID);
        break;

      case 2:
        // ASM @0> movabs addr, %rax ;
        //     @a> mov write_id, %edx ;
        //     @f> mov %dx, (%rax) ;
        expected_len = 18;
        assert(len >= expected_len);
        *at = start + 0xf;

        // @0
        *cnext++ = 0x48;
        *cnext++ = 0xb8;
        *reinterpret_cast<std::uint64_t *>(cnext) =
            static_cast<std::uint64_t>(addr);
        cnext += sizeof(std::uint64_t);

        // @a
        *cnext++ = 0xba;
        *reinterpret_cast<std::uint32_t *>(cnext) = write_id;
        cnext += sizeof(std::uint32_t);

        // @f
        *cnext++ = 0x66;
        *cnext++ = 0x89;
        *cnext++ = 0x10;
        break;

      default:
        throw std::logic_error("Not supported");
    }
  }

  assert((cnext - static_cast<char *>(code)) ==
         static_cast<std::ptrdiff_t>(expected_len));
  return expected_len;
}

inline std::size_t Backend_X86_64::ReadModifyWrite(types::Addr addr,
                                                   types::WriteID write_id,
                                                   types::InstPtr start,
                                                   void *code, std::size_t len,
                                                   types::InstPtr *at) const {
  char *cnext = static_cast<char *>(code);
  std::size_t expected_len = 0;

  assert(write_id != 0);

  switch (sizeof(types::WriteID)) {
    case 1:
      // ASM @0> mov write_id, %al
      expected_len = 2;
      assert(len >= expected_len);

      // @0
      *cnext++ = 0xb0;
      *reinterpret_cast<types::WriteID *>(cnext) = write_id;
      cnext += sizeof(types::WriteID);
      break;

    case 2:
      // ASM @0> mov write_id, %eax
      expected_len = 5;
      assert(len >= expected_len);

      // @0
      *cnext++ = 0xb8;
      *reinterpret_cast<std::uint32_t *>(cnext) = write_id;
      cnext += sizeof(std::uint32_t);
      break;

    default:
      throw std::logic_error("Not supported");
  }

  if (addr <= static_cast<types::Addr>(0xffffffff)) {
    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @2> mov addr, %edx
        //     @7> lock xchg %al, (%rdx)
        expected_len = 10;
        assert(len >= expected_len);
        *at = start + 0x7;

        // @2
        *cnext++ = 0xba;
        *reinterpret_cast<std::uint32_t *>(cnext) =
            static_cast<std::uint32_t>(addr);
        cnext += sizeof(std::uint32_t);

        // @7
        *cnext++ = 0xf0;
        *cnext++ = 0x86;
        *cnext++ = 0x02;
        break;

      default:
        throw std::logic_error("Not supported");
    }
  } else {
    switch (sizeof(types::WriteID)) {
      case 1:
        // ASM @2> movabs addr, %rdx ;
        //     @c> lock xchg %al, (%rdx) ;
        expected_len = 15;
        assert(len >= expected_len);
        *at = start + 0xc;

        // @2
        *cnext++ = 0x48;
        *cnext++ = 0xba;
        *reinterpret_cast<std::uint64_t *>(cnext) =
            static_cast<std::uint64_t>(addr);
        cnext += sizeof(std::uint64_t);

        // @c
        *cnext++ = 0xf0;
        *cnext++ = 0x86;
        *cnext++ = 0x02;
        break;

      case 2:
        // ASM @5> movabs addr, %rdx ;
        //     @f> lock xchg %ax, (%rdx) ;
        expected_len = 19;
        assert(len >= expected_len);
        *at = start + 0xf;

        // @2
        *cnext++ = 0x48;
        *cnext++ = 0xba;
        *reinterpret_cast<std::uint64_t *>(cnext) =
            static_cast<std::uint64_t>(addr);
        cnext += sizeof(std::uint64_t);

        // @c
        *cnext++ = 0x66;
        *cnext++ = 0xf0;
        *cnext++ = 0x87;
        *cnext++ = 0x02;
        break;

      default:
        throw std::logic_error("Not supported");
    }
  }

  assert((cnext - static_cast<char *>(code)) ==
         static_cast<std::ptrdiff_t>(expected_len));
  return expected_len;
}

inline std::size_t Backend_X86_64::CacheFlush(types::Addr addr, void *code,
                                              std::size_t len) const {
  char *cnext = static_cast<char *>(code);
  std::size_t expected_len = 0;

  if (addr <= static_cast<types::Addr>(0xffffffff)) {
    // ASM @0> clflush addr ;
    expected_len = 8;
    assert(len >= expected_len);

    // @0
    *cnext++ = 0x0f;
    *cnext++ = 0xae;
    *cnext++ = 0x3c;
    *cnext++ = 0x25;
    *reinterpret_cast<std::uint32_t *>(cnext) =
        static_cast<std::uint32_t>(addr);
    cnext += sizeof(std::uint32_t);
  } else {
    // ASM @0> mov addr, %rdx ;
    //     @a> clflush (%rdx) ;
    expected_len = 13;
    assert(len >= expected_len);

    // @0
    *cnext++ = 0x48;
    *cnext++ = 0xba;
    *reinterpret_cast<std::uint64_t *>(cnext) =
        static_cast<std::uint64_t>(addr);
    cnext += sizeof(std::uint64_t);

    // @a
    *cnext++ = 0x0f;
    *cnext++ = 0xae;
    *cnext++ = 0x3a;
  }

  assert((cnext - static_cast<char *>(code)) ==
         static_cast<std::ptrdiff_t>(expected_len));
  return expected_len;
}

}  // namespace strong
}  // namespace codegen
}  // namespace mc2lib

#endif /* MC2LIB_CODEGEN_OPS_X86_64_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
