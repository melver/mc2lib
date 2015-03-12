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

#ifndef MC2LIB_CODEGEN_OPS_X86_64_HPP_
#define MC2LIB_CODEGEN_OPS_X86_64_HPP_

#include "generic.hpp"

#include <cstdint>

namespace mc2lib {
namespace codegen {
namespace ops {

inline std::size_t
Return::emit_X86_64(types::InstPtr start, AssemblerState *asms,
                    void *code, std::size_t len)
{
    assert(len >= 1);
    // ASM> retq ;
    *static_cast<char*>(code) = 0xc3;
    return 1;
}

inline std::size_t
Delay::emit_X86_64(types::InstPtr start, AssemblerState *asms,
                   void *code, std::size_t len)
{
    char *cnext = static_cast<char*>(code);

    assert(len >= length_);
    for (std::size_t i = 0; i < length_; ++i) {
        // ASM> nop ;
        *cnext++ = 0x90;
    }

    assert((cnext - static_cast<char*>(code)) ==
            static_cast<std::ptrdiff_t>(length_));
    return length_;
}

inline std::size_t
Read::emit_X86_64(types::InstPtr start, AssemblerState *asms,
                  void *code, std::size_t len)
{
    char *cnext = static_cast<char*>(code);
    std::size_t expected_len = 0;

    if (addr_ <= static_cast<types::Addr>(0xffffffff)) {
        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @0> movzbl addr_, %eax ;
                expected_len = 8; assert(len >= expected_len);
                at_ = start;

                // @0
                *cnext++ = 0x0f; *cnext++ = 0xb6;
                *cnext++ = 0x04; *cnext++ = 0x25;
                *reinterpret_cast<std::uint32_t*>(cnext) = static_cast<std::uint32_t>(addr_);
                cnext += sizeof(std::uint32_t);
                break;

            default:
                assert(false);
        }
    } else {
        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @0> movabs addr_, %al ;
                expected_len = 9; assert(len >= expected_len);
                at_ = start;

                // @0
                *cnext++ = 0xa0;
                break;

            case 2:
                // ASM @0> movabs addr_, %ax ;
                expected_len = 10; assert(len >= expected_len);
                at_ = start;

                // @0
                *cnext++ = 0x66; *cnext++ = 0xa1;
                break;

            default:
                assert(false);
        }

        *reinterpret_cast<std::uint64_t*>(cnext) = static_cast<std::uint64_t>(addr_);
        cnext += sizeof(std::uint64_t);
    }

    assert((cnext - static_cast<char*>(code)) ==
            static_cast<std::ptrdiff_t>(expected_len));
    return expected_len;
}

inline std::size_t
ReadAddrDp::emit_X86_64(types::InstPtr start, AssemblerState *asms,
                        void *code, std::size_t len)
{
    char *cnext = static_cast<char*>(code);
    std::size_t expected_len = 0;

    // ASM @0> xor %rax, %rax
    expected_len = 3; assert(len >= expected_len);

    // @0
    *cnext++ = 0x48; *cnext++ = 0x31; *cnext++ = 0xc0;

    if (addr_ <= static_cast<types::Addr>(0xffffffff)) {
        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @3> movzbl addr_(%rax), %eax ;
                expected_len = 10; assert(len >= expected_len);
                at_ = start + 3;

                // @3
                *cnext++ = 0x0f; *cnext++ = 0xb6;
                *cnext++ = 0x80;
                *reinterpret_cast<std::uint32_t*>(cnext) = static_cast<std::uint32_t>(addr_);
                cnext += sizeof(std::uint32_t);
                break;

            default:
                assert(false);
        }
    } else {
        // ASM @03> movabs addr_, %rdx ;
        //     @0d> add %rdx, %rax ;
        expected_len = 19; assert(len >= expected_len);
        at_ = start + 0x10;

        // @03
        *cnext++ = 0x48; *cnext++ = 0xba;
        *reinterpret_cast<std::uint64_t*>(cnext) = static_cast<std::uint64_t>(addr_);
        cnext += sizeof(std::uint64_t);

        // @0d
        *cnext++ = 0x48; *cnext++ = 0x01; *cnext++ = 0xd0;

        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @10> movzbl (%rax), %eax ;
                // @10
                *cnext++ = 0x0f; *cnext++ = 0xb6; *cnext++ = 0x00;
                break;

            case 2:
                // ASM @10> movzwl (%rax), %eax ;
                // @10
                *cnext++ = 0x0f; *cnext++ = 0xb7; *cnext++ = 0x00;
                break;

            default:
                assert(false);
        }
    }

    assert((cnext - static_cast<char*>(code)) ==
            static_cast<std::ptrdiff_t>(expected_len));
    return expected_len;
}

inline std::size_t
Write::emit_X86_64(types::InstPtr start, AssemblerState *asms,
                   void *code, std::size_t len)
{
    char *cnext = static_cast<char*>(code);
    std::size_t expected_len = 0;

    assert(write_id_ != 0);

    if (addr_ <= static_cast<types::Addr>(0xffffffff)) {
        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @0> movb write_id_, addr_ ;
                expected_len = 8; assert(len >= expected_len);
                at_ = start;

                // @0
                *cnext++ = 0xc6; *cnext++ = 0x04;
                *cnext++ = 0x25;

                *reinterpret_cast<std::uint32_t*>(cnext) = static_cast<std::uint32_t>(addr_);
                cnext += sizeof(std::uint32_t);

                *reinterpret_cast<types::WriteID*>(cnext) = write_id_;
                cnext += sizeof(types::WriteID);
                break;

            default:
                assert(false);
        }
    } else {
        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @0> movabs addr_, %rax    ;
                //     @a> movb write_id_, (%rax) ;
                expected_len = 13; assert(len >= expected_len);
                at_ = start + 0xa;

                // @0
                *cnext++ = 0x48; *cnext++ = 0xb8;
                *reinterpret_cast<std::uint64_t*>(cnext) = static_cast<std::uint64_t>(addr_);
                cnext += sizeof(std::uint64_t);

                // @a
                *cnext++ = 0xc6; *cnext++ = 0x00;
                *reinterpret_cast<types::WriteID*>(cnext) = write_id_;
                cnext += sizeof(types::WriteID);
                break;

            case 2:
                // ASM @0> movabs addr_, %rax ;
                //     @a> mov write_id_, %edx ;
                //     @f> mov %dx, (%rax) ;
                expected_len = 18; assert(len >= expected_len);
                at_ = start + 0xf;

                // @0
                *cnext++ = 0x48; *cnext++ = 0xb8;
                *reinterpret_cast<std::uint64_t*>(cnext) = static_cast<std::uint64_t>(addr_);
                cnext += sizeof(std::uint64_t);

                // @a
                *cnext++ = 0xba;
                *reinterpret_cast<std::uint32_t*>(cnext) = write_id_;
                cnext += sizeof(std::uint32_t);

                // @f
                *cnext++ = 0x66; *cnext++ = 0x89; *cnext++ = 0x10;
                break;

            default:
                assert(false);
        }
    }

    assert((cnext - static_cast<char*>(code)) ==
            static_cast<std::ptrdiff_t>(expected_len));
    return expected_len;
}

inline std::size_t
ReadModifyWrite::emit_X86_64(types::InstPtr start, AssemblerState *asms,
                             void *code, std::size_t len)
{
    char *cnext = static_cast<char*>(code);
    std::size_t expected_len = 0;

    assert(write_id_ != 0);

    switch (sizeof(types::WriteID)) {
        case 1:
            // ASM @0> mov write_id_, %al
            expected_len = 2; assert(len >= expected_len);

            // @0
            *cnext++ = 0xb0;
            *reinterpret_cast<types::WriteID*>(cnext) = write_id_;
            cnext += sizeof(types::WriteID);
            break;

        case 2:
            // ASM @0> mov write_id_, %eax
            expected_len = 5; assert(len >= expected_len);

            // @0
            *cnext++ = 0xb8;
            *reinterpret_cast<std::uint32_t*>(cnext) = write_id_;
            cnext += sizeof(std::uint32_t);
            break;

        default:
            assert(false);
    }

    if (addr_ <= static_cast<types::Addr>(0xffffffff)) {
        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @2> mov addr_, %edx
                //     @7> lock xchg %al, (%rdx)
                expected_len = 10; assert(len >= expected_len);
                at_ = start + 0x7;

                // @2
                *cnext++ = 0xba;
                *reinterpret_cast<std::uint32_t*>(cnext) = static_cast<std::uint32_t>(addr_);
                cnext += sizeof(std::uint32_t);

                // @7
                *cnext++ = 0xf0; *cnext++ = 0x86; *cnext++ = 0x02;
                break;

            default:
                assert(false);
        }
    } else {
        switch (sizeof(types::WriteID)) {
            case 1:
                // ASM @2> movabs addr_, %rdx ;
                //     @c> lock xchg %al, (%rdx) ;
                expected_len = 15; assert(len >= expected_len);
                at_ = start + 0xc;

                // @2
                *cnext++ = 0x48; *cnext++ = 0xba;
                *reinterpret_cast<std::uint64_t*>(cnext) = static_cast<std::uint64_t>(addr_);
                cnext += sizeof(std::uint64_t);

                // @c
                *cnext++ = 0xf0; *cnext++ = 0x86; *cnext++ = 0x02;
                break;

            case 2:
                // ASM @5> movabs addr_, %rdx ;
                //     @f> lock xchg %ax, (%rdx) ;
                expected_len = 19; assert(len >= expected_len);
                at_ = start + 0xf;

                // @2
                *cnext++ = 0x48; *cnext++ = 0xba;
                *reinterpret_cast<std::uint64_t*>(cnext) = static_cast<std::uint64_t>(addr_);
                cnext += sizeof(std::uint64_t);

                // @c
                *cnext++ = 0x66; *cnext++ = 0xf0; *cnext++ = 0x87; *cnext++ = 0x02;
                break;

            default:
                assert(false);
        }
    }

    assert((cnext - static_cast<char*>(code)) ==
            static_cast<std::ptrdiff_t>(expected_len));
    return expected_len;
}

inline std::size_t
CacheFlush::emit_X86_64(types::InstPtr start, AssemblerState *asms,
                        void *code, std::size_t len)
{
    char *cnext = static_cast<char*>(code);
    std::size_t expected_len = 0;

    if (addr_ <= static_cast<types::Addr>(0xffffffff)) {
        // ASM @0> clflush addr_ ;
        expected_len = 8; assert(len >= expected_len);

        // @0
        *cnext++ = 0x0f; *cnext++ = 0xae;
        *cnext++ = 0x3c; *cnext++ = 0x25;
        *reinterpret_cast<std::uint32_t*>(cnext) = static_cast<std::uint32_t>(addr_);
        cnext += sizeof(std::uint32_t);
    } else {
        // ASM @0> mov addr_, %rdx ;
        //     @a> clflush (%rdx) ;
        expected_len = 13; assert(len >= expected_len);

        // @0
        *cnext++ = 0x48; *cnext++ = 0xba;
        *reinterpret_cast<std::uint64_t*>(cnext) = static_cast<std::uint64_t>(addr_);
        cnext += sizeof(std::uint64_t);

        // @a
        *cnext++ = 0x0f; *cnext++ = 0xae; *cnext++ = 0x3a;
    }

    assert((cnext - static_cast<char*>(code)) ==
            static_cast<std::ptrdiff_t>(expected_len));
    return expected_len;
}

} /* namespace ops */
} /* namespace codegen */
} /* namespace mc2lib */

#endif /* MC2LIB_CODEGEN_OPS_X86_64_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
