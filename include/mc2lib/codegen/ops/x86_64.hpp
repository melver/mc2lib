/*
 * Copyright (c) 2014, Marco Elver
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

namespace mc2lib {
namespace codegen {
namespace ops {

inline std::size_t
Return::emit_X86_64(AssemblerState *asms,
                    mc::model14::Arch_TSO *arch, InstPtr start,
                    void *code, std::size_t len)
{
    assert(len >= 1);
    // ASM> retq ;
    *static_cast<char*>(code) = 0xc3;
    return 1;
}

inline std::size_t
Read::emit_X86_64(AssemblerState *asms,
                  mc::model14::Arch_TSO *arch, InstPtr start,
                  void *code, std::size_t len)
{
    const std::size_t EXPECTED_LEN = 9;
    char *cnext = static_cast<char*>(code);

    assert(len >= EXPECTED_LEN);

    // ASM 0> movabs addr_, %al ;
    event_ = asms->make_read<1>(pid(), mc::Event::Read, addr_)[0];

    // @0
    *cnext++ = 0xa0;
    *reinterpret_cast<mc::Event::Addr*>(cnext) = addr_;
    cnext += sizeof(mc::Event::Addr);

    assert((cnext - static_cast<char*>(code)) == EXPECTED_LEN);
    return EXPECTED_LEN;
}

inline std::size_t
Write::emit_X86_64(AssemblerState *asms,
                   mc::model14::Arch_TSO *arch, InstPtr start,
                   void *code, std::size_t len)
{
    const std::size_t EXPECTED_LEN = 13;
    char *cnext = static_cast<char*>(code);

    assert(len >= EXPECTED_LEN);

    // ASM 0> movabs addr_, %rax    ;
    //     a> movb write_id, (%rax) ;
    WriteID write_id = 0;
    event_ = asms->make_write<1>(pid(), mc::Event::Read, addr_, &write_id)[0];

    // 0
    *cnext++ = 0x48; *cnext++ = 0xb8;
    *reinterpret_cast<mc::Event::Addr*>(cnext) = addr_;
    cnext += sizeof(mc::Event::Addr);

    // a
    *cnext++ = 0xc6; *cnext++ = 0x00;
    *reinterpret_cast<WriteID*>(cnext) = write_id;
    cnext += sizeof(WriteID);

    assert((cnext - static_cast<char*>(code)) == EXPECTED_LEN);
    return EXPECTED_LEN;
}

} /* namespace ops */
} /* namespace codegen */
} /* namespace mc2lib */

#endif /* MC2LIB_CODEGEN_OPS_X86_64_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
