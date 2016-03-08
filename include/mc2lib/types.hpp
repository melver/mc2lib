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

#ifndef MC2LIB_TYPES_HPP_
#define MC2LIB_TYPES_HPP_

#include <cstdint>

namespace mc2lib {

/**
 * @namespace mc2lib::types
 * @brief Common types.
 */
namespace types {

/**
 * @brief Template class of common types, permitting specialization.
 *
 * Can be specialized to declare custom types without overwriting types.hh;
 * however, this appraoch depends on user specializing before including any
 * mc2lib header file. Do *not* define more than 1 per project!
 */
template <bool use_specialized>
struct Types {
  typedef std::uint64_t Addr;
  typedef std::uint16_t Pid;
  typedef std::uint16_t Poi;
  typedef Addr InstPtr;
  typedef std::uint8_t WriteID;
};

/**
 * @brief Address type.
 */
typedef typename Types<true>::Addr Addr;

/**
 * @brief Processor/thread ID type.
 */
typedef typename Types<true>::Pid Pid;

/**
 * @brief Program order index type.
 */
typedef typename Types<true>::Poi Poi;

/**
 * @brief Instruction pointer type.
 */
typedef typename Types<true>::InstPtr InstPtr;

/**
 * @brief Write ID type.
 */
typedef typename Types<true>::WriteID WriteID;

}  // namespace types
}  // namespace mc2lib

#endif /* MC2LIB_TYPES_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
