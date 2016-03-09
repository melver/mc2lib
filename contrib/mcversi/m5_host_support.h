/*
 * Copyright (c) 2014-2015 Marco Elver
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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
 *
 * Authors: Marco Elver
 */

#ifndef HOST_SUPPORT_H_
#define HOST_SUPPORT_H_

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "m5op.h"

#ifndef CACHELINE_SIZE
#  define CACHELINE_SIZE 64
#endif

#ifndef BARRIER_USE_QUIESCE
#  define BARRIER_USE_QUIESCE 1
#endif

#ifdef M5OP_ADDR
void *m5_mem = NULL;
#endif

#if defined(__x86_64__)

inline void
full_memory_barrier(void)
{
	__asm__ __volatile__ ( "mfence" ::: "memory" );
}

inline void
flush_cache_line(volatile void *addr)
{
	__asm__ __volatile__ ("clflush (%0)" :: "r" (addr) : "memory");
}

#else /* defined(__x86_64__) */
#  error "Unsupported architecture!"
#endif

/*
 * Host pseudo-op wrapper functions.
 */

#define host_mark_test_mem_range m5_mark_test_mem_range
#define host_make_test_thread m5_make_test_thread

inline void
host_verify_reset_conflict(void **used_addrs, uint64_t len)
{
	while(!m5_verify_reset_conflict(used_addrs, len));
}

inline void
host_verify_reset_all(void **used_addrs, uint64_t len)
{
	while(!m5_verify_reset_all(used_addrs, len));
}

inline void
barrier_wait_precise(uint64_t nt)
{
	full_memory_barrier();

#if BARRIER_USE_QUIESCE
	while(m5_barrier_async(nt, 1));

	full_memory_barrier();
#endif

	while(m5_barrier_async(nt, 0));

	full_memory_barrier();
}

inline void
barrier_wait_coarse(uint64_t nt)
{
#if BARRIER_USE_QUIESCE
	full_memory_barrier();

	while(m5_barrier_async(nt, 1));

	full_memory_barrier();
#else
	barrier_wait_precise(nt);
#endif
}

#ifdef M5OP_ADDR
inline void
map_m5_mem(void)
{
    int fd;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        perror("Cannot open /dev/mem");
        exit(1);
    }

    m5_mem = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, M5OP_ADDR);
    if (!m5_mem) {
        perror("Cannot mmap /dev/mem");
        exit(1);
    }
}
#endif

inline void
host_init(void)
{
#ifdef M5OP_ADDR
	map_m5_mem();
#endif
}

inline void
roi_begin(void)
{
	m5_dumpreset_stats(0, 0);
}

inline void
roi_end(void)
{
	m5_fail(0, 42);
}

#endif /* HOST_SUPPORT_H_ */
