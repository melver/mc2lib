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

#ifndef _GNU_SOURCE
#  define _GNU_SOURCE
#endif

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/shm.h>

#include "host_support.h"

#ifndef MAX_CODE_SIZE
#  define MAX_CODE_SIZE (4096*16)
#endif

#ifndef MAX_THREADS
#  define MAX_THREADS 64
#endif

static size_t num_threads = 0;

static char *test_mem = NULL;
static size_t test_mem_bytes = 0;
static size_t test_mem_stride = 0;

static pthread_barrier_t barrier;
static pthread_t thread_main_id;

static inline void
reset_test_mem(void **used_addrs, size_t len)
{
#if !HOST_ZERO_TEST_MEM
	if (used_addrs != NULL) {
		// NULL marks end of list.
		for (size_t i = 0; i < len && used_addrs[i]; ++i) {
			// Mask stride due encoding (see mc2lib RandomFactory)
			memset(used_addrs[i], 0, (test_mem_stride & 0xffff));
		}
	} else {
		memset(test_mem, 0, test_mem_bytes);
	}

	full_memory_barrier();
#endif

	if (used_addrs != NULL) {
		// NULL marks end of list.
		for (size_t i = 0; i < len && used_addrs[i]; ++i) {
			flush_cache_line(used_addrs[i]);
		}
	} else {
#if defined(CACHELINE_SIZE) && CACHELINE_SIZE != 0
		// Fallback
		for (size_t i = 0; i < test_mem_bytes; i += CACHELINE_SIZE) {
			flush_cache_line(&test_mem[i]);
		}
#endif
	}

	full_memory_barrier();
}

static inline void
barrier_wait_pthread(void)
{
	int rc = pthread_barrier_wait(&barrier);
	assert(rc == 0 || rc == PTHREAD_BARRIER_SERIAL_THREAD);
}

void*
thread_func(void *arg)
{
	const size_t test_iterations = (size_t) arg;
	const pthread_t thread_self = pthread_self();

	void *code = mmap(NULL, MAX_CODE_SIZE,
	                  PROT_READ | PROT_WRITE | PROT_EXEC,
	                  MAP_ANONYMOUS | MAP_PRIVATE,
	                  -1, 0);
	assert(code != NULL);
	memset(code, 0, MAX_CODE_SIZE);

	void (*thread_test)() = GET_CALLABLE_THREAD(code);

	void **used_addrs = NULL;
#if MAX_USED_ADDRS_SIZE
	if (thread_self == thread_main_id) {
		used_addrs = (void**)malloc(MAX_USED_ADDRS_SIZE);
		assert(used_addrs != NULL);
		memset(used_addrs, 0, MAX_USED_ADDRS_SIZE);
	}
#endif

	barrier_wait_pthread();

	while (1) {
		host_make_test_thread(code, MAX_CODE_SIZE);
		full_memory_barrier();

		for (size_t i = 0; i < test_iterations; ++i) {
			barrier_wait_precise(num_threads);

			full_memory_barrier();

			thread_test();

			full_memory_barrier();

			barrier_wait_coarse(num_threads);

			if (i + 1 < test_iterations
			    && thread_self == thread_main_id) {
				host_verify_reset_conflict(used_addrs,
				                           MAX_USED_ADDRS_SIZE);
				reset_test_mem(used_addrs, MAX_USED_ADDRS_SIZE);
			}
		}

		if (thread_self == thread_main_id) {
			host_verify_reset_all(used_addrs, MAX_USED_ADDRS_SIZE);
			reset_test_mem(used_addrs, MAX_USED_ADDRS_SIZE);
		}

		barrier_wait_coarse(num_threads);
	}

#if MAX_USED_ADDRS_SIZE
	if (thread_self == thread_main_id) {
		free(used_addrs);
	}
#endif

	munmap(code, MAX_CODE_SIZE);
	return NULL;
}

static void
setaffinity_thread(size_t pid, pthread_t thread)
{
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(pid, &cpuset);
	int rc = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
	assert(!rc);
}

static void
setaffinity_attr(size_t pid, pthread_attr_t *attr)
{
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(pid, &cpuset);
	int rc = pthread_attr_setaffinity_np(attr, sizeof(cpu_set_t), &cpuset);
	assert(!rc);
}

static void
spawn_threads(size_t test_iterations)
{
	assert(MAX_THREADS <= CPU_SETSIZE);
	assert(num_threads <= MAX_THREADS);
	assert(num_threads > 0);

	// Reuse main thread.
	pthread_t thread_ids[MAX_THREADS - 1];
	pthread_attr_t attr;

	int rc = pthread_barrier_init(&barrier, NULL, num_threads);
	assert(!rc);

	thread_main_id = pthread_self();
	setaffinity_thread(0, thread_main_id);

	printf("Spawning threads ...\n");

	for (size_t i = 0; i < num_threads - 1; ++i) {
		rc = pthread_attr_init(&attr);
		assert(!rc);

		setaffinity_attr(i + 1, &attr);

		pthread_attr_getinheritsched(&attr, &rc);
		assert(rc == PTHREAD_INHERIT_SCHED);

		rc = pthread_create(&thread_ids[i], &attr, thread_func,
				    (void*) test_iterations);
		assert(!rc);

		rc = pthread_attr_destroy(&attr);
		assert(!rc);
	}

	roi_begin();

	printf("Running tests ...\n");
	thread_func((void*) test_iterations);

	for (size_t i = 0; i < num_threads - 1; ++i) {
		pthread_join(thread_ids[i], NULL);
	}

	printf("All tests complete.\n");

	roi_end();

	pthread_barrier_destroy(&barrier);
}

static void
setup_sched(void)
{
	const char *env_policy = getenv("MC2_SCHED_POLICY");
	if (!env_policy) {
		// Do nothing.
		return;
	}

	// defaults
	int policy = SCHED_FIFO;
	struct sched_param sp = {
		.sched_priority = sched_get_priority_max(policy) - 20
	};

	if (env_policy[0] != '\0') {
		if (!strcmp(env_policy, "SCHED_RR")) {
			policy = SCHED_RR;
		} else if (!strcmp(env_policy, "SCHED_FIFO")) {
			policy = SCHED_FIFO;
		} else {
			perror("Invalid MC2_SCHED_POLICY!");
			exit(1);
		}
	}

	const char *env_prio = getenv("MC2_SCHED_PRIO");
	if (env_prio) {
		sp.sched_priority = atoi(env_prio);
		assert(sp.sched_priority != 0);
	}

	if (sched_setscheduler(0, policy, &sp) == -1) {
		perror("sched_setscheduler failed!");
		exit(1);
	} else {
		printf("Set RT scheduler: %d @ %d\n", policy,
		       sp.sched_priority);
	}
}

static void
usage(const char *progname)
{
	printf("Usage: %s <num-threads> <test-iterations> <test-mem-bytes> "
	       "<test-mem-stride> [<test-mem-addr> [<synonym-count>]]\n",
	       progname);

	exit(42);
}

int
main(int argc, char *argv[])
{
	if (argc <= 4) usage(argv[0]);

	host_init();
	setup_sched();

	// Get number of threads
	num_threads = strtoull(argv[1], NULL, 0);
	if (!num_threads) usage(argv[0]);
	printf("Threads: %zu\n", num_threads);

	// Get iterations per test
	size_t test_iterations = strtoull(argv[2], NULL, 0);
	if (!test_iterations) usage(argv[0]);
	printf("Test iterations: %zu\n", test_iterations);

	// Initialize test memory
	test_mem_bytes = strtoull(argv[3], NULL, 0);
	if (!test_mem_bytes) usage(argv[0]);

	test_mem_stride = strtoull(argv[4], NULL, 0);
	if (!test_mem_stride) usage(argv[0]);

	assert(test_mem == NULL);
	size_t synonym_count = 0;
	if (argc > 5) {
		// Optionally set test-memory base address.
		test_mem = (char*)strtoull(argv[5], NULL, 0);

		if (test_mem && argc > 6) {
			synonym_count = strtoull(argv[6], NULL, 0);
		}
	}

	int shm_id;
	if (synonym_count) {
		shm_id = shmget(IPC_PRIVATE, test_mem_bytes,
				IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR);
		assert(shm_id != -1);
		test_mem = (char*)shmat(shm_id, test_mem, 0);

		for (size_t i = 0; i < synonym_count; ++i) {
			char* synonym_mem = (char*)shmat(shm_id,
					test_mem + (test_mem_bytes * (i+1)), 0);
			assert(synonym_mem != (char*)-1);
			printf("Test memory synonym @ 0x%tx\n",
			       (ptrdiff_t)synonym_mem);
		}
	} else if (test_mem) {
		test_mem = (char*)mmap(test_mem, test_mem_bytes,
				       PROT_READ | PROT_WRITE,
				       MAP_ANONYMOUS | MAP_PRIVATE | MAP_FIXED,
				       0, 0);
		assert(test_mem != MAP_FAILED);
	} else {
		test_mem = (char*)malloc(test_mem_bytes);
		assert(test_mem != NULL);
	}

	printf("Test memory: %zu bytes (stride=0x%zx) @ 0x%tx\n",
	       test_mem_bytes, test_mem_stride, (ptrdiff_t)test_mem);

	// Let host know of memory range
#if HOST_ZERO_TEST_MEM
	// Need to access memory once if host wants to access page-table
	// entries.
#  if defined(CACHELINE_SIZE) && CACHELINE_SIZE != 0
	for (size_t i = 0; i < test_mem_bytes; i += CACHELINE_SIZE) {
		test_mem[i] = 0x42;
	}
#  else
	memset(test_mem, 0, test_mem_bytes);
#  endif
	full_memory_barrier();
#endif

	host_mark_test_mem_range(test_mem,
			(test_mem + (test_mem_bytes * (synonym_count+1)) - 1),
			test_mem_stride,
			(synonym_count ? (void*)(test_mem_bytes - 1) : NULL));

	reset_test_mem(NULL, 0);

	spawn_threads(test_iterations);

	// cleanup
	if (synonym_count) {
		shmdt(test_mem);
		for (size_t i = 0; i < synonym_count; ++i) {
			char* synonym_mem = test_mem + (test_mem_bytes * (i+1));
			shmdt(synonym_mem);
		}
		shmctl(shm_id, IPC_RMID, 0);
	} else if (argc > 5 && strtoull(argv[5], NULL, 0)) {
		munmap(test_mem, test_mem_bytes);
	} else {
		free(test_mem);
	}

	return EXIT_SUCCESS;
}

