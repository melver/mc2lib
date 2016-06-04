#!/bin/sh
#
# This code is licensed under the BSD 3-Clause license. See the LICENSE file in
# the project root for license terms.
#
# 8KB configuration from McVerSi paper.

# Optionally set this to one of:
# - SCHED_RR
# - SCHED_FIFO
export MC2_SCHED_POLICY=

# Run with as many threads as reported by nproc; 10 iterations per test;
# allocate 15729152 bytes of memory, with
#
# - stride = (0x09140010 & ((1 << 16) - 1)) = 16 bytes;
# - chunks of size (1 << ((0x09140010 & (0xff << 24)) >> 24)) = 512 bytes;
# - and valid chunk base addresses offset by multiples of
#   (1 << ((0x09140010 & (0xff << 16)) >> 16)) = 1048576 bytes from first base address.
#
# and first base address of memory is 0x100000000 (optionally set, but used
# here to force 64bit address instructions by the code generator). This
# effectively uses 8KB of memory distributed across 16MB.

/mcversi/guest_workload $(nproc) 10 0xf00200 0x09140010 0x100000000
