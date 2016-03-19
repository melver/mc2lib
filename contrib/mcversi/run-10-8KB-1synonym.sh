#!/bin/sh
#
# 8KB configuration with virtual address synonyms (2 virtual addresses per 1
# physical).

/mcversi/guest_workload $(nproc) 10 0x1000000 0x09140010 0x100000000 1
