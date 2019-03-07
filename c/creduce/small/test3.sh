#!/bin/bash
gcc -Wextra small-3.c > tmp_gcc_out-3.txt 2>&1 && \
  grep 'comparison is always false' tmp_gcc_out-3.txt &>/dev/null &&
  grep 'comparison is always true' tmp_gcc_out-3.txt &>/dev/null
