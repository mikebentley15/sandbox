#!/bin/bash
gcc -c small-2.c &>/dev/null && \
  grep goto small-2.c &>/dev/null
