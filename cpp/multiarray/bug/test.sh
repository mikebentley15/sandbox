#!/bin/bash

g++ -O0 minimal-manual-and-creduce.cpp &>/dev/null && \
  ! g++ -O3 minimal-manual-and-creduce.cpp &>/dev/null

