#!/bin/bash

g++ -O0 minimal.cpp &>/dev/null && \
  ! g++ -O3 minimal.cpp &>/dev/null

