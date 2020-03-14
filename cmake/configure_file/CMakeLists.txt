cmake_minimum_required(VERSION 3.5)
project(copy_flit_files)

set(flit_dir ${CMAKE_SOURCE_DIR}/flit)

foreach(flit_file custom.mk main.cpp tests/Empty.cpp)
  configure_file(
    ${flit_dir}/${flit_file}
    ${CMAKE_BINARY_DIR}/flit/${flit_file}
    COPYONLY)
endforeach()

# toml_string(<outvar> [<val1> ...])
#
# Replace and populate outvar variable with the values encased in a string
# suitable for a string value for a TOML file.
#
# Example:
#   toml_string(compiler_flags
#     -O3
#     -funsafe-math
#   )
#
# ${compiler_flags} == "'-O3 -funsafe-math'"
function(toml_string outvar)
  string(REPLACE ";" " " _TMP_STR "${ARGN}")
  set(${outvar} "'${_TMP_STR}'" PARENT_SCOPE)
endfunction()

# toml_list(<outvar> [<val1> ...])
#
# Replace and populate the outvar variable with the values encased in a string
# suitable for a list of strings for a TOML file.
#
# Example:
#   toml_list(compiler_optimization_levels -O0 -O1 -O2 -O3 "")
#
# ${compiler_optimization_levels} == "[
#   '-O0',
#   '-O1',
#   '-O2',
#   '-O3',
#   ''
# ]"
function(toml_list outvar)
  message(STATUS "ARGN = '${ARGN}'")
  string(REPLACE ";" "',\n  '" _LIST_VALS "${ARGN}")
  set(${outvar} "[\n  '${_LIST_VALS}'\n]" PARENT_SCOPE)
endfunction()

toml_string(compiler_binary        clang++)
toml_string(compiler_name          clang++)
toml_string(compiler_type          clang)
toml_string(compiler_fixed_flags   -gxx-name=/usr/local/bin/g++-8)
toml_string(compiler_fixed_ldflags
  -use-ld=gold
  -gxx-name=/usr/local/bin/g++-8
)
toml_list(compiler_optimization_levels
  -O0
  -O1
  -O2
  -O3
)
toml_list(compiler_switches_list
  -fassociative-math
  -fexcess-precision=fast
  -fexcess-precision=standard
  -ffinite-math-only
  -ffloat-store
  -ffp-contract=on
  -fmerge-all-constants
  -fno-trapping-math
  -freciprocal-math
  -frounding-math
  -fsignaling-nans
  -fsingle-precision-constant
  -funsafe-math-optimizations
  -march=core-avx2
  -mavx
  "-mavx2 -mfma"
  "-mfpmath=sse -mtune=native"
  ""
)

configure_file(
  ${flit_dir}/flit-config.toml.in
  ${CMAKE_BINARY_DIR}/flit/flit-config.toml
  @ONLY)
