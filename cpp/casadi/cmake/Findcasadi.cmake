# Assumes Casadi is installed within Python 3
#
# It uses the Python 3 interpreter to find where site packages are stored so
# that it can look for casadi-config.cmake there.

find_package(PythonInterp 3 REQUIRED)
execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
  "from site import getsitepackages; print(';'.join(getsitepackages()))"
  OUTPUT_VARIABLE PYTHON_SITE_MODULES
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${PYTHON_SITE_MODULES}")

find_package(casadi 3.5 CONFIG)
