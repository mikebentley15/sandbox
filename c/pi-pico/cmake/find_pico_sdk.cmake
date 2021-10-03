# Note: importing the Pico SDK must be done before project()

option(PICO_SDK_FETCH_FROM_GIT
  "Download the Pi Pico SDK from GitHub instead of using an existing copy"
  OFF
)

if (NOT PICO_SDK_FETCH_FROM_GIT)
  set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk"
      CACHE STRING
      "Location of the Pico SDK (defaults to using the git submodule)"
  )
  if (NOT EXISTS "${PICO_SDK_PATH}/pico_sdk_init.cmake")
    message(FATAL_ERROR
      "Cannot find Pico SDK, please initialize the git submodule or set PICO_SDK_PATH")
  endif()
  include("${PICO_SDK_PATH}/pico_sdk_init.cmake")
else()
  include("cmake/pico_sdk_import.cmake")
endif()

