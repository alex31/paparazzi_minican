cmake_minimum_required(VERSION 3.16)
project(imav_monitor LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIBCANARD_DIR "/home/alex/DEV/STM32/UAVCAN/libcanard" CACHE PATH
    "Path to the libcanard source directory")
set(DSDLC_DIR "/home/alex/DEV/STM32/UAVCAN/DSDLC" CACHE PATH
    "Path to the generated DroneCAN DSDL C code")

if(NOT EXISTS "${LIBCANARD_DIR}/canard.c")
  message(FATAL_ERROR
    "libcanard not found at ${LIBCANARD_DIR}. Set -DLIBCANARD_DIR=...")
endif()
if(NOT EXISTS "${DSDLC_DIR}/include/uavcan.protocol.debug.KeyValue.h")
  message(FATAL_ERROR
    "DSDLC headers not found at ${DSDLC_DIR}. Set -DDSDLC_DIR=...")
endif()

find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
  set(QT_WIDGETS Qt6::Widgets)
else()
  find_package(Qt5 COMPONENTS Widgets REQUIRED)
  set(QT_WIDGETS Qt5::Widgets)
endif()

add_library(imav_detection_state STATIC
  src/detection_state.cpp
  src/detection_state.hpp
)
target_include_directories(imav_detection_state PUBLIC src)

add_executable(imav_monitor
  src/main.cpp
  src/main_window.cpp
  src/main_window.hpp
  src/history_widget.cpp
  src/history_widget.hpp
  src/socket_can.cpp
  src/socket_can.hpp
  src/uavcan_receiver.cpp
  src/uavcan_receiver.hpp
  ${DSDLC_DIR}/src/uavcan.protocol.debug.KeyValue.c
  ${LIBCANARD_DIR}/canard.c
)

target_compile_definitions(imav_monitor PRIVATE
  CANARD_ENABLE_CANFD=1
  CANARD_ENABLE_TAO_OPTION=1
)
target_include_directories(imav_monitor PRIVATE
  src
  ${LIBCANARD_DIR}
  ${DSDLC_DIR}/include
)
target_link_libraries(imav_monitor PRIVATE
  imav_detection_state
  ${QT_WIDGETS}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # libcanard's public C header intentionally uses flexible array members.
  target_compile_options(imav_monitor PRIVATE -Wall -Wextra)
  target_compile_options(imav_detection_state PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(detection_state_test
    tests/detection_state_test.cpp
  )
  target_link_libraries(detection_state_test PRIVATE imav_detection_state)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(detection_state_test PRIVATE
      -Wall -Wextra -Wpedantic)
  endif()
  add_test(NAME detection_state_test COMMAND detection_state_test)
endif()
