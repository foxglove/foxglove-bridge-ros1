cmake_minimum_required(VERSION 3.17)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)
endif()
if(POLICY CMP0024)
  cmake_policy(SET CMP0024 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0024 NEW)
endif()

project(foxglove_bridge LANGUAGES CXX VERSION 3.4.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Avoid DOWNLOAD_EXTRACT_TIMESTAMP warning for CMake >= 3.24
if (POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

# Remote access requires glibc >= 2.35 on Linux. Detect the build host's glibc
# (when not cross-compiling) so we can lower the default to OFF on older
# systems and warn if the user pins it ON.
set(_glibc_detected FALSE)
set(_glibc_too_old FALSE)
set(_glibc_version "")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT CMAKE_CROSSCOMPILING)
  execute_process(
    COMMAND getconf GNU_LIBC_VERSION
    OUTPUT_VARIABLE _glibc_version_output
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _glibc_result
  )
  if(_glibc_result EQUAL 0 AND _glibc_version_output MATCHES "glibc ([0-9]+)\\.([0-9]+)")
    set(_glibc_major ${CMAKE_MATCH_1})
    set(_glibc_minor ${CMAKE_MATCH_2})
    set(_glibc_detected TRUE)
    set(_glibc_version "${_glibc_major}.${_glibc_minor}")
    if(_glibc_major LESS 2 OR (_glibc_major EQUAL 2 AND _glibc_minor LESS 35))
      set(_glibc_too_old TRUE)
    endif()
  endif()
endif()

set(_remote_access_default ON)
if(NOT DEFINED CACHE{FOXGLOVE_BRIDGE_REMOTE_ACCESS})
  if(_glibc_too_old)
    message(STATUS "Detected glibc ${_glibc_version} (< 2.35); defaulting FOXGLOVE_BRIDGE_REMOTE_ACCESS to OFF")
    set(_remote_access_default OFF)
  elseif(NOT _glibc_detected)
    message(STATUS "Could not detect glibc version, defaulting FOXGLOVE_BRIDGE_REMOTE_ACCESS to ON")
  endif()
endif()
option(FOXGLOVE_BRIDGE_REMOTE_ACCESS "Build with remote access gateway support" ${_remote_access_default})

if(FOXGLOVE_BRIDGE_REMOTE_ACCESS AND _glibc_too_old)
  message(WARNING "FOXGLOVE_BRIDGE_REMOTE_ACCESS=ON but build host has glibc ${_glibc_version} (< 2.35); the link step is likely to fail.")
endif()

macro(enable_strict_compiler_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror -Wold-style-cast -Wfloat-equal -Wmost -Wunused-exception-parameter)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror -Wold-style-cast -Wfloat-equal)
  endif()
endmacro()

find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Detect big-endian architectures
include(TestBigEndian)
TEST_BIG_ENDIAN(ENDIAN)
if (ENDIAN)
  add_compile_definitions(ARCH_IS_BIG_ENDIAN=1)
endif()

find_package(catkin REQUIRED COMPONENTS
  resource_retriever
  roscpp
  rosgraph_msgs
  topic_tools
  ros_babel_fish
)

# Intentionally exports nothing: this is a leaf package; no downstream
# package builds against foxglove_bridge_lib.
catkin_package()

# Fetch the released SDK zip from GitHub. It contains the prebuilt C libraries
# (with remote access), the C++ wrapper sources, and the CMake package config
# consumed below. To build against a locally-modified SDK instead, override
# FETCHCONTENT_SOURCE_DIR_FOXGLOVE_SDK (cmake's standard FetchContent override)
# to point at a `make build-cpp-dist`-produced `cpp/dist` tree — the
# Makefile's docker-test-local-sdk target does this for you (see README.md).
set(FOXGLOVE_SDK_VERSION "0.25.1")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(FOXGLOVE_SDK_PLATFORM "aarch64-unknown-linux-gnu")
  set(FOXGLOVE_SDK_SHA "c70e35c78f0e37f3ba8d0251cd31f353bb10b1fd106200dc6c5f409ed4df1918")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(FOXGLOVE_SDK_PLATFORM "x86_64-unknown-linux-gnu")
  set(FOXGLOVE_SDK_SHA "0b9d348df3a8e98d2c2cee2360cc9f52e83bb3445044debd771d1b46dc358be4")
else()
  message(FATAL_ERROR "Unsupported platform/architecture combination: ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}")
endif()
set(FOXGLOVE_SDK_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv${FOXGLOVE_SDK_VERSION}/foxglove-v${FOXGLOVE_SDK_VERSION}-cpp-${FOXGLOVE_SDK_PLATFORM}.zip")

include(FetchContent)
FetchContent_Declare(
  foxglove_sdk
  URL ${FOXGLOVE_SDK_URL}
  URL_HASH SHA256=${FOXGLOVE_SDK_SHA}
)
FetchContent_MakeAvailable(foxglove_sdk)

# The dist's CMake package config defines IMPORTED targets for the prebuilt C libraries
# (foxglove-static, foxglove-shared) and the foxglove_sdk_add_cpp_library() helper, which
# compiles the SDK's C++ wrapper sources against our toolchain. find_package looks for
# the config at <sdk>/lib/cmake/foxglove-sdk/foxglove-sdkConfig.cmake.
find_package(foxglove-sdk CONFIG REQUIRED HINTS "${foxglove_sdk_SOURCE_DIR}")

# Always build the C++ wrapper as a SHARED library: with RA, dynamic-link the RA
# cdylib (libfoxglove.so); without RA, statically link the staticlib into the wrapper.
if (FOXGLOVE_BRIDGE_REMOTE_ACCESS)
  foxglove_sdk_add_cpp_library(foxglove_cpp_shared TYPE SHARED REMOTE_ACCESS ON)
else()
  foxglove_sdk_add_cpp_library(foxglove_cpp_shared TYPE SHARED REMOTE_ACCESS OFF)
endif()
# INSTALL_RPATH only: build-tree binaries keep CMake's default build rpath,
# which points at the linked libraries' actual locations (including the SDK
# dist's libfoxglove.so) — so a plain catkin_make_isolated without --install
# produces runnable binaries. The install step rewrites to the $ORIGIN-
# relative rpaths for the installed layout.
set_target_properties(foxglove_cpp_shared PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

# Stamp the version and git hash into version.cpp so the bridge can identify
# itself at startup. FOXGLOVE_BRIDGE_GIT_HASH may be passed on the cmake
# command line — the docker build does this, since .dockerignore keeps .git
# out of the build context. Otherwise it is captured from git at configure
# time; a rebuild after new commits without re-running cmake keeps the stale
# hash, which is close enough for triaging bug reports.
if(NOT FOXGLOVE_BRIDGE_GIT_HASH)
  find_program(GIT_SCM git DOC "Git version control")
  if(GIT_SCM)
    execute_process(
      COMMAND ${GIT_SCM} describe --always --dirty --exclude=*
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      OUTPUT_VARIABLE FOXGLOVE_BRIDGE_GIT_HASH
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
  endif()
endif()
if(NOT FOXGLOVE_BRIDGE_GIT_HASH)
  set(FOXGLOVE_BRIDGE_GIT_HASH "unknown")
endif()
set(FOXGLOVE_BRIDGE_VERSION "${CMAKE_PROJECT_VERSION}")
configure_file(src/version.cpp.in src/version.cpp @ONLY)

add_library(foxglove_bridge_lib SHARED
  src/capabilities.cpp
  src/param_interface.cpp
  src/ros1_foxglove_bridge.cpp
  src/service_utils.cpp
  src/transport_manager.cpp
  # Generated:
  ${CMAKE_CURRENT_BINARY_DIR}/src/version.cpp
)
target_include_directories(foxglove_bridge_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(foxglove_bridge_lib SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(foxglove_bridge_lib
  PUBLIC foxglove_cpp_shared ${catkin_LIBRARIES}
  PRIVATE Threads::Threads
)
# The FOXGLOVE_REMOTE_ACCESS compile definition propagates PUBLIC from
# foxglove_cpp_shared (set by the SDK's foxglove_sdk_add_cpp_library).
enable_strict_compiler_warnings(foxglove_bridge_lib)
set_target_properties(foxglove_bridge_lib PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

add_executable(foxglove_bridge
  src/ros1_foxglove_bridge_node.cpp
)
target_link_libraries(foxglove_bridge
  foxglove_bridge_lib
)
enable_strict_compiler_warnings(foxglove_bridge)
set_target_properties(foxglove_bridge PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../lib"
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(websocketpp REQUIRED)

  # The smoke test launches a master + the bridge (smoke.test) and exercises it
  # with the in-repo ws-protocol test client (tests/client).
  add_rostest_gtest(smoke_test tests/smoke.test tests/smoke_test.cpp)
  # ASIO_STANDALONE makes websocketpp dispatch to standalone asio (header-only,
  # supplied by the `asio` system package) rather than boost::asio.
  target_compile_definitions(smoke_test PRIVATE ASIO_STANDALONE)
  target_include_directories(smoke_test PRIVATE tests)
  target_link_libraries(smoke_test
    foxglove_bridge_lib
    nlohmann_json::nlohmann_json
    websocketpp::websocketpp
  )
  enable_strict_compiler_warnings(smoke_test)
endif()

install(TARGETS foxglove_bridge_lib foxglove_cpp_shared
  ARCHIVE DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
# The remote-access cdylib must live alongside the wrapper for its $ORIGIN rpath.
if (FOXGLOVE_BRIDGE_REMOTE_ACCESS)
  install(FILES $<TARGET_FILE:foxglove-shared> DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
endif()

install(TARGETS foxglove_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
