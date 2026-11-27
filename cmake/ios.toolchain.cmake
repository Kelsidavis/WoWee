# CMake toolchain file for iOS / iPadOS cross-compilation.
# Usage:
#   cmake -S . -B build-ios \
#     -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake \
#     -DCMAKE_OSX_ARCHITECTURES=arm64 \
#     -DCMAKE_OSX_DEPLOYMENT_TARGET=16.0

set(CMAKE_SYSTEM_NAME iOS)
set(CMAKE_SYSTEM_PROCESSOR arm64)

set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "")
set(CMAKE_OSX_DEPLOYMENT_TARGET "16.0" CACHE STRING "")

# Release is the default, and the reason is worth writing down.
#
# The Xcode generator ignores CMAKE_BUILD_TYPE for the project it writes -
# CMAKE_XCODE_CONFIGURATION is the configuration it builds by default - and the
# debug configuration of this project is not a build to measure with:
#
#   - enableValidation follows NDEBUG, so a debug build loads the Khronos
#     validation layer, which intercepts and checks every Vulkan call. The
#     renderer makes thousands of them per frame.
#   - the release configuration compiles at -O3 and the debug one at -Og.
#
# A debug build is for the crash it was built to catch. Ask for it explicitly
# with -configuration Debug on the xcodebuild line.
set(CMAKE_XCODE_CONFIGURATION "Release" CACHE STRING "")
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "")

# Let Xcode provide the sysroot; do not hardcode.
if(NOT CMAKE_OSX_SYSROOT)
    set(CMAKE_OSX_SYSROOT "iphoneos" CACHE STRING "iOS SDK")
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
