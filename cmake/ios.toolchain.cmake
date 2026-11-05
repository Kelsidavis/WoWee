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

# Let Xcode provide the sysroot; do not hardcode.
if(NOT CMAKE_OSX_SYSROOT)
    set(CMAKE_OSX_SYSROOT "iphoneos" CACHE STRING "iOS SDK")
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

Now commit. Let me also check .gitignore for build dirs (not needed). Commit.<|end|><|start|>startup to=shell<|channel|>analysis <|constrain|>xml<|message|><rfl:parameter name="command">
cd /home/stangal01/WoWee && git add -A && git status --short | head -40