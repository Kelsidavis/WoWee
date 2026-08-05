cmake_minimum_required(VERSION 3.15)
project(wowee VERSION 1.0.0 LANGUAGES C CXX)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Explicitly tag optimized configs so runtime defaults can enforce low-noise logging.
add_compile_definitions(
    $<$<CONFIG:Release>:WOWEE_RELEASE_LOGGING>
    $<$<CONFIG:RelWithDebInfo>:WOWEE_RELEASE_LOGGING>
    $<$<CONFIG:MinSizeRel>:WOWEE_RELEASE_LOGGING>
)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(WIN32)
    # Needed for Vulkan Win32 external memory/semaphore handle types used by FSR3 interop.
    add_compile_definitions(VK_USE_PLATFORM_WIN32_KHR WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# Options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(WOWEE_BUILD_TESTS "Build tests" ON)
option(WOWEE_SYSTEM_LUA "Link against an installed Lua 5.1 instead of the vendored copy" OFF)
option(WOWEE_ENABLE_ASAN "Enable AddressSanitizer + UBSan (applied to all build configs; pairs well with Debug)" OFF)
option(WOWEE_ENABLE_TRACY "Enable Tracy profiler instrumentation" OFF)
option(WOWEE_ENABLE_AMD_FSR2 "Enable AMD FidelityFX FSR2 backend when SDK is present" ON)
option(WOWEE_ENABLE_AMD_FSR3_FRAMEGEN "Enable AMD FidelityFX SDK FSR3 frame generation interface probe when SDK is present" ON)
option(WOWEE_BUILD_AMD_FSR3_RUNTIME "Build native AMD FidelityFX VK runtime (Path A) from extern/FidelityFX-SDK/Kits" ON)

# AMD FidelityFX FSR2 SDK detection (drop-in under extern/FidelityFX-FSR2)
set(WOWEE_AMD_FSR2_DIR ${CMAKE_SOURCE_DIR}/extern/FidelityFX-FSR2)
set(WOWEE_AMD_FSR2_HEADER ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/ffx_fsr2.h)
set(WOWEE_AMD_FSR2_VK_PERM_HEADER ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/vk/shaders/ffx_fsr2_accumulate_pass_permutations.h)
set(WOWEE_AMD_FSR2_VK_VENDOR_DIR ${CMAKE_SOURCE_DIR}/third_party/fsr2_vk_permutations)

# Upstream SDK checkouts may not ship generated Vulkan permutation headers.
# If we have a vendored snapshot, copy it into the SDK tree before detection.
if(WOWEE_ENABLE_AMD_FSR2 AND EXISTS ${WOWEE_AMD_FSR2_HEADER} AND NOT EXISTS ${WOWEE_AMD_FSR2_VK_PERM_HEADER} AND EXISTS ${WOWEE_AMD_FSR2_VK_VENDOR_DIR})
    file(GLOB WOWEE_AMD_FSR2_VK_VENDOR_HEADERS "${WOWEE_AMD_FSR2_VK_VENDOR_DIR}/ffx_fsr2_*pass*.h")
    list(LENGTH WOWEE_AMD_FSR2_VK_VENDOR_HEADERS WOWEE_AMD_FSR2_VK_VENDOR_COUNT)
    if(WOWEE_AMD_FSR2_VK_VENDOR_COUNT GREATER 0)
        file(MAKE_DIRECTORY ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/vk/shaders)
        file(COPY ${WOWEE_AMD_FSR2_VK_VENDOR_HEADERS} DESTINATION ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/vk/shaders)
        message(STATUS "AMD FSR2: bootstrapped ${WOWEE_AMD_FSR2_VK_VENDOR_COUNT} vendored Vulkan permutation headers")
    endif()
endif()

if(WOWEE_ENABLE_AMD_FSR2 AND EXISTS ${WOWEE_AMD_FSR2_HEADER} AND EXISTS ${WOWEE_AMD_FSR2_VK_PERM_HEADER})
    message(STATUS "AMD FSR2 SDK detected at ${WOWEE_AMD_FSR2_DIR}")
    add_compile_definitions(WOWEE_HAS_AMD_FSR2=1)
    add_compile_definitions(FFX_GCC=1)

    # AMD FSR2 Vulkan backend sources (official SDK implementation)
    set(WOWEE_AMD_FSR2_SOURCES
        ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/ffx_assert.cpp
        ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/ffx_fsr2.cpp
        ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/vk/ffx_fsr2_vk.cpp
        ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/vk/shaders/ffx_fsr2_shaders_vk.cpp
    )
    add_library(wowee_fsr2_amd_vk STATIC ${WOWEE_AMD_FSR2_SOURCES})
    set_target_properties(wowee_fsr2_amd_vk PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
    target_include_directories(wowee_fsr2_amd_vk PUBLIC
        ${WOWEE_AMD_FSR2_DIR}/src
        ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api
        ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/vk
        ${WOWEE_AMD_FSR2_DIR}/src/ffx-fsr2-api/vk/shaders
    )
    set(WOWEE_FFX_COMPAT_HEADER ${CMAKE_SOURCE_DIR}/include/third_party/ffx_fsr2_compat.h)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(wowee_fsr2_amd_vk PRIVATE
            "-include${WOWEE_FFX_COMPAT_HEADER}"
            -Wno-deprecated-declarations
        )
    elseif(MSVC)
        target_compile_options(wowee_fsr2_amd_vk PRIVATE
            "/FI${WOWEE_FFX_COMPAT_HEADER}"
        )
    endif()
    target_link_libraries(wowee_fsr2_amd_vk PUBLIC Vulkan::Vulkan)
else()
    add_compile_definitions(WOWEE_HAS_AMD_FSR2=0)
    if(WOWEE_ENABLE_AMD_FSR2)
        if(NOT EXISTS ${WOWEE_AMD_FSR2_HEADER})
            message(WARNING "AMD FSR2 SDK not found at ${WOWEE_AMD_FSR2_DIR}; using internal fallback implementation.")
        elseif(NOT EXISTS ${WOWEE_AMD_FSR2_VK_PERM_HEADER})
            message(WARNING "AMD FSR2 SDK found, but generated Vulkan permutation headers are missing (e.g. ${WOWEE_AMD_FSR2_VK_PERM_HEADER}); using internal fallback implementation.")
        endif()
    endif()
endif()

# AMD FidelityFX SDK (FSR3 frame generation interfaces) detection under extern/FidelityFX-SDK
set(WOWEE_AMD_FFX_SDK_KITS_DIR ${CMAKE_SOURCE_DIR}/extern/FidelityFX-SDK/Kits/FidelityFX)
set(WOWEE_AMD_FFX_SDK_KITS_FG_HEADER ${WOWEE_AMD_FFX_SDK_KITS_DIR}/framegeneration/include/ffx_framegeneration.h)

set(WOWEE_AMD_FFX_SDK_KITS_READY FALSE)
if(EXISTS ${WOWEE_AMD_FFX_SDK_KITS_DIR}/upscalers/fsr3/include/ffx_fsr3upscaler.h
   AND EXISTS ${WOWEE_AMD_FFX_SDK_KITS_DIR}/framegeneration/fsr3/include/ffx_frameinterpolation.h
   AND EXISTS ${WOWEE_AMD_FFX_SDK_KITS_DIR}/framegeneration/fsr3/include/ffx_opticalflow.h
   AND EXISTS ${WOWEE_AMD_FFX_SDK_KITS_DIR}/backend/vk/ffx_vk.h)
    set(WOWEE_AMD_FFX_SDK_KITS_READY TRUE)
endif()

if(WOWEE_ENABLE_AMD_FSR3_FRAMEGEN AND WOWEE_AMD_FFX_SDK_KITS_READY)
    message(STATUS "AMD FidelityFX-SDK framegen headers detected at ${WOWEE_AMD_FFX_SDK_KITS_DIR} (Kits layout)")
    add_compile_definitions(WOWEE_AMD_FFX_SDK_KITS=1)
    add_compile_definitions(WOWEE_HAS_AMD_FSR3_FRAMEGEN=1)
    add_library(wowee_fsr3_framegen_amd_vk_probe STATIC
        src/rendering/amd_fsr3_framegen_probe.cpp
    )
    set_target_properties(wowee_fsr3_framegen_amd_vk_probe PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
    target_include_directories(wowee_fsr3_framegen_amd_vk_probe PUBLIC
        ${WOWEE_AMD_FFX_SDK_KITS_DIR}/upscalers/include
        ${WOWEE_AMD_FFX_SDK_KITS_DIR}/upscalers/fsr3/include
        ${WOWEE_AMD_FFX_SDK_KITS_DIR}/framegeneration/fsr3/include
        ${WOWEE_AMD_FFX_SDK_KITS_DIR}/framegeneration/include
        ${WOWEE_AMD_FFX_SDK_KITS_DIR}/backend/vk
        ${WOWEE_AMD_FFX_SDK_KITS_DIR}/api/internal
        ${WOWEE_AMD_FFX_SDK_KITS_DIR}/api/include
    )
    target_link_libraries(wowee_fsr3_framegen_amd_vk_probe PUBLIC Vulkan::Vulkan)

    if(WOWEE_BUILD_AMD_FSR3_RUNTIME)
        message(STATUS "AMD FSR3 Path A runtime target enabled: build with target 'wowee_fsr3_official_runtime_copy'")
        set(WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR ${CMAKE_BINARY_DIR}/ffx_sdk_runtime_build)
        if(WIN32)
            set(WOWEE_AMD_FSR3_RUNTIME_NAME amd_fidelityfx_vk.dll)
            if(CMAKE_CONFIGURATION_TYPES)
                set(WOWEE_AMD_FSR3_RUNTIME_SRC ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}/$<CONFIG>/amd_fidelityfx_vk.dll)
            else()
                set(WOWEE_AMD_FSR3_RUNTIME_SRC ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}/amd_fidelityfx_vk.dll)
            endif()
        elseif(APPLE)
            set(WOWEE_AMD_FSR3_RUNTIME_NAME libamd_fidelityfx_vk.dylib)
            if(CMAKE_CONFIGURATION_TYPES)
                set(WOWEE_AMD_FSR3_RUNTIME_SRC ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}/$<CONFIG>/libamd_fidelityfx_vk.dylib)
            else()
                set(WOWEE_AMD_FSR3_RUNTIME_SRC ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}/libamd_fidelityfx_vk.dylib)
            endif()
        else()
            set(WOWEE_AMD_FSR3_RUNTIME_NAME libamd_fidelityfx_vk.so)
            set(WOWEE_AMD_FSR3_RUNTIME_SRC ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}/libamd_fidelityfx_vk.so)
        endif()

        if(CMAKE_BUILD_TYPE)
            set(WOWEE_AMD_FSR3_RUNTIME_BUILD_TYPE ${CMAKE_BUILD_TYPE})
        else()
            set(WOWEE_AMD_FSR3_RUNTIME_BUILD_TYPE Release)
        endif()

        # Locate bash at configure time so the build-time COMMAND works on Windows
        # (cmake custom commands run via cmd.exe on Windows, so bare 'bash' is not found).
        find_program(BASH_EXECUTABLE bash
            HINTS
                /usr/bin
                /bin
                "${MSYS2_PATH}/usr/bin"
                "$ENV{MSYS2_PATH}/usr/bin"
                "C:/msys64/usr/bin"
                "D:/msys64/usr/bin"
        )
        if(BASH_EXECUTABLE)
            add_custom_target(wowee_fsr3_official_runtime_build
                COMMAND ${CMAKE_COMMAND}
                    -S ${WOWEE_AMD_FFX_SDK_KITS_DIR}
                    -B ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}
                    -DCMAKE_BUILD_TYPE=${WOWEE_AMD_FSR3_RUNTIME_BUILD_TYPE}
                    -DFFX_BUILD_VK=ON
                    -DFFX_BUILD_FRAMEGENERATION=ON
                    -DFFX_BUILD_UPSCALER=ON
                COMMAND ${BASH_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/generate_ffx_sdk_vk_permutations.sh
                    ${CMAKE_SOURCE_DIR}/extern/FidelityFX-SDK
                COMMAND ${CMAKE_COMMAND}
                    --build ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}
                    --config $<CONFIG>
                    --parallel
                COMMENT "Building native AMD FSR3 runtime (Path A) from FidelityFX-SDK Kits"
                VERBATIM
            )
        else()
            message(STATUS "bash not found; VK permutation headers will not be auto-generated")
            add_custom_target(wowee_fsr3_official_runtime_build
                COMMAND ${CMAKE_COMMAND}
                    -S ${WOWEE_AMD_FFX_SDK_KITS_DIR}
                    -B ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}
                    -DCMAKE_BUILD_TYPE=${WOWEE_AMD_FSR3_RUNTIME_BUILD_TYPE}
                    -DFFX_BUILD_VK=ON
                    -DFFX_BUILD_FRAMEGENERATION=ON
                    -DFFX_BUILD_UPSCALER=ON
                COMMAND ${CMAKE_COMMAND}
                    --build ${WOWEE_AMD_FSR3_RUNTIME_BUILD_DIR}
                    --config $<CONFIG>
                    --parallel
                COMMENT "Building native AMD FSR3 runtime (Path A) from FidelityFX-SDK Kits (no permutation bootstrap)"
                VERBATIM
            )
        endif()

        add_custom_target(wowee_fsr3_official_runtime_copy
            COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${WOWEE_AMD_FSR3_RUNTIME_SRC}
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${WOWEE_AMD_FSR3_RUNTIME_NAME}
            DEPENDS wowee_fsr3_official_runtime_build
            COMMENT "Copying native AMD FSR3 runtime to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
            VERBATIM
        )
    endif()

else()
    add_compile_definitions(WOWEE_HAS_AMD_FSR3_FRAMEGEN=0)
    add_compile_definitions(WOWEE_AMD_FFX_SDK_KITS=0)
    if(WOWEE_ENABLE_AMD_FSR3_FRAMEGEN)
        if(EXISTS ${WOWEE_AMD_FFX_SDK_KITS_FG_HEADER})
            message(STATUS "FidelityFX-SDK Kits layout detected at ${WOWEE_AMD_FFX_SDK_KITS_DIR}, but required FSR3 headers are incomplete. FSR3 framegen interface probe disabled.")
        else()
            message(WARNING "AMD FidelityFX-SDK Kits headers not found at ${WOWEE_AMD_FFX_SDK_KITS_DIR}; FSR3 framegen interface probe disabled.")
        endif()
    endif()
endif()

# Opcode registry generation/validation
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_custom_target(opcodes-generate
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/gen_opcode_registry.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating opcode registry include fragments"
    )

    add_custom_target(opcodes-validate
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/validate_opcode_maps.py --root ${CMAKE_SOURCE_DIR}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS opcodes-generate
        COMMENT "Validating canonical opcode registry and expansion maps"
    )
endif()

# Find required packages
find_package(SDL2 REQUIRED)
find_package(Vulkan QUIET)
if(NOT Vulkan_FOUND)
    # For Windows cross-compilation the host pkg-config finds the Linux libvulkan-dev
    # and injects /usr/include as an INTERFACE_INCLUDE_DIRECTORY, which causes
    # MinGW clang to pull in glibc headers (bits/libc-header-start.h) instead of
    # the MinGW sysroot headers.  Skip the host pkg-config path entirely and instead
    # locate Vulkan via vcpkg-installed vulkan-headers or the MinGW toolchain.
    if(CMAKE_CROSSCOMPILING AND WIN32)
        # The cross-compile build script generates a Vulkan import library
        # (libvulkan-1.a) in ${CMAKE_BINARY_DIR}/vulkan-import from the headers.
        set(_VULKAN_IMPORT_DIR "${CMAKE_BINARY_DIR}/vulkan-import")

        find_package(VulkanHeaders CONFIG QUIET)
        if(VulkanHeaders_FOUND)
            if(NOT TARGET Vulkan::Vulkan)
                add_library(Vulkan::Vulkan INTERFACE IMPORTED)
            endif()
            # Vulkan::Headers is provided by vcpkg's vulkan-headers port and carries
            # the correct MinGW include path — no Linux system headers involved.
            set_property(TARGET Vulkan::Vulkan APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Vulkan::Headers)
            # Link against the Vulkan loader import library (vulkan-1.dll).
            if(EXISTS "${_VULKAN_IMPORT_DIR}/libvulkan-1.a")
                set_property(TARGET Vulkan::Vulkan APPEND PROPERTY
                    INTERFACE_LINK_DIRECTORIES "${_VULKAN_IMPORT_DIR}")
                set_property(TARGET Vulkan::Vulkan APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES vulkan-1)
            endif()
            set(Vulkan_FOUND TRUE)
            message(STATUS "Found Vulkan headers for Windows cross-compile via vcpkg VulkanHeaders")
        else()
            # Last-resort: check the LLVM-MinGW toolchain sysroot directly.
            find_path(_VULKAN_MINGW_INCLUDE NAMES vulkan/vulkan.h
                PATHS /opt/llvm-mingw/x86_64-w64-mingw32/include NO_DEFAULT_PATH)
            if(_VULKAN_MINGW_INCLUDE)
                if(NOT TARGET Vulkan::Vulkan)
                    add_library(Vulkan::Vulkan INTERFACE IMPORTED)
                endif()
                set_target_properties(Vulkan::Vulkan PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES "${_VULKAN_MINGW_INCLUDE}")
                # Link against the Vulkan loader import library (vulkan-1.dll).
                if(EXISTS "${_VULKAN_IMPORT_DIR}/libvulkan-1.a")
                    set_property(TARGET Vulkan::Vulkan APPEND PROPERTY
                        INTERFACE_LINK_DIRECTORIES "${_VULKAN_IMPORT_DIR}")
                    set_property(TARGET Vulkan::Vulkan APPEND PROPERTY
                        INTERFACE_LINK_LIBRARIES vulkan-1)
                endif()
                set(Vulkan_FOUND TRUE)
                message(STATUS "Found Vulkan headers in LLVM-MinGW sysroot: ${_VULKAN_MINGW_INCLUDE}")
            endif()
        endif()
    elseif(CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        # macOS cross-compilation: use vcpkg-installed vulkan-headers.
        # The host pkg-config would find Linux libvulkan-dev headers which the
        # macOS cross-compiler cannot use (different sysroot).
        find_package(VulkanHeaders CONFIG QUIET)
        if(VulkanHeaders_FOUND)
            if(NOT TARGET Vulkan::Vulkan)
                add_library(Vulkan::Vulkan INTERFACE IMPORTED)
            endif()
            set_property(TARGET Vulkan::Vulkan APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Vulkan::Headers)
            set(Vulkan_FOUND TRUE)
            message(STATUS "Found Vulkan headers for macOS cross-compile via vcpkg VulkanHeaders")
        endif()
    else()
        # Fallback: some distros / CMake versions need pkg-config to locate Vulkan.
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(VULKAN_PKG vulkan)
            if(VULKAN_PKG_FOUND)
                add_library(Vulkan::Vulkan INTERFACE IMPORTED)
                set_target_properties(Vulkan::Vulkan PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES "${VULKAN_PKG_INCLUDE_DIRS}"
                    INTERFACE_LINK_LIBRARIES "${VULKAN_PKG_LIBRARIES}"
                )
                if(VULKAN_PKG_LIBRARY_DIRS)
                    set_property(TARGET Vulkan::Vulkan APPEND PROPERTY
                        INTERFACE_LINK_DIRECTORIES "${VULKAN_PKG_LIBRARY_DIRS}")
                endif()
                set(Vulkan_FOUND TRUE)
                message(STATUS "Found Vulkan via pkg-config: ${VULKAN_PKG_LIBRARIES}")
            endif()
        endif()
    endif()
    if(NOT Vulkan_FOUND)
        message(FATAL_ERROR "Could not find Vulkan. Install libvulkan-dev (Linux), vulkan-loader (macOS), or the Vulkan SDK (Windows).")
    endif()
endif()
# macOS cross-compilation: the Vulkan loader (MoltenVK) is not available at link
# time. Allow unresolved Vulkan symbols — they are resolved at runtime.
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(WOWEE_MACOS_CROSS_COMPILE TRUE)
endif()
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
if(WIN32)
    find_package(PkgConfig QUIET)
else()
    find_package(PkgConfig REQUIRED)
endif()
if(PkgConfig_FOUND)
    pkg_check_modules(FFMPEG libavformat libavcodec libswscale libavutil)
else()
    # Fallback for MSVC/vcpkg — find FFmpeg libraries manually
    find_path(FFMPEG_INCLUDE_DIRS libavformat/avformat.h)
    find_library(AVFORMAT_LIB NAMES avformat)
    find_library(AVCODEC_LIB  NAMES avcodec)
    find_library(AVUTIL_LIB   NAMES avutil)
    find_library(SWSCALE_LIB  NAMES swscale)
    set(FFMPEG_LIBRARIES ${AVFORMAT_LIB} ${AVCODEC_LIB} ${AVUTIL_LIB} ${SWSCALE_LIB})
endif()
if(FFMPEG_INCLUDE_DIRS AND AVFORMAT_LIB)
    set(HAVE_FFMPEG TRUE)
    message(STATUS "Found FFmpeg: ${AVFORMAT_LIB}")
elseif(FFMPEG_FOUND)
    set(HAVE_FFMPEG TRUE)
else()
    set(HAVE_FFMPEG FALSE)
    message(WARNING "FFmpeg not found — video_player will be disabled. Install via vcpkg: ffmpeg:x64-windows")
endif()

# Unicorn Engine (x86 emulator for cross-platform Warden module execution)
find_library(UNICORN_LIBRARY NAMES unicorn)
find_path(UNICORN_INCLUDE_DIR unicorn/unicorn.h)
if(NOT UNICORN_LIBRARY OR NOT UNICORN_INCLUDE_DIR)
    message(WARNING "Unicorn Engine not found. Install with: sudo apt-get install libunicorn-dev")
    message(WARNING "Warden emulation will be disabled")
    set(HAVE_UNICORN FALSE)
else()
    message(STATUS "Found Unicorn Engine: ${UNICORN_LIBRARY}")
    set(HAVE_UNICORN TRUE)
endif()

# GLM (header-only math library)
find_package(glm QUIET)
if(NOT glm_FOUND)
    message(STATUS "GLM not found, will use system includes or download")
endif()
# GLM GTX extensions (quaternion, norm, etc.) require this flag on newer GLM versions
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(WIN32)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# SPIR-V shader compilation via glslc
find_program(GLSLC glslc HINTS ${Vulkan_GLSLC_EXECUTABLE} "$ENV{VULKAN_SDK}/bin")
if(GLSLC)
    message(STATUS "Found glslc: ${GLSLC}")
else()
    message(WARNING "glslc not found. Install the Vulkan SDK or vulkan-tools package.")
    message(WARNING "Shaders will not be compiled to SPIR-V.")
endif()

# Function to compile GLSL shaders to SPIR-V
function(compile_shaders TARGET_NAME)
    set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders)
    # Compile in place, next to the GLSL. The .spv files are tracked (they are the
    # fallback when glslc is absent), and the POST_BUILD step below copies the whole
    # source assets/ tree over the runtime one — so compiling into the runtime tree
    # would just be overwritten by the committed .spv, silently running stale shaders.
    set(SPV_DIR ${SHADER_DIR})
    file(MAKE_DIRECTORY ${SPV_DIR})

    file(GLOB GLSL_SOURCES "${SHADER_DIR}/*.glsl")
    set(SPV_OUTPUTS)

    foreach(GLSL_FILE ${GLSL_SOURCES})
        get_filename_component(FILE_NAME ${GLSL_FILE} NAME)
        # e.g. skybox.vert.glsl -> skybox.vert.spv
        string(REGEX REPLACE "\\.glsl$" ".spv" SPV_NAME ${FILE_NAME})
        set(SPV_FILE ${SPV_DIR}/${SPV_NAME})

        # Determine shader stage from filename
        if(FILE_NAME MATCHES "\\.vert\\.glsl$")
            set(SHADER_STAGE vertex)
        elseif(FILE_NAME MATCHES "\\.frag\\.glsl$")
            set(SHADER_STAGE fragment)
        elseif(FILE_NAME MATCHES "\\.comp\\.glsl$")
            set(SHADER_STAGE compute)
        elseif(FILE_NAME MATCHES "\\.geom\\.glsl$")
            set(SHADER_STAGE geometry)
        else()
            message(WARNING "Cannot determine shader stage for: ${FILE_NAME}")
            continue()
        endif()

        add_custom_command(
            OUTPUT ${SPV_FILE}
            COMMAND ${GLSLC} -fshader-stage=${SHADER_STAGE} -O ${GLSL_FILE} -o ${SPV_FILE}
            DEPENDS ${GLSL_FILE}
            COMMENT "Compiling SPIR-V: ${FILE_NAME} -> ${SPV_NAME}"
            VERBATIM
        )
        list(APPEND SPV_OUTPUTS ${SPV_FILE})
    endforeach()

    add_custom_target(${TARGET_NAME}_shaders ALL DEPENDS ${SPV_OUTPUTS})
    add_dependencies(${TARGET_NAME} ${TARGET_NAME}_shaders)
endfunction()

# StormLib for MPQ extraction tool (not needed for main executable)
find_library(STORMLIB_LIBRARY NAMES StormLib stormlib storm)
find_path(STORMLIB_INCLUDE_DIR StormLib.h PATH_SUFFIXES StormLib)

# Include ImGui as a static library (Vulkan backend)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui)
if(EXISTS ${IMGUI_DIR})
    add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
        ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp
    )
    target_include_directories(imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC SDL2::SDL2 Vulkan::Vulkan ${CMAKE_DL_LIBS})
else()
    message(WARNING "ImGui not found in extern/imgui. Clone it with:")
    message(WARNING "  git clone https://github.com/ocornut/imgui.git extern/imgui")
endif()

# vk-bootstrap (Vulkan device/instance setup)
set(VK_BOOTSTRAP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/vk-bootstrap)
if(EXISTS ${VK_BOOTSTRAP_DIR})
    add_library(vk-bootstrap STATIC
        ${VK_BOOTSTRAP_DIR}/src/VkBootstrap.cpp
    )
    target_include_directories(vk-bootstrap PUBLIC ${VK_BOOTSTRAP_DIR}/src)
    target_link_libraries(vk-bootstrap PUBLIC Vulkan::Vulkan)
else()
    message(FATAL_ERROR "vk-bootstrap not found in extern/vk-bootstrap")
endif()

# Source files
set(WOWEE_SOURCES
    # Core
    src/core/application.cpp
    src/core/entity_spawner.cpp
    src/core/entity_spawner_player.cpp
    src/core/entity_spawner_processing.cpp
    src/core/appearance_composer.cpp
    src/core/helm_visual.cpp
    src/core/world_loader.cpp
    src/core/npc_interaction_callback_handler.cpp
    src/core/audio_callback_handler.cpp
    src/core/entity_spawn_callback_handler.cpp
    src/core/animation_callback_handler.cpp
    src/core/transport_callback_handler.cpp
    src/core/world_entry_callback_handler.cpp
    src/core/ui_screen_callback_handler.cpp
    src/core/window.cpp
    src/core/input.cpp
    src/core/logger.cpp
    src/core/memory_monitor.cpp
    src/core/app_clock.cpp
    src/core/config_paths.cpp

    # Math
    src/math/spline.cpp

    # Network
    src/network/socket.cpp
    src/network/packet.cpp
    src/network/tcp_socket.cpp
    src/network/world_socket.cpp

    # Auth
    src/auth/auth_handler.cpp
    src/auth/auth_opcodes.cpp
    src/auth/auth_packets.cpp
    src/auth/pin_auth.cpp
    src/auth/integrity.cpp
    src/auth/srp.cpp
    src/auth/big_num.cpp
    src/auth/crypto.cpp
    src/auth/rc4.cpp
    src/auth/vanilla_crypt.cpp

    # Game
    src/game/expansion_profile.cpp
    src/game/opcode_table.cpp
    src/game/update_field_table.cpp
    src/game/game_handler.cpp
    src/game/game_handler_packets.cpp
    src/game/game_handler_callbacks.cpp
    src/game/chat_handler.cpp
    src/game/movement_handler.cpp
    src/game/combat_handler.cpp
    src/game/spell_handler.cpp
    src/game/inventory_handler.cpp
    src/game/social_handler.cpp
    src/game/quest_handler.cpp
    src/game/entity_controller.cpp
    src/game/warden_handler.cpp
    src/game/warden_crypto.cpp
    src/game/warden_module.cpp
    src/game/warden_emulator.cpp
    src/game/warden_memory.cpp
    src/game/transport_manager.cpp
    src/game/transport_path_repository.cpp
    src/game/transport_clock_sync.cpp
    src/game/transport_animator.cpp
    src/game/world.cpp
    src/game/player.cpp
    src/game/entity.cpp
    src/game/opcodes.cpp
    src/game/world_packets.cpp
    src/game/world_packets_social.cpp
    src/game/world_packets_entity.cpp
    src/game/world_packets_world.cpp
    src/game/world_packets_economy.cpp
    src/game/spline_packet.cpp
    src/game/faction_hostility.cpp
    src/game/packet_parsers_tbc.cpp
    src/game/packet_parsers_classic.cpp
    src/game/character.cpp
    src/game/zone_manager.cpp
    src/game/inventory.cpp

    # Audio
    src/audio/audio_engine.cpp
    src/audio/audio_coordinator.cpp
    src/audio/music_manager.cpp
    src/audio/footstep_manager.cpp
    src/audio/activity_sound_manager.cpp
    src/audio/mount_sound_manager.cpp
    src/audio/npc_voice_manager.cpp
    src/audio/player_voice_manager.cpp
    src/audio/ambient_sound_manager.cpp
    src/audio/ui_sound_manager.cpp
    src/audio/combat_sound_manager.cpp
    src/audio/spell_sound_manager.cpp
    src/audio/movement_sound_manager.cpp

    # Pipeline (asset loaders)
    src/pipeline/blp_loader.cpp
    src/pipeline/dbc_loader.cpp
    src/pipeline/asset_manager.cpp
    src/pipeline/asset_manifest.cpp
    src/pipeline/loose_file_reader.cpp
    src/pipeline/m2_loader.cpp
    src/pipeline/wmo_loader.cpp
    src/pipeline/adt_loader.cpp
    src/pipeline/wdt_loader.cpp
    src/pipeline/wowee_terrain_loader.cpp
    src/pipeline/wowee_model.cpp
    src/pipeline/wowee_model_fromm2.cpp
    src/pipeline/wowee_building.cpp
    src/pipeline/wowee_collision.cpp
    src/pipeline/wowee_light.cpp
    src/pipeline/wowee_weather.cpp
    src/pipeline/wowee_world_map.cpp
    src/pipeline/wowee_sound.cpp
    src/pipeline/wowee_spawns.cpp
    src/pipeline/wowee_items.cpp
    src/pipeline/wowee_loot.cpp
    src/pipeline/wowee_creatures.cpp
    src/pipeline/wowee_quests.cpp
    src/pipeline/wowee_objects.cpp
    src/pipeline/wowee_factions.cpp
    src/pipeline/wowee_locks.cpp
    src/pipeline/wowee_skills.cpp
    src/pipeline/wowee_spells.cpp
    src/pipeline/wowee_achievements.cpp
    src/pipeline/wowee_trainers.cpp
    src/pipeline/wowee_gossip.cpp
    src/pipeline/wowee_taxi.cpp
    src/pipeline/wowee_talents.cpp
    src/pipeline/wowee_maps.cpp
    src/pipeline/wowee_chars.cpp
    src/pipeline/wowee_tokens.cpp
    src/pipeline/wowee_triggers.cpp
    src/pipeline/wowee_titles.cpp
    src/pipeline/wowee_events.cpp
    src/pipeline/wowee_mounts.cpp
    src/pipeline/wowee_battlegrounds.cpp
    src/pipeline/wowee_mail.cpp
    src/pipeline/wowee_gems.cpp
    src/pipeline/wowee_guilds.cpp
    src/pipeline/wowee_conditions.cpp
    src/pipeline/wowee_pets.cpp
    src/pipeline/wowee_auction.cpp
    src/pipeline/wowee_channels.cpp
    src/pipeline/wowee_cinematics.cpp
    src/pipeline/wowee_glyphs.cpp
    src/pipeline/wowee_vehicles.cpp
    src/pipeline/wowee_holidays.cpp
    src/pipeline/wowee_liquids.cpp
    src/pipeline/wowee_animations.cpp
    src/pipeline/wowee_spell_visuals.cpp
    src/pipeline/wowee_world_state_ui.cpp
    src/pipeline/wowee_player_conditions.cpp
    src/pipeline/wowee_trade_skills.cpp
    src/pipeline/wowee_creature_equipment.cpp
    src/pipeline/wowee_item_sets.cpp
    src/pipeline/wowee_game_tips.cpp
    src/pipeline/wowee_companions.cpp
    src/pipeline/wowee_spell_mechanics.cpp
    src/pipeline/wowee_keybindings.cpp
    src/pipeline/wowee_spell_schools.cpp
    src/pipeline/wowee_lfg.cpp
    src/pipeline/wowee_macros.cpp
    src/pipeline/wowee_char_features.cpp
    src/pipeline/wowee_pvp.cpp
    src/pipeline/wowee_bags.cpp
    src/pipeline/wowee_runes.cpp
    src/pipeline/wowee_loading_screens.cpp
    src/pipeline/wowee_item_suffixes.cpp
    src/pipeline/wowee_combat_ratings.cpp
    src/pipeline/wowee_unit_movement.cpp
    src/pipeline/wowee_quest_sorts.cpp
    src/pipeline/wowee_spell_ranges.cpp
    src/pipeline/wowee_spell_cast_times.cpp
    src/pipeline/wowee_spell_durations.cpp
    src/pipeline/wowee_spell_cooldowns.cpp
    src/pipeline/wowee_creature_families.cpp
    src/pipeline/wowee_spell_power_costs.cpp
    src/pipeline/wowee_glyph_slots.cpp
    src/pipeline/wowee_creature_difficulties.cpp
    src/pipeline/wowee_item_materials.cpp
    src/pipeline/wowee_player_spawn_profiles.cpp
    src/pipeline/wowee_talent_tabs.cpp
    src/pipeline/wowee_currency_types.cpp
    src/pipeline/wowee_spell_reagents.cpp
    src/pipeline/wowee_achievement_criteria.cpp
    src/pipeline/wowee_spell_effect_types.cpp
    src/pipeline/wowee_spell_aura_types.cpp
    src/pipeline/wowee_item_qualities.cpp
    src/pipeline/wowee_skill_costs.cpp
    src/pipeline/wowee_item_flags.cpp
    src/pipeline/wowee_npc_services.cpp
    src/pipeline/wowee_token_rewards.cpp
    src/pipeline/wowee_spell_procs.cpp
    src/pipeline/wowee_creature_patrols.cpp
    src/pipeline/wowee_boss_encounters.cpp
    src/pipeline/wowee_instance_lockouts.cpp
    src/pipeline/wowee_stable_slots.cpp
    src/pipeline/wowee_stat_curves.cpp
    src/pipeline/wowee_action_bars.cpp
    src/pipeline/wowee_group_compositions.cpp
    src/pipeline/wowee_hearth_binds.cpp
    src/pipeline/wowee_server_broadcasts.cpp
    src/pipeline/wowee_combat_maneuvers.cpp
    src/pipeline/wowee_realm_list.cpp
    src/pipeline/wowee_emotes.cpp
    src/pipeline/wowee_buff_book.cpp
    src/pipeline/wowee_tabards.cpp
    src/pipeline/wowee_spell_markers.cpp
    src/pipeline/wowee_learning_notifications.cpp
    src/pipeline/wowee_creature_resists.cpp
    src/pipeline/wowee_pet_talents.cpp
    src/pipeline/wowee_heroic_scaling.cpp
    src/pipeline/wowee_reputation_rewards.cpp
    src/pipeline/wowee_minimap_levels.cpp
    src/pipeline/wowee_pet_care.cpp
    src/pipeline/wowee_movie_credits.cpp
    src/pipeline/wowee_spell_variants.cpp
    src/pipeline/wowee_voiceovers.cpp
    src/pipeline/wowee_trade_rules.cpp
    src/pipeline/wowee_word_filters.cpp
    src/pipeline/wowee_raid_markers.cpp
    src/pipeline/wowee_loot_modes.cpp
    src/pipeline/wowee_sky_params.cpp
    src/pipeline/wowee_server_config.cpp
    src/pipeline/wowee_anniversary_events.cpp
    src/pipeline/wowee_pvp_ranks.cpp
    src/pipeline/wowee_localization.cpp
    src/pipeline/wowee_global_channels.cpp
    src/pipeline/wowee_addon_manifest.cpp
    src/pipeline/wowee_spell_pack.cpp
    src/pipeline/wowee_player_movement_anim.cpp
    src/pipeline/wowee_transit_schedule.cpp
    src/pipeline/wowee_mage_portals.cpp
    src/pipeline/wowee_combat_stats.cpp
    src/pipeline/wowee_guild_bank.cpp
    src/pipeline/wowee_quest_graph.cpp
    src/pipeline/wowee_crafting_recipes.cpp
    src/pipeline/wowee_world_locations.cpp
    src/pipeline/wowee_soulbind_rules.cpp
    src/pipeline/wowee_creature_behavior.cpp
    src/pipeline/wowee_random_property.cpp
    src/pipeline/wowee_spell_proc_rules.cpp
    src/pipeline/wowee_auction_houses.cpp
    src/pipeline/wowee_battleground_rewards.cpp
    src/pipeline/wowee_sound_swap.cpp
    src/pipeline/wowee_tutorial_steps.cpp
    src/pipeline/wowee_chat_commands.cpp
    src/pipeline/wowee_camera_presets.cpp
    src/pipeline/wowee_combat_formulas.cpp
    src/pipeline/wowee_chat_links.cpp
    src/pipeline/custom_zone_discovery.cpp
    src/pipeline/dbc_layout.cpp

    src/pipeline/terrain_mesh.cpp

    # Rendering (Vulkan infrastructure)
    src/rendering/vk_context.cpp
    src/rendering/vk_utils.cpp
    src/rendering/vk_shader.cpp
    src/rendering/vk_texture.cpp
    src/rendering/vk_buffer.cpp
    src/rendering/vk_pipeline.cpp
    src/rendering/vk_render_target.cpp

    # Rendering
    src/rendering/renderer.cpp
    src/rendering/amd_fsr3_runtime.cpp
    src/rendering/camera.cpp
    src/rendering/camera_controller.cpp
    src/rendering/material.cpp
    src/rendering/terrain_renderer.cpp
    src/rendering/terrain_manager.cpp
    src/rendering/frustum.cpp
    src/rendering/performance_hud.cpp
    src/rendering/water_renderer.cpp
    src/rendering/skybox.cpp
    src/rendering/celestial.cpp
    src/rendering/starfield.cpp
    src/rendering/clouds.cpp
    src/rendering/lens_flare.cpp
    src/rendering/weather.cpp
    src/rendering/lightning.cpp
    src/rendering/lighting_manager.cpp
    src/rendering/zone_ambience.cpp
    src/rendering/sky_system.cpp
    src/rendering/character_renderer.cpp
    src/rendering/character_preview.cpp
    src/rendering/wmo_renderer.cpp
    src/rendering/m2_renderer.cpp
    src/rendering/m2_renderer_render.cpp
    src/rendering/m2_renderer_particles.cpp
    src/rendering/m2_renderer_instance.cpp
    src/rendering/m2_model_classifier.cpp
    src/rendering/render_graph.cpp
    src/rendering/hiz_system.cpp
    src/rendering/quest_marker_renderer.cpp
    src/rendering/footprint_renderer.cpp
    src/rendering/minimap.cpp
    src/rendering/world_map/coordinate_projection.cpp
    src/rendering/world_map/map_resolver.cpp
    src/rendering/world_map/exploration_state.cpp
    src/rendering/world_map/zone_metadata.cpp
    src/rendering/world_map/data_repository.cpp
    src/rendering/world_map/view_state_machine.cpp
    src/rendering/world_map/composite_renderer.cpp
    src/rendering/world_map/overlay_renderer.cpp
    src/rendering/world_map/input_handler.cpp
    src/rendering/world_map/world_map_facade.cpp
    src/rendering/world_map/layers/player_marker_layer.cpp
    src/rendering/world_map/layers/party_dot_layer.cpp
    src/rendering/world_map/layers/taxi_node_layer.cpp
    src/rendering/world_map/layers/poi_marker_layer.cpp
    src/rendering/world_map/layers/quest_poi_layer.cpp
    src/rendering/world_map/layers/corpse_marker_layer.cpp
    src/rendering/world_map/layers/rare_tracker_layer.cpp
    src/rendering/world_map/layers/chest_tracker_layer.cpp
    src/rendering/world_map/layers/zone_highlight_layer.cpp
    src/rendering/world_map/layers/coordinate_display.cpp
    src/rendering/world_map/layers/subzone_tooltip_layer.cpp
    src/rendering/swim_effects.cpp
    src/rendering/mount_dust.cpp
    src/rendering/levelup_effect.cpp
    src/rendering/charge_effect.cpp
    src/rendering/spell_visual_system.cpp
    src/rendering/post_process_pipeline.cpp
    src/rendering/overlay_system.cpp
    src/rendering/animation_controller.cpp
    src/rendering/animation/animation_ids.cpp
    src/rendering/animation/emote_registry.cpp
    src/rendering/animation/footstep_driver.cpp
    src/rendering/animation/sfx_state_driver.cpp
    src/rendering/animation/anim_capability_probe.cpp
    src/rendering/animation/locomotion_fsm.cpp
    src/rendering/animation/combat_fsm.cpp
    src/rendering/animation/activity_fsm.cpp
    src/rendering/animation/mount_fsm.cpp
    src/rendering/animation/character_animator.cpp  # Renamed from player_animator.cpp; npc_animator.cpp removed
    src/rendering/animation/animation_manager.cpp
    src/rendering/loading_screen.cpp

    # UI
    src/ui/ui_manager.cpp
    src/ui/widget_tree.cpp
    src/ui/xml_parser.cpp
    src/ui/framexml_emitter.cpp
    src/ui/interface_fonts.cpp
    src/ui/widget_renderer.cpp
    src/ui/ui_raid_icons.cpp
    src/ui/auth_screen.cpp
    src/ui/realm_screen.cpp
    src/ui/character_create_screen.cpp
    src/ui/character_screen.cpp
    src/ui/game_screen.cpp
    src/ui/scene_pick.cpp
    src/ui/game_screen_frames.cpp
    src/ui/game_screen_hud.cpp
    src/ui/game_screen_minimap.cpp
    src/ui/chat_panel.cpp
    src/ui/chat/chat_settings.cpp
    src/ui/chat/chat_input.cpp
    src/ui/chat/chat_utils.cpp
    src/ui/chat/chat_tab_manager.cpp
    src/ui/chat/chat_bubble_manager.cpp
    src/ui/chat/chat_markup_parser.cpp
    src/ui/chat/chat_markup_renderer.cpp
    src/ui/chat/item_tooltip_renderer.cpp
    src/ui/chat/chat_command_registry.cpp
    src/ui/chat/chat_tab_completer.cpp
    src/ui/chat/macro_evaluator.cpp
    src/ui/chat/macro_eval_convenience.cpp
    src/ui/chat/game_state_adapter.cpp
    src/ui/chat/input_modifier_adapter.cpp
    src/ui/chat/commands/system_commands.cpp
    src/ui/chat/commands/social_commands.cpp
    src/ui/chat/commands/channel_commands.cpp
    src/ui/chat/commands/combat_commands.cpp
    src/ui/chat/commands/group_commands.cpp
    src/ui/chat/commands/guild_commands.cpp
    src/ui/chat/commands/target_commands.cpp
    src/ui/chat/commands/emote_commands.cpp
    src/ui/chat/commands/misc_commands.cpp
    src/ui/chat/commands/help_commands.cpp
    src/ui/chat/commands/gm_commands.cpp
    src/ui/toast_manager.cpp
    src/ui/dialog_manager.cpp
    src/ui/settings_panel.cpp
    src/ui/combat_ui.cpp
    src/ui/social_panel.cpp
    src/ui/action_bar_panel.cpp
    src/ui/window_manager.cpp
    src/ui/crafting_window.cpp
    src/ui/gm_command_screen.cpp
    src/ui/bis_gear_data.cpp
    src/ui/inventory_screen.cpp
    src/ui/quest_log_screen.cpp
    src/ui/spellbook_screen.cpp
    src/ui/talent_screen.cpp
    src/ui/keybinding_manager.cpp

    # Addons
    src/addons/addon_manager.cpp
    src/addons/lua_engine.cpp
    src/addons/lua_unit_api.cpp
    src/addons/lua_spell_api.cpp
    src/addons/lua_inventory_api.cpp
    src/addons/lua_quest_api.cpp
    src/addons/lua_social_api.cpp
    src/addons/lua_system_api.cpp
    src/addons/lua_action_api.cpp
    src/addons/toc_parser.cpp

    # Main
    src/main.cpp
)

set(WOWEE_HEADERS
    include/core/application.hpp
    include/core/window.hpp
    include/core/input.hpp
    include/core/logger.hpp

    include/network/socket.hpp
    include/network/packet.hpp
    include/network/tcp_socket.hpp
    include/network/world_socket.hpp
    include/network/net_platform.hpp

    include/platform/process.hpp

    include/auth/auth_handler.hpp
    include/auth/auth_opcodes.hpp
    include/auth/auth_packets.hpp
    include/auth/srp.hpp
    include/auth/big_num.hpp
    include/auth/crypto.hpp

    include/game/game_handler.hpp
    include/game/world.hpp
    include/game/player.hpp
    include/game/entity.hpp
    include/game/opcodes.hpp
    include/game/zone_manager.hpp
    include/game/inventory.hpp
    include/game/spell_defines.hpp
    include/game/group_defines.hpp
    include/game/world_packets.hpp
    include/game/character.hpp

    include/audio/audio_engine.hpp
    include/audio/music_manager.hpp
    include/audio/footstep_manager.hpp
    include/audio/activity_sound_manager.hpp
    include/audio/mount_sound_manager.hpp
    include/audio/npc_voice_manager.hpp
    include/audio/player_voice_manager.hpp
    include/audio/ambient_sound_manager.hpp
    include/audio/ui_sound_manager.hpp
    include/audio/combat_sound_manager.hpp
    include/audio/spell_sound_manager.hpp
    include/audio/movement_sound_manager.hpp

    include/pipeline/blp_loader.hpp
    include/pipeline/asset_manifest.hpp
    include/pipeline/loose_file_reader.hpp
    include/pipeline/m2_loader.hpp
    include/pipeline/wmo_loader.hpp
    include/pipeline/adt_loader.hpp
    include/pipeline/wdt_loader.hpp
    include/pipeline/dbc_loader.hpp
    include/pipeline/terrain_mesh.hpp

    include/rendering/vk_context.hpp
    include/rendering/vk_utils.hpp
    include/rendering/vk_shader.hpp
    include/rendering/vk_texture.hpp
    include/rendering/vk_buffer.hpp
    include/rendering/vk_pipeline.hpp
    include/rendering/vk_render_target.hpp
    include/rendering/renderer.hpp
    include/rendering/camera.hpp
    include/rendering/camera_controller.hpp
    include/rendering/material.hpp
    include/rendering/terrain_renderer.hpp
    include/rendering/terrain_manager.hpp
    include/rendering/frustum.hpp
    include/rendering/performance_hud.hpp
    include/rendering/water_renderer.hpp
    include/rendering/skybox.hpp
    include/rendering/celestial.hpp
    include/rendering/starfield.hpp
    include/rendering/clouds.hpp
    include/rendering/lens_flare.hpp
    include/rendering/weather.hpp
    include/rendering/lightning.hpp
    include/rendering/swim_effects.hpp
    include/rendering/world_map.hpp
    include/rendering/world_map/world_map_types.hpp
    include/rendering/world_map/coordinate_projection.hpp
    include/rendering/world_map/map_resolver.hpp
    include/rendering/world_map/exploration_state.hpp
    include/rendering/world_map/zone_metadata.hpp
    include/rendering/world_map/data_repository.hpp
    include/rendering/world_map/view_state_machine.hpp
    include/rendering/world_map/composite_renderer.hpp
    include/rendering/world_map/overlay_renderer.hpp
    include/rendering/world_map/input_handler.hpp
    include/rendering/world_map/world_map_facade.hpp
    include/rendering/world_map/layers/player_marker_layer.hpp
    include/rendering/world_map/layers/party_dot_layer.hpp
    include/rendering/world_map/layers/taxi_node_layer.hpp
    include/rendering/world_map/layers/poi_marker_layer.hpp
    include/rendering/world_map/layers/quest_poi_layer.hpp
    include/rendering/world_map/layers/corpse_marker_layer.hpp
    include/rendering/world_map/layers/rare_tracker_layer.hpp
    include/rendering/world_map/layers/chest_tracker_layer.hpp
    include/rendering/world_map/layers/zone_highlight_layer.hpp
    include/rendering/world_map/layers/coordinate_display.hpp
    include/rendering/world_map/layers/subzone_tooltip_layer.hpp
    include/rendering/character_renderer.hpp
    include/rendering/character_preview.hpp
    include/rendering/wmo_renderer.hpp
    include/rendering/loading_screen.hpp

    include/ui/ui_manager.hpp
    include/ui/auth_screen.hpp
    include/ui/realm_screen.hpp
    include/ui/character_create_screen.hpp
    include/ui/character_screen.hpp
    include/ui/game_screen.hpp
    include/ui/inventory_screen.hpp
    include/ui/spellbook_screen.hpp
    include/ui/talent_screen.hpp
    include/ui/keybinding_manager.hpp
)

# macOS needs an Objective-C++ shim for AppKit behavior that has no SDL
# equivalent (press-and-hold accents).  window.cpp calls into it, so every
# target that compiles window.cpp must link it.
set(WOWEE_MACOS_SOURCES)
if(APPLE)
    enable_language(OBJCXX)
    set(WOWEE_MACOS_SOURCES src/core/macos_platform.mm)
endif()

set(WOWEE_PLATFORM_SOURCES)
if(WIN32)
    # Copy icon into build tree so windres can find it via the relative path
    # in wowee.rc ("assets\\wowee.ico").  Tell the RC compiler to also search
    # the build directory — GNU windres uses cwd (already the build dir) but
    # llvm-windres resolves relative to the .rc file, so it needs the hint.
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/assets/Wowee.ico
        ${CMAKE_CURRENT_BINARY_DIR}/assets/wowee.ico
        COPYONLY
    )
    set(CMAKE_RC_FLAGS "${CMAKE_RC_FLAGS} -I ${CMAKE_CURRENT_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR}")
    list(APPEND WOWEE_PLATFORM_SOURCES resources/wowee.rc)
endif()

# ---- Lua 5.1 ----
#
# Vendored by default and system-provided on request, and the target the rest of
# this file links against is called lua51 either way.
#
# Off by default rather than "use it if it happens to be installed": which Lua a
# build linked would otherwise depend on what was lying around on the machine,
# and the interface this client runs is a large body of Lua whose behaviour is
# the thing under test. A distribution packaging this wants the system copy and
# can say so; everyone else gets the same interpreter as CI.
#
# 5.1 exactly. The addon and FrameXML code this runs is written for it — 5.2
# removed setfenv and the arg table, 5.3 changed integer division and string
# coercion — so a 5.4 found under the name "Lua" is not a substitute, and a
# wrong-version link fails at configure time rather than at the first addon.
#
# The check is not redundant with the version given to find_package: that is a
# minimum, and CMake's own FindLua reports a 5.4 install as satisfying it. Asked
# for 5.1 on a machine carrying only 5.4, find_package succeeds and says so.
if(WOWEE_SYSTEM_LUA)
    find_package(Lua 5.1 REQUIRED)
    if(NOT LUA_VERSION_STRING MATCHES "^5\\.1")
        message(FATAL_ERROR
            "WOWEE_SYSTEM_LUA is on and the Lua found is ${LUA_VERSION_STRING}. "
            "This client needs 5.1 (Debian and Ubuntu call it liblua5.1-0-dev), "
            "or turn the option off to build the vendored copy.")
    endif()
    add_library(lua51 INTERFACE)
    target_include_directories(lua51 SYSTEM INTERFACE ${LUA_INCLUDE_DIR})
    target_link_libraries(lua51 INTERFACE ${LUA_LIBRARIES})
    message(STATUS "Lua: system ${LUA_VERSION_STRING} (${LUA_LIBRARIES})")
else()

# ---- Lua 5.1.5 (vendored, static library) ----
set(LUA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/lua-5.1.5/src)
set(LUA_SOURCES
    ${LUA_DIR}/lapi.c    ${LUA_DIR}/lcode.c   ${LUA_DIR}/ldebug.c
    ${LUA_DIR}/ldo.c     ${LUA_DIR}/ldump.c   ${LUA_DIR}/lfunc.c
    ${LUA_DIR}/lgc.c     ${LUA_DIR}/llex.c    ${LUA_DIR}/lmem.c
    ${LUA_DIR}/lobject.c ${LUA_DIR}/lopcodes.c ${LUA_DIR}/lparser.c
    ${LUA_DIR}/lstate.c  ${LUA_DIR}/lstring.c ${LUA_DIR}/ltable.c
    ${LUA_DIR}/ltm.c     ${LUA_DIR}/lundump.c ${LUA_DIR}/lvm.c
    ${LUA_DIR}/lzio.c    ${LUA_DIR}/lauxlib.c ${LUA_DIR}/lbaselib.c
    ${LUA_DIR}/ldblib.c  ${LUA_DIR}/liolib.c  ${LUA_DIR}/lmathlib.c
    ${LUA_DIR}/loslib.c  ${LUA_DIR}/ltablib.c ${LUA_DIR}/lstrlib.c
    ${LUA_DIR}/linit.c
)
add_library(lua51 STATIC ${LUA_SOURCES})
set_target_properties(lua51 PROPERTIES LINKER_LANGUAGE C C_STANDARD 99 POSITION_INDEPENDENT_CODE ON)
target_include_directories(lua51 PUBLIC ${LUA_DIR})
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lua51 PRIVATE -w)
endif()
message(STATUS "Lua: vendored 5.1.5 (set WOWEE_SYSTEM_LUA=ON to link an installed one)")

endif()

# Create executable
add_executable(wowee ${WOWEE_SOURCES} ${WOWEE_HEADERS} ${WOWEE_PLATFORM_SOURCES} ${WOWEE_MACOS_SOURCES})

# Tracy profiler — zero overhead when WOWEE_ENABLE_TRACY is OFF
if(WOWEE_ENABLE_TRACY)
    target_sources(wowee PRIVATE ${CMAKE_SOURCE_DIR}/extern/tracy/public/TracyClient.cpp)
    target_compile_definitions(wowee PRIVATE TRACY_ENABLE)
    target_include_directories(wowee SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/extern/tracy/public)
    message(STATUS "Tracy profiler: ENABLED")
endif()

if(TARGET opcodes-generate)
    add_dependencies(wowee opcodes-generate)
endif()
# macOS cross-compilation: MoltenVK is not available at link time.
# Allow unresolved Vulkan symbols — resolved at runtime. Scoped to wowee only.
if(WOWEE_MACOS_CROSS_COMPILE)
    target_link_options(wowee PRIVATE "-undefined" "dynamic_lookup")
endif()

# FidelityFX-SDK headers can trigger compiler-specific pragma/unused-static noise
# when included through the runtime bridge; keep suppression scoped to that TU.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set_source_files_properties(src/rendering/amd_fsr3_runtime.cpp PROPERTIES
        COMPILE_OPTIONS "-Wno-unknown-pragmas;-Wno-unused-variable"
    )
endif()

# Compile GLSL shaders to SPIR-V
if(GLSLC)
    compile_shaders(wowee)
endif()

# Version header generated from `git describe`. Regenerated on every build (the
# script rewrites the header only when the version changed), so tagging a release
# is enough to update the version shown in the client.
set(WOWEE_VERSION_HEADER ${CMAKE_BINARY_DIR}/generated/core/version.hpp)
set(WOWEE_VERSION_SCRIPT_ARGS
    -DSRC_DIR=${CMAKE_SOURCE_DIR}
    -DIN_FILE=${CMAKE_SOURCE_DIR}/include/core/version.hpp.in
    -DOUT_FILE=${WOWEE_VERSION_HEADER}
    -P ${CMAKE_SOURCE_DIR}/cmake/GitVersion.cmake
)
# Once now, so the header exists before the first compile.
execute_process(COMMAND ${CMAKE_COMMAND} ${WOWEE_VERSION_SCRIPT_ARGS})
add_custom_target(wowee_version ALL
    COMMAND ${CMAKE_COMMAND} ${WOWEE_VERSION_SCRIPT_ARGS}
    BYPRODUCTS ${WOWEE_VERSION_HEADER}
    COMMENT "Resolving version from git"
    VERBATIM
)
add_dependencies(wowee wowee_version)

# Include directories
target_include_directories(wowee PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/generated
)
# Vendored headers as SYSTEM to suppress third-party warnings
target_include_directories(wowee SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/vk-bootstrap/src
)
if(HAVE_FFMPEG)
    target_include_directories(wowee PRIVATE ${FFMPEG_INCLUDE_DIRS})
endif()

# Link libraries
target_link_libraries(wowee PRIVATE
    SDL2::SDL2
    Vulkan::Vulkan
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
    ZLIB::ZLIB
    lua51
    ${CMAKE_DL_LIBS}
)

if(HAVE_FFMPEG)
    target_compile_definitions(wowee PRIVATE HAVE_FFMPEG)
    target_link_libraries(wowee PRIVATE ${FFMPEG_LIBRARIES})
    if(FFMPEG_LIBRARY_DIRS)
        target_link_directories(wowee PRIVATE ${FFMPEG_LIBRARY_DIRS})
    endif()
endif()

# Platform-specific libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(wowee PRIVATE X11)
endif()
if(APPLE)
    target_link_libraries(wowee PRIVATE "-framework Foundation")
endif()
if(WIN32)
    target_link_libraries(wowee PRIVATE ws2_32)
    # SDL2main provides WinMain entry point on Windows
    if(TARGET SDL2::SDL2main)
        target_link_libraries(wowee PRIVATE SDL2::SDL2main)
    endif()
endif()

# Link ImGui if available
if(TARGET imgui)
    target_link_libraries(wowee PRIVATE imgui)
endif()

# Link vk-bootstrap
if(TARGET vk-bootstrap)
    target_link_libraries(wowee PRIVATE vk-bootstrap)
endif()

if(TARGET wowee_fsr2_amd_vk)
    target_link_libraries(wowee PRIVATE wowee_fsr2_amd_vk)
endif()
if(TARGET wowee_fsr3_framegen_amd_vk_probe)
    target_link_libraries(wowee PRIVATE wowee_fsr3_framegen_amd_vk_probe)
endif()
if(TARGET wowee_fsr3_official_runtime_copy)
    # FSR3 Path A runtime is an opt-in artifact; build explicitly with:
    #   cmake --build build --target wowee_fsr3_official_runtime_copy
    # Do NOT add as a hard dependency of wowee — it would break arm64 and Windows CI
    # (no DXC available on arm64; bash context issues on MSYS2 Windows).
endif()

# Link Unicorn if available
if(HAVE_UNICORN)
    target_link_libraries(wowee PRIVATE ${UNICORN_LIBRARY})
    target_include_directories(wowee PRIVATE ${UNICORN_INCLUDE_DIR})
    target_compile_definitions(wowee PRIVATE HAVE_UNICORN)
endif()

# Prefer GLM's explicit header-only target. GLM 1.0 may build glm::glm as a
# shared library, which creates an unnecessary runtime dependency in app bundles.
if(TARGET glm::glm-header-only)
    target_link_libraries(wowee PRIVATE glm::glm-header-only)
elseif(TARGET glm::glm)
    target_link_libraries(wowee PRIVATE glm::glm)
elseif(glm_FOUND)
    target_include_directories(wowee PRIVATE ${GLM_INCLUDE_DIRS})
endif()

# Compiler warnings
if(MSVC)
    target_compile_options(wowee PRIVATE /W4)
else()
    target_compile_options(wowee PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
    # GCC LTO emits -Wodr for FFX enum-name mismatches across SDK generations.
    # We intentionally keep FSR2+FSR3 integrations in separate TUs and suppress
    # this linker-time diagnostic to avoid CI noise.
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(wowee PRIVATE -Wno-odr)
        target_link_options(wowee PRIVATE -Wno-odr)
    endif()
endif()

# Debug build flags
if(MSVC)
    # /ZI  — Edit-and-Continue debug info (works with hot-reload in VS 2022)
    # /RTC1 — stack-frame and uninitialised-variable runtime checks (Debug only)
    # /sdl  — additional SDL security checks
    # /Od   — disable optimisation so stepping matches source lines exactly
    target_compile_options(wowee PRIVATE
        $<$<CONFIG:Debug>:/ZI /RTC1 /sdl /Od>
    )
    # Ensure the linker emits a .pdb alongside the .exe for every config
    target_link_options(wowee PRIVATE
        $<$<CONFIG:Debug>:/DEBUG:FULL>
        $<$<CONFIG:RelWithDebInfo>:/DEBUG:FASTLINK>
    )
else()
    # -g3          — maximum DWARF debug info (includes macro definitions)
    # -Og          — optimise for debugging (better than -O0, keeps most frames)
    # -fno-omit-frame-pointer — preserve frame pointers so stack traces are clean
    # Frame pointers in all configs (negligible perf cost, critical for crash backtraces)
    target_compile_options(wowee PRIVATE
        -fno-omit-frame-pointer
        $<$<CONFIG:Debug>:-g3 -Og>
        $<$<CONFIG:RelWithDebInfo>:-g>
    )
endif()

# ── Unit tests (Catch2) ──────────────────────────────────────
if(WOWEE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# AddressSanitizer — catch buffer overflows, use-after-free, etc.
# Enable with:  cmake ... -DWOWEE_ENABLE_ASAN=ON -DCMAKE_BUILD_TYPE=Debug
if(WOWEE_ENABLE_ASAN)
    if(MSVC)
        target_compile_options(wowee PRIVATE /fsanitize=address)
        # ASAN on MSVC requires the dynamic CRT (/MD or /MDd)
        target_compile_options(wowee PRIVATE
            $<$<CONFIG:Debug>:/MDd>
            $<$<CONFIG:Release>:/MD>
        )
    else()
        target_compile_options(wowee PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(wowee PRIVATE -fsanitize=address,undefined)
    endif()
    message(STATUS "AddressSanitizer + UBSan: ENABLED")
endif()

# Release build optimizations
include(CheckIPOSupported)
check_ipo_supported(RESULT _ipo_supported OUTPUT _ipo_error)
if(_ipo_supported)
    set_property(TARGET wowee PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()
if(NOT MSVC)
    # -O3: more aggressive inlining and auto-vectorization vs CMake's default -O2
    target_compile_options(wowee PRIVATE $<$<CONFIG:Release>:-O3>)
    # -fvisibility=hidden: keeps all symbols internal by default, shrinks binary
    # and gives the linker and optimizer more freedom to dead-strip and inline
    target_compile_options(wowee PRIVATE $<$<CONFIG:Release>:-fvisibility=hidden -fvisibility-inlines-hidden>)
endif()

# Copy assets next to the executable (runs every build, not just configure).
# Uses $<TARGET_FILE_DIR:wowee> so MSVC multi-config generators place assets
# in bin/Debug/ or bin/Release/ alongside the exe, not the common bin/ parent.
add_custom_command(TARGET wowee POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:wowee>/assets
    COMMENT "Syncing assets to $<TARGET_FILE_DIR:wowee>/assets"
)

# Symlink Data/ next to the executable so expansion profiles, opcode tables,
# and other runtime data files are found when running from the build directory.
if(NOT WIN32)
    add_custom_command(TARGET wowee POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink
            ${CMAKE_CURRENT_SOURCE_DIR}/Data
            $<TARGET_FILE_DIR:wowee>/Data
        COMMENT "Symlinking Data to $<TARGET_FILE_DIR:wowee>/Data"
    )
endif()

# On Windows, SDL 2.28+ uses LoadLibraryExW with LOAD_LIBRARY_SEARCH_DEFAULT_DIRS
# which does NOT include System32. Copy vulkan-1.dll into the output directory so
# SDL_Vulkan_LoadLibrary can locate it without needing a full system PATH search.
if(WIN32)
    find_file(VULKAN_DLL vulkan-1.dll
        PATHS "$ENV{SystemRoot}/System32" "$ENV{VULKAN_SDK}/Bin"
        NO_DEFAULT_PATH)
    if(VULKAN_DLL)
        add_custom_command(TARGET wowee POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${VULKAN_DLL}" "$<TARGET_FILE_DIR:wowee>/vulkan-1.dll"
            COMMENT "Copying vulkan-1.dll to output directory"
        )
    else()
        message(STATUS "  vulkan-1.dll not found — skipping copy (MSYS2 provides it via PATH)")
    endif()
endif()

# Install targets
install(TARGETS wowee
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Note: tool install rules are placed next to each target definition below.

# Install built-in assets (exclude proprietary music)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/assets
    DESTINATION bin
    PATTERN "Original Music" EXCLUDE)

# On Windows, install any DLLs that were bundled into the build output dir
# (populated by the CI workflow's DLL-bundling step before cpack runs)
if(WIN32)
    install(DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
        DESTINATION bin
        FILES_MATCHING PATTERN "*.dll")
endif()

# Linux desktop integration (launcher + icon)
if(UNIX AND NOT APPLE)
    set(WOWEE_LINUX_ICON_PATH "${CMAKE_INSTALL_FULL_DATAROOTDIR}/icons/hicolor/256x256/apps/wowee.png")
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/wowee.desktop.in
        ${CMAKE_CURRENT_BINARY_DIR}/wowee.desktop
        @ONLY
    )

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wowee.desktop
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/applications)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/assets/Wowee.png
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/icons/hicolor/256x256/apps
        RENAME wowee.png)
endif()

# ---- Tool: asset_extract (MPQ → loose files) ----
if(STORMLIB_LIBRARY AND STORMLIB_INCLUDE_DIR)
    add_executable(asset_extract
        tools/asset_extract/main.cpp
        tools/asset_extract/extractor.cpp
        tools/asset_extract/path_mapper.cpp
        tools/asset_extract/manifest_writer.cpp
        tools/asset_extract/open_format_emitter.cpp
        src/pipeline/dbc_loader.cpp
        src/pipeline/blp_loader.cpp
        src/pipeline/m2_loader.cpp
        src/pipeline/wmo_loader.cpp
        src/pipeline/adt_loader.cpp
        src/pipeline/wowee_model.cpp
        src/pipeline/wowee_building.cpp
        src/pipeline/wowee_collision.cpp
        src/core/logger.cpp
    )
    target_include_directories(asset_extract PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/asset_extract
        ${STORMLIB_INCLUDE_DIR}
    )
    target_include_directories(asset_extract SYSTEM PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/extern
    )
    target_link_libraries(asset_extract PRIVATE
        ${STORMLIB_LIBRARY}
        ZLIB::ZLIB
        Threads::Threads
    )
    if(WIN32)
        find_library(WININET_LIB wininet)
        find_library(BZ2_LIB bz2)
        find_library(TOMCRYPT_LIB NAMES tomcrypt libtomcrypt)
        find_library(TOMMATH_LIB NAMES tommath libtommath)
        if(WININET_LIB)
            target_link_libraries(asset_extract PRIVATE ${WININET_LIB})
        endif()
        if(BZ2_LIB)
            target_link_libraries(asset_extract PRIVATE ${BZ2_LIB})
        endif()
        if(TOMCRYPT_LIB AND TOMMATH_LIB)
            # Static StormLib exposes its LibTomCrypt/LibTomMath dependencies.
            target_link_libraries(asset_extract PRIVATE ${TOMCRYPT_LIB} ${TOMMATH_LIB})
        else()
            message(FATAL_ERROR
                "Static StormLib on Windows requires LibTomCrypt and LibTomMath")
        endif()
    endif()
    set_target_properties(asset_extract PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
    install(TARGETS asset_extract RUNTIME DESTINATION bin)
    message(STATUS "  asset_extract tool: ENABLED")
else()
    message(STATUS "  asset_extract tool: DISABLED (requires StormLib)")
endif()

# ---- Tool: dbc_to_csv (DBC → CSV text) ----
add_executable(dbc_to_csv
    tools/dbc_to_csv/main.cpp
    src/pipeline/dbc_loader.cpp
    src/core/logger.cpp
)
target_include_directories(dbc_to_csv PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
)
target_link_libraries(dbc_to_csv PRIVATE Threads::Threads)
set_target_properties(dbc_to_csv PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
install(TARGETS dbc_to_csv RUNTIME DESTINATION bin)

# ---- Tool: auth_probe (LOGON_CHALLENGE probe) ----
add_executable(auth_probe
    tools/auth_probe/main.cpp
    src/auth/auth_packets.cpp
    src/auth/auth_opcodes.cpp
    src/auth/crypto.cpp
    src/network/packet.cpp
    src/network/socket.cpp
    src/network/tcp_socket.cpp
    src/core/logger.cpp
)
target_include_directories(auth_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(auth_probe PRIVATE Threads::Threads OpenSSL::Crypto
    $<$<BOOL:${WIN32}>:ws2_32>)
set_target_properties(auth_probe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
install(TARGETS auth_probe RUNTIME DESTINATION bin)

# ---- Tool: auth_login_probe (challenge + proof probe) ----
add_executable(auth_login_probe
    tools/auth_login_probe/main.cpp
    src/auth/auth_packets.cpp
    src/auth/auth_opcodes.cpp
    src/auth/crypto.cpp
    src/auth/integrity.cpp
    src/auth/big_num.cpp
    src/auth/srp.cpp
    src/network/packet.cpp
    src/network/socket.cpp
    src/network/tcp_socket.cpp
    src/core/logger.cpp
)
target_include_directories(auth_login_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(auth_login_probe PRIVATE Threads::Threads OpenSSL::Crypto
    $<$<BOOL:${WIN32}>:ws2_32>)
set_target_properties(auth_login_probe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
install(TARGETS auth_login_probe RUNTIME DESTINATION bin)

# ---- Tool: blp_convert (BLP ↔ PNG) ----
add_executable(blp_convert
    tools/blp_convert/main.cpp
    src/pipeline/blp_loader.cpp
    src/core/logger.cpp
)
target_include_directories(blp_convert PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
)
target_link_libraries(blp_convert PRIVATE Threads::Threads)
set_target_properties(blp_convert PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
install(TARGETS blp_convert RUNTIME DESTINATION bin)

# ---- Tool: wowee_editor (Standalone World Editor) ----
add_executable(wowee_editor
    tools/editor/main.cpp
    tools/editor/cli_gen_audio.cpp
    tools/editor/cli_zone_packs.cpp
    tools/editor/cli_audits.cpp
    tools/editor/cli_readmes.cpp
    tools/editor/cli_zone_inventory.cpp
    tools/editor/cli_project_inventory.cpp
    tools/editor/cli_help.cpp
    tools/editor/cli_gen_texture.cpp
    tools/editor/cli_gen_mesh.cpp
    tools/editor/cli_mesh_io.cpp
    tools/editor/cli_mesh_edit.cpp
    tools/editor/cli_wom_info.cpp
    tools/editor/cli_format_validate.cpp
    tools/editor/cli_convert.cpp
    tools/editor/cli_format_info.cpp
    tools/editor/cli_pack.cpp
    tools/editor/cli_content_info.cpp
    tools/editor/cli_zone_info.cpp
    tools/editor/cli_data_tree.cpp
    tools/editor/cli_diff.cpp
    tools/editor/cli_spawn_audit.cpp
    tools/editor/cli_items.cpp
    tools/editor/cli_extract_info.cpp
    tools/editor/cli_export.cpp
    tools/editor/cli_bake.cpp
    tools/editor/cli_migrate.cpp
    tools/editor/cli_convert_single.cpp
    tools/editor/cli_validate_interop.cpp
    tools/editor/cli_glb_inspect.cpp
    tools/editor/cli_wom_io.cpp
    tools/editor/cli_world_io.cpp
    tools/editor/cli_info_tree.cpp
    tools/editor/cli_info_bytes.cpp
    tools/editor/cli_info_extents.cpp
    tools/editor/cli_info_water.cpp
    tools/editor/cli_info_density.cpp
    tools/editor/cli_info_audio.cpp
    tools/editor/cli_world_info.cpp
    tools/editor/cli_world_map.cpp
    tools/editor/cli_sound_catalog.cpp
    tools/editor/cli_spawns_catalog.cpp
    tools/editor/cli_items_catalog.cpp
    tools/editor/cli_loot_catalog.cpp
    tools/editor/cli_creatures_catalog.cpp
    tools/editor/cli_quests_catalog.cpp
    tools/editor/cli_objects_catalog.cpp
    tools/editor/cli_factions_catalog.cpp
    tools/editor/cli_locks_catalog.cpp
    tools/editor/cli_skills_catalog.cpp
    tools/editor/cli_spells_catalog.cpp
    tools/editor/cli_achievements_catalog.cpp
    tools/editor/cli_trainers_catalog.cpp
    tools/editor/cli_gossip_catalog.cpp
    tools/editor/cli_taxi_catalog.cpp
    tools/editor/cli_talents_catalog.cpp
    tools/editor/cli_maps_catalog.cpp
    tools/editor/cli_chars_catalog.cpp
    tools/editor/cli_tokens_catalog.cpp
    tools/editor/cli_triggers_catalog.cpp
    tools/editor/cli_titles_catalog.cpp
    tools/editor/cli_events_catalog.cpp
    tools/editor/cli_mounts_catalog.cpp
    tools/editor/cli_battlegrounds_catalog.cpp
    tools/editor/cli_mail_catalog.cpp
    tools/editor/cli_gems_catalog.cpp
    tools/editor/cli_guilds_catalog.cpp
    tools/editor/cli_conditions_catalog.cpp
    tools/editor/cli_pets_catalog.cpp
    tools/editor/cli_auction_catalog.cpp
    tools/editor/cli_channels_catalog.cpp
    tools/editor/cli_cinematics_catalog.cpp
    tools/editor/cli_glyphs_catalog.cpp
    tools/editor/cli_vehicles_catalog.cpp
    tools/editor/cli_holidays_catalog.cpp
    tools/editor/cli_liquids_catalog.cpp
    tools/editor/cli_list_formats.cpp
    tools/editor/cli_info_magic.cpp
    tools/editor/cli_animations_catalog.cpp
    tools/editor/cli_spell_visuals_catalog.cpp
    tools/editor/cli_format_table.cpp
    tools/editor/cli_summary_dir.cpp
    tools/editor/cli_rename_magic.cpp
    tools/editor/cli_world_state_ui_catalog.cpp
    tools/editor/cli_player_conditions_catalog.cpp
    tools/editor/cli_trade_skills_catalog.cpp
    tools/editor/cli_creature_equipment_catalog.cpp
    tools/editor/cli_item_sets_catalog.cpp
    tools/editor/cli_touch_tree.cpp
    tools/editor/cli_game_tips_catalog.cpp
    tools/editor/cli_companions_catalog.cpp
    tools/editor/cli_spell_mechanics_catalog.cpp
    tools/editor/cli_keybindings_catalog.cpp
    tools/editor/cli_tree_summary_md.cpp
    tools/editor/cli_spell_schools_catalog.cpp
    tools/editor/cli_lfg_catalog.cpp
    tools/editor/cli_catalog_grep.cpp
    tools/editor/cli_diff_headers.cpp
    tools/editor/cli_audit_tree.cpp
    tools/editor/cli_magic_fix.cpp
    tools/editor/cli_bulk_validate.cpp
    tools/editor/cli_bulk_json.cpp
    tools/editor/cli_diff_tree.cpp
    tools/editor/cli_orphan_jsons.cpp
    tools/editor/cli_list_by_magic.cpp
    tools/editor/cli_catalog_stats.cpp
    tools/editor/cli_macros_catalog.cpp
    tools/editor/cli_char_features_catalog.cpp
    tools/editor/cli_pvp_catalog.cpp
    tools/editor/cli_bags_catalog.cpp
    tools/editor/cli_runes_catalog.cpp
    tools/editor/cli_loading_screens_catalog.cpp
    tools/editor/cli_item_suffixes_catalog.cpp
    tools/editor/cli_combat_ratings_catalog.cpp
    tools/editor/cli_unit_movement_catalog.cpp
    tools/editor/cli_quest_sorts_catalog.cpp
    tools/editor/cli_spell_ranges_catalog.cpp
    tools/editor/cli_spell_cast_times_catalog.cpp
    tools/editor/cli_spell_durations_catalog.cpp
    tools/editor/cli_spell_cooldowns_catalog.cpp
    tools/editor/cli_creature_families_catalog.cpp
    tools/editor/cli_spell_power_costs_catalog.cpp
    tools/editor/cli_glyph_slots_catalog.cpp
    tools/editor/cli_creature_difficulties_catalog.cpp
    tools/editor/cli_item_materials_catalog.cpp
    tools/editor/cli_player_spawn_profiles_catalog.cpp
    tools/editor/cli_talent_tabs_catalog.cpp
    tools/editor/cli_currency_types_catalog.cpp
    tools/editor/cli_spell_reagents_catalog.cpp
    tools/editor/cli_achievement_criteria_catalog.cpp
    tools/editor/cli_spell_effect_types_catalog.cpp
    tools/editor/cli_spell_aura_types_catalog.cpp
    tools/editor/cli_item_qualities_catalog.cpp
    tools/editor/cli_skill_costs_catalog.cpp
    tools/editor/cli_item_flags_catalog.cpp
    tools/editor/cli_npc_services_catalog.cpp
    tools/editor/cli_token_rewards_catalog.cpp
    tools/editor/cli_spell_procs_catalog.cpp
    tools/editor/cli_creature_patrols_catalog.cpp
    tools/editor/cli_boss_encounters_catalog.cpp
    tools/editor/cli_instance_lockouts_catalog.cpp
    tools/editor/cli_stable_slots_catalog.cpp
    tools/editor/cli_stat_curves_catalog.cpp
    tools/editor/cli_action_bars_catalog.cpp
    tools/editor/cli_group_compositions_catalog.cpp
    tools/editor/cli_hearth_binds_catalog.cpp
    tools/editor/cli_server_broadcasts_catalog.cpp
    tools/editor/cli_combat_maneuvers_catalog.cpp
    tools/editor/cli_realm_list_catalog.cpp
    tools/editor/cli_emotes_catalog.cpp
    tools/editor/cli_buff_book_catalog.cpp
    tools/editor/cli_tabards_catalog.cpp
    tools/editor/cli_spell_markers_catalog.cpp
    tools/editor/cli_learning_notifications_catalog.cpp
    tools/editor/cli_creature_resists_catalog.cpp
    tools/editor/cli_pet_talents_catalog.cpp
    tools/editor/cli_heroic_scaling_catalog.cpp
    tools/editor/cli_reputation_rewards_catalog.cpp
    tools/editor/cli_minimap_levels_catalog.cpp
    tools/editor/cli_pet_care_catalog.cpp
    tools/editor/cli_movie_credits_catalog.cpp
    tools/editor/cli_spell_variants_catalog.cpp
    tools/editor/cli_voiceovers_catalog.cpp
    tools/editor/cli_trade_rules_catalog.cpp
    tools/editor/cli_word_filters_catalog.cpp
    tools/editor/cli_raid_markers_catalog.cpp
    tools/editor/cli_loot_modes_catalog.cpp
    tools/editor/cli_sky_params_catalog.cpp
    tools/editor/cli_server_config_catalog.cpp
    tools/editor/cli_anniversary_events_catalog.cpp
    tools/editor/cli_pvp_ranks_catalog.cpp
    tools/editor/cli_localization_catalog.cpp
    tools/editor/cli_global_channels_catalog.cpp
    tools/editor/cli_addon_manifest_catalog.cpp
    tools/editor/cli_spell_pack_catalog.cpp
    tools/editor/cli_player_movement_anim_catalog.cpp
    tools/editor/cli_transit_schedule_catalog.cpp
    tools/editor/cli_mage_portals_catalog.cpp
    tools/editor/cli_combat_stats_catalog.cpp
    tools/editor/cli_guild_bank_catalog.cpp
    tools/editor/cli_quest_graph_catalog.cpp
    tools/editor/cli_crafting_recipes_catalog.cpp
    tools/editor/cli_world_locations_catalog.cpp
    tools/editor/cli_soulbind_rules_catalog.cpp
    tools/editor/cli_creature_behavior_catalog.cpp
    tools/editor/cli_random_property_catalog.cpp
    tools/editor/cli_spell_proc_rules_catalog.cpp
    tools/editor/cli_auction_houses_catalog.cpp
    tools/editor/cli_battleground_rewards_catalog.cpp
    tools/editor/cli_sound_swap_catalog.cpp
    tools/editor/cli_tutorial_steps_catalog.cpp
    tools/editor/cli_chat_commands_catalog.cpp
    tools/editor/cli_camera_presets_catalog.cpp
    tools/editor/cli_combat_formulas_catalog.cpp
    tools/editor/cli_chat_links_catalog.cpp
    tools/editor/cli_catalog_pluck.cpp
    tools/editor/cli_catalog_find.cpp
    tools/editor/cli_catalog_by_name.cpp
    tools/editor/cli_catalog_id_range.cpp
    tools/editor/cli_quest_objective.cpp
    tools/editor/cli_quest_reward.cpp
    tools/editor/cli_clone.cpp
    tools/editor/cli_remove.cpp
    tools/editor/cli_add.cpp
    tools/editor/cli_random.cpp
    tools/editor/cli_items_export.cpp
    tools/editor/cli_items_mutate.cpp
    tools/editor/cli_zone_create.cpp
    tools/editor/cli_tiles.cpp
    tools/editor/cli_zone_mgmt.cpp
    tools/editor/cli_strip.cpp
    tools/editor/cli_repair.cpp
    tools/editor/cli_makefile.cpp
    tools/editor/cli_zone_list.cpp
    tools/editor/cli_tilemap.cpp
    tools/editor/cli_deps.cpp
    tools/editor/cli_for_each.cpp
    tools/editor/cli_check.cpp
    tools/editor/cli_introspect.cpp
    tools/editor/cli_subprocess.cpp
    tools/editor/cli_texture_helpers.cpp
    tools/editor/cli_mesh_info.cpp
    tools/editor/cli_zone_data.cpp
    tools/editor/cli_project_actions.cpp
    tools/editor/cli_zone_export.cpp
    tools/editor/cli_arg_required.cpp
    tools/editor/cli_multi_arg_required.cpp
    tools/editor/cli_weld.cpp
    tools/editor/cli_box_emitter.cpp
    tools/editor/cli_dispatch.cpp
    tools/editor/editor_app.cpp
    tools/editor/editor_camera.cpp
    tools/editor/editor_viewport.cpp
    tools/editor/editor_ui.cpp
    tools/editor/editor_brush.cpp
    tools/editor/editor_history.cpp
    tools/editor/terrain_editor.cpp
    tools/editor/texture_painter.cpp
    tools/editor/object_placer.cpp
    tools/editor/npc_spawner.cpp
    tools/editor/npc_presets.cpp
    tools/editor/sql_exporter.cpp
    tools/editor/server_module_gen.cpp
    tools/editor/quest_editor.cpp
    tools/editor/transform_gizmo.cpp
    tools/editor/zone_manifest.cpp
    tools/editor/content_pack.cpp
    tools/editor/wowee_terrain.cpp
    tools/editor/editor_project.cpp
    tools/editor/texture_exporter.cpp
    tools/editor/dbc_exporter.cpp
    tools/editor/asset_browser.cpp
    tools/editor/editor_water.cpp
    tools/editor/editor_markers.cpp
    tools/editor/adt_writer.cpp

    # Pipeline (asset loading)
    src/pipeline/blp_loader.cpp
    src/pipeline/dbc_loader.cpp
    src/pipeline/dbc_layout.cpp
    src/pipeline/asset_manager.cpp
    src/pipeline/asset_manifest.cpp
    src/pipeline/loose_file_reader.cpp
    src/pipeline/m2_loader.cpp
    src/pipeline/wmo_loader.cpp
    src/pipeline/adt_loader.cpp
    src/pipeline/wdt_loader.cpp
    src/pipeline/wowee_terrain_loader.cpp
    src/pipeline/wowee_model.cpp
    src/pipeline/wowee_model_fromm2.cpp
    src/pipeline/wowee_building.cpp
    src/pipeline/wowee_collision.cpp
    src/pipeline/wowee_light.cpp
    src/pipeline/wowee_weather.cpp
    src/pipeline/wowee_world_map.cpp
    src/pipeline/wowee_sound.cpp
    src/pipeline/wowee_spawns.cpp
    src/pipeline/wowee_items.cpp
    src/pipeline/wowee_loot.cpp
    src/pipeline/wowee_creatures.cpp
    src/pipeline/wowee_quests.cpp
    src/pipeline/wowee_objects.cpp
    src/pipeline/wowee_factions.cpp
    src/pipeline/wowee_locks.cpp
    src/pipeline/wowee_skills.cpp
    src/pipeline/wowee_spells.cpp
    src/pipeline/wowee_achievements.cpp
    src/pipeline/wowee_trainers.cpp
    src/pipeline/wowee_gossip.cpp
    src/pipeline/wowee_taxi.cpp
    src/pipeline/wowee_talents.cpp
    src/pipeline/wowee_maps.cpp
    src/pipeline/wowee_chars.cpp
    src/pipeline/wowee_tokens.cpp
    src/pipeline/wowee_triggers.cpp
    src/pipeline/wowee_titles.cpp
    src/pipeline/wowee_events.cpp
    src/pipeline/wowee_mounts.cpp
    src/pipeline/wowee_battlegrounds.cpp
    src/pipeline/wowee_mail.cpp
    src/pipeline/wowee_gems.cpp
    src/pipeline/wowee_guilds.cpp
    src/pipeline/wowee_conditions.cpp
    src/pipeline/wowee_pets.cpp
    src/pipeline/wowee_auction.cpp
    src/pipeline/wowee_channels.cpp
    src/pipeline/wowee_cinematics.cpp
    src/pipeline/wowee_glyphs.cpp
    src/pipeline/wowee_vehicles.cpp
    src/pipeline/wowee_holidays.cpp
    src/pipeline/wowee_liquids.cpp
    src/pipeline/wowee_animations.cpp
    src/pipeline/wowee_spell_visuals.cpp
    src/pipeline/wowee_world_state_ui.cpp
    src/pipeline/wowee_player_conditions.cpp
    src/pipeline/wowee_trade_skills.cpp
    src/pipeline/wowee_creature_equipment.cpp
    src/pipeline/wowee_item_sets.cpp
    src/pipeline/wowee_game_tips.cpp
    src/pipeline/wowee_companions.cpp
    src/pipeline/wowee_spell_mechanics.cpp
    src/pipeline/wowee_keybindings.cpp
    src/pipeline/wowee_spell_schools.cpp
    src/pipeline/wowee_lfg.cpp
    src/pipeline/wowee_macros.cpp
    src/pipeline/wowee_char_features.cpp
    src/pipeline/wowee_pvp.cpp
    src/pipeline/wowee_bags.cpp
    src/pipeline/wowee_runes.cpp
    src/pipeline/wowee_loading_screens.cpp
    src/pipeline/wowee_item_suffixes.cpp
    src/pipeline/wowee_combat_ratings.cpp
    src/pipeline/wowee_unit_movement.cpp
    src/pipeline/wowee_quest_sorts.cpp
    src/pipeline/wowee_spell_ranges.cpp
    src/pipeline/wowee_spell_cast_times.cpp
    src/pipeline/wowee_spell_durations.cpp
    src/pipeline/wowee_spell_cooldowns.cpp
    src/pipeline/wowee_creature_families.cpp
    src/pipeline/wowee_spell_power_costs.cpp
    src/pipeline/wowee_glyph_slots.cpp
    src/pipeline/wowee_creature_difficulties.cpp
    src/pipeline/wowee_item_materials.cpp
    src/pipeline/wowee_player_spawn_profiles.cpp
    src/pipeline/wowee_talent_tabs.cpp
    src/pipeline/wowee_currency_types.cpp
    src/pipeline/wowee_spell_reagents.cpp
    src/pipeline/wowee_achievement_criteria.cpp
    src/pipeline/wowee_spell_effect_types.cpp
    src/pipeline/wowee_spell_aura_types.cpp
    src/pipeline/wowee_item_qualities.cpp
    src/pipeline/wowee_skill_costs.cpp
    src/pipeline/wowee_item_flags.cpp
    src/pipeline/wowee_npc_services.cpp
    src/pipeline/wowee_token_rewards.cpp
    src/pipeline/wowee_spell_procs.cpp
    src/pipeline/wowee_creature_patrols.cpp
    src/pipeline/wowee_boss_encounters.cpp
    src/pipeline/wowee_instance_lockouts.cpp
    src/pipeline/wowee_stable_slots.cpp
    src/pipeline/wowee_stat_curves.cpp
    src/pipeline/wowee_action_bars.cpp
    src/pipeline/wowee_group_compositions.cpp
    src/pipeline/wowee_hearth_binds.cpp
    src/pipeline/wowee_server_broadcasts.cpp
    src/pipeline/wowee_combat_maneuvers.cpp
    src/pipeline/wowee_realm_list.cpp
    src/pipeline/wowee_emotes.cpp
    src/pipeline/wowee_buff_book.cpp
    src/pipeline/wowee_tabards.cpp
    src/pipeline/wowee_spell_markers.cpp
    src/pipeline/wowee_learning_notifications.cpp
    src/pipeline/wowee_creature_resists.cpp
    src/pipeline/wowee_pet_talents.cpp
    src/pipeline/wowee_heroic_scaling.cpp
    src/pipeline/wowee_reputation_rewards.cpp
    src/pipeline/wowee_minimap_levels.cpp
    src/pipeline/wowee_pet_care.cpp
    src/pipeline/wowee_movie_credits.cpp
    src/pipeline/wowee_spell_variants.cpp
    src/pipeline/wowee_voiceovers.cpp
    src/pipeline/wowee_trade_rules.cpp
    src/pipeline/wowee_word_filters.cpp
    src/pipeline/wowee_raid_markers.cpp
    src/pipeline/wowee_loot_modes.cpp
    src/pipeline/wowee_sky_params.cpp
    src/pipeline/wowee_server_config.cpp
    src/pipeline/wowee_anniversary_events.cpp
    src/pipeline/wowee_pvp_ranks.cpp
    src/pipeline/wowee_localization.cpp
    src/pipeline/wowee_global_channels.cpp
    src/pipeline/wowee_addon_manifest.cpp
    src/pipeline/wowee_spell_pack.cpp
    src/pipeline/wowee_player_movement_anim.cpp
    src/pipeline/wowee_transit_schedule.cpp
    src/pipeline/wowee_mage_portals.cpp
    src/pipeline/wowee_combat_stats.cpp
    src/pipeline/wowee_guild_bank.cpp
    src/pipeline/wowee_quest_graph.cpp
    src/pipeline/wowee_crafting_recipes.cpp
    src/pipeline/wowee_world_locations.cpp
    src/pipeline/wowee_soulbind_rules.cpp
    src/pipeline/wowee_creature_behavior.cpp
    src/pipeline/wowee_random_property.cpp
    src/pipeline/wowee_spell_proc_rules.cpp
    src/pipeline/wowee_auction_houses.cpp
    src/pipeline/wowee_battleground_rewards.cpp
    src/pipeline/wowee_sound_swap.cpp
    src/pipeline/wowee_tutorial_steps.cpp
    src/pipeline/wowee_chat_commands.cpp
    src/pipeline/wowee_camera_presets.cpp
    src/pipeline/wowee_combat_formulas.cpp
    src/pipeline/wowee_chat_links.cpp
    src/pipeline/custom_zone_discovery.cpp
    src/pipeline/terrain_mesh.cpp

    # Rendering core
    src/rendering/vk_context.cpp
    src/rendering/vk_utils.cpp
    src/rendering/vk_shader.cpp
    src/rendering/vk_texture.cpp
    src/rendering/vk_buffer.cpp
    src/rendering/vk_pipeline.cpp
    src/rendering/camera.cpp
    src/rendering/terrain_renderer.cpp
    src/rendering/m2_renderer.cpp
    src/rendering/m2_renderer_instance.cpp
    src/rendering/m2_renderer_particles.cpp
    src/rendering/m2_renderer_render.cpp
    src/rendering/m2_model_classifier.cpp
    src/rendering/wmo_renderer.cpp
    src/rendering/frustum.cpp

    # Core
    src/core/window.cpp
    src/core/logger.cpp
    src/core/memory_monitor.cpp

    # stb_image (needed by AssetManager for PNG overrides)
    tools/editor/stb_image_impl.cpp

    ${WOWEE_MACOS_SOURCES}
)
target_include_directories(wowee_editor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/editor
)
target_include_directories(wowee_editor SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/vk-bootstrap/src
)
target_link_libraries(wowee_editor PRIVATE
    SDL2::SDL2
    Vulkan::Vulkan
    Threads::Threads
    ZLIB::ZLIB
    ${CMAKE_DL_LIBS}
    imgui
    vk-bootstrap
)
if(TARGET glm::glm-header-only)
    target_link_libraries(wowee_editor PRIVATE glm::glm-header-only)
elseif(TARGET glm::glm)
    target_link_libraries(wowee_editor PRIVATE glm::glm)
elseif(glm_FOUND)
    target_include_directories(wowee_editor PRIVATE ${GLM_INCLUDE_DIRS})
endif()
if(UNIX AND NOT APPLE)
    find_package(X11 QUIET)
    if(X11_FOUND)
        target_link_libraries(wowee_editor PRIVATE X11)
    endif()
endif()
if(APPLE)
    target_link_libraries(wowee_editor PRIVATE "-framework Foundation")
endif()
if(WIN32)
    target_link_libraries(wowee_editor PRIVATE ws2_32)
    if(TARGET SDL2::SDL2main)
        target_link_libraries(wowee_editor PRIVATE SDL2::SDL2main)
    endif()
endif()
if(NOT MSVC)
    # The editor's CLI handlers share a `(int& i, int argc, char** argv)`
    # signature so they can be plugged into a function-pointer dispatch
    # table; many handlers don't reference argc directly. Suppress that
    # warning here rather than littering 30+ handlers with [[maybe_unused]].
    target_compile_options(wowee_editor PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-missing-field-initializers
        -Wno-unused-parameter)
endif()
if(GLSLC)
    add_dependencies(wowee_editor wowee_shaders)
endif()
set_target_properties(wowee_editor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
install(TARGETS wowee_editor RUNTIME DESTINATION bin)
message(STATUS "  wowee_editor tool: ENABLED")

# Print configuration summary
message(STATUS "")
message(STATUS "Wowee Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  SDL2: ${SDL2_VERSION}")
message(STATUS "  OpenSSL: ${OPENSSL_VERSION}")
message(STATUS "  ImGui: ${IMGUI_DIR}")
message(STATUS "  ASAN: ${WOWEE_ENABLE_ASAN}")
message(STATUS "")

# ---- CPack packaging ----
set(CPACK_PACKAGE_NAME "wowee")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "World of Warcraft client emulator")
set(CPACK_PACKAGE_VENDOR "Wowee")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Wowee")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

if(WIN32)
    set(CPACK_GENERATOR "NSIS")
    set(CPACK_NSIS_DISPLAY_NAME "Wowee")
    set(CPACK_NSIS_PACKAGE_NAME "Wowee")
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    # Run wowee from bin/ so that ./assets/ resolves correctly.
    # SetOutPath sets the shortcut's working directory in NSIS.
    set(CPACK_NSIS_CREATE_ICONS_EXTRA
        "SetOutPath '$INSTDIR\\\\bin'\nCreateShortCut '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Wowee.lnk' '$INSTDIR\\\\bin\\\\wowee.exe'")
    set(CPACK_NSIS_DELETE_ICONS_EXTRA
        "Delete '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Wowee.lnk'")
elseif(APPLE)
    set(CPACK_GENERATOR "DragNDrop")
else()
    # Linux — generate postinst/prerm wrapper scripts
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/packaging)
    # postinst: write a wrapper script at /usr/local/bin/wowee that cd's to
    # the install dir so ./assets/ resolves correctly.
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/packaging/postinst
[[#!/bin/sh
cat > /usr/local/bin/wowee << 'WOWEE_WRAPPER'
#!/bin/sh
cd /opt/wowee/bin
exec ./wowee "$@"
WOWEE_WRAPPER
chmod +x /usr/local/bin/wowee
]])
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/packaging/prerm
"#!/bin/sh\nrm -f /usr/local/bin/wowee\n")
    file(CHMOD
        ${CMAKE_CURRENT_BINARY_DIR}/packaging/postinst
        ${CMAKE_CURRENT_BINARY_DIR}/packaging/prerm
        PERMISSIONS
            OWNER_EXECUTE OWNER_WRITE OWNER_READ
            GROUP_EXECUTE GROUP_READ
            WORLD_EXECUTE WORLD_READ)

    set(CPACK_GENERATOR "DEB")
    set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/wowee")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Wowee")
    set(CPACK_DEBIAN_PACKAGE_SECTION "games")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "libsdl2-2.0-0, libvulkan1, libssl3, zlib1g")
    set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
        "${CMAKE_CURRENT_BINARY_DIR}/packaging/postinst;${CMAKE_CURRENT_BINARY_DIR}/packaging/prerm")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
    endif()
endif()

include(CPack)

# Bundled addons ship next to the executable, so the client finds them without
# anyone copying files into an extracted game install. AddonManager searches
# here as well as the game data's own Interface/AddOns.
add_custom_target(copy_bundled_addons ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/addons $<TARGET_FILE_DIR:wowee>/addons
    COMMENT "Copying bundled addons")
add_dependencies(copy_bundled_addons wowee)

