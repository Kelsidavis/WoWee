cmake_minimum_required(VERSION 3.15)
project(wowee VERSION 1.0.0 LANGUAGES CXX)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(WOWEE_BUILD_TESTS "Build tests" OFF)
option(WOWEE_ENABLE_ASAN "Enable AddressSanitizer (Debug builds)" OFF)

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
find_package(Vulkan REQUIRED)
# GL/GLEW kept temporarily for unconverted sub-renderers during Vulkan migration.
# These files compile against GL types but their code is never called — the Vulkan
# path is the only active rendering backend. Remove in Phase 7 when all renderers
# are converted and grep confirms zero GL references.
find_package(OpenGL QUIET)
find_package(GLEW QUIET)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
if(WIN32)
    # Use vcpkg's FFmpeg CMake integration on Windows
    find_package(FFMPEG QUIET)
    if(FFMPEG_FOUND)
        set(HAVE_FFMPEG TRUE)
        message(STATUS "Found FFmpeg (vcpkg): ${FFMPEG_LIBRARIES}")
    else()
        set(HAVE_FFMPEG FALSE)
        message(WARNING "FFmpeg not found — video_player will be disabled. Install via vcpkg: ffmpeg:x64-windows")
    endif()
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FFMPEG libavformat libavcodec libswscale libavutil)
    if(FFMPEG_FOUND)
        set(HAVE_FFMPEG TRUE)
        message(STATUS "Found FFmpeg (pkg-config): ${FFMPEG_LIBRARIES}")
    else()
        set(HAVE_FFMPEG FALSE)
        message(WARNING "FFmpeg not found — video_player will be disabled.")
    endif()
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

# Vulkan Memory Allocator
find_package(VulkanMemoryAllocator CONFIG REQUIRED)

# GLM (header-only math library)
find_package(glm QUIET)
if(NOT glm_FOUND)
    message(STATUS "GLM not found, will use system includes or download")
endif()
# GLM GTX extensions (quaternion, norm, etc.) require this flag on newer GLM versions
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL GLM_FORCE_DEPTH_ZERO_TO_ONE)
if(WIN32)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
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
    set(SPV_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets/shaders)
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
    src/core/window.cpp
    src/core/input.cpp
    src/core/logger.cpp
    src/core/memory_monitor.cpp

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
    src/game/warden_crypto.cpp
    src/game/warden_module.cpp
    src/game/warden_emulator.cpp
    src/game/warden_memory.cpp
    src/game/transport_manager.cpp
    src/game/world.cpp
    src/game/player.cpp
    src/game/entity.cpp
    src/game/opcodes.cpp
    src/game/world_packets.cpp
    src/game/packet_parsers_tbc.cpp
    src/game/packet_parsers_classic.cpp
    src/game/character.cpp
    src/game/zone_manager.cpp
    src/game/inventory.cpp

    # Audio
    src/audio/audio_engine.cpp
    src/audio/music_manager.cpp
    src/audio/footstep_manager.cpp
    src/audio/activity_sound_manager.cpp
    src/audio/mount_sound_manager.cpp
    src/audio/npc_voice_manager.cpp
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
    src/rendering/shader.cpp
    src/rendering/texture.cpp
    src/rendering/mesh.cpp
    src/rendering/camera.cpp
    src/rendering/camera_controller.cpp
    src/rendering/material.cpp
    src/rendering/scene.cpp
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
    src/rendering/sky_system.cpp
    src/rendering/character_renderer.cpp
    src/rendering/character_preview.cpp
    src/rendering/wmo_renderer.cpp
    src/rendering/m2_renderer.cpp
    src/rendering/quest_marker_renderer.cpp
    src/rendering/minimap.cpp
    src/rendering/world_map.cpp
    src/rendering/swim_effects.cpp
    src/rendering/mount_dust.cpp
    src/rendering/levelup_effect.cpp
    src/rendering/charge_effect.cpp
    src/rendering/loading_screen.cpp
    $<$<BOOL:${HAVE_FFMPEG}>:${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/video_player.cpp>

    # UI
    src/ui/ui_manager.cpp
    src/ui/auth_screen.cpp
    src/ui/realm_screen.cpp
    src/ui/character_create_screen.cpp
    src/ui/character_screen.cpp
    src/ui/game_screen.cpp
    src/ui/inventory_screen.cpp
    src/ui/quest_log_screen.cpp
    src/ui/spellbook_screen.cpp
    src/ui/talent_screen.cpp

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
    include/rendering/shader.hpp
    include/rendering/texture.hpp
    include/rendering/mesh.hpp
    include/rendering/camera.hpp
    include/rendering/camera_controller.hpp
    include/rendering/material.hpp
    include/rendering/scene.hpp
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
    include/rendering/character_renderer.hpp
    include/rendering/character_preview.hpp
    include/rendering/wmo_renderer.hpp
    include/rendering/loading_screen.hpp
    include/rendering/video_player.hpp

    include/ui/ui_manager.hpp
    include/ui/auth_screen.hpp
    include/ui/realm_screen.hpp
    include/ui/character_create_screen.hpp
    include/ui/character_screen.hpp
    include/ui/game_screen.hpp
    include/ui/inventory_screen.hpp
    include/ui/spellbook_screen.hpp
    include/ui/talent_screen.hpp
)

set(WOWEE_PLATFORM_SOURCES)
if(WIN32)
    # Copy icon into build tree so llvm-rc can find it via the relative path in wowee.rc
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/assets/Wowee.ico
        ${CMAKE_CURRENT_BINARY_DIR}/assets/wowee.ico
        COPYONLY
    )
    list(APPEND WOWEE_PLATFORM_SOURCES resources/wowee.rc)
endif()

# Create executable
add_executable(wowee ${WOWEE_SOURCES} ${WOWEE_HEADERS} ${WOWEE_PLATFORM_SOURCES})
if(TARGET opcodes-generate)
    add_dependencies(wowee opcodes-generate)
endif()

# Compile GLSL shaders to SPIR-V
if(GLSLC)
    compile_shaders(wowee)
endif()

# Include directories
target_include_directories(wowee PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
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
    ${CMAKE_DL_LIBS}
)

# GL/GLEW linked temporarily for unconverted sub-renderers (removed in Phase 7)
if(TARGET OpenGL::GL)
    target_link_libraries(wowee PRIVATE OpenGL::GL)
endif()
if(TARGET GLEW::GLEW)
    target_link_libraries(wowee PRIVATE GLEW::GLEW)
endif()

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

# Link Vulkan Memory Allocator
target_link_libraries(wowee PRIVATE GPUOpen::VulkanMemoryAllocator)

# Link Unicorn if available
if(HAVE_UNICORN)
    target_link_libraries(wowee PRIVATE ${UNICORN_LIBRARY})
    target_include_directories(wowee PRIVATE ${UNICORN_INCLUDE_DIR})
    target_compile_definitions(wowee PRIVATE HAVE_UNICORN)
endif()

# Link GLM if found
if(TARGET glm::glm)
    target_link_libraries(wowee PRIVATE glm::glm)
elseif(glm_FOUND)
    target_include_directories(wowee PRIVATE ${GLM_INCLUDE_DIRS})
endif()

# Compiler warnings
if(MSVC)
    target_compile_options(wowee PRIVATE /W4)
else()
    target_compile_options(wowee PRIVATE -Wall -Wextra -Wpedantic)
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
    target_compile_options(wowee PRIVATE
        $<$<CONFIG:Debug>:-g3 -Og -fno-omit-frame-pointer>
        $<$<CONFIG:RelWithDebInfo>:-g -fno-omit-frame-pointer>
    )
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
        target_compile_options(wowee PRIVATE -fsanitize=address -fno-omit-frame-pointer)
        target_link_options(wowee PRIVATE -fsanitize=address)
    endif()
    message(STATUS "AddressSanitizer: ENABLED")
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

# On Windows, SDL 2.28+ uses LoadLibraryExW with LOAD_LIBRARY_SEARCH_DEFAULT_DIRS
# which does NOT include System32. Copy vulkan-1.dll into the output directory so
# SDL_Vulkan_LoadLibrary can locate it without needing a full system PATH search.
if(WIN32)
    add_custom_command(TARGET wowee POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$ENV{SystemRoot}/System32/vulkan-1.dll"
            "$<TARGET_FILE_DIR:wowee>/vulkan-1.dll"
        COMMENT "Copying vulkan-1.dll to output directory"
    )
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
        src/pipeline/dbc_loader.cpp
        src/core/logger.cpp
    )
    target_include_directories(asset_extract PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/asset_extract
        ${STORMLIB_INCLUDE_DIR}
    )
    target_link_libraries(asset_extract PRIVATE
        ${STORMLIB_LIBRARY}
        ZLIB::ZLIB
        Threads::Threads
    )
    if(WIN32)
        target_link_libraries(asset_extract PRIVATE wininet bz2)
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
