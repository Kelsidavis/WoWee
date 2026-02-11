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

# Find required packages
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavformat libavcodec libswscale libavutil)

# GLM (header-only math library)
find_package(glm QUIET)
if(NOT glm_FOUND)
    message(STATUS "GLM not found, will use system includes or download")
endif()

# StormLib for MPQ archives
find_library(STORMLIB_LIBRARY NAMES StormLib stormlib storm)
find_path(STORMLIB_INCLUDE_DIR StormLib.h PATH_SUFFIXES StormLib)

if(NOT STORMLIB_LIBRARY OR NOT STORMLIB_INCLUDE_DIR)
    message(WARNING "StormLib not found. You may need to build it manually.")
    message(WARNING "Get it from: https://github.com/ladislav-zezula/StormLib")
endif()

# Include ImGui as a static library (we'll add the sources)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui)
if(EXISTS ${IMGUI_DIR})
    add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC SDL2::SDL2 OpenGL::GL ${CMAKE_DL_LIBS})
    target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
else()
    message(WARNING "ImGui not found in extern/imgui. Clone it with:")
    message(WARNING "  git clone https://github.com/ocornut/imgui.git extern/imgui")
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
    src/auth/srp.cpp
    src/auth/big_num.cpp
    src/auth/crypto.cpp
    src/auth/rc4.cpp

    # Game
    src/game/game_handler.cpp
    src/game/transport_manager.cpp
    src/game/world.cpp
    src/game/player.cpp
    src/game/entity.cpp
    src/game/opcodes.cpp
    src/game/world_packets.cpp
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
    src/pipeline/mpq_manager.cpp
    src/pipeline/blp_loader.cpp
    src/pipeline/dbc_loader.cpp
    src/pipeline/asset_manager.cpp
    src/pipeline/m2_loader.cpp
    src/pipeline/wmo_loader.cpp
    src/pipeline/adt_loader.cpp
    src/pipeline/terrain_mesh.cpp

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
    src/rendering/loading_screen.cpp
    src/rendering/video_player.cpp

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

    include/pipeline/mpq_manager.hpp
    include/pipeline/blp_loader.hpp
    include/pipeline/m2_loader.hpp
    include/pipeline/wmo_loader.hpp
    include/pipeline/adt_loader.hpp
    include/pipeline/dbc_loader.hpp
    include/pipeline/terrain_mesh.hpp

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
    list(APPEND WOWEE_PLATFORM_SOURCES resources/wowee.rc)
endif()

# Create executable
add_executable(wowee ${WOWEE_SOURCES} ${WOWEE_HEADERS} ${WOWEE_PLATFORM_SOURCES})

# Include directories
target_include_directories(wowee PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
    ${FFMPEG_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(wowee PRIVATE
    SDL2::SDL2
    OpenGL::GL
    GLEW::GLEW
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
    ZLIB::ZLIB
    ${CMAKE_DL_LIBS}
)

target_link_libraries(wowee PRIVATE ${FFMPEG_LIBRARIES})
if (FFMPEG_LIBRARY_DIRS)
    target_link_directories(wowee PRIVATE ${FFMPEG_LIBRARY_DIRS})
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

# Link StormLib if found
if(STORMLIB_LIBRARY AND STORMLIB_INCLUDE_DIR)
    target_link_libraries(wowee PRIVATE ${STORMLIB_LIBRARY})
    target_include_directories(wowee PRIVATE ${STORMLIB_INCLUDE_DIR})
    target_compile_definitions(wowee PRIVATE HAVE_STORMLIB)
endif()

# Link ImGui if available
if(TARGET imgui)
    target_link_libraries(wowee PRIVATE imgui)
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

# Copy assets to build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets
     DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Install targets
install(TARGETS wowee
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

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

# Print configuration summary
message(STATUS "")
message(STATUS "Wowee Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  SDL2: ${SDL2_VERSION}")
message(STATUS "  OpenSSL: ${OPENSSL_VERSION}")
message(STATUS "  StormLib: ${STORMLIB_LIBRARY}")
message(STATUS "  ImGui: ${IMGUI_DIR}")
message(STATUS "")
