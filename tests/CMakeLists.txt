# Unit test infrastructure using Catch2 v3 (amalgamated)

# Catch2 amalgamated as a library target
add_library(catch2_main STATIC
    ${CMAKE_SOURCE_DIR}/extern/catch2/catch_amalgamated.cpp
)
target_include_directories(catch2_main PUBLIC
    ${CMAKE_SOURCE_DIR}/extern/catch2
)
# Catch2 v3 needs C++17 minimum
target_compile_features(catch2_main PUBLIC cxx_std_17)

# ── ASAN / UBSan propagation ────────────────────────────────
# Collect all test target names so we can apply sanitizer flags at the end.
set(ALL_TEST_TARGETS "")

# Helper: register a test target for ASAN/UBSan if enabled.
# Also links Winsock on Windows: several tests pull in networking sources
# (auth, socket code) whose symbols live in ws2_32 there rather than libc.
# Linking it for every test keeps a new test from breaking the Windows build.
macro(register_test_target _target)
    list(APPEND ALL_TEST_TARGETS ${_target})
    if(WIN32)
        target_link_libraries(${_target} PRIVATE ws2_32)
    endif()
endmacro()

# Shared source files used across multiple tests
set(TEST_COMMON_SOURCES
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)

# Include directories matching the main target
set(TEST_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)
set(TEST_SYSTEM_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/extern
)

# Stock-style hill climbing regression test. Run directly with:
#   cmake --build build --target test_hill_climbing && ctest --test-dir build -R hill_climbing
add_executable(test_hill_climbing
    test_movement_limits.cpp
)
target_include_directories(test_hill_climbing PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_hill_climbing SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_hill_climbing PRIVATE catch2_main)
# glm is not vendored — it comes from find_package(glm), and its include path
# arrives with the imported target rather than through any directory this file
# lists. Adding TEST_SYSTEM_INCLUDE_DIRS above does not supply it, which is why
# the first attempt at this failed CI unchanged: Linux has a system copy under
# /usr/include and compiles either way, so only macOS ever reports it.
# Same two-branch check as the top level, where GLM 1.0 may build glm::glm as a
# real library and expose the headers as glm::glm-header-only.
if(TARGET glm::glm-header-only)
    target_link_libraries(test_hill_climbing PRIVATE glm::glm-header-only)
elseif(TARGET glm::glm)
    target_link_libraries(test_hill_climbing PRIVATE glm::glm)
endif()
add_test(NAME hill_climbing COMMAND test_hill_climbing)
register_test_target(test_hill_climbing)

# ── test_framexml ───────────────────────────────────────────
add_executable(test_framexml
    test_framexml.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/xml_parser.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/framexml_emitter.cpp
)
target_include_directories(test_framexml PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_framexml SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_framexml PRIVATE catch2_main)
add_test(NAME framexml COMMAND test_framexml)
register_test_target(test_framexml)

# ── test_widget_tree ────────────────────────────────────────
add_executable(test_widget_tree
    test_widget_tree.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/widget_tree.cpp
)
target_include_directories(test_widget_tree PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_widget_tree SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_widget_tree PRIVATE catch2_main)
add_test(NAME widget_tree COMMAND test_widget_tree)
register_test_target(test_widget_tree)

# ── test_packet ──────────────────────────────────────────────
add_executable(test_packet
    test_packet.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
)
target_include_directories(test_packet PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_packet SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_packet PRIVATE catch2_main)
add_test(NAME packet COMMAND test_packet)
register_test_target(test_packet)

# ── test_srp ─────────────────────────────────────────────────
add_executable(test_srp
    test_srp.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/auth/srp.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/big_num.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
)
target_include_directories(test_srp PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_srp SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_srp PRIVATE catch2_main OpenSSL::SSL OpenSSL::Crypto)
add_test(NAME srp COMMAND test_srp)
register_test_target(test_srp)

# ── test_spline ──────────────────────────────────────────────
add_executable(test_spline
    test_spline.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/math/spline.cpp
    ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_spline PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_spline SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_spline PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_spline PRIVATE glm::glm)
endif()
add_test(NAME spline COMMAND test_spline)
register_test_target(test_spline)

# ── test_transport_path_repo ─────────────────────────────────
add_executable(test_transport_path_repo
    test_transport_path_repo.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/game/transport_path_repository.cpp
    ${CMAKE_SOURCE_DIR}/src/math/spline.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_loader.cpp
)
target_include_directories(test_transport_path_repo PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_transport_path_repo SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_transport_path_repo PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_transport_path_repo PRIVATE glm::glm)
endif()
add_test(NAME transport_path_repo COMMAND test_transport_path_repo)
register_test_target(test_transport_path_repo)

# ── test_opcode_table ────────────────────────────────────────
add_executable(test_opcode_table
    test_opcode_table.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/game/opcode_table.cpp
)
target_include_directories(test_opcode_table PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_opcode_table SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_opcode_table PRIVATE catch2_main)
add_test(NAME opcode_table COMMAND test_opcode_table)
register_test_target(test_opcode_table)

# ── test_chat_types ─────────────────────────────────────────────
# A chat type is a byte off the wire and the event name is built from it, so a
# value missing from the enum is a line of chat that never appears — no error,
# no log. The whole run between LOOT and the battleground block was absent once.
add_executable(test_chat_types test_chat_types.cpp)
target_include_directories(test_chat_types PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_chat_types SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_chat_types PRIVATE catch2_main)
add_test(NAME chat_types COMMAND test_chat_types)
register_test_target(test_chat_types)

# ── test_taxi_packets ────────────────────────────────────────
# The multi-hop flight request. A field the server does not read was being
# written between the guid and the node count, so the count was read as a node
# and every route longer than one hop was refused.
add_executable(test_taxi_packets
    test_taxi_packets.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/opcode_table.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/character.cpp
    ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_world.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_entity.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_social.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_economy.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_classic.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_tbc.cpp
)
target_include_directories(test_taxi_packets PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_taxi_packets SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_taxi_packets PRIVATE catch2_main SDL2::SDL2 ZLIB::ZLIB
                                                OpenSSL::SSL OpenSSL::Crypto)
if(TARGET glm::glm)
    target_link_libraries(test_taxi_packets PRIVATE glm::glm)
endif()
if(TARGET Vulkan::Vulkan)
    target_link_libraries(test_taxi_packets PRIVATE Vulkan::Vulkan)
endif()
add_test(NAME taxi_packets COMMAND test_taxi_packets)
register_test_target(test_taxi_packets)

# ── test_use_item_packet ─────────────────────────────────────
add_executable(test_use_item_packet
    test_use_item_packet.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/opcode_table.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/character.cpp
    ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_world.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_entity.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_social.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_economy.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_classic.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_tbc.cpp
)
target_include_directories(test_use_item_packet PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_use_item_packet SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_use_item_packet PRIVATE catch2_main SDL2::SDL2 ZLIB::ZLIB
                                                   OpenSSL::SSL OpenSSL::Crypto)
if(TARGET glm::glm)
    target_link_libraries(test_use_item_packet PRIVATE glm::glm)
endif()
if(TARGET Vulkan::Vulkan)
    target_link_libraries(test_use_item_packet PRIVATE Vulkan::Vulkan)
endif()
add_test(NAME use_item_packet COMMAND test_use_item_packet)
register_test_target(test_use_item_packet)

# ── test_mail_packets ───────────────────────────────────────
add_executable(test_mail_packets
    test_mail_packets.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/opcode_table.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/character.cpp
    ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_world.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_entity.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_social.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_economy.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_classic.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_tbc.cpp
)
target_include_directories(test_mail_packets PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_mail_packets SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_mail_packets PRIVATE catch2_main SDL2::SDL2 ZLIB::ZLIB
                                                 OpenSSL::SSL OpenSSL::Crypto)
if(TARGET glm::glm)
    target_link_libraries(test_mail_packets PRIVATE glm::glm)
endif()
if(TARGET Vulkan::Vulkan)
    target_link_libraries(test_mail_packets PRIVATE Vulkan::Vulkan)
endif()
add_test(NAME mail_packets COMMAND test_mail_packets)
register_test_target(test_mail_packets)

# ── test_raid_target_packets ────────────────────────────────
add_executable(test_raid_target_packets
    test_raid_target_packets.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/opcode_table.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/character.cpp
    ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_world.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_entity.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_social.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_economy.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_classic.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_tbc.cpp
)
target_include_directories(test_raid_target_packets PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_raid_target_packets SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_raid_target_packets PRIVATE catch2_main SDL2::SDL2 ZLIB::ZLIB
                                                 OpenSSL::SSL OpenSSL::Crypto)
if(TARGET glm::glm)
    target_link_libraries(test_raid_target_packets PRIVATE glm::glm)
endif()
if(TARGET Vulkan::Vulkan)
    target_link_libraries(test_raid_target_packets PRIVATE Vulkan::Vulkan)
endif()
add_test(NAME raid_target_packets COMMAND test_raid_target_packets)
register_test_target(test_raid_target_packets)

# ── test_entity ──────────────────────────────────────────────
add_executable(test_entity
    test_entity.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/game/entity.cpp
    ${CMAKE_SOURCE_DIR}/src/math/spline.cpp
)
target_include_directories(test_entity PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_entity SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_entity PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_entity PRIVATE glm::glm)
endif()
add_test(NAME entity COMMAND test_entity)
register_test_target(test_entity)

# ── test_dbc_loader ──────────────────────────────────────────
add_executable(test_dbc_loader
    test_dbc_loader.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_layout.cpp
)
target_include_directories(test_dbc_loader PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_dbc_loader SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_dbc_loader PRIVATE catch2_main)
add_test(NAME dbc_loader COMMAND test_dbc_loader)
register_test_target(test_dbc_loader)

# ── test_m2_structs ──────────────────────────────────────────
add_executable(test_m2_structs
    test_m2_structs.cpp
    ../src/rendering/m2_model_classifier.cpp
)
target_include_directories(test_m2_structs PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_structs SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_m2_structs PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_m2_structs PRIVATE glm::glm)
endif()
add_test(NAME m2_structs COMMAND test_m2_structs)
register_test_target(test_m2_structs)

# ── test_m2_color_alpha ──────────────────────────────────────
# Loads the peasant lumberjack carry models (WotLK v264 + vanilla v256) and
# pins the M2 color-alpha track parsing that hides the duplicate wood bundle.
add_executable(test_m2_color_alpha
    test_m2_color_alpha.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/m2_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_m2_color_alpha PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_color_alpha SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_m2_color_alpha PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_m2_color_alpha PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_m2_color_alpha PRIVATE glm::glm)
endif()
add_test(NAME m2_color_alpha COMMAND test_m2_color_alpha)
register_test_target(test_m2_color_alpha)

# ── test_m2_footstep_events ──────────────────────────────────
# Loads real WotLK v264 + vanilla v256 models and pins the $FSD footfall
# event parsing that syncs footstep audio to foot-strike keyframes.
add_executable(test_m2_footstep_events
    test_m2_footstep_events.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/m2_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_m2_footstep_events PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_footstep_events SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_m2_footstep_events PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_m2_footstep_events PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_m2_footstep_events PRIVATE glm::glm)
endif()
add_test(NAME m2_footstep_events COMMAND test_m2_footstep_events)
register_test_target(test_m2_footstep_events)

# ── test_blp_loader ──────────────────────────────────────────
add_executable(test_blp_loader
    test_blp_loader.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/pipeline/blp_loader.cpp
)
target_include_directories(test_blp_loader PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_blp_loader SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_blp_loader PRIVATE catch2_main)
add_test(NAME blp_loader COMMAND test_blp_loader)
register_test_target(test_blp_loader)

# ── test_frustum ─────────────────────────────────────────────
add_executable(test_frustum
    test_frustum.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/frustum.cpp
)
target_include_directories(test_frustum PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_frustum SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_frustum PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_frustum PRIVATE glm::glm)
endif()
add_test(NAME frustum COMMAND test_frustum)
register_test_target(test_frustum)

# ── test_packed_time ─────────────────────────────────────────
# Header-only domain logic: the one uint32 that carries a whole date, which
# four separate readers in this tree got wrong in four different ways.
add_executable(test_packed_time
    test_packed_time.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_packed_time PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_packed_time SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_packed_time PRIVATE catch2_main)
add_test(NAME packed_time COMMAND test_packed_time)
register_test_target(test_packed_time)

# ── test_animation_ids ───────────────────────────────────────
add_executable(test_animation_ids
    test_animation_ids.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/rendering/animation/animation_ids.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_loader.cpp
)
target_include_directories(test_animation_ids PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_animation_ids SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_animation_ids PRIVATE catch2_main)
add_test(NAME animation_ids COMMAND test_animation_ids)
register_test_target(test_animation_ids)

# ── test_locomotion_fsm ──────────────────────────────────────
add_executable(test_locomotion_fsm
    test_locomotion_fsm.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/rendering/animation/locomotion_fsm.cpp
)
target_include_directories(test_locomotion_fsm PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_locomotion_fsm SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_locomotion_fsm PRIVATE catch2_main)
add_test(NAME locomotion_fsm COMMAND test_locomotion_fsm)
register_test_target(test_locomotion_fsm)

# ── test_combat_fsm ──────────────────────────────────────────
add_executable(test_combat_fsm
    test_combat_fsm.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/rendering/animation/combat_fsm.cpp
)
target_include_directories(test_combat_fsm PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_combat_fsm SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_combat_fsm PRIVATE catch2_main)
add_test(NAME combat_fsm COMMAND test_combat_fsm)
register_test_target(test_combat_fsm)

# ── test_activity_fsm ────────────────────────────────────────
add_executable(test_activity_fsm
    test_activity_fsm.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/rendering/animation/activity_fsm.cpp
)
target_include_directories(test_activity_fsm PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_activity_fsm SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_activity_fsm PRIVATE catch2_main)
add_test(NAME activity_fsm COMMAND test_activity_fsm)
register_test_target(test_activity_fsm)

# NPC animator tests removed — NpcAnimator replaced by generic CharacterAnimator

# ── test_anim_capability ─────────────────────────────────────
# Header-only struct tests — no source files needed
add_executable(test_anim_capability
    test_anim_capability.cpp
)
target_include_directories(test_anim_capability PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_anim_capability SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_anim_capability PRIVATE catch2_main)
add_test(NAME anim_capability COMMAND test_anim_capability)
register_test_target(test_anim_capability)

# ── test_indoor_shadows ──────────────────────────────────────
add_executable(test_indoor_shadows
    test_indoor_shadows.cpp
)
target_include_directories(test_indoor_shadows PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_indoor_shadows SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_indoor_shadows PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_indoor_shadows PRIVATE glm::glm)
endif()
add_test(NAME indoor_shadows COMMAND test_indoor_shadows)
register_test_target(test_indoor_shadows)

# ── test_zone_ambience ─────────────────────────────────────────
add_executable(test_zone_ambience
    test_zone_ambience.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/zone_ambience.cpp
)
target_include_directories(test_zone_ambience PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_zone_ambience SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_zone_ambience PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_zone_ambience PRIVATE glm::glm)
endif()
add_test(NAME zone_ambience COMMAND test_zone_ambience)
register_test_target(test_zone_ambience)

# ── test_transport_components ────────────────────────────────
add_executable(test_transport_components
    test_transport_components.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/game/transport_clock_sync.cpp
    ${CMAKE_SOURCE_DIR}/src/game/transport_animator.cpp
    ${CMAKE_SOURCE_DIR}/src/math/spline.cpp
)
target_include_directories(test_transport_components PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_transport_components SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_transport_components PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_transport_components PRIVATE glm::glm)
endif()
add_test(NAME transport_components COMMAND test_transport_components)
register_test_target(test_transport_components)

# ── test_world_map ────────────────────────────────────────────
add_executable(test_world_map
    test_world_map.cpp
)
target_include_directories(test_world_map PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_world_map SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_world_map PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_world_map PRIVATE glm::glm)
endif()
add_test(NAME world_map COMMAND test_world_map)
register_test_target(test_world_map)

# ── test_world_map_coordinate_projection ──────────────────────
add_executable(test_world_map_coordinate_projection
    test_world_map_coordinate_projection.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/world_map/coordinate_projection.cpp
)
target_include_directories(test_world_map_coordinate_projection PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_world_map_coordinate_projection SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_world_map_coordinate_projection PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_world_map_coordinate_projection PRIVATE glm::glm)
endif()
add_test(NAME world_map_coordinate_projection COMMAND test_world_map_coordinate_projection)
register_test_target(test_world_map_coordinate_projection)

# ── test_world_map_map_resolver ───────────────────────────────
add_executable(test_world_map_map_resolver
    test_world_map_map_resolver.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/world_map/map_resolver.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/world_map/coordinate_projection.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_world_map_map_resolver PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_world_map_map_resolver SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_world_map_map_resolver PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_world_map_map_resolver PRIVATE glm::glm)
endif()
add_test(NAME world_map_map_resolver COMMAND test_world_map_map_resolver)
register_test_target(test_world_map_map_resolver)

# ── test_world_map_view_state_machine ─────────────────────────
add_executable(test_world_map_view_state_machine
    test_world_map_view_state_machine.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/world_map/view_state_machine.cpp
)
target_include_directories(test_world_map_view_state_machine PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_world_map_view_state_machine SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_world_map_view_state_machine PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_world_map_view_state_machine PRIVATE glm::glm)
endif()
add_test(NAME world_map_view_state_machine COMMAND test_world_map_view_state_machine)
register_test_target(test_world_map_view_state_machine)

# ── test_world_map_exploration_state ──────────────────────────
add_executable(test_world_map_exploration_state
    test_world_map_exploration_state.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/world_map/exploration_state.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/world_map/coordinate_projection.cpp
)
target_include_directories(test_world_map_exploration_state PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_world_map_exploration_state SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_world_map_exploration_state PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_world_map_exploration_state PRIVATE glm::glm)
endif()
add_test(NAME world_map_exploration_state COMMAND test_world_map_exploration_state)
register_test_target(test_world_map_exploration_state)

# ── test_world_map_zone_metadata ──────────────────────────────
add_executable(test_world_map_zone_metadata
    test_world_map_zone_metadata.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/world_map/zone_metadata.cpp
)
target_include_directories(test_world_map_zone_metadata PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_world_map_zone_metadata SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_world_map_zone_metadata PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_world_map_zone_metadata PRIVATE glm::glm)
endif()
add_test(NAME world_map_zone_metadata COMMAND test_world_map_zone_metadata)
register_test_target(test_world_map_zone_metadata)

# ── test_chat_markup_parser ──────────────────────────────────
add_executable(test_chat_markup_parser
    test_chat_markup_parser.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/ui/chat/chat_markup_parser.cpp
)
target_include_directories(test_chat_markup_parser PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_chat_markup_parser SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/imgui
)
target_link_libraries(test_chat_markup_parser PRIVATE catch2_main)
add_test(NAME chat_markup_parser COMMAND test_chat_markup_parser)
register_test_target(test_chat_markup_parser)

# ── test_macro_evaluator ─────────────────────────────────────
add_executable(test_macro_evaluator
    test_macro_evaluator.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/ui/chat/macro_evaluator.cpp
)
target_include_directories(test_macro_evaluator PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_macro_evaluator SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_macro_evaluator PRIVATE catch2_main)
add_test(NAME macro_evaluator COMMAND test_macro_evaluator)
register_test_target(test_macro_evaluator)

# ── test_chat_tab_completer ──────────────────────────────────
add_executable(test_chat_tab_completer
    test_chat_tab_completer.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/ui/chat/chat_tab_completer.cpp
)
target_include_directories(test_chat_tab_completer PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_chat_tab_completer SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_chat_tab_completer PRIVATE catch2_main)
add_test(NAME chat_tab_completer COMMAND test_chat_tab_completer)
register_test_target(test_chat_tab_completer)

# ── test_gm_commands ─────────────────────────────────────────
add_executable(test_gm_commands
    test_gm_commands.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_gm_commands PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_gm_commands SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_gm_commands PRIVATE catch2_main)
add_test(NAME gm_commands COMMAND test_gm_commands)
register_test_target(test_gm_commands)

# ── test_open_formats ────────────────────────────────────────
add_executable(test_open_formats
    test_open_formats.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_building.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_collision.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_terrain_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wmo_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_open_formats PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_open_formats SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_open_formats PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_open_formats PRIVATE glm::glm)
endif()
add_test(NAME open_formats COMMAND test_open_formats)
register_test_target(test_open_formats)

# ── test_open_format_emitter ─────────────────────────────────
# Locks in the asset_extract → wowee open-format conversion path.
add_executable(test_open_format_emitter
    test_open_format_emitter.cpp
    ${CMAKE_SOURCE_DIR}/tools/asset_extract/open_format_emitter.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/blp_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/m2_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wmo_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/adt_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_model.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_building.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_collision.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_open_format_emitter PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)
target_include_directories(test_open_format_emitter SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/nlohmann
)
target_link_libraries(test_open_format_emitter PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_open_format_emitter PRIVATE glm::glm)
endif()
add_test(NAME open_format_emitter COMMAND test_open_format_emitter)
register_test_target(test_open_format_emitter)

# ── test_camera ──────────────────────────────────────────────
add_executable(test_camera
    test_camera.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/camera.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_camera PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_camera SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_camera PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_camera PRIVATE glm::glm)
endif()
add_test(NAME camera COMMAND test_camera)
register_test_target(test_camera)

# ── test_editor_units (SQL escape, quest validation, …) ─────
add_executable(test_editor_units
    test_editor_units.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/sql_exporter.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/npc_spawner.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/quest_editor.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/content_pack.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/editor_brush.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/object_placer.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_editor_units PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools/editor
)
target_include_directories(test_editor_units SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/nlohmann
)
target_link_libraries(test_editor_units PRIVATE catch2_main)
# npc_spawner.hpp pulls in <glm/glm.hpp>; link it when available so the
# test target builds on hosts where glm is system-installed (Homebrew etc).
if(TARGET glm::glm)
    target_link_libraries(test_editor_units PRIVATE glm::glm)
endif()
add_test(NAME editor_units COMMAND test_editor_units)
register_test_target(test_editor_units)

# ── test_expansion_profile ──────────────────────────────────
add_executable(test_expansion_profile
    test_expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_expansion_profile PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_expansion_profile SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_expansion_profile PRIVATE catch2_main)
add_test(NAME expansion_profile COMMAND test_expansion_profile)
register_test_target(test_expansion_profile)

# ── test_buff_bar_layout ────────────────────────────────────
# Header-only layout arithmetic — no ImGui needed.
add_executable(test_buff_bar_layout
    test_buff_bar_layout.cpp
)
target_include_directories(test_buff_bar_layout PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_buff_bar_layout SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_buff_bar_layout PRIVATE catch2_main)
add_test(NAME buff_bar_layout COMMAND test_buff_bar_layout)
register_test_target(test_buff_bar_layout)

# ── test_dbc_layouts ────────────────────────────────────────
# Reads the layout files shipped in Data/expansions, so it needs the source dir.
add_executable(test_dbc_layouts
    test_dbc_layouts.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_layout.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_loader.cpp
)
target_include_directories(test_dbc_layouts PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_dbc_layouts SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_dbc_layouts PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_dbc_layouts PRIVATE catch2_main)
add_test(NAME dbc_layouts COMMAND test_dbc_layouts)
register_test_target(test_dbc_layouts)

# ── test_pet_action ─────────────────────────────────────────
add_executable(test_pet_action
    test_pet_action.cpp
)
target_include_directories(test_pet_action PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_pet_action SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_pet_action PRIVATE catch2_main)
add_test(NAME pet_action COMMAND test_pet_action)
register_test_target(test_pet_action)

# ── test_spell_classification ───────────────────────────────
add_executable(test_spell_classification
    test_spell_classification.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_spell_classification PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_spell_classification SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_spell_classification PRIVATE catch2_main)
add_test(NAME spell_classification COMMAND test_spell_classification)
register_test_target(test_spell_classification)

# ── test_realm_list ─────────────────────────────────────────
add_executable(test_realm_list
    test_realm_list.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/auth_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/auth_opcodes.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/big_num.cpp
)
target_include_directories(test_realm_list PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_realm_list SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_realm_list PRIVATE catch2_main OpenSSL::SSL OpenSSL::Crypto)
add_test(NAME realm_list COMMAND test_realm_list)
register_test_target(test_realm_list)

# ── test_quest_rewards ──────────────────────────────────────
add_executable(test_quest_rewards
    test_quest_rewards.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/opcode_table.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/character.cpp
    ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_world.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_entity.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_social.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_economy.cpp
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_classic.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_tbc.cpp
)
target_include_directories(test_quest_rewards PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_quest_rewards SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_quest_rewards PRIVATE catch2_main SDL2::SDL2 ZLIB::ZLIB
                                                 OpenSSL::SSL OpenSSL::Crypto)
if(TARGET glm::glm)
    target_link_libraries(test_quest_rewards PRIVATE glm::glm)
endif()
if(TARGET Vulkan::Vulkan)
    target_link_libraries(test_quest_rewards PRIVATE Vulkan::Vulkan)
endif()
add_test(NAME quest_rewards COMMAND test_quest_rewards)
register_test_target(test_quest_rewards)

# ── test_inventory_sort ─────────────────────────────────────
add_executable(test_inventory_sort
    test_inventory_sort.cpp
    ${CMAKE_SOURCE_DIR}/src/game/inventory.cpp
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(test_inventory_sort PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_inventory_sort SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_inventory_sort PRIVATE catch2_main)
add_test(NAME inventory_sort COMMAND test_inventory_sort)
register_test_target(test_inventory_sort)

# ── test_quest_progress ─────────────────────────────────────
add_executable(test_quest_progress
    test_quest_progress.cpp
)
target_include_directories(test_quest_progress PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_quest_progress SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_quest_progress PRIVATE catch2_main)
add_test(NAME quest_progress COMMAND test_quest_progress)
register_test_target(test_quest_progress)

# ── ASAN / UBSan for test targets ────────────────────────────
if(WOWEE_ENABLE_ASAN AND NOT MSVC)
    foreach(_t IN LISTS ALL_TEST_TARGETS)
        target_compile_options(${_t} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${_t} PRIVATE -fsanitize=address,undefined)
    endforeach()
    # catch2_main must also be compiled with the same flags
    target_compile_options(catch2_main PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(catch2_main PRIVATE -fsanitize=address,undefined)
    message(STATUS "Test targets: ASAN + UBSan ENABLED")
endif()

# ── test_m2_classifier_foliage ───────────────────────────────
# Pins the wind-sway classification against substring collisions: foliage
# tokens are matched inside concatenated names, so "thorn" landed inside
# Stranglethorn and made every troll ruin sway like a tree.
add_executable(test_m2_classifier_foliage
    test_m2_classifier_foliage.cpp
    ${CMAKE_SOURCE_DIR}/src/rendering/m2_model_classifier.cpp
)
target_include_directories(test_m2_classifier_foliage PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_classifier_foliage SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_m2_classifier_foliage PRIVATE catch2_main)
if(TARGET glm::glm)
    target_link_libraries(test_m2_classifier_foliage PRIVATE glm::glm)
endif()
add_test(NAME m2_classifier_foliage COMMAND test_m2_classifier_foliage)
register_test_target(test_m2_classifier_foliage)
