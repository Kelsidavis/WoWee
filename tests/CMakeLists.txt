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
    # A global property, not a list variable. This macro is called both at file
    # scope and from inside wowee_add_test, which is a function -- and
    # list(APPEND) inside a function writes to that function's scope and is
    # discarded on return. Every test registered through wowee_add_test was
    # therefore missing from the list: 79 of ~144 targets were in it, and the
    # sanitizer loop that walks it silently skipped the rest.
    set_property(GLOBAL APPEND PROPERTY WOWEE_ALL_TEST_TARGETS ${_target})
    if(WIN32)
        target_link_libraries(${_target} PRIVATE ws2_32)
    endif()
endmacro()

# Shared source files used across multiple tests: see wowee_test_common below,
# defined once TEST_INCLUDE_DIRS exists.

# Include directories matching the main target
# GLM, resolved the three ways find_package can present it.
#
# It is not vendored: it comes from find_package(glm QUIET), and its include
# path arrives with the imported target rather than through any directory this
# file lists. Three shapes exist in the wild - GLM 1.0 exposes the headers as
# glm::glm-header-only and may build glm::glm as a real library beside it, and
# some distributions provide neither target, only GLM_INCLUDE_DIRS. A target
# that asks for glm::glm alone gets nothing in the third case and fails on
# glm/glm.hpp.
#
# Linux has a system copy under /usr/include and compiles either way, so this
# only ever shows up somewhere else - which is what makes it easy to leave
# broken. The main target and the editor have walked all three branches since
# they were written; this is the same ladder, once, for the tests.
function(wowee_test_link_glm target)
    if(TARGET glm::glm-header-only)
        target_link_libraries(${target} PRIVATE glm::glm-header-only)
    elseif(TARGET glm::glm)
        target_link_libraries(${target} PRIVATE glm::glm)
    elseif(glm_FOUND)
        target_include_directories(${target} PRIVATE ${GLM_INCLUDE_DIRS})
    endif()
endfunction()

set(TEST_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)
set(TEST_SYSTEM_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/extern
)

# Shared sources every test may opt into, compiled once.
#
# These used to be a plain source list spliced into each test's add_executable,
# which meant one object file per test that asked for it: logger.cpp alone was
# compiled 112 times in a clean build. An OBJECT library builds it once and the
# generator expression below splices the same object into each executable, so
# every existing ${TEST_COMMON_SOURCES} call site keeps working untouched.
#
# It stays a source-list-shaped variable rather than a link library on purpose.
# The opt-in property the wowee_add_test comment relies on - that a test which
# does not ask for the logger fails to link if its subject starts using one -
# only survives while omitting the variable omits the symbols.
add_library(wowee_test_common OBJECT
    ${CMAKE_SOURCE_DIR}/src/core/logger.cpp
)
target_include_directories(wowee_test_common PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(wowee_test_common SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_features(wowee_test_common PRIVATE cxx_std_20)
set(TEST_COMMON_SOURCES $<TARGET_OBJECTS:wowee_test_common>)

# Helper: a test executable with the scaffolding every one of them wants.
#
# Each test was registered by hand in nine lines of which six never varied:
# the two include-directory calls, catch2_main, glm, add_test and
# register_test_target. Copying one block to make the next is how a test came
# to link sources it did not need and how another came to miss its own.
#
# Sources are passed in full and nothing is added: several tests deliberately
# do not link TEST_COMMON_SOURCES, because a pure-domain header that starts
# using the logger should fail to link in its standalone test rather than
# quietly work. Adding it for everyone would remove that.
#
# The ctest name is the target with "test_" stripped, which is what every block
# already wrote by hand.
#
#   wowee_add_test(test_foo SOURCES test_foo.cpp ${TEST_COMMON_SOURCES})
#
# Tests that also need SDL2, OpenSSL or Vulkan keep their own block: the link
# set is the interesting part of those.
function(wowee_add_test target)
    cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
    add_executable(${target} ${ARG_SOURCES})
    target_include_directories(${target} PRIVATE ${TEST_INCLUDE_DIRS})
    target_include_directories(${target} SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE catch2_main)
    wowee_test_link_glm(${target})
    string(REGEX REPLACE "^test_" "" _name ${target})
    add_test(NAME ${_name} COMMAND ${target})
    register_test_target(${target})
endfunction()

# The packet layer, and what a test of it links.
#
# Sixteen tests named these thirteen sources and these five libraries one by
# one - the parsers pull in the rest of the layer through their shared base, so
# a test of any one packet needs all of it. Copying a neighbouring block to
# make a new one is how the guild roster test came to name the character-list
# test's sources, and it is the whole reason both of them are written out at
# all.
set(TEST_PACKET_SOURCES
    ${CMAKE_SOURCE_DIR}/src/auth/crypto.cpp
    ${CMAKE_SOURCE_DIR}/src/game/character.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/opcode_table.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_classic.cpp
    ${CMAKE_SOURCE_DIR}/src/game/packet_parsers_tbc.cpp
    ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_economy.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_entity.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_social.cpp
    ${CMAKE_SOURCE_DIR}/src/game/world_packets_world.cpp
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
)

# A test of the packet layer: its own source, the common set, the layer, and
# the libraries the layer drags in.
function(wowee_add_packet_test target)
    cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
    add_executable(${target} ${target}.cpp ${TEST_COMMON_SOURCES}
                             ${TEST_PACKET_SOURCES} ${ARG_SOURCES})
    target_include_directories(${target} PRIVATE ${TEST_INCLUDE_DIRS})
    target_include_directories(${target} SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE catch2_main SDL2::SDL2 ZLIB::ZLIB
                                            OpenSSL::SSL OpenSSL::Crypto)
    wowee_test_link_glm(${target})
    if(TARGET Vulkan::Vulkan)
        target_link_libraries(${target} PRIVATE Vulkan::Vulkan)
    endif()
    string(REGEX REPLACE "^test_" "" _name ${target})
    add_test(NAME ${_name} COMMAND ${target})
    register_test_target(${target})
endfunction()



# What a vendor sell asks for. Header-only arithmetic, no game state.
wowee_add_test(test_sell_count
    SOURCES test_sell_count.cpp)

# Stock-style hill climbing regression test. Run directly with:
#   cmake --build build --target test_hill_climbing && ctest --test-dir build -R hill_climbing
wowee_add_test(test_hill_climbing
    SOURCES test_movement_limits.cpp)

# ── test_cli_catalog_paths ──────────────────────────────────
# Working a catalog's base name back out of a path the user typed. Pure header
# in tools/editor, so the test reaches into that directory for it.
add_executable(test_cli_catalog_paths test_cli_catalog_paths.cpp ${TEST_COMMON_SOURCES})
target_include_directories(test_cli_catalog_paths PRIVATE
    ${TEST_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/tools/editor)
target_include_directories(test_cli_catalog_paths SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_cli_catalog_paths PRIVATE catch2_main)
add_test(NAME cli_catalog_paths COMMAND test_cli_catalog_paths)
register_test_target(test_cli_catalog_paths)

# ── test_shared_rules ───────────────────────────────────────
# The rules this codebase collapsed out of many copies into one. A rule with a
# single home is a rule one mistake breaks everywhere, so the cases that were
# actually wrong in one of the copies are pinned here.
# Pure rules only - the item texture paths and the settings vocabulary are
# header-only, and skinPathForM2 comes from m2_loader, which knows nothing about
# where bytes come from.
wowee_add_test(test_shared_rules
    SOURCES test_shared_rules.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/m2_loader.cpp
            ${CMAKE_SOURCE_DIR}/src/ui/settings_schema.cpp
            ${TEST_COMMON_SOURCES})

# ── test_geoset_rules ────────────────────────────────────────
# Choosing a geoset when the model lacks the one asked for. Pure header, so the
# test needs nothing but Catch.
add_executable(test_geoset_rules test_geoset_rules.cpp ${TEST_COMMON_SOURCES})
target_include_directories(test_geoset_rules PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_geoset_rules SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_geoset_rules PRIVATE catch2_main)
add_test(NAME geoset_rules COMMAND test_geoset_rules)
register_test_target(test_geoset_rules)

# ── test_framexml ───────────────────────────────────────────
add_executable(test_framexml
    test_framexml.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/xml_parser.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/framexml_emitter.cpp
)
target_include_directories(test_framexml PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_framexml SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_framexml PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_framexml PRIVATE catch2_main)
add_test(NAME framexml COMMAND test_framexml)
register_test_target(test_framexml)

# ── test_framexml_takeover ──────────────────────────────────
# Which interface draws what in the configuration a run actually gets. The
# transition's state is "FrameXML draws all of it", and that rested on a list
# of forty-nine names and a grouping rule agreeing about fifty-two elements,
# in a file where either can be edited alone. An element falling out is not an
# error - this client just draws its own version again.
add_executable(test_framexml_takeover
    test_framexml_takeover.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/framexml_takeover.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_framexml_takeover PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_framexml_takeover SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_framexml_takeover PRIVATE catch2_main)
add_test(NAME framexml_takeover COMMAND test_framexml_takeover)
register_test_target(test_framexml_takeover)

# ── test_lua_handler_globals ────────────────────────────────
# The pre-3.0 handler globals, which are a pure function of the Lua stack and
# so need no game, window or interface - while the fault they prevent is the
# largest one this client can have: `this` nil on the first line of every
# OnLoad in a 1.12 interface, so every frame it declares reaches the screen at
# once and none of them where it belongs.
add_executable(test_lua_handler_globals
    test_lua_handler_globals.cpp
    ${CMAKE_SOURCE_DIR}/src/addons/lua_handler_globals.cpp
)
target_include_directories(test_lua_handler_globals PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_lua_handler_globals SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/extern/lua-5.1.5/src)
target_link_libraries(test_lua_handler_globals PRIVATE catch2_main lua51)
wowee_test_link_glm(test_lua_handler_globals)
add_test(NAME lua_handler_globals COMMAND test_lua_handler_globals)
register_test_target(test_lua_handler_globals)

# ── test_lua_generic_for ────────────────────────────────────
# `for k, v in t do` over a plain table - the Lua 5.0 form 1.12's FrameXML is
# written in. It raised on the vendored 5.1, and raising while a file is read
# takes the whole file with it: six of FrameXML's died on this one construct,
# the player, target and party frames among them.
add_executable(test_lua_generic_for
    test_lua_generic_for.cpp
)
target_include_directories(test_lua_generic_for PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_lua_generic_for SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/extern/lua-5.1.5/src)
target_link_libraries(test_lua_generic_for PRIVATE catch2_main lua51)
wowee_test_link_glm(test_lua_generic_for)
add_test(NAME lua_generic_for COMMAND test_lua_generic_for)
register_test_target(test_lua_generic_for)

# ── test_lua_arg_coercion ───────────────────────────────────
# The lenient numeric argument, which is a pure function of the Lua stack and
# so needs no game, window or interface - while the fault it prevents is
# invisible at every other level: a raise inside a click handler is swallowed,
# so an auction search that never sent looked exactly like one that found
# nothing.
add_executable(test_lua_arg_coercion
    test_lua_arg_coercion.cpp
)
target_include_directories(test_lua_arg_coercion PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_lua_arg_coercion SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/extern/lua-5.1.5/src)
target_link_libraries(test_lua_arg_coercion PRIVATE catch2_main lua51)
wowee_test_link_glm(test_lua_arg_coercion)
add_test(NAME lua_arg_coercion COMMAND test_lua_arg_coercion)
register_test_target(test_lua_arg_coercion)

# ── framexml_emit ────────────────────────────────────────────
# Not a test: a one-file dump of what the emitter produces, for answering
# questions about emitted Lua without starting the client.
add_executable(framexml_emit
    ${CMAKE_SOURCE_DIR}/tools/framexml_emit.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/framexml_emitter.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/xml_parser.cpp
)
target_include_directories(framexml_emit PRIVATE ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/lua-5.1.5/src)
target_include_directories(framexml_emit SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(framexml_emit PRIVATE lua51)

# ── test_settings_schema_consistency ─────────────────────────
# Three readers build controls out of the schema and none can notice a row that
# contradicts itself. A dropdown whose range says three entries while its
# labels name two offers two and writes indices the labels cannot name.
add_executable(test_settings_schema_consistency
    test_settings_schema_consistency.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/settings_schema.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_settings_schema_consistency PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_settings_schema_consistency SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_settings_schema_consistency PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_settings_schema_consistency PRIVATE catch2_main)
add_test(NAME settings_schema_consistency COMMAND test_settings_schema_consistency)
register_test_target(test_settings_schema_consistency)

# ── test_settings_panel_layout ───────────────────────────────
# The options panels are generated from the schema by Lua, and nothing else
# checks the arithmetic between them. A control laid out past the bottom of a
# panel registers, refreshes and answers correctly - it is just not on screen.
add_executable(test_settings_panel_layout
    test_settings_panel_layout.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/settings_schema.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_settings_panel_layout PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_settings_panel_layout SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_settings_panel_layout PRIVATE catch2_main)
add_test(NAME settings_panel_layout COMMAND test_settings_panel_layout)
register_test_target(test_settings_panel_layout)

# ── test_addon_lua_snippets ──────────────────────────────────
# The Lua this client injects into the interface lives as C++ string literals,
# which nothing compiles until the client runs - and a syntax error there is a
# false from executeString and a line in a warning-only log. This asks Lua
# whether they parse.
add_executable(test_addon_lua_snippets
    test_addon_lua_snippets.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_addon_lua_snippets PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_addon_lua_snippets SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/extern/lua-5.1.5/src)
target_link_libraries(test_addon_lua_snippets PRIVATE catch2_main lua51)
add_test(NAME addon_lua_snippets COMMAND test_addon_lua_snippets)
register_test_target(test_addon_lua_snippets)

# ── framexml_compiles ───────────────────────────────────────
# Not a unit test: it emits every real FrameXML file and asks Lua itself
# whether the result compiles. The unit tests above check cases someone
# thought of; this checks all 126, and a file that will not compile is lost
# whole rather than degraded. Skipped when the game data is not present.
add_executable(framexml_compile_check
    ${CMAKE_SOURCE_DIR}/tools/framexml_compile_check.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/xml_parser.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/framexml_emitter.cpp
)
target_include_directories(framexml_compile_check PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(framexml_compile_check SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/extern/lua-5.1.5/src)
target_link_libraries(framexml_compile_check PRIVATE lua51)
add_test(NAME framexml_compiles COMMAND framexml_compile_check
         ${CMAKE_SOURCE_DIR}/Data/interface/framexml)
set_tests_properties(framexml_compiles PROPERTIES SKIP_RETURN_CODE 77)

# The Blizzard addons go through the same emitter and were never checked by it.
# Their XML is where the load-on-demand panels come from - a file that will not
# compile takes its whole panel with it, and only when someone opens it.
add_test(NAME addon_xml_compiles COMMAND framexml_compile_check
         ${CMAKE_SOURCE_DIR}/Data/interface/addons)
set_tests_properties(addon_xml_compiles PROPERTIES SKIP_RETURN_CODE 77)

# ── test_taxi_map_projection ────────────────────────────────
# The order of the coordinate conversions behind a flight map marker. Silent
# when wrong - every marker still lands somewhere, just not where it belongs.
wowee_add_test(test_taxi_map_projection
    SOURCES test_taxi_map_projection.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/coordinate_projection.cpp)

# ── test_inventory_slots ─────────────────────────────────────────
# Where the bank, its bags and the keyring sit. A wrong slot here moves an
# item somewhere real that nobody asked for, so the numbers are pinned.
add_executable(test_inventory_slots test_inventory_slots.cpp)
target_include_directories(test_inventory_slots PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_inventory_slots SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_inventory_slots PRIVATE catch2_main)
add_test(NAME inventory_slots COMMAND test_inventory_slots)
register_test_target(test_inventory_slots)

# ── test_jit_write ───────────────────────────────────────────────
# Writing to a MAP_JIT mapping, which Apple Silicon refuses until the thread
# asks. The Warden module image is mmap'd that way and then zeroed, and the
# zeroing raised SIGBUS - a crash with no bad pointer in it, found by fuzzing
# the loader with bytes a server could send.
add_executable(test_jit_write test_jit_write.cpp)
target_include_directories(test_jit_write PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_jit_write SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_jit_write PRIVATE catch2_main)
add_test(NAME jit_write COMMAND test_jit_write)
register_test_target(test_jit_write)

# ── test_warden_reloc ────────────────────────────────────────────
# Where a Warden relocation entry is allowed to point. The module comes from
# the server, so both of these decide where an attacker-supplied number may be
# written; one of them let a four-byte write land at image + 0xFFFFFFFF.
add_executable(test_warden_reloc test_warden_reloc.cpp)
target_include_directories(test_warden_reloc PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_warden_reloc SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_warden_reloc PRIVATE catch2_main)
add_test(NAME warden_reloc COMMAND test_warden_reloc)
register_test_target(test_warden_reloc)

# ── sweep_guard ──────────────────────────────────────────────────
# The fast interface sweeps, run as a test so they stop depending on someone
# remembering. Each fault they catch raises nothing, logs nothing and fails no
# other test: a panel drawn twice, a binding answering short, a manifest entry
# resolving to no file, bootstrap Lua replacing the binding beneath it. Under
# three seconds for all of them; the slower reports stay manual.
if(Python3_Interpreter_FOUND)
    add_test(NAME sweep_guard
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/sweep_guard.py)
endif()

# ── unicorn_stub_compiles ────────────────────────────────────────
# Unicorn is optional and every build here has it, so the stub branch the
# Warden emulator falls back to compiles in no configuration anyone runs. It
# rotted that way: issue #115 was a stub defining a method the header does
# not declare, found by someone building without Unicorn and invisible to us.
# Parses the two translation units with the define removed. A second or two.
if(Python3_Interpreter_FOUND)
    add_test(NAME unicorn_stub_compiles
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_SOURCE_DIR}/tools/unicorn_stub_check.py
                     ${CMAKE_BINARY_DIR})
endif()

# ── android_data_profiles ───────────────────────────────────────
# The profiles in tools/android decide what an 18 GB extraction is cut down to
# for a phone. A pattern that stops matching drops files silently and only says
# so on a device. It has happened once already: * crossed a separator and a
# trailing /** was compared literally, so one exclusion matched nothing at all.
# Needs no extraction, so it runs here rather than by hand. Instant.
if(Python3_Interpreter_FOUND)
    add_test(NAME android_data_profiles
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_SOURCE_DIR}/tools/android/test_data_profiles.py)
endif()

# ── test_chat_types ─────────────────────────────────────────────
# A chat type is a byte off the wire and the event name is built from it, so a
# value missing from the enum is a line of chat that never appears - no error,
# no log. The whole run between LOOT and the battleground block was absent once.
wowee_add_test(test_chat_types
    SOURCES test_chat_types.cpp)

# ── test_chat_window_background ─────────────────────────────────
# The chat panel is a white image the interface tints, so the colour this client
# saves is the whole of what it looks like - and it saved white at full alpha,
# which covers the chat with the text lost in it. Fixing the default left every
# install that had already written that white unfixed, and it came back as a
# report after the fix had shipped.
wowee_add_test(test_chat_window_background
    SOURCES test_chat_window_background.cpp)

# ── test_text_tokens ────────────────────────────────────────
# The server writes the player as a blank - "$N, you have done well" - and the
# client fills it in. Nothing did on the chat path, so every scripted NPC in the
# game addressed the player as "$N". Pure: the substitution takes the four facts
# it needs about the reader rather than a client.
add_executable(test_text_tokens test_text_tokens.cpp ${CMAKE_SOURCE_DIR}/src/game/text_tokens.cpp)
target_include_directories(test_text_tokens PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_text_tokens SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_text_tokens PRIVATE catch2_main)
add_test(NAME text_tokens COMMAND test_text_tokens)
register_test_target(test_text_tokens)

# ── test_reputation_standing ────────────────────────────────
# Header-only and constexpr, so this needs nothing linked: the point is that the
# thresholds have one home now and stay put.
add_executable(test_reputation_standing test_reputation_standing.cpp)
target_include_directories(test_reputation_standing PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_reputation_standing SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_reputation_standing PRIVATE catch2_main)
add_test(NAME reputation_standing COMMAND test_reputation_standing)
register_test_target(test_reputation_standing)

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

# ── test_text_edit ──────────────────────────────────────────
wowee_add_test(test_text_edit SOURCES
    test_text_edit.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/text_edit.cpp
)

# ── test_escape_action ──────────────────────────────────────
add_executable(test_escape_action
    test_escape_action.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/escape_action.cpp
)
target_include_directories(test_escape_action PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_escape_action SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_escape_action PRIVATE catch2_main)
add_test(NAME escape_action COMMAND test_escape_action)
register_test_target(test_escape_action)

# ── test_placement_rotation ─────────────────────────────────
wowee_add_test(test_placement_rotation
    SOURCES test_placement_rotation.cpp)

# ── test_water_mask ─────────────────────────────────────────
add_executable(test_water_mask
    test_water_mask.cpp
)
target_include_directories(test_water_mask PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_water_mask SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_water_mask PRIVATE catch2_main)
add_test(NAME water_mask COMMAND test_water_mask)
register_test_target(test_water_mask)

# ── test_addon_globals ──────────────────────────────────────
add_executable(test_addon_globals
    test_addon_globals.cpp
)
target_include_directories(test_addon_globals PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_addon_globals SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_addon_globals PRIVATE catch2_main)
add_test(NAME addon_globals COMMAND test_addon_globals)
register_test_target(test_addon_globals)

# ── test_monster_move_facing ─────────────────────────────────
# SMSG_MONSTER_MOVE's move-type byte and the facing after it, which three
# expansion parsers used to read for themselves. A wrong length here misreads
# every field that follows.
wowee_add_test(test_monster_move_facing
    SOURCES test_monster_move_facing.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
            ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp)

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

# ── test_bit_packet ──────────────────────────────────────────
add_executable(test_bit_packet
    test_bit_packet.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
)
target_include_directories(test_bit_packet PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_bit_packet SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_bit_packet PRIVATE catch2_main)
add_test(NAME bit_packet COMMAND test_bit_packet)
register_test_target(test_bit_packet)

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

# ── test_m2_ribbon_tracks ────────────────────────────────────
# A ribbon emitter's alpha and visibility tracks, whose keys are fixed16 and
# bytes rather than floats. Real-asset coverage; skips when unextracted.
add_executable(test_m2_ribbon_tracks
    test_m2_ribbon_tracks.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/pipeline/m2_loader.cpp
)
target_include_directories(test_m2_ribbon_tracks PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_ribbon_tracks SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_m2_ribbon_tracks PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_m2_ribbon_tracks PRIVATE catch2_main)
wowee_test_link_glm(test_m2_ribbon_tracks)
add_test(NAME m2_ribbon_tracks COMMAND test_m2_ribbon_tracks)
register_test_target(test_m2_ribbon_tracks)

# ── test_shapeshift_forms ────────────────────────────────────
# The stance bar's three questions, which used to be three different lists.
wowee_add_test(test_shapeshift_forms
    SOURCES test_shapeshift_forms.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/shapeshift_forms.cpp)

# ── test_tile_size_agreement ─────────────────────────────────
# The ADT tile size, which three headers each declared and two files spelled
# with two fewer decimals. Pins the value against its own arithmetic.
wowee_add_test(test_tile_size_agreement
    SOURCES test_tile_size_agreement.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/pipeline/terrain_mesh.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_alpha.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_loader.cpp)

# ── test_chunk_surface ───────────────────────────────────────
# Where a point inside a terrain chunk sits in the world. The clutter and
# doodad scatterers each had their own copy, including the crossed axes.
wowee_add_test(test_chunk_surface
    SOURCES test_chunk_surface.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/pipeline/terrain_mesh.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_alpha.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_loader.cpp)

# ── test_spline_body ─────────────────────────────────────────
# The body of SMSG_MONSTER_MOVE, whose head two parsers shared and neither
# covered. Checked against AzerothCore's own WriteCommonMonsterMovePart.
wowee_add_test(test_spline_body
    SOURCES test_spline_body.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
            ${CMAKE_SOURCE_DIR}/src/network/packet.cpp)

# ── test_spline ──────────────────────────────────────────────
wowee_add_test(test_spline
    SOURCES test_spline.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/math/spline.cpp
            ${CMAKE_SOURCE_DIR}/src/game/spline_packet.cpp
            ${CMAKE_SOURCE_DIR}/src/network/packet.cpp)

# ── test_transport_path_repo ─────────────────────────────────
wowee_add_test(test_transport_path_repo
    SOURCES test_transport_path_repo.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/transport_path_repository.cpp
            ${CMAKE_SOURCE_DIR}/src/math/spline.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/dbc_loader.cpp)

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

# ── test_equipment_set_packets ───────────────────────────────
# Every guid in the equipment-set family is packed and all twenty were read and
# written as fixed eight-byte values - plus an ignore mask that the server does
# not send at all. Saving a set could never have worked.
add_executable(test_equipment_set_packets
    test_equipment_set_packets.cpp
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
)
target_include_directories(test_equipment_set_packets PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_equipment_set_packets SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_equipment_set_packets PRIVATE catch2_main)
add_test(NAME equipment_set_packets COMMAND test_equipment_set_packets)
register_test_target(test_equipment_set_packets)

# ── test_auction_sort_packet ─────────────────────────────────
# The sort block on CMSG_AUCTION_LIST_ITEMS was a hardcoded zero, and the
# browse tab re-queries rather than sorting locally, so its column headers did
# nothing at all. Two orderings have to agree and they run opposite ways.
wowee_add_test(test_auction_sort_packet
    SOURCES test_auction_sort_packet.cpp
            ${CMAKE_SOURCE_DIR}/src/network/packet.cpp)

# ── test_achievement_criteria ────────────────────────────────
# The criteria progress record. Its counter is a packed guid and was read as a
# fixed uint64, which put every field behind it out of place - and in the login
# message, where records repeat until a sentinel, took the whole list with it.
add_executable(test_achievement_criteria
    test_achievement_criteria.cpp
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
)
target_include_directories(test_achievement_criteria PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_achievement_criteria SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_achievement_criteria PRIVATE catch2_main)
add_test(NAME achievement_criteria COMMAND test_achievement_criteria)
register_test_target(test_achievement_criteria)

# ── test_taxi_packets ────────────────────────────────────────
# The multi-hop flight request. A field the server does not read was being
# written between the guid and the node count, so the count was read as a node
# and every route longer than one hop was refused.
wowee_add_packet_test(test_taxi_packets)

# ── test_use_item_packet ─────────────────────────────────────
wowee_add_packet_test(test_use_item_packet)

# ── test_normal_map ────────────────────────────────────────────────────
# The normal-map pass the character and WMO renderers used to duplicate. Both
# copies lived inside a renderer class and needed a Vulkan context to reach, so
# this is its first test.
wowee_add_test(test_normal_map
    SOURCES test_normal_map.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/rendering/normal_map.cpp)

# ── test_spatial_grid ────────────────────────────────────────
# The uniform grid the M2 and WMO renderers narrow collision queries with. Both
# had their own copy, private to a renderer and unreachable without a Vulkan
# context, so this is its first test.
wowee_add_test(test_spatial_grid
    SOURCES test_spatial_grid.cpp
            ${TEST_COMMON_SOURCES})

# ── test_cata_movement ─────────────────────────────────────────────────
# The 109 Cataclysm movement layouts derived from a 4.3.4 core, and the reader
# that executes them. Runs from the source dir for the derived JSON.
add_executable(test_cata_movement
    test_cata_movement.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/game/cata_movement.cpp
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp)
target_include_directories(test_cata_movement PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_cata_movement SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_cata_movement PRIVATE catch2_main)
wowee_test_link_glm(test_cata_movement)
add_test(NAME cata_movement COMMAND test_cata_movement
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
register_test_target(test_cata_movement)

# ── test_base_fallback ─────────────────────────────────────────────────
# Whether one client's asset tree may answer for another's missing files.
add_executable(test_base_fallback
    test_base_fallback.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_base_fallback PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_base_fallback SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_base_fallback PRIVATE catch2_main)
add_test(NAME base_fallback COMMAND test_base_fallback)
register_test_target(test_base_fallback)

# ── test_asset_manifest_load ───────────────────────────────────────────
# The 32 MB asset manifest, pinned so a change to how it is read cannot change
# what it read. Skips when Data/manifest.json is not in the checkout.
add_executable(test_asset_manifest_load
    test_asset_manifest_load.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/pipeline/asset_manifest.cpp)
target_include_directories(test_asset_manifest_load PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_asset_manifest_load SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_asset_manifest_load PRIVATE catch2_main)
wowee_test_link_glm(test_asset_manifest_load)
add_test(NAME asset_manifest_load COMMAND test_asset_manifest_load
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
register_test_target(test_asset_manifest_load)

# ── test_macro_text ──────────────────────────────────────────
# Reading a macro body. The action bar and the chat panel each had their own
# copy of the line walk, neither with a test.
wowee_add_test(test_macro_text
    SOURCES test_macro_text.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/ui/macro_text.cpp)

# ── test_water_surface_grid ──────────────────────────────────
# Projecting a world position onto a water surface's grid, which
# water_renderer.cpp did in four places and never tested.
wowee_add_test(test_water_surface_grid
    SOURCES test_water_surface_grid.cpp
            ${TEST_COMMON_SOURCES})

# ── test_dxt_block ───────────────────────────────────────────
# The colour half of a DXT block, decoded three times in blp_loader.cpp and
# never tested; the BLP tests stop at header validation.
wowee_add_test(test_dxt_block
    SOURCES test_dxt_block.cpp
            ${TEST_COMMON_SOURCES})

# ── test_vertex_layouts ──────────────────────────────────────
# The WMO and terrain vertex layouts against the shaders that read them. Each
# layout was stated three or four times, once with the stride and every offset
# as a literal; the shader source is the outside oracle.
add_executable(test_vertex_layouts
    test_vertex_layouts.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_vertex_layouts PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_vertex_layouts SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_vertex_layouts PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_vertex_layouts PRIVATE catch2_main)
wowee_test_link_glm(test_vertex_layouts)
add_test(NAME vertex_layouts COMMAND test_vertex_layouts)
register_test_target(test_vertex_layouts)

# ── test_movement_flags ──────────────────────────────────────
# What the movement flag word means. Two handlers spelled these out against
# their own copy of the flags; they now both ask MovementInfo. Header-only, so
# no sources beyond the common set.
wowee_add_test(test_movement_flags
    SOURCES test_movement_flags.cpp
            ${TEST_COMMON_SOURCES})

# ── test_attacker_state_layout ─────────────────────────────────────────
# SMSG_ATTACKERSTATEUPDATE for classic and TBC, pinned before the shared head
# was factored out. Same link set as the other two layout tests.
wowee_add_packet_test(test_attacker_state_layout)

# ── test_spell_damage_log_layout ───────────────────────────────────────
# SMSG_SPELLNONMELEEDAMAGELOG before WotLK, pinned before the Classic and TBC
# readers were merged. The expansions differ by one field - overkill - and
# reading it where it is absent shifts everything after it by four bytes.
wowee_add_packet_test(test_spell_damage_log_layout)

# ── test_prewotlk_build_packets ────────────────────────────────────────
# The three client packets Classic and TBC build identically, pinned before
# the builders were merged. Each is a WotLK packet minus a leading packed GUID
# or a trailing uint32, and sending the extra bytes fails silently.
wowee_add_packet_test(test_prewotlk_build_packets)

# ── test_item_query_layout ─────────────────────────────────────────────
# SMSG_ITEM_QUERY_SINGLE_RESPONSE for classic and TBC, pinned before the two
# readers were merged. Same link set as test_char_enum_layout.
wowee_add_packet_test(test_item_query_layout)

# ── test_guild_bank_list ───────────────────────────────────────────────
# SMSG_GUILD_BANK_LIST, whose tab-name block is optional and was gated on the
# wrong condition - so a tab query that answered with the names attached had
# every field after them read a block early, and the vault drew empty.
wowee_add_packet_test(test_guild_bank_list)

# ── test_gameobject_query_layout ───────────────────────────────────────
# SMSG_GAMEOBJECT_QUERY_RESPONSE, parsed three times, differing only in how
# many strings sit before the twenty-four data fields.
wowee_add_packet_test(test_gameobject_query_layout)

# ── test_guild_roster_layout ───────────────────────────────────────────
# SMSG_GUILD_ROSTER, parsed by three separate functions with no coverage at
# all. Same link set as test_char_enum_layout, and for the same reason.
wowee_add_packet_test(test_guild_roster_layout)

# ── test_char_enum_layout ──────────────────────────────────────────────
# SMSG_CHAR_ENUM, parsed by three separate functions with no coverage at all.
# Links the same set test_taxi_packets does: the classic and TBC parsers pull
# in the rest of the packet layer through their shared base.
wowee_add_packet_test(test_char_enum_layout)

# ── test_loot_response_packet ─────────────────────────────────────────
# The quest-item section is WotLK's, and whether the parser looks for it comes
# from this client's configured expansion rather than from the realm. A
# mismatch used to lose the whole response - regular items included - so a
# chest opened and no window came up.
wowee_add_packet_test(test_loot_response_packet)

# ── test_battlefield_status_packet ─────────────────────────────────────
# SMSG_BATTLEFIELD_STATUS, whose header was read two bytes short.
wowee_add_packet_test(test_battlefield_status_packet)

# ── test_lfg_join_packet ─────────────────────────────────────
# Every field of CMSG_LFG_JOIN, which was wrong in every field and silent.
wowee_add_packet_test(test_lfg_join_packet)

# ── test_learn_talent_packet ─────────────────────────────────
# The rank field counts from zero on the wire and from one everywhere else.
wowee_add_packet_test(test_learn_talent_packet)

# ── test_item_query_sockets ─────────────────────────────────
# The socket block of the item query, which the layout sweep cannot reach.
wowee_add_packet_test(test_item_query_sockets)

# ── test_item_stat_names ────────────────────────────────────
# Which of the two stat-name tables answers for the five primaries. A random
# suffix rolls raw ITEM_MOD types and was reading the table that names only the
# extras, so every animal suffix listed no stats at all.
wowee_add_test(test_item_stat_names SOURCES test_item_stat_names.cpp)

# ── test_mail_packets ───────────────────────────────────────
wowee_add_packet_test(test_mail_packets)

# ── test_raid_target_packets ────────────────────────────────
wowee_add_packet_test(test_raid_target_packets)

# ── test_entity ──────────────────────────────────────────────
wowee_add_test(test_entity
    SOURCES test_entity.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/entity.cpp
            ${CMAKE_SOURCE_DIR}/src/math/spline.cpp)

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
wowee_add_test(test_m2_structs
    SOURCES test_m2_structs.cpp
            ../src/rendering/m2_model_classifier.cpp)

# ── test_m2_color_alpha ──────────────────────────────────────
# Loads the peasant lumberjack carry models (WotLK v264 + vanilla v256) and
# pins the M2 color-alpha track parsing that hides the duplicate wood bundle.
add_executable(test_m2_color_alpha
    test_m2_color_alpha.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/m2_loader.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_m2_color_alpha PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_color_alpha SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_m2_color_alpha PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_m2_color_alpha PRIVATE catch2_main)
wowee_test_link_glm(test_m2_color_alpha)
add_test(NAME m2_color_alpha COMMAND test_m2_color_alpha)
register_test_target(test_m2_color_alpha)

# ── test_m2_footstep_events ──────────────────────────────────
# Loads real WotLK v264 + vanilla v256 models and pins the $FSD footfall
# event parsing that syncs footstep audio to foot-strike keyframes.
add_executable(test_m2_footstep_events
    test_m2_footstep_events.cpp
    ${CMAKE_SOURCE_DIR}/src/pipeline/m2_loader.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_m2_footstep_events PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_footstep_events SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_m2_footstep_events PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_m2_footstep_events PRIVATE catch2_main)
wowee_test_link_glm(test_m2_footstep_events)
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

# ── test_blp_alpha_scan ──────────────────────────────────────
# M2 textures were decoded to RGBA8 so something could walk every fourth byte
# and learn whether the texture had any transparency. The block scan that
# replaced it has to answer the same, on real files rather than on the cases
# someone thought to write down.
add_executable(test_blp_alpha_scan
    test_blp_alpha_scan.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/pipeline/blp_loader.cpp
)
target_include_directories(test_blp_alpha_scan PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_blp_alpha_scan SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_blp_alpha_scan PRIVATE WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_blp_alpha_scan PRIVATE catch2_main)
add_test(NAME blp_alpha_scan COMMAND test_blp_alpha_scan)
register_test_target(test_blp_alpha_scan)

# ── test_frustum ─────────────────────────────────────────────
wowee_add_test(test_frustum
    SOURCES test_frustum.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/frustum.cpp)

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

# ── test_chat_filters ────────────────────────────────────────
add_executable(test_chat_filters
    test_chat_filters.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_chat_filters PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_chat_filters SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_chat_filters PRIVATE catch2_main)
add_test(NAME chat_filters COMMAND test_chat_filters)
register_test_target(test_chat_filters)

# ── test_nameplate_stacking ──────────────────────────────────
add_executable(test_nameplate_stacking
    test_nameplate_stacking.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_nameplate_stacking PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_nameplate_stacking SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_nameplate_stacking PRIVATE catch2_main)
add_test(NAME nameplate_stacking COMMAND test_nameplate_stacking)
register_test_target(test_nameplate_stacking)

# ── test_combat_text_filter ──────────────────────────────────
add_executable(test_combat_text_filter
    test_combat_text_filter.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_combat_text_filter PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_combat_text_filter SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_combat_text_filter PRIVATE catch2_main)
add_test(NAME combat_text_filter COMMAND test_combat_text_filter)
register_test_target(test_combat_text_filter)

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

# NPC animator tests removed - NpcAnimator replaced by generic CharacterAnimator

# ── test_anim_capability ─────────────────────────────────────
# Header-only struct tests - no source files needed
add_executable(test_anim_capability
    test_anim_capability.cpp
)
target_include_directories(test_anim_capability PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_anim_capability SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_anim_capability PRIVATE catch2_main)
add_test(NAME anim_capability COMMAND test_anim_capability)
register_test_target(test_anim_capability)

# ── test_indoor_shadows ──────────────────────────────────────
wowee_add_test(test_indoor_shadows
    SOURCES test_indoor_shadows.cpp)

# ── test_zone_ambience ─────────────────────────────────────────
wowee_add_test(test_zone_ambience
    SOURCES test_zone_ambience.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/zone_ambience.cpp)

# ── test_transport_components ────────────────────────────────
wowee_add_test(test_transport_components
    SOURCES test_transport_components.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/transport_clock_sync.cpp
            ${CMAKE_SOURCE_DIR}/src/game/transport_animator.cpp
            ${CMAKE_SOURCE_DIR}/src/math/spline.cpp)

# ── test_world_map ────────────────────────────────────────────
wowee_add_test(test_world_map
    SOURCES test_world_map.cpp)

# ── test_world_map_coordinate_projection ──────────────────────
wowee_add_test(test_world_map_coordinate_projection
    SOURCES test_world_map_coordinate_projection.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/coordinate_projection.cpp)

# ── test_world_map_map_resolver ───────────────────────────────
wowee_add_test(test_world_map_map_resolver
    SOURCES test_world_map_map_resolver.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/map_resolver.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/coordinate_projection.cpp
            ${TEST_COMMON_SOURCES})

# ── test_world_map_view_state_machine ─────────────────────────
wowee_add_test(test_world_map_view_state_machine
    SOURCES test_world_map_view_state_machine.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/view_state_machine.cpp)

# ── test_world_map_exploration_state ──────────────────────────
wowee_add_test(test_world_map_exploration_state
    SOURCES test_world_map_exploration_state.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/exploration_state.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/coordinate_projection.cpp)

# ── test_world_map_zone_metadata ──────────────────────────────
wowee_add_test(test_world_map_zone_metadata
    SOURCES test_world_map_zone_metadata.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/world_map/zone_metadata.cpp)

# ── test_text_wrap ───────────────────────────────────────────
add_executable(test_text_wrap
    test_text_wrap.cpp
)
target_include_directories(test_text_wrap PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_text_wrap SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
)
target_link_libraries(test_text_wrap PRIVATE catch2_main)
add_test(NAME text_wrap COMMAND test_text_wrap)
register_test_target(test_text_wrap)

# ── test_text_markup ─────────────────────────────────────────
add_executable(test_text_markup
    test_text_markup.cpp
)
target_include_directories(test_text_markup PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_text_markup SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
)
target_link_libraries(test_text_markup PRIVATE catch2_main)
add_test(NAME text_markup COMMAND test_text_markup)
register_test_target(test_text_markup)

# ── test_link_hit ────────────────────────────────────────────
add_executable(test_link_hit
    test_link_hit.cpp
)
target_include_directories(test_link_hit PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_link_hit SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
)
target_link_libraries(test_link_hit PRIVATE catch2_main)
add_test(NAME link_hit COMMAND test_link_hit)
register_test_target(test_link_hit)

# ── test_minimap_projection ──────────────────────────────────
add_executable(test_minimap_projection
    test_minimap_projection.cpp
)
target_include_directories(test_minimap_projection PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_minimap_projection SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
)
target_link_libraries(test_minimap_projection PRIVATE catch2_main)
wowee_test_link_glm(test_minimap_projection)
add_test(NAME minimap_projection COMMAND test_minimap_projection)
register_test_target(test_minimap_projection)
register_test_target(test_text_wrap)

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

# ── test_font_em_scale ───────────────────────────────────────
# The head/hhea read that makes a font height mean an em, as FrameXML means it,
# rather than the ascender-to-descender span ImGui fits into it. Absolute byte
# offsets into a file this client did not write, so they are pinned.
add_executable(test_font_em_scale
    test_font_em_scale.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/interface_fonts.cpp
)
target_include_directories(test_font_em_scale PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_font_em_scale SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/imgui
)
target_link_libraries(test_font_em_scale PRIVATE catch2_main imgui)
add_test(NAME font_em_scale COMMAND test_font_em_scale)
register_test_target(test_font_em_scale)

# ── test_quest_text ──────────────────────────────────────────
# Which of a quest's five strings holds the line a finished quest shows. The
# pick used to be "whichever is longer", which answered a different question.
wowee_add_test(test_quest_text SOURCES test_quest_text.cpp)

# ── test_interface_text_lines ────────────────────────────────
# How many lines a label takes, measured on demand rather than read off the
# last frame. The quest tracker sizes every row from that answer, and a stale
# zero drew each wrapped objective over the row beneath it.
add_executable(test_interface_text_lines
    test_interface_text_lines.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/interface_fonts.cpp
)
target_include_directories(test_interface_text_lines PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_interface_text_lines SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/imgui
)
target_link_libraries(test_interface_text_lines PRIVATE catch2_main imgui)
add_test(NAME interface_text_lines COMMAND test_interface_text_lines)
register_test_target(test_interface_text_lines)

# ── test_npc_flags ───────────────────────────────────────────
# UNIT_NPC_FLAGS bit by bit. A wrong one is invisible - the mask just always
# answers no - and NPC_FLAG_VENDOR was 0x04, a bit nothing sets, so no merchant
# ever read as one.
wowee_add_test(test_npc_flags SOURCES test_npc_flags.cpp)

# ── test_scene_pick ──────────────────────────────────────────
# Which of ScenePick's answers a caller wants. closestGuid and resolve()
# disagree exactly where a wide object stands beside an NPC, which is where the
# vendor cursor was asking the wrong one.
wowee_add_test(test_scene_pick SOURCES test_scene_pick.cpp)

# ── test_plural_escape ───────────────────────────────────────
# WoW's |4singular:plural; escape, which nothing resolved - so the logout
# prompt read "5 |4Second:Seconds; until logout" and seventy-four other
# strings did the same.
add_executable(test_plural_escape
    test_plural_escape.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/plural_escape.cpp
)
target_include_directories(test_plural_escape PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_plural_escape SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_plural_escape PRIVATE catch2_main)
add_test(NAME plural_escape COMMAND test_plural_escape)
register_test_target(test_plural_escape)

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
wowee_add_test(test_open_formats
    SOURCES test_open_formats.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_building.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_collision.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/wowee_terrain_loader.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/wmo_loader.cpp
            ${TEST_COMMON_SOURCES})

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
    ${TEST_COMMON_SOURCES}
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
wowee_test_link_glm(test_open_format_emitter)
add_test(NAME open_format_emitter COMMAND test_open_format_emitter)
register_test_target(test_open_format_emitter)

# ── test_camera ──────────────────────────────────────────────
wowee_add_test(test_camera
    SOURCES test_camera.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/camera.cpp
            ${TEST_COMMON_SOURCES})

# ── test_gltf_terrain_bin ────────────────────────────────────
# The BIN chunk of a terrain .glb, packed byte for byte by two exporters.
# Checked against the glTF 2.0 layout rather than against the old code.
add_executable(test_gltf_terrain_bin
    test_gltf_terrain_bin.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_gltf_terrain_bin PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools/editor
)
target_include_directories(test_gltf_terrain_bin SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/nlohmann
)
target_link_libraries(test_gltf_terrain_bin PRIVATE catch2_main)
wowee_test_link_glm(test_gltf_terrain_bin)
add_test(NAME gltf_terrain_bin COMMAND test_gltf_terrain_bin)
register_test_target(test_gltf_terrain_bin)

# ── test_editor_units (SQL escape, quest validation, …) ─────
add_executable(test_editor_units
    test_editor_units.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/sql_exporter.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/npc_spawner.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/quest_editor.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/content_pack.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/editor_brush.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/object_placer.cpp
    ${TEST_COMMON_SOURCES}
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
wowee_test_link_glm(test_editor_units)
add_test(NAME editor_units COMMAND test_editor_units)
register_test_target(test_editor_units)
register_test_target(test_editor_units)

# ── test_expansion_profile ──────────────────────────────────
add_executable(test_expansion_profile
    test_expansion_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/game/expansion_profile.cpp
    ${TEST_COMMON_SOURCES}
)
target_include_directories(test_expansion_profile PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_expansion_profile SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_expansion_profile PRIVATE catch2_main)
add_test(NAME expansion_profile COMMAND test_expansion_profile)
register_test_target(test_expansion_profile)

# ── test_buff_bar_layout ────────────────────────────────────
# Header-only layout arithmetic - no ImGui needed.
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

# ── test_update_field_table ─────────────────────────────────
add_executable(test_update_field_table
    test_update_field_table.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/game/update_field_table.cpp
)
target_include_directories(test_update_field_table PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_update_field_table SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_update_field_table PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_update_field_table PRIVATE catch2_main)
add_test(NAME update_field_table COMMAND test_update_field_table)
register_test_target(test_update_field_table)

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

# ── test_quest_query_layout ─────────────────────────────────
# Where the strings start in the quest query response. Pure arithmetic against
# a body built the way the server writes it: reading from the wrong offset does
# not fail, it hands back the next string along and shifts everything after it.
add_executable(test_quest_query_layout test_quest_query_layout.cpp)
target_include_directories(test_quest_query_layout PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_quest_query_layout SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_quest_query_layout PRIVATE catch2_main)
add_test(NAME quest_query_layout COMMAND test_quest_query_layout)
register_test_target(test_quest_query_layout)

# ── test_quest_rewards ──────────────────────────────────────
wowee_add_packet_test(test_quest_rewards)

# ── test_inventory_sort ─────────────────────────────────────
add_executable(test_inventory_sort
    test_inventory_sort.cpp
    ${CMAKE_SOURCE_DIR}/src/game/inventory.cpp
    ${TEST_COMMON_SOURCES}
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
    # ...and so must wowee_test_common, for the same reason: its objects are
    # linked straight into the sanitized test executables, and mixing
    # instrumented and uninstrumented objects is how ASAN starts reporting
    # nonsense.
    target_compile_options(wowee_test_common PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    message(STATUS "Test targets: ASAN + UBSan ENABLED")
endif()

# ── test_m2_classifier_foliage ───────────────────────────────
# Pins the wind-sway classification against substring collisions: foliage
# tokens are matched inside concatenated names, so "thorn" landed inside
# Stranglethorn and made every troll ruin sway like a tree.
wowee_add_test(test_m2_classifier_foliage
    SOURCES test_m2_classifier_foliage.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/m2_model_classifier.cpp)

# ── test_helper_creature_models ─────────────────────────────
# The deliberately invisible creatures scripts hang effects on. The list was
# written out twice in the spawner and named only invisiblestalker, so
# invisibleman and the World\\Scale measuring boxes stood in the world as
# solid objects.
wowee_add_test(test_helper_creature_models
    SOURCES test_helper_creature_models.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/m2_model_classifier.cpp)

# ── test_terrain_slope_limit ────────────────────────────────
# The 50 degree limit as it applies to the ground. It governed WMO and M2
# floors; the terrain query answers a height and no normal, so it was never
# applied there and any mountain could be walked straight up.
wowee_add_test(test_terrain_slope_limit
    SOURCES test_terrain_slope_limit.cpp)

# ── test_particle_density_floor ─────────────────────────────
# Particle density against the floor that holds a flame together. Applying
# the setting after the floor instead of before it takes every candle in the
# world out at the lowest setting.
wowee_add_test(test_particle_density_floor
    SOURCES test_particle_density_floor.cpp)

# ── test_m2_star_layer ──────────────────────────────────────
# Which batch of a sky model is its stars. The clouds, planets and nebulae
# of the same dome sit in the same directory, one texture holds both stars
# and clouds, and a floor doodad in Ulduar is named for stars without being
# a sky - so suppressing the wrong one deletes the sky instead of its stars.
wowee_add_test(test_m2_star_layer
    SOURCES test_m2_star_layer.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/m2_model_classifier.cpp)

# ── test_calendar_packet ────────────────────────────────────
add_executable(test_calendar_packet
    test_calendar_packet.cpp
    ${TEST_COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/network/packet.cpp
    ${CMAKE_SOURCE_DIR}/src/game/calendar_data.cpp
)
target_include_directories(test_calendar_packet PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_calendar_packet SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_calendar_packet PRIVATE catch2_main SDL2::SDL2 ZLIB::ZLIB
                                                 OpenSSL::SSL OpenSSL::Crypto)
wowee_test_link_glm(test_calendar_packet)
if(TARGET Vulkan::Vulkan)
    target_link_libraries(test_calendar_packet PRIVATE Vulkan::Vulkan)
endif()
add_test(NAME calendar_packet COMMAND test_calendar_packet)
register_test_target(test_calendar_packet)

# ── test_calendar_month ─────────────────────────────────────
add_executable(test_calendar_month test_calendar_month.cpp)
target_include_directories(test_calendar_month PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_calendar_month SYSTEM PRIVATE ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_calendar_month PRIVATE catch2_main)
add_test(NAME calendar_month COMMAND test_calendar_month)
register_test_target(test_calendar_month)

# ── test_catalog_entry_key ───────────────────────────────────
# Which field of a catalog entry is its id. Four commands guessed it from the
# field's name and disagreed; the format table now declares it. The oracle is
# each format's own --info handler, so the test reads the editor sources.
add_executable(test_catalog_entry_key
    test_catalog_entry_key.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/cli_format_table.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_catalog_entry_key PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools/editor
)
target_include_directories(test_catalog_entry_key SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/nlohmann
)
target_compile_definitions(test_catalog_entry_key PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_catalog_entry_key PRIVATE catch2_main)
add_test(NAME catalog_entry_key COMMAND test_catalog_entry_key)
register_test_target(test_catalog_entry_key)

# ── test_catalog_subprocess ──────────────────────────────────
# How the catalog searches reach a file: they re-invoke this executable with
# the format's --info flag. Four commands carried the same four steps. The
# oracle for the quoting is POSIX sh, which the test runs.
add_executable(test_catalog_subprocess
    test_catalog_subprocess.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_catalog_subprocess PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools/editor
)
target_include_directories(test_catalog_subprocess SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/extern/nlohmann
)
target_link_libraries(test_catalog_subprocess PRIVATE catch2_main)
add_test(NAME catalog_subprocess COMMAND test_catalog_subprocess)
register_test_target(test_catalog_subprocess)

# ── test_cli_paths ───────────────────────────────────────────
# Finding the files of one format under a directory. Eleven commands walked
# the tree themselves for .wom alone. The oracle is a tree the test writes.
add_executable(test_cli_paths
    test_cli_paths.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_cli_paths PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools/editor
)
target_include_directories(test_cli_paths SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
)
target_link_libraries(test_cli_paths PRIVATE catch2_main)
add_test(NAME cli_paths COMMAND test_cli_paths)
register_test_target(test_cli_paths)

# ── test_obj_parse ───────────────────────────────────────────
# Reading a Wavefront OBJ, which two importers each implemented. The oracle
# is the OBJ format: 1-based indices, negative indices relative to the pool
# as it stands, and three independently indexed pools.
add_executable(test_obj_parse
    test_obj_parse.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_obj_parse PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools/editor
)
target_include_directories(test_obj_parse SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS}
)
target_link_libraries(test_obj_parse PRIVATE catch2_main)
wowee_test_link_glm(test_obj_parse)
add_test(NAME obj_parse COMMAND test_obj_parse)
register_test_target(test_obj_parse)

# ── test_model_bounds ────────────────────────────────────────
# The box and radius a .wom carries. Eighteen places computed them three
# different ways. The oracle is what the field means: M2Loader sets it from
# the M2 header's boundingRadius, measured from the model origin.
wowee_add_test(test_model_bounds
    SOURCES test_model_bounds.cpp
            ${TEST_COMMON_SOURCES})

# ── test_format_lookup ───────────────────────────────────────
# Finding a format by extension, which three commands each implemented.
# --fix-magic renames on the answer. The oracle is the table itself: every
# row's extension has to find that row.
add_executable(test_format_lookup
    test_format_lookup.cpp
    ${CMAKE_SOURCE_DIR}/tools/editor/cli_format_table.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_format_lookup PRIVATE
    ${TEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools/editor
)
target_include_directories(test_format_lookup SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_format_lookup PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_format_lookup PRIVATE catch2_main)
add_test(NAME format_lookup COMMAND test_format_lookup)
register_test_target(test_format_lookup)

# ── test_wmo_material_class ──────────────────────────────────
# Which WMO materials are glass. The oracle is the shipped WMO data: the
# MOMT flags the artist set, not the texture's file name.
add_executable(test_wmo_material_class
    test_wmo_material_class.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_wmo_material_class PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_wmo_material_class SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_wmo_material_class PRIVATE catch2_main)
add_test(NAME wmo_material_class COMMAND test_wmo_material_class)
register_test_target(test_wmo_material_class)

# ── test_asset_token_name ────────────────────────────────────
# Which part of an asset path a token check may read. A directory name
# poisons every token under it: Item/TextureComponents/LegLowerTexture
# spells "glow", so every leg and boot texture was being colour-keyed.
wowee_add_test(test_asset_token_name
    SOURCES test_asset_token_name.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/m2_model_classifier.cpp)

# ── test_pom_quality_one_scale ──────────────────────────────
# Parallax quality is one scale and everywhere it is offered has to agree.
# The login screen offered two entries of a three entry setting and wrote the
# wrong index for the two it did offer, so its "High" asked for Medium.
add_executable(test_pom_quality_one_scale
    test_pom_quality_one_scale.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_pom_quality_one_scale PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_pom_quality_one_scale SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_pom_quality_one_scale PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_pom_quality_one_scale PRIVATE catch2_main)
add_test(NAME pom_quality_one_scale COMMAND test_pom_quality_one_scale)
register_test_target(test_pom_quality_one_scale)

# ── test_settings_apply_on_load ──────────────────────────────
# A graphics setting read from the config file has to reach something.
# Loading only filled the panel's pending fields, so a view distance chosen
# at the login screen was shown on the slider and never applied.
add_executable(test_settings_apply_on_load
    test_settings_apply_on_load.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/settings_schema.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_settings_apply_on_load PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_settings_apply_on_load SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_settings_apply_on_load PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_settings_apply_on_load PRIVATE catch2_main)
add_test(NAME settings_apply_on_load COMMAND test_settings_apply_on_load)
register_test_target(test_settings_apply_on_load)

# ── test_light_coords ────────────────────────────────────────
# Where Light.dbc says a light volume is. The scale was 1.0 against a file
# storing thirty-sixths of a yard on mirrored, swapped, corner-origin axes,
# so no volume matched anywhere and every zone fell back to default light.
wowee_add_test(test_light_coords
    SOURCES test_light_coords.cpp
            ${TEST_COMMON_SOURCES})

# ── test_light_bands ─────────────────────────────────────────
# Which LightParams row and channel a light band belongs to, and where its
# fields are. The block mapping was off by one in both terms and the field
# indices by one, so 12024 of 15300 colour bands read as empty.
add_executable(test_light_bands
    test_light_bands.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_light_bands PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_light_bands SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_light_bands PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_light_bands PRIVATE catch2_main)
wowee_test_link_glm(test_light_bands)
add_test(NAME light_bands COMMAND test_light_bands)
register_test_target(test_light_bands)

# ── test_light_headroom ──────────────────────────────────────
# Keeping ambient + diffuse inside 1.0. 82% of the client's LightParams
# rows exceed it at noon, and per-channel clipping shifts hue: Mulgore's
# grass went searing yellow, Teldrassil's canopies magenta.
wowee_add_test(test_light_headroom
    SOURCES test_light_headroom.cpp
            ${TEST_COMMON_SOURCES})

# ── test_m2_view_distance ────────────────────────────────────
# How far one M2 instance draws. Three separate widenings - a size factor,
# an animation factor and the game object floor - each pushed past the view
# distance the player asked for, so trees and mailboxes hung in the air
# beyond where the terrain and the WMOs stop.
add_executable(test_m2_view_distance
    test_m2_view_distance.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_m2_view_distance PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_view_distance SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_m2_view_distance PRIVATE catch2_main)
add_test(NAME m2_view_distance COMMAND test_m2_view_distance)
register_test_target(test_m2_view_distance)

# ── test_light_volume_order ──────────────────────────────────
# Which of two overlapping light volumes is blended first. Equal weights
# are the ordinary case, not a coincidence, and an unstable sort decided
# them differently every time the list changed - so the sky model flipped
# frame to frame while walking.
add_executable(test_light_volume_order
    test_light_volume_order.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_light_volume_order PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_light_volume_order SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_light_volume_order PRIVATE catch2_main)
add_test(NAME light_volume_order COMMAND test_light_volume_order)
register_test_target(test_light_volume_order)

# ── test_m2_blend_mode ───────────────────────────────────────
# Whether an M2 batch is alpha tested. Additive batches were forced onto
# the cutout pipeline when their texture had no alpha, which drew every
# glow card as a flat opaque disc.
add_executable(test_m2_blend_mode
    test_m2_blend_mode.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_m2_blend_mode PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_blend_mode SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_m2_blend_mode PRIVATE catch2_main)
add_test(NAME m2_blend_mode COMMAND test_m2_blend_mode)
register_test_target(test_m2_blend_mode)

# ── test_m2_glow_card ────────────────────────────────────────
# What the renderer does with a glow-card batch. Three decisions the two
# render passes each made for themselves, in expressions identical down to
# the constants; a batch is skipped by one pass and drawn by the other, so
# drift there draws the card twice or never.
add_executable(test_m2_glow_card
    test_m2_glow_card.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_m2_glow_card PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_glow_card SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_m2_glow_card PRIVATE catch2_main)
add_test(NAME m2_glow_card COMMAND test_m2_glow_card)
register_test_target(test_m2_glow_card)

# ── test_m2_color_track ──────────────────────────────────────
# Where an M2 colour track's first key is. WotLK nests an array per
# sequence; vanilla points straight at the values. Reading one as the other
# gives a plausible colour rather than a failure.
add_executable(test_m2_color_track
    test_m2_color_track.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_m2_color_track PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_m2_color_track SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_link_libraries(test_m2_color_track PRIVATE catch2_main)
add_test(NAME m2_color_track COMMAND test_m2_color_track)
register_test_target(test_m2_color_track)

# ── test_dbc_layout_columns ──────────────────────────────────
# Every column a layout names has to exist in the file it describes. A DBC
# asked for a field past the end answers zero for every row, forever, which
# is how the light bands read as empty for as long as they did.
add_executable(test_dbc_layout_columns
    test_dbc_layout_columns.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_dbc_layout_columns PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_dbc_layout_columns SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_dbc_layout_columns PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_dbc_layout_columns PRIVATE catch2_main)
add_test(NAME dbc_layout_columns COMMAND test_dbc_layout_columns)
register_test_target(test_dbc_layout_columns)

# ── test_update_field_indices ────────────────────────────────
# Update field slot numbers against AzerothCore's UpdateFields.h. Reading
# the wrong slot returns a neighbouring field, usually zero for a guid, so
# the feature looks unsent rather than misread.
add_executable(test_update_field_indices
    test_update_field_indices.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_update_field_indices PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_update_field_indices SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_update_field_indices PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_update_field_indices PRIVATE catch2_main)
add_test(NAME update_field_indices COMMAND test_update_field_indices)
register_test_target(test_update_field_indices)

# ── test_opcode_tables ───────────────────────────────────────
# Opcode numbers and the structure around them. All 1306 WotLK numbers were
# checked against AzerothCore's Opcodes.h and agreed; this holds them there
# and pins the _extends inheritance the smaller tables rely on.
add_executable(test_opcode_tables
    test_opcode_tables.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_opcode_tables PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_opcode_tables SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_opcode_tables PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_opcode_tables PRIVATE catch2_main)
add_test(NAME opcode_tables COMMAND test_opcode_tables)
register_test_target(test_opcode_tables)

# ── test_movement_flag_bits ──────────────────────────────────
# Movement flag bits against AzerothCore's UnitDefines.h. FEATHER_FALL and
# WATER_WALK sat on PENDING_STOP and PENDING_STRAFE_STOP, so the client
# asserted a pending stop in every packet while those states were active.
wowee_add_test(test_movement_flag_bits
    SOURCES test_movement_flag_bits.cpp
            ${TEST_COMMON_SOURCES})

# ── test_spell_icon_columns ──────────────────────────────────
# Which columns of SpellIcon.dbc hold the id and the path, against the shipped
# file. Five places carried their own copy of these two numbers, and a wrong
# column answers empty for every row rather than failing.
add_executable(test_spell_icon_columns
    test_spell_icon_columns.cpp
    ${TEST_COMMON_SOURCES})
target_include_directories(test_spell_icon_columns PRIVATE ${TEST_INCLUDE_DIRS})
target_include_directories(test_spell_icon_columns SYSTEM PRIVATE
    ${TEST_SYSTEM_INCLUDE_DIRS})
target_compile_definitions(test_spell_icon_columns PRIVATE
    WOWEE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(test_spell_icon_columns PRIVATE catch2_main)
wowee_test_link_glm(test_spell_icon_columns)
add_test(NAME spell_icon_columns COMMAND test_spell_icon_columns)
register_test_target(test_spell_icon_columns)

# ── test_json_table_scan ─────────────────────────────────────
# The scanner both expansion table loaders now share. The value parser is the
# point: stoul on "0x12" in base ten answers 0 without raising, and 0 is a
# real opcode and a real update field index.
wowee_add_test(test_json_table_scan
    SOURCES test_json_table_scan.cpp
            ${TEST_COMMON_SOURCES})

# ── test_auction_slot_filter ─────────────────────────────────
# The auction house's equipment-slot filter. FrameXML numbers the rows it is
# given from one, so the number that comes back is a position in the subset
# offered for that class rather than in the whole slot table.
wowee_add_test(test_auction_slot_filter
    SOURCES test_auction_slot_filter.cpp
            ${TEST_COMMON_SOURCES})

# ── test_player_model_path ───────────────────────────────────
# Which body a character wears. Gender alone does not answer it: a nonbinary
# character picks one, and the preview, the world spawn and the skin textures
# each read that choice differently.
wowee_add_test(test_player_model_path
    SOURCES test_player_model_path.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/character.cpp)

# ── test_equipment_hash ──────────────────────────────────────
# The fingerprint a cache redraws on when the outfit changes. Two copies
# existed, each deciding for itself which fields change how an item looks.
wowee_add_test(test_equipment_hash
    SOURCES test_equipment_hash.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/character.cpp)

# ── test_quest_giver_status ──────────────────────────────────
# DIALOG_STATUS_* and the mark it draws. Five of the eleven values had no name,
# and a value with no name drew nothing at all.
wowee_add_test(test_quest_giver_status
    SOURCES test_quest_giver_status.cpp
            ${TEST_COMMON_SOURCES})

# ── test_collision_geometry ──────────────────────────────────
# The ray, box and focus tests every collision query is built from. Both
# renderers had their own copy, and a difference moves where the world is solid.
wowee_add_test(test_collision_geometry
    SOURCES test_collision_geometry.cpp
            ${TEST_COMMON_SOURCES})

# ── test_race_gender_suffix ──────────────────────────────────
# The two-letter race code per-race models are suffixed with. Written out twice
# - the helmet resolver and the shoulder attachment - and a wrong suffix reads
# as "no per-race cut" rather than as an error.
wowee_add_test(test_race_gender_suffix
    SOURCES test_race_gender_suffix.cpp
            ${TEST_COMMON_SOURCES})

# ── test_melee_anim_chains ───────────────────────────────────
# The attack animation fallback chains, which were written out twice - in the
# controller that plays them and in the probe that reports what a model can do.
wowee_add_test(test_melee_anim_chains
    SOURCES test_melee_anim_chains.cpp
            ${TEST_COMMON_SOURCES})

# ── test_sample_pick ─────────────────────────────────────────
# Which of a sound bank's samples to play. Three banks wrote out the same
# loop, and its one rule - choose among the loaded ones - is a one-character
# difference from silence.
wowee_add_test(test_sample_pick
    SOURCES test_sample_pick.cpp
            ${TEST_COMMON_SOURCES})

# ── test_footstep_paths ──────────────────────────────────────
# The names the footstep sounds are stored under. Six families, none derivable
# from another, and a name that stops matching the archive is silent.
wowee_add_test(test_footstep_paths
    SOURCES test_footstep_paths.cpp
            ${TEST_COMMON_SOURCES})

# ── test_placement_transform ─────────────────────────────────
# The model matrix a doodad or a building placement becomes. The two renderers
# composed the rotation in different orders, and nothing on flat ground can
# tell those apart.
wowee_add_test(test_placement_transform
    SOURCES test_placement_transform.cpp
            ${TEST_COMMON_SOURCES})

# ── test_wmo_group_path ──────────────────────────────────────
# The names a WMO's group file might carry. Five loaders tried different
# numbers of them, so a building spelled ".WMO" loaded its interior on some
# paths and silently did not on others.
wowee_add_test(test_wmo_group_path
    SOURCES test_wmo_group_path.cpp
            ${TEST_COMMON_SOURCES})

# ── test_env_flag ────────────────────────────────────────────
# Reading a boolean out of an environment variable. Five files had their own
# copy and two rules: one read "off" as enabled because it tested only the
# first character.
wowee_add_test(test_env_flag
    SOURCES test_env_flag.cpp
            ${TEST_COMMON_SOURCES})

# ── test_inventory_count ─────────────────────────────────────
# How many of an item the bags hold. Counted in five places with two rules:
# the Lua bindings read an occupied slot with a zero stack as one item and the
# client's own counters read it as none, so a reagent could be in the bags and
# absent from the crafting check at the same time.
wowee_add_test(test_inventory_count
    SOURCES test_inventory_count.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/game/inventory.cpp)

# ── test_gather_spells ───────────────────────────────────────
# The mining and herbalism rank ids, which were written out four times across
# two files. A list that gains a rank in one copy and not the others leaves the
# player unable to gather with that rank, silently.
wowee_add_test(test_gather_spells
    SOURCES test_gather_spells.cpp
            ${TEST_COMMON_SOURCES})

# ── test_group_list_parse ────────────────────────────────────
# SMSG_GROUP_LIST built as the server writes it and read back, in both its
# shapes. The dungeon finder shape was read a byte long - a seventh field that
# is never sent - so every member of every finder group came out shifted and
# the member count was three bytes of the real count plus a letter of a name.
wowee_add_packet_test(test_group_list_parse)

# ── test_party_wire_codes ────────────────────────────────────
# Party operation and result codes against AzerothCore. PARTY_OP_SWAP is 4
# and SWAP was declared 3, which is not an operation at all, so the field
# could never be recognised as a swap.
wowee_add_test(test_party_wire_codes
    SOURCES test_party_wire_codes.cpp
            ${TEST_COMMON_SOURCES})

# ── test_spell_mod_ops ───────────────────────────────────────
# Spell modifier op numbers against AzerothCore's SpellDefines.h. Ops 0-19
# were right; from 20 the list was the Cataclysm one on a 3.3.5 head, so op
# 22 was named for dispel resistance where the server means a periodic.
wowee_add_test(test_spell_mod_ops
    SOURCES test_spell_mod_ops.cpp
            ${TEST_COMMON_SOURCES})

# ── test_adt_truncation ──────────────────────────────────────
# What the ADT parser does with a file that lies about its own sizes. The
# chunk readers took a bare pointer and a length as separate arguments and
# trusted every caller's arithmetic; one of those sums wrapped, turning a
# four-byte MCAL into a memcpy length of 1.8e19. Malformed input only -- run
# under --asan, where an overrun fails rather than passing unnoticed.
wowee_add_test(test_adt_truncation
    SOURCES test_adt_truncation.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_loader.cpp)

# ── test_blp_truncation ──────────────────────────────────────
# What the BLP loader does with a texture that lies about its own sizes. The
# decompressors derived how much source to read from width and height and
# bounded only the destination, so a 4096x4096 DXT1 header with an eight-byte
# mip level read two megabytes out of an eight-byte buffer. Malformed input
# only -- run under --asan, where the overrun fails rather than passing.
wowee_add_test(test_blp_truncation
    SOURCES test_blp_truncation.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/pipeline/blp_loader.cpp)

# ── test_grass_blade_layout ────────────────────────────
# The C++ and GLSL views of a grass blade have to agree byte for byte, and a
# disagreement is silent: it compiles, passes validation, and renders the wrong
# thing. Header-only on purpose -- grass_blade.hpp needs nothing but glm, so
# this stays a fast test rather than one that drags in a Vulkan context.
wowee_add_test(test_grass_blade_layout
    SOURCES test_grass_blade_layout.cpp)

# ── test_adt_alpha ─────────────────────────────────────
# MCAL alpha maps, and the row and column at the end of a four-bit one that are
# not data. Left alone they put a hard line between every pair of chunks.
wowee_add_test(test_adt_alpha
    SOURCES test_adt_alpha.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_alpha.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_loader.cpp)

# ── test_grass_terrain ─────────────────────────────────
# Whether grass grows at a point, answered from the ADT's own ground-effect
# ids rather than from a classifier of ours. Headless: the chunks are built by
# hand, so this needs no device, no tile and no world.
wowee_add_test(test_grass_terrain
    SOURCES test_grass_terrain.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/pipeline/grass_terrain.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/grass_profile.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/terrain_mesh.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_alpha.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/adt_loader.cpp)

# ── test_grass_population ──────────────────────────────
# Blades generated from terrain suitability. Most of these cases are one
# property from different angles: a blade's identity comes from its world
# lattice cell and nothing else, so rebuilding the population as the player
# moves cannot shuffle the field.
wowee_add_test(test_grass_population
    SOURCES test_grass_population.cpp
            ${TEST_COMMON_SOURCES}
            ${CMAKE_SOURCE_DIR}/src/pipeline/grass_population.cpp)

# ── test_grass_profile ─────────────────────────────────
# What grass looks like on a patch of ground, from the detail doodads the map
# places there. The asset names carry the classification, so nothing here knows
# what a zone is - a profile needing a per-zone correction would be the wrong
# shape entirely.
wowee_add_test(test_grass_profile
    SOURCES test_grass_profile.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/grass_profile.cpp)

# ── test_grass_biomes ──────────────────────────────────
# The per-zone look table: parsing, subzone-over-zone matching, and that an
# override multiplies the scales and replaces the colours it names while
# leaving everything else the effect data derived.
wowee_add_test(test_grass_biomes
    SOURCES test_grass_biomes.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/grass_biomes.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/grass_profile.cpp)

# ── test_grass_clearing ────────────────────────────────
# The clearings the placed world presses into the grass: box distance, the
# cleared ring, the ease band, and the minimum across overlapping sources.
wowee_add_test(test_grass_clearing
    SOURCES test_grass_clearing.cpp
            ${CMAKE_SOURCE_DIR}/src/pipeline/grass_clearing.cpp)

# ── test_grass_cull_math ───────────────────────────────
# The frustum arithmetic the grass cull performs, run for a camera turned all
# the way round. Grass vanishing in one world direction is a claim about this
# and nothing else, and it is cheaper to measure than to reason about.
wowee_add_test(test_grass_cull_math
    SOURCES test_grass_cull_math.cpp
            ${CMAKE_SOURCE_DIR}/src/rendering/frustum.cpp)

# ── Sanitizers, once every test target exists ────────────────
# This must be the last thing in this file. The loop walks ALL_TEST_TARGETS,
# which register_test_target appends to as each test is declared, so the
# equivalent block higher up only ever saw the targets above it -- 47 tests are
# declared after it. `./test.sh --asan` was building those uninstrumented and
# reporting them as passing.
#
# Re-applying to targets the earlier block already covered is harmless: the
# flags are identical and repeat on the command line.
get_property(ALL_TEST_TARGETS GLOBAL PROPERTY WOWEE_ALL_TEST_TARGETS)
list(LENGTH ALL_TEST_TARGETS _wowee_test_target_count)
message(STATUS "Test targets registered: ${_wowee_test_target_count}")

if(WOWEE_ENABLE_ASAN AND NOT MSVC)
    foreach(_t IN LISTS ALL_TEST_TARGETS)
        target_compile_options(${_t} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${_t} PRIVATE -fsanitize=address,undefined)
    endforeach()
endif()

# A precompiled header for the test targets was tried here and removed: with
# ~140 executables each generating its own PCH, building them cost as much as
# the re-parsing it saved (950.8 s of CPU against 945.2 s without). The main
# target benefits because it is one target over 413 objects; the tests are the
# opposite shape. Do not re-add it without measuring.
