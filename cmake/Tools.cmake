# Command-line tools built alongside the client.
#
# Each is optional and self-contained — the asset extractor, the DBC
# converter, the two auth probes, the BLP converter and the standalone
# world editor — and none of them is part of the client's own link.
# Extracted from the top-level CMakeLists.txt, which had grown past two
# thousand lines and was asked about in #107. Moved verbatim: this is the same
# text in a file of its own, included from the same scope, so every variable it
# reads and every one it sets behaves exactly as before.

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
