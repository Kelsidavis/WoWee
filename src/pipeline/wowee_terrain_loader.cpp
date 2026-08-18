#include "core/coordinates.hpp"
#include "pipeline/wowee_terrain_loader.hpp"
#include "core/logger.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <cstring>
#include <cmath>

namespace wowee {
namespace pipeline {

static constexpr uint32_t WHM_MAGIC = 0x314D4857; // "WHM1"

bool WoweeTerrainLoader::exists(const std::string& basePath) {
    return std::filesystem::exists(basePath + ".whm") &&
           std::filesystem::exists(basePath + ".wot");
}

bool WoweeTerrainLoader::loadHeightmap(const std::string& whmPath, ADTTerrain& terrain) {
    std::ifstream f(whmPath, std::ios::binary);
    if (!f) return false;

    uint32_t magic, chunks, verts;
    f.read(reinterpret_cast<char*>(&magic), 4);
    if (magic != WHM_MAGIC) {
        LOG_ERROR("Not a WHM file: ", whmPath);
        return false;
    }
    f.read(reinterpret_cast<char*>(&chunks), 4);
    f.read(reinterpret_cast<char*>(&verts), 4);

    if (chunks != 256 || verts != 145) {
        LOG_ERROR("WHM unexpected dimensions: ", chunks, " chunks, ", verts, " verts");
        return false;
    }

    terrain.loaded = true;
    terrain.version = 18;

    for (int ci = 0; ci < 256; ci++) {
        auto& chunk = terrain.chunks[ci];
        chunk.heightMap.loaded = true;
        chunk.indexX = ci % 16;
        chunk.indexY = ci / 16;
        chunk.flags = 0;
        chunk.holes = 0;

        float base;
        f.read(reinterpret_cast<char*>(&base), 4);
        // Reject NaN/inf chunk base height - would break collision/pathing
        // and produce non-finite vertex positions in the terrain mesh.
        if (!std::isfinite(base)) base = 0.0f;
        chunk.position[2] = base;

        f.read(reinterpret_cast<char*>(chunk.heightMap.heights.data()), 145 * 4);
        // Same guard applied per-vertex.
        for (auto& h : chunk.heightMap.heights) {
            if (!std::isfinite(h)) h = 0.0f;
        }

        // Read alpha map data (may not be present in older WHM files).
        // Reject overlong alphaSize to keep the per-chunk block alignment -
        // skipping a 100MB alpha block would leave the next chunk's
        // baseHeight read parsing alpha bytes as floats.
        uint32_t alphaSize = 0;
        if (f.read(reinterpret_cast<char*>(&alphaSize), 4) && alphaSize > 0) {
            if (alphaSize > 65536) {
                LOG_ERROR("WHM chunk ", ci, " alphaSize rejected (",
                          alphaSize, "): ", whmPath);
                return false;
            }
            chunk.alphaMap.resize(alphaSize);
            f.read(reinterpret_cast<char*>(chunk.alphaMap.data()), alphaSize);
        }

        // Default normals (up)
        for (int i = 0; i < 145; i++) {
            chunk.normals[i * 3 + 0] = 0;
            chunk.normals[i * 3 + 1] = 0;
            chunk.normals[i * 3 + 2] = 127;
        }
    }

    LOG_INFO("WHM loaded: ", whmPath, " (256 chunks, 145 verts each)");
    return true;
}

bool WoweeTerrainLoader::loadMetadata(const std::string& wotPath, ADTTerrain& terrain) {
    std::ifstream f(wotPath);
    if (!f) return false;

    try {
        auto j = nlohmann::json::parse(f);

        terrain.coord.x = j.value("tileX", 0);
        terrain.coord.y = j.value("tileY", 0);
        // Out-of-range tile coords would compute chunk positions tens of
        // thousands of units away from any other zone tile. Clamp to the
        // 64x64 grid so loaded terrain always lands at a valid spot.
        if (terrain.coord.x < 0 || terrain.coord.x > 63) terrain.coord.x = 32;
        if (terrain.coord.y < 0 || terrain.coord.y > 63) terrain.coord.y = 32;

        // Compute chunk world positions from tile coordinates
        float tileSize = core::coords::TILE_SIZE;
        float chunkSize = tileSize / 16.0f;
        for (int cy = 0; cy < 16; cy++) {
            for (int cx = 0; cx < 16; cx++) {
                auto& chunk = terrain.chunks[cy * 16 + cx];
                chunk.position[0] = (32.0f - terrain.coord.x) * tileSize - cx * chunkSize;
                chunk.position[1] = (32.0f - terrain.coord.y) * tileSize - cy * chunkSize;
            }
        }

        // Parse textures (cap at 1024 - far above any realistic ADT)
        if (j.contains("textures") && j["textures"].is_array()) {
            constexpr size_t kMaxTextures = 1024;
            for (const auto& tex : j["textures"]) {
                if (terrain.textures.size() >= kMaxTextures) break;
                if (tex.is_string() && !tex.get<std::string>().empty())
                    terrain.textures.push_back(tex.get<std::string>());
            }
        }

        // Parse chunk layers - WoW ADT supports max 4 layers per chunk;
        // cap to 8 to allow some headroom without unbounded growth.
        if (j.contains("chunkLayers") && j["chunkLayers"].is_array()) {
            const auto& layers = j["chunkLayers"];
            for (int ci = 0; ci < std::min(256, static_cast<int>(layers.size())); ci++) {
                const auto& cl = layers[ci];
                if (cl.contains("layers") && cl["layers"].is_array()) {
                    for (const auto& texId : cl["layers"]) {
                        if (terrain.chunks[ci].layers.size() >= 8) break;
                        TextureLayer layer{};
                        // Range-check: get<uint32_t> throws on negative/oversize.
                        int64_t raw = texId.is_number_integer()
                            ? texId.get<int64_t>() : 0;
                        if (raw < 0 || raw > 0xFFFFFFFFll) raw = 0;
                        layer.textureId = static_cast<uint32_t>(raw);
                        layer.flags = terrain.chunks[ci].layers.empty() ? 0 : 0x100;
                        terrain.chunks[ci].layers.push_back(layer);
                    }
                }
                if (cl.contains("holes")) {
                    int64_t raw = cl["holes"].is_number_integer()
                        ? cl["holes"].get<int64_t>() : 0;
                    if (raw < 0 || raw > 0xFFFF) raw = 0;
                    terrain.chunks[ci].holes = static_cast<uint16_t>(raw);
                }
                // Optional, absent from most custom maps: quads where ground
                // effect doodads (grass) must not grow, as in the ADT format.
                if (cl.contains("noEffectDoodad") &&
                    cl["noEffectDoodad"].is_number_unsigned()) {
                    terrain.chunks[ci].noEffectDoodad =
                        cl["noEffectDoodad"].get<uint64_t>();
                }
            }
        }

        // Parse water data
        if (j.contains("water") && j["water"].is_array()) {
            for (const auto& w : j["water"]) {
                if (w.is_null()) continue;
                int wci = w.value("chunk", -1);
                if (wci < 0 || wci >= 256) continue;
                ADTTerrain::WaterLayer wl;
                wl.liquidType = w.value("type", 0u);
                // Known WoW liquid types: 0=water, 1=ocean, 2=magma, 3=slime.
                // Out-of-range values would default to plain water in render
                // but might break server-side liquid behaviour.
                if (wl.liquidType > 3) wl.liquidType = 0;
                wl.maxHeight = w.value("height", 0.0f);
                // NaN water height would produce NaN vertex positions and
                // a degenerate GPU draw, or crash the water mesh build.
                if (!std::isfinite(wl.maxHeight)) wl.maxHeight = 0.0f;
                wl.minHeight = wl.maxHeight;
                wl.x = 0; wl.y = 0; wl.width = 9; wl.height = 9;
                wl.heights.assign(81, wl.maxHeight);
                wl.mask.assign(8, 0xFF);
                terrain.waterData[wci].layers.push_back(wl);
            }
        }

        // Parse doodad placements. n.get<std::string> throws on non-string
        // entries - guard with is_string and cap the list at 65536 (uint32
        // nameId range is far larger but real zones top out around ~5k).
        if (j.contains("doodadNames") && j["doodadNames"].is_array()) {
            constexpr size_t kMaxNames = 65536;
            for (const auto& n : j["doodadNames"]) {
                if (terrain.doodadNames.size() >= kMaxNames) break;
                if (n.is_string()) terrain.doodadNames.push_back(n.get<std::string>());
            }
        }
        // Helper used by both doodad and WMO loaders below.
        auto san3 = [](float& a, float& b, float& c) {
            if (!std::isfinite(a)) a = 0.0f;
            if (!std::isfinite(b)) b = 0.0f;
            if (!std::isfinite(c)) c = 0.0f;
        };
        // Caps match the editor-side ObjectPlacer (100k for objects). Per-
        // tile, real ADTs cap at ~64k MDDF entries (uint32 indices) and
        // far fewer in practice (~5k for dense zones).
        constexpr size_t kMaxPlacements = 100'000;
        if (j.contains("doodads") && j["doodads"].is_array()) {
            for (const auto& jd : j["doodads"]) {
                if (terrain.doodadPlacements.size() >= kMaxPlacements) break;
                ADTTerrain::DoodadPlacement dp{};
                dp.nameId = jd.value("nameId", 0u);
                dp.uniqueId = jd.value("uniqueId", 0u);
                if (jd.contains("pos") && jd["pos"].size() >= 3) {
                    dp.position[0] = jd["pos"][0]; dp.position[1] = jd["pos"][1]; dp.position[2] = jd["pos"][2];
                }
                if (jd.contains("rot") && jd["rot"].size() >= 3) {
                    dp.rotation[0] = jd["rot"][0]; dp.rotation[1] = jd["rot"][1]; dp.rotation[2] = jd["rot"][2];
                }
                san3(dp.position[0], dp.position[1], dp.position[2]);
                san3(dp.rotation[0], dp.rotation[1], dp.rotation[2]);
                dp.scale = jd.value("scale", 1024);
                dp.flags = jd.value("flags", 0);
                terrain.doodadPlacements.push_back(dp);
            }
        }

        // Parse WMO placements (same guards as doodadNames above).
        if (j.contains("wmoNames") && j["wmoNames"].is_array()) {
            constexpr size_t kMaxWmoNames = 65536;
            for (const auto& n : j["wmoNames"]) {
                if (terrain.wmoNames.size() >= kMaxWmoNames) break;
                if (n.is_string()) terrain.wmoNames.push_back(n.get<std::string>());
            }
        }
        if (j.contains("wmos") && j["wmos"].is_array()) {
            for (const auto& jw : j["wmos"]) {
                if (terrain.wmoPlacements.size() >= kMaxPlacements) break;
                ADTTerrain::WMOPlacement wp{};
                wp.nameId = jw.value("nameId", 0u);
                wp.uniqueId = jw.value("uniqueId", 0u);
                if (jw.contains("pos") && jw["pos"].size() >= 3) {
                    wp.position[0] = jw["pos"][0]; wp.position[1] = jw["pos"][1]; wp.position[2] = jw["pos"][2];
                }
                if (jw.contains("rot") && jw["rot"].size() >= 3) {
                    wp.rotation[0] = jw["rot"][0]; wp.rotation[1] = jw["rot"][1]; wp.rotation[2] = jw["rot"][2];
                }
                san3(wp.position[0], wp.position[1], wp.position[2]);
                san3(wp.rotation[0], wp.rotation[1], wp.rotation[2]);
                wp.flags = jw.value("flags", 0);
                wp.doodadSet = jw.value("doodadSet", 0);
                terrain.wmoPlacements.push_back(wp);
            }
        }

        LOG_INFO("WOT loaded: ", wotPath, " (tile [", terrain.coord.x, ",", terrain.coord.y,
                 "], ", terrain.textures.size(), " textures, ",
                 terrain.doodadPlacements.size(), " doodads, ",
                 terrain.wmoPlacements.size(), " WMOs)");
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to parse WOT: ", e.what());
        return false;
    }
}

bool WoweeTerrainLoader::load(const std::string& basePath, ADTTerrain& terrain) {
    if (!loadHeightmap(basePath + ".whm", terrain)) return false;
    if (!loadMetadata(basePath + ".wot", terrain)) return false;
    return true;
}

} // namespace pipeline
} // namespace wowee
