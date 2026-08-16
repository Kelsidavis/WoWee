#include "pipeline/wowee_vertex_sanitize.hpp"
#include "pipeline/wowee_binary_io.hpp"
#include "pipeline/wowee_building.hpp"
#include "pipeline/wmo_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/quaternion.hpp>
#include <fstream>
#include <filesystem>
#include <cstring>
#include <cmath>
#include <unordered_map>

namespace wowee {
namespace pipeline {

static constexpr uint32_t WOB_MAGIC = 0x31424F57; // "WOB1"

bool WoweeBuildingLoader::exists(const std::string& basePath) {
    return std::filesystem::exists(basePath + ".wob");
}

WoweeBuilding WoweeBuildingLoader::load(const std::string& basePath) {
    WoweeBuilding bld;
    std::ifstream f(basePath + ".wob", std::ios::binary);
    if (!f) return bld;

    uint32_t magic;
    f.read(reinterpret_cast<char*>(&magic), 4);
    if (magic != WOB_MAGIC) return bld;

    uint32_t groupCount, portalCount, doodadCount;
    f.read(reinterpret_cast<char*>(&groupCount), 4);
    f.read(reinterpret_cast<char*>(&portalCount), 4);
    f.read(reinterpret_cast<char*>(&doodadCount), 4);
    f.read(reinterpret_cast<char*>(&bld.boundRadius), 4);
    // Sanity bounds. Real WMOs are usually 1-50 groups; >4096 indicates a
    // corrupted header that would OOM us when we resize() vectors.
    if (groupCount > 4096 || portalCount > 8192 || doodadCount > 65536) {
        LOG_ERROR("WOB header rejected (groups=", groupCount,
                  " portals=", portalCount, " doodads=", doodadCount, "): ", basePath);
        return WoweeBuilding{};
    }
    if (!std::isfinite(bld.boundRadius) || bld.boundRadius < 0.0f) bld.boundRadius = 1.0f;

    uint16_t nameLen;
    f.read(reinterpret_cast<char*>(&nameLen), 2);
    // Same desync risk as material texture paths: overlong length
    // means the actual bytes are still on disk and reading 0 bytes
    // would shift every subsequent length+data pair.
    if (nameLen > 1024) {
        LOG_ERROR("WOB name length rejected (", nameLen, "): ", basePath);
        return WoweeBuilding{};
    }
    bld.name.resize(nameLen);
    f.read(bld.name.data(), nameLen);

    for (uint32_t gi = 0; gi < groupCount; gi++) {
        WoweeBuilding::Group grp;
        uint16_t gnLen;
        f.read(reinterpret_cast<char*>(&gnLen), 2);
        if (gnLen > 1024) {
            LOG_ERROR("WOB group ", gi, " name length rejected (",
                      gnLen, "): ", basePath);
            return WoweeBuilding{};
        }
        grp.name.resize(gnLen);
        f.read(grp.name.data(), gnLen);

        uint32_t vc, ic, tc;
        f.read(reinterpret_cast<char*>(&vc), 4);
        f.read(reinterpret_cast<char*>(&ic), 4);
        f.read(reinterpret_cast<char*>(&tc), 4);
        // Per-group sanity. WMO groups cap at 65k vertices in practice
        // (uint16 indices). Tex paths per group rarely exceed 16.
        if (vc > 1'000'000 || ic > 4'000'000 || tc > 1024) {
            LOG_ERROR("WOB group ", gi, " counts rejected (verts=", vc,
                      " indices=", ic, " textures=", tc, "): ", basePath);
            return WoweeBuilding{};
        }
        uint8_t outdoor;
        f.read(reinterpret_cast<char*>(&outdoor), 1);
        grp.isOutdoor = (outdoor != 0);
        f.read(reinterpret_cast<char*>(&grp.boundMin), 12);
        f.read(reinterpret_cast<char*>(&grp.boundMax), 12);

        grp.vertices.resize(vc);
        f.read(reinterpret_cast<char*>(grp.vertices.data()), vc * sizeof(WoweeBuilding::Vertex));
        // Sanitize vertex floats - WMO renderer matrix math is sensitive
        // and a NaN position can desync the entire group's draw state.
        sanitizeVertices(grp.vertices);
        grp.indices.resize(ic);
        f.read(reinterpret_cast<char*>(grp.indices.data()), ic * 4);
        // Same out-of-range index clamp as the WOM loader - bad indices
        // would crash the GPU draw on the WMO group.
        const uint32_t vMax = vc > 0 ? vc - 1 : 0;
        for (auto& idx : grp.indices) {
            if (idx > vMax) idx = 0;
        }

        // Helper: clear path on traversal/absolute attempt.
        auto rejectTraversal = [](std::string& s, const char* what) {
            if (s.find("..") != std::string::npos ||
                (!s.empty() && (s[0] == '/' || s[0] == '\\')) ||
                (s.size() >= 2 && s[1] == ':')) {
                LOG_WARNING("WOB ", what, " path rejected (traversal): ", s);
                s.clear();
            }
        };
        for (uint32_t ti = 0; ti < tc; ti++) {
            uint16_t tl;
            f.read(reinterpret_cast<char*>(&tl), 2);
            // tl > 1024 means we'd previously zero-out tl and skip the
            // read - but the original bytes are still on disk, so the
            // next iteration would read garbage as the next length+path.
            // Reject the whole load instead of silently desyncing.
            if (tl > 1024) {
                LOG_ERROR("WOB group ", gi, " texture path length rejected (",
                          tl, "): ", basePath);
                return WoweeBuilding{};
            }
            std::string tp(tl, '\0');
            f.read(tp.data(), tl);
            rejectTraversal(tp, "group texture");
            grp.texturePaths.push_back(tp);
        }

        // Read material data (v1.1+). Reject the whole load on a count
        // overflow rather than silently dropping the materials and leaving
        // the file pointer misaligned (next group's name would read
        // material bytes as garbage).
        uint32_t mc = 0;
        if (f.read(reinterpret_cast<char*>(&mc), 4) && mc > 0) {
            if (mc > 256) {
                LOG_ERROR("WOB group ", gi, " material count rejected (",
                          mc, "): ", basePath);
                return WoweeBuilding{};
            }
            for (uint32_t mi = 0; mi < mc; mi++) {
                WoweeBuilding::Material mat;
                uint16_t pl;
                f.read(reinterpret_cast<char*>(&pl), 2);
                // Same load-desync risk as group texture paths above -
                // overlong path means stale on-disk bytes would become
                // the next material's length+data. Reject the load.
                if (pl > 1024) {
                    LOG_ERROR("WOB group ", gi, " material ", mi,
                              " texture path length rejected (", pl, "): ", basePath);
                    return WoweeBuilding{};
                }
                mat.texturePath.resize(pl);
                f.read(mat.texturePath.data(), pl);
                rejectTraversal(mat.texturePath, "material texture");
                f.read(reinterpret_cast<char*>(&mat.flags), 4);
                f.read(reinterpret_cast<char*>(&mat.shader), 4);
                f.read(reinterpret_cast<char*>(&mat.blendMode), 4);
                grp.materials.push_back(mat);
            }
        }

        bld.groups.push_back(std::move(grp));
    }

    for (uint32_t pi = 0; pi < portalCount; pi++) {
        WoweeBuilding::Portal portal;
        f.read(reinterpret_cast<char*>(&portal.groupA), 4);
        f.read(reinterpret_cast<char*>(&portal.groupB), 4);
        uint32_t pvCount;
        f.read(reinterpret_cast<char*>(&pvCount), 4);
        // Real portals are 4-12 vertices; >4096 is corrupt.
        if (pvCount > 4096) {
            LOG_ERROR("WOB portal ", pi, " vertex count rejected (", pvCount, "): ", basePath);
            return WoweeBuilding{};
        }
        portal.vertices.resize(pvCount);
        f.read(reinterpret_cast<char*>(portal.vertices.data()), pvCount * 12);
        // Sanitize vertex floats - NaN portal vertices break the WMO
        // portal-frustum cull and would draw the whole interior every frame.
        for (auto& v : portal.vertices) {
            if (!std::isfinite(v.x)) v.x = 0.0f;
            if (!std::isfinite(v.y)) v.y = 0.0f;
            if (!std::isfinite(v.z)) v.z = 0.0f;
        }
        // Validate group indices are in range - out-of-range groupA/groupB
        // would index past wmo.groups during cull and segfault.
        const int32_t maxGroup = static_cast<int32_t>(groupCount);
        if (portal.groupA >= maxGroup) portal.groupA = -1;
        if (portal.groupB >= maxGroup) portal.groupB = -1;
        bld.portals.push_back(portal);
    }

    for (uint32_t di = 0; di < doodadCount; di++) {
        WoweeBuilding::DoodadPlacement dp;
        uint16_t pl;
        f.read(reinterpret_cast<char*>(&pl), 2);
        if (pl > 1024) {
            LOG_ERROR("WOB doodad ", di, " model path length rejected (",
                      pl, "): ", basePath);
            return WoweeBuilding{};
        }
        dp.modelPath.resize(pl);
        f.read(dp.modelPath.data(), pl);
        // Reject path-traversal in doodad model paths - these end up in
        // outModel.doodadNames and are passed to the asset manager. While
        // the manager only reads files, '..' paths in custom_zones could
        // probe for files outside the assets/ tree.
        if (dp.modelPath.find("..") != std::string::npos ||
            (!dp.modelPath.empty() && (dp.modelPath[0] == '/' || dp.modelPath[0] == '\\')) ||
            (dp.modelPath.size() >= 2 && dp.modelPath[1] == ':')) {
            LOG_WARNING("WOB doodad path rejected (traversal): ", dp.modelPath);
            dp.modelPath.clear();
        }
        f.read(reinterpret_cast<char*>(&dp.position), 12);
        f.read(reinterpret_cast<char*>(&dp.rotation), 12);
        f.read(reinterpret_cast<char*>(&dp.scale), 4);
        // Guard against corrupted scale (older WoBs that hadn't initialized
        // the field, or NaNs from a partial write). The renderer would
        // collapse the doodad to a point with scale 0.
        if (!std::isfinite(dp.scale) || dp.scale <= 0.0001f) dp.scale = 1.0f;
        // Same NaN scrub for position/rotation - doodads with non-finite
        // transforms produce NaN model matrices and crash the GPU.
        for (int k = 0; k < 3; k++) {
            if (!std::isfinite(dp.position[k])) dp.position[k] = 0.0f;
            if (!std::isfinite(dp.rotation[k])) dp.rotation[k] = 0.0f;
        }
        bld.doodads.push_back(dp);
    }

    LOG_INFO("WOB loaded: ", basePath, " (", groupCount, " groups, ",
             portalCount, " portals, ", doodadCount, " doodads)");
    return bld;
}

bool WoweeBuildingLoader::save(const WoweeBuilding& bld, const std::string& basePath) {
    namespace fs = std::filesystem;
    ensureParentDirectory(basePath);

    std::ofstream f(basePath + ".wob", std::ios::binary);
    if (!f) return false;

    f.write(reinterpret_cast<const char*>(&WOB_MAGIC), 4);
    // Cap header counts at the load-side limits so a pathological build
    // can't write a file the loader rejects whole. Same caps the load
    // bounds-check uses (4096 groups / 8192 portals / 65536 doodads).
    uint32_t gc = static_cast<uint32_t>(std::min<size_t>(bld.groups.size(), 4096));
    uint32_t pc = static_cast<uint32_t>(std::min<size_t>(bld.portals.size(), 8192));
    uint32_t dc = static_cast<uint32_t>(std::min<size_t>(bld.doodads.size(), 65536));
    f.write(reinterpret_cast<const char*>(&gc), 4);
    f.write(reinterpret_cast<const char*>(&pc), 4);
    f.write(reinterpret_cast<const char*>(&dc), 4);
    // Same float sanitize as WOM/WOC saves - defaults to 1.0 if non-finite.
    float boundRadius = std::isfinite(bld.boundRadius) && bld.boundRadius >= 0.0f
                            ? bld.boundRadius : 1.0f;
    f.write(reinterpret_cast<const char*>(&boundRadius), 4);

    // Truncate length-prefixed strings to fit their u16 length field - without
    // this, names over 65535 chars would silently get a wrap-around length and
    // produce a corrupt file (the string text would be longer than the saved
    // length and shift everything after it).
    auto writeStr = [&](const std::string& s, size_t maxLen = 1024) {
        uint16_t len = static_cast<uint16_t>(std::min<size_t>(s.size(), maxLen));
        f.write(reinterpret_cast<const char*>(&len), 2);
        f.write(s.data(), len);
    };
    writeStr(bld.name);

    // Iterate using the capped counts so the body matches the header.
    for (uint32_t gi = 0; gi < gc; gi++) {
        const auto& grp = bld.groups[gi];
        writeStr(grp.name);

        // Per-group counts capped at the load-side limits (1M verts /
        // 4M indices / 1024 texture paths). Without these caps a huge
        // group would write a header the loader rejects, leaking data.
        uint32_t vc = static_cast<uint32_t>(std::min<size_t>(grp.vertices.size(), 1'000'000));
        uint32_t ic = static_cast<uint32_t>(std::min<size_t>(grp.indices.size(), 4'000'000));
        uint32_t tc = static_cast<uint32_t>(std::min<size_t>(grp.texturePaths.size(), 1024));
        f.write(reinterpret_cast<const char*>(&vc), 4);
        f.write(reinterpret_cast<const char*>(&ic), 4);
        f.write(reinterpret_cast<const char*>(&tc), 4);
        uint8_t outdoor = grp.isOutdoor ? 1 : 0;
        f.write(reinterpret_cast<const char*>(&outdoor), 1);
        // Sanitize group bounds - non-finite would corrupt cull frustum.
        glm::vec3 bMin = grp.boundMin, bMax = grp.boundMax;
        for (int k = 0; k < 3; k++) {
            if (!std::isfinite(bMin[k])) bMin[k] = 0.0f;
            if (!std::isfinite(bMax[k])) bMax[k] = 0.0f;
        }
        f.write(reinterpret_cast<const char*>(&bMin), 12);
        f.write(reinterpret_cast<const char*>(&bMax), 12);

        // Sanitize vertices on the way out - same scrub the load side does.
        // Without this, a manually-constructed WoweeBuilding with NaN-laced
        // vertices would persist them into the file and have the load-time
        // guard clean up forever after.
        std::vector<WoweeBuilding::Vertex> sanVerts = grp.vertices;
        sanitizeVertices(sanVerts);
        f.write(reinterpret_cast<const char*>(sanVerts.data()),
                vc * sizeof(WoweeBuilding::Vertex));
        // Clamp out-of-range indices on save too - symmetric with load.
        const uint32_t vMax = vc > 0 ? vc - 1 : 0;
        std::vector<uint32_t> sanIdx = grp.indices;
        for (auto& idx : sanIdx) if (idx > vMax) idx = 0;
        f.write(reinterpret_cast<const char*>(sanIdx.data()), ic * 4);

        // Write only the capped number of texture paths so the body
        // matches the header (tc) on round-trip.
        for (uint32_t ti = 0; ti < tc; ti++) writeStr(grp.texturePaths[ti]);

        // Write material data - cap at 256 to match load-side limit so a
        // pathological in-memory count can't write a file the loader will
        // reject and produce a partially-zero build on round-trip.
        uint32_t mc = static_cast<uint32_t>(
            std::min<size_t>(grp.materials.size(), 256));
        f.write(reinterpret_cast<const char*>(&mc), 4);
        for (uint32_t mi = 0; mi < mc; mi++) {
            const auto& mat = grp.materials[mi];
            writeStr(mat.texturePath);
            f.write(reinterpret_cast<const char*>(&mat.flags), 4);
            f.write(reinterpret_cast<const char*>(&mat.shader), 4);
            f.write(reinterpret_cast<const char*>(&mat.blendMode), 4);
        }
    }

    for (uint32_t pi = 0; pi < pc; pi++) {
        const auto& portal = bld.portals[pi];
        f.write(reinterpret_cast<const char*>(&portal.groupA), 4);
        f.write(reinterpret_cast<const char*>(&portal.groupB), 4);
        // Cap per-portal vertex count at the load limit (4096). Real
        // portals are 4-12 verts; >4096 would be rejected on round-trip.
        uint32_t pvCount = static_cast<uint32_t>(
            std::min<size_t>(portal.vertices.size(), 4096));
        f.write(reinterpret_cast<const char*>(&pvCount), 4);
        // Sanitize vertices on the way out - NaN portal vertices break
        // the WMO portal-frustum cull and fail-back to drawing the entire
        // building, defeating the indoor optimization.
        std::vector<glm::vec3> sanPortal(portal.vertices.begin(),
                                          portal.vertices.begin() + pvCount);
        for (auto& v : sanPortal) {
            if (!std::isfinite(v.x)) v.x = 0.0f;
            if (!std::isfinite(v.y)) v.y = 0.0f;
            if (!std::isfinite(v.z)) v.z = 0.0f;
        }
        f.write(reinterpret_cast<const char*>(sanPortal.data()), pvCount * 12);
    }

    for (uint32_t di = 0; di < dc; di++) {
        const auto& dp = bld.doodads[di];
        writeStr(dp.modelPath);
        glm::vec3 pos = dp.position, rot = dp.rotation;
        for (int k = 0; k < 3; k++) {
            if (!std::isfinite(pos[k])) pos[k] = 0.0f;
            if (!std::isfinite(rot[k])) rot[k] = 0.0f;
        }
        float scale = (std::isfinite(dp.scale) && dp.scale > 0.0001f) ? dp.scale : 1.0f;
        f.write(reinterpret_cast<const char*>(&pos), 12);
        f.write(reinterpret_cast<const char*>(&rot), 12);
        f.write(reinterpret_cast<const char*>(&scale), 4);
    }

    LOG_INFO("WOB saved: ", basePath, ".wob (", gc, " groups)");
    return true;
}

bool WoweeBuildingLoader::toWMOModel(const WoweeBuilding& building, WMOModel& outModel) {
    if (building.groups.empty()) return false;

    outModel.nGroups = static_cast<uint32_t>(building.groups.size());
    outModel.groups.clear();
    outModel.textures.clear();
    outModel.materials.clear();
    outModel.portals.clear();
    outModel.portalVertices.clear();
    outModel.portalRefs.clear();

    // Build a global texture index from per-material texturePath strings.
    // First-group materials become the WMO material list; each unique texture
    // gets one entry in outModel.textures.

    // Reverse the .blp -> .png conversion that fromWMO did, since the WMO
    // renderer's PNG override system only triggers when the requested texture
    // path ends in .blp (it then probes for a .png next to the cache file).
    auto textureIndex = [&](const std::string& path) -> uint32_t {
        std::string p = path;
        if (p.size() >= 4) {
            std::string ext = p.substr(p.size() - 4);
            if (ext == ".png" || ext == ".PNG") p = p.substr(0, p.size() - 4) + ".blp";
        }
        if (p.empty()) return 0;
        for (uint32_t i = 0; i < outModel.textures.size(); i++) {
            if (outModel.textures[i] == p) return i;
        }
        outModel.textures.push_back(p);
        return static_cast<uint32_t>(outModel.textures.size() - 1);
    };

    // Collect unique materials across all groups. WMO has a single global
    // materials array shared by every group's batches; pulling only from
    // group[0] dropped per-group materials. Dedupe by (texture, blend, flags).
    auto materialKey = [](const WoweeBuilding::Material& m) {
        return m.texturePath + "|" + std::to_string(m.flags) + "|" + std::to_string(m.blendMode);
    };
    std::unordered_map<std::string, uint32_t> materialIndex;
    for (const auto& grp : building.groups) {
        for (const auto& mat : grp.materials) {
            std::string key = materialKey(mat);
            if (materialIndex.count(key)) continue;
            WMOMaterial wm{};
            wm.flags = mat.flags;
            wm.shader = mat.shader;
            wm.blendMode = mat.blendMode;
            wm.texture1 = textureIndex(mat.texturePath);
            wm.color1 = 0;
            wm.texture2 = 0;
            wm.color2 = 0;
            wm.texture3 = 0;
            wm.color3 = 0;
            materialIndex[key] = static_cast<uint32_t>(outModel.materials.size());
            outModel.materials.push_back(wm);
        }
    }

    for (const auto& grp : building.groups) {
        WMOGroup wmoGroup;
        wmoGroup.name = grp.name;
        wmoGroup.boundingBoxMin = grp.boundMin;
        wmoGroup.boundingBoxMax = grp.boundMax;
        if (grp.isOutdoor) wmoGroup.flags |= 0x08;

        wmoGroup.vertices.reserve(grp.vertices.size());
        for (const auto& v : grp.vertices) {
            WMOVertex wv;
            wv.position = v.position;
            wv.normal = v.normal;
            wv.texCoord = v.texCoord;
            wv.color = v.color;
            wmoGroup.vertices.push_back(wv);
        }

        wmoGroup.indices.reserve(grp.indices.size());
        // WMO format uses uint16 indices, so each group must stay under 65k
        // verts. WoB allows uint32 - log + clamp instead of silently
        // wrapping and producing garbage triangles in the renderer.
        bool warnedTrunc = false;
        for (uint32_t idx : grp.indices) {
            if (idx > 0xFFFF) {
                if (!warnedTrunc) {
                    LOG_WARNING("toWMOModel: group '", grp.name,
                                "' has index > 65535 (clamping to 0)");
                    warnedTrunc = true;
                }
                wmoGroup.indices.push_back(0);
            } else {
                wmoGroup.indices.push_back(static_cast<uint16_t>(idx));
            }
        }

        outModel.groups.push_back(std::move(wmoGroup));
    }

    // Reconstruct portal vertices + refs from WoB's higher-level portal struct.
    for (const auto& wp : building.portals) {
        WMOPortal portal{};
        portal.startVertex = static_cast<uint16_t>(outModel.portalVertices.size());
        portal.vertexCount = static_cast<uint16_t>(wp.vertices.size());
        portal.planeIndex = 0;
        for (const auto& v : wp.vertices) outModel.portalVertices.push_back(v);
        uint16_t portalIdx = static_cast<uint16_t>(outModel.portals.size());
        outModel.portals.push_back(portal);
        if (wp.groupA >= 0) {
            outModel.portalRefs.push_back({.portalIndex = portalIdx, .groupIndex = static_cast<uint16_t>(wp.groupA), .side = -1, .padding = 0});
        }
        if (wp.groupB >= 0) {
            outModel.portalRefs.push_back({.portalIndex = portalIdx, .groupIndex = static_cast<uint16_t>(wp.groupB), .side = 1, .padding = 0});
        }
    }

    // Restore doodads. WMODoodad keys nameIndex by MODN byte offset; we just
    // assign sequential offsets here since none of our paths share suffixes.
    outModel.doodads.clear();
    outModel.doodadNames.clear();
    outModel.doodadSets.clear();
    uint32_t doodadOffset = 0;
    for (const auto& dp : building.doodads) {
        // Convert WOM extension back to M2 for the runtime that may not have a
        // WOM-aware loader for in-WMO doodads yet.
        std::string mp = dp.modelPath;
        auto dot = mp.rfind('.');
        if (dot != std::string::npos) {
            std::string ext = mp.substr(dot + 1);
            if (ext == "wom") mp = mp.substr(0, dot) + ".m2";
        }
        outModel.doodadNames[doodadOffset] = mp;

        WMODoodad d{};
        d.nameIndex = doodadOffset;
        d.position = dp.position;
        // Convert euler degrees -> quaternion using the same convention that
        // glm::eulerAngles uses (so this round-trips cleanly with fromWMO).
        d.rotation = glm::quat(glm::radians(dp.rotation));
        d.scale = dp.scale;
        d.color = glm::vec4(1.0f);
        outModel.doodads.push_back(d);
        doodadOffset += static_cast<uint32_t>(mp.size() + 1);
    }

    // Renderer uses doodadSets[0] to know which slice of doodads to render.
    // Without it, even with doodads populated, nothing draws. Emit a default
    // "Set_$DefaultGlobal" set covering every doodad.
    if (!outModel.doodads.empty()) {
        WMODoodadSet ds{};
        std::strncpy(ds.name, "Set_$DefaultGlobal", sizeof(ds.name) - 1);
        ds.startIndex = 0;
        ds.count = static_cast<uint32_t>(outModel.doodads.size());
        ds.padding = 0;
        outModel.doodadSets.push_back(ds);
    }

    return true;
}

WoweeBuilding WoweeBuildingLoader::fromWMO(const WMOModel& wmo, const std::string& name) {
    WoweeBuilding bld;
    bld.name = name.empty() ? "Converted WMO" : name;

    float maxDist = 0.0f;
    for (const auto& grp : wmo.groups) {
        WoweeBuilding::Group wobGroup;
        wobGroup.name = grp.name;
        wobGroup.isOutdoor = (grp.flags & 0x08) != 0;
        wobGroup.boundMin = grp.boundingBoxMin;
        wobGroup.boundMax = grp.boundingBoxMax;

        wobGroup.vertices.reserve(grp.vertices.size());
        for (const auto& v : grp.vertices) {
            WoweeBuilding::Vertex wv;
            wv.position = v.position;
            wv.normal = v.normal;
            wv.texCoord = v.texCoord;
            wv.color = v.color;
            // Sanitize on conversion so a corrupt source WMO doesn't poison
            // the WOB and then surprise us on later reload.
            if (!std::isfinite(wv.position.x)) wv.position.x = 0.0f;
            if (!std::isfinite(wv.position.y)) wv.position.y = 0.0f;
            if (!std::isfinite(wv.position.z)) wv.position.z = 0.0f;
            if (!std::isfinite(wv.normal.x)) wv.normal.x = 0.0f;
            if (!std::isfinite(wv.normal.y)) wv.normal.y = 0.0f;
            if (!std::isfinite(wv.normal.z)) wv.normal.z = 1.0f;
            if (!std::isfinite(wv.texCoord.x)) wv.texCoord.x = 0.0f;
            if (!std::isfinite(wv.texCoord.y)) wv.texCoord.y = 0.0f;
            for (int c = 0; c < 4; c++)
                if (!std::isfinite(wv.color[c])) wv.color[c] = 1.0f;
            wobGroup.vertices.push_back(wv);

            float d = glm::length(wv.position);
            if (std::isfinite(d) && d > maxDist) maxDist = d;
        }

        wobGroup.indices.reserve(grp.indices.size());
        for (uint16_t idx : grp.indices)
            wobGroup.indices.push_back(static_cast<uint32_t>(idx));

        for (const auto& mat : wmo.materials) {
            WoweeBuilding::Material wobMat;
            wobMat.flags = mat.flags;
            wobMat.shader = mat.shader;
            wobMat.blendMode = mat.blendMode;
            if (mat.texture1 < wmo.textures.size()) {
                std::string texPath = wmo.textures[mat.texture1];
                auto dot = texPath.rfind('.');
                if (dot != std::string::npos)
                    texPath = texPath.substr(0, dot) + ".png";
                wobMat.texturePath = texPath;
                wobGroup.texturePaths.push_back(texPath);
            }
            wobGroup.materials.push_back(wobMat);
        }

        bld.groups.push_back(std::move(wobGroup));
    }

    bld.boundRadius = maxDist;

    // Extract portals (vertex polygons + group links via MOPR refs).
    // Each MOPR ref links a portal to one of its two adjacent groups; pairing
    // refs with the same portalIndex gives us groupA/groupB for that portal.
    for (size_t pi = 0; pi < wmo.portals.size(); pi++) {
        const auto& wmoPortal = wmo.portals[pi];
        WoweeBuilding::Portal wp;
        wp.groupA = -1;
        wp.groupB = -1;
        for (const auto& ref : wmo.portalRefs) {
            if (ref.portalIndex != static_cast<uint16_t>(pi)) continue;
            if (wp.groupA < 0) wp.groupA = ref.groupIndex;
            else if (wp.groupB < 0) wp.groupB = ref.groupIndex;
        }
        for (uint16_t vi = 0; vi < wmoPortal.vertexCount; vi++) {
            uint32_t idx = wmoPortal.startVertex + vi;
            if (idx < wmo.portalVertices.size()) {
                wp.vertices.push_back(wmo.portalVertices[idx]);
            }
        }
        if (!wp.vertices.empty()) bld.portals.push_back(std::move(wp));
    }

    for (const auto& doodad : wmo.doodads) {
        auto nameIt = wmo.doodadNames.find(doodad.nameIndex);
        if (nameIt == wmo.doodadNames.end()) continue;

        WoweeBuilding::DoodadPlacement dp;
        dp.modelPath = nameIt->second;
        auto dot = dp.modelPath.rfind('.');
        if (dot != std::string::npos)
            dp.modelPath = dp.modelPath.substr(0, dot) + ".wom";
        dp.position = doodad.position;
        // Sanitize position before euler conversion. NaN in the source
        // quaternion would propagate to NaN euler angles and ruin the
        // doodad transform forever after; same for the position floats.
        for (int k = 0; k < 3; k++)
            if (!std::isfinite(dp.position[k])) dp.position[k] = 0.0f;
        glm::quat q(doodad.rotation.w, doodad.rotation.x,
                     doodad.rotation.y, doodad.rotation.z);
        if (!std::isfinite(q.w) || !std::isfinite(q.x) ||
            !std::isfinite(q.y) || !std::isfinite(q.z))
            q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // identity
        dp.rotation = glm::degrees(glm::eulerAngles(q));
        for (int k = 0; k < 3; k++)
            if (!std::isfinite(dp.rotation[k])) dp.rotation[k] = 0.0f;
        dp.scale = (std::isfinite(doodad.scale) && doodad.scale > 0.0001f)
                       ? doodad.scale : 1.0f;
        bld.doodads.push_back(dp);
    }

    LOG_INFO("WOB from WMO: ", bld.name, " (", bld.groups.size(), " groups, ",
             bld.doodads.size(), " doodads)");
    return bld;
}

WoweeBuilding WoweeBuildingLoader::tryLoadByGamePath(
    const std::string& gamePath,
    const std::vector<std::string>& extraPrefixes) {
    std::string base = gamePath;
    auto dot = base.rfind('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    std::replace(base.begin(), base.end(), '\\', '/');
    auto tryPrefix = [&](const std::string& prefix) -> WoweeBuilding {
        std::string full = prefix + base;
        if (exists(full)) {
            auto wob = load(full);
            if (wob.isValid()) return wob;
        }
        return {};
    };
    for (const auto& p : extraPrefixes) {
        if (auto w = tryPrefix(p); w.isValid()) return w;
    }
    for (const char* p : {"custom_zones/buildings/", "output/buildings/"}) {
        if (auto w = tryPrefix(p); w.isValid()) return w;
    }
    return {};
}

} // namespace pipeline
} // namespace wowee
