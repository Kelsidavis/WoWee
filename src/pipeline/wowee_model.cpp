#include "pipeline/wowee_vertex_sanitize.hpp"
#include "pipeline/wowee_binary_io.hpp"
#include "pipeline/wowee_model.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <filesystem>
#include <cstring>
#include <cmath>

namespace wowee {
namespace pipeline {

static constexpr uint32_t WOM_MAGIC = 0x314D4F57; // "WOM1"
static constexpr uint32_t WOM2_MAGIC = 0x324D4F57; // "WOM2"
static constexpr uint32_t WOM3_MAGIC = 0x334D4F57; // "WOM3"

bool WoweeModelLoader::exists(const std::string& basePath) {
    return std::filesystem::exists(basePath + ".wom");
}

WoweeModel WoweeModelLoader::load(const std::string& basePath) {
    WoweeModel model;
    std::string womPath = basePath + ".wom";

    std::ifstream f(womPath, std::ios::binary);
    if (!f) return model;

    uint32_t magic;
    f.read(reinterpret_cast<char*>(&magic), 4);
    bool isV2 = (magic == WOM2_MAGIC || magic == WOM3_MAGIC);
    bool isV3 = (magic == WOM3_MAGIC);
    if (magic != WOM_MAGIC && magic != WOM2_MAGIC && magic != WOM3_MAGIC) return model;
    model.version = isV3 ? 3 : (isV2 ? 2 : 1);

    uint32_t vertCount, indexCount, texCount;
    f.read(reinterpret_cast<char*>(&vertCount), 4);
    f.read(reinterpret_cast<char*>(&indexCount), 4);
    f.read(reinterpret_cast<char*>(&texCount), 4);
    f.read(reinterpret_cast<char*>(&model.boundRadius), 4);
    f.read(reinterpret_cast<char*>(&model.boundMin), 12);
    f.read(reinterpret_cast<char*>(&model.boundMax), 12);
    // Sanity bounds. Real M2 models cap at 65k vertices (uint16 indices) and
    // typically 64 textures. Reject obviously corrupted counts before we
    // try to allocate huge vertex/index buffers.
    if (vertCount > 1'000'000 || indexCount > 4'000'000 || texCount > 1024) {
        LOG_ERROR("WOM header rejected (verts=", vertCount,
                  " indices=", indexCount, " textures=", texCount, "): ", basePath);
        return WoweeModel{};
    }

    // Bound sanity - radius drives M2 culling, min/max drive collision AABBs.
    // NaN/inf would either cull-out the model or crash the cull math.
    if (!std::isfinite(model.boundRadius) || model.boundRadius < 0.0f)
        model.boundRadius = 1.0f;
    auto sanitiseVec = [](glm::vec3& v) {
        if (!std::isfinite(v.x)) v.x = 0.0f;
        if (!std::isfinite(v.y)) v.y = 0.0f;
        if (!std::isfinite(v.z)) v.z = 0.0f;
    };
    sanitiseVec(model.boundMin);
    sanitiseVec(model.boundMax);

    uint16_t nameLen;
    f.read(reinterpret_cast<char*>(&nameLen), 2);
    // Save caps name at 1024; reject anything longer to keep load
    // alignment with future-version files predictable.
    if (nameLen > 1024) {
        LOG_ERROR("WOM name length rejected (", nameLen, "): ", basePath);
        return WoweeModel{};
    }
    model.name.resize(nameLen);
    f.read(model.name.data(), nameLen);

    // Read vertices (WOM1: 32 bytes/vert, WOM2: 40 bytes with bone data)
    model.vertices.resize(vertCount);
    if (isV2) {
        f.read(reinterpret_cast<char*>(model.vertices.data()),
               vertCount * sizeof(WoweeModel::Vertex));
    } else {
        // WOM1 backward compat: read 32-byte vertices (no bone data)
        struct V1Vertex { glm::vec3 pos; glm::vec3 norm; glm::vec2 uv; };
        for (uint32_t i = 0; i < vertCount; i++) {
            V1Vertex v1;
            f.read(reinterpret_cast<char*>(&v1), sizeof(V1Vertex));
            model.vertices[i].position = v1.pos;
            model.vertices[i].normal = v1.norm;
            model.vertices[i].texCoord = v1.uv;
        }
    }
    // Sanitize per-vertex floats. NaN/inf positions crash the M2 vertex
    // shader (silent device-lost on some drivers) - safer to render the
    // vertex at the origin than corrupt the whole pipeline.
    sanitizeVertices(model.vertices);

    model.indices.resize(indexCount);
    f.read(reinterpret_cast<char*>(model.indices.data()), indexCount * 4);
    // Clamp out-of-range indices - these would index past the vertex buffer
    // and crash the GPU vertex shader. Replace with 0 rather than drop, so
    // triangle counts stay aligned (a degenerate triangle is harmless,
    // an off-by-one indexing the wrong vertex is silent corruption).
    const uint32_t vMax = vertCount > 0 ? vertCount - 1 : 0;
    for (auto& idx : model.indices) {
        if (idx > vMax) idx = 0;
    }

    for (uint32_t i = 0; i < texCount; i++) {
        uint16_t pathLen;
        f.read(reinterpret_cast<char*>(&pathLen), 2);
        // Same desync risk as elsewhere - pathLen=0 with the actual
        // bytes still on disk would shift every subsequent length+data
        // pair. Reject the whole load instead of silently dropping.
        if (pathLen > 1024) {
            LOG_ERROR("WOM texture path ", i, " length rejected (",
                      pathLen, "): ", basePath);
            return WoweeModel{};
        }
        std::string path(pathLen, '\0');
        f.read(path.data(), pathLen);
        // Reject path-traversal - texture paths from a hostile WOM are fed
        // to the asset manager and could probe files outside assets/.
        if (path.find("..") != std::string::npos ||
            (!path.empty() && (path[0] == '/' || path[0] == '\\')) ||
            (path.size() >= 2 && path[1] == ':')) {
            LOG_WARNING("WOM texture path rejected (traversal): ", path);
            path.clear();
        }
        model.texturePaths.push_back(path);
    }

    // WOM2: read bones and animations
    if (isV2) {
        uint32_t boneCount = 0;
        if (f.read(reinterpret_cast<char*>(&boneCount), 4) && boneCount > 0 && boneCount <= 512) {
            model.bones.resize(boneCount);
            for (uint32_t bi = 0; bi < boneCount; bi++) {
                auto& bone = model.bones[bi];
                f.read(reinterpret_cast<char*>(&bone.keyBoneId), 4);
                f.read(reinterpret_cast<char*>(&bone.parentBone), 2);
                f.read(reinterpret_cast<char*>(&bone.pivot), 12);
                f.read(reinterpret_cast<char*>(&bone.flags), 4);
                // Sanitize pivot - bones with NaN pivots produce broken
                // skeleton matrices that ripple into every child bone.
                if (!std::isfinite(bone.pivot.x)) bone.pivot.x = 0.0f;
                if (!std::isfinite(bone.pivot.y)) bone.pivot.y = 0.0f;
                if (!std::isfinite(bone.pivot.z)) bone.pivot.z = 0.0f;
                // parentBone must be < boneCount (or -1) - out-of-range
                // parents would cause a use-after-free during bone-matrix
                // computation that walks the parent chain.
                if (bone.parentBone >= 0 &&
                    static_cast<uint32_t>(bone.parentBone) >= boneCount) {
                    bone.parentBone = -1;
                }
            }
        }

        uint32_t animCount = 0;
        // Track total keyframes loaded; cap at 10M total to prevent a
        // pathological model (e.g. 1024 anims × 512 bones × 10K keys = 5B
        // keyframes attempted otherwise).
        size_t totalKeyframes = 0;
        constexpr size_t kMaxTotalKeyframes = 10'000'000;
        if (f.read(reinterpret_cast<char*>(&animCount), 4) && animCount > 0 && animCount <= 1024) {
            model.animations.resize(animCount);
            for (uint32_t ai = 0; ai < animCount; ai++) {
                auto& anim = model.animations[ai];
                f.read(reinterpret_cast<char*>(&anim.id), 4);
                f.read(reinterpret_cast<char*>(&anim.durationMs), 4);
                f.read(reinterpret_cast<char*>(&anim.movingSpeed), 4);
                // Reject NaN movingSpeed; it leaks into displacement maths.
                if (!std::isfinite(anim.movingSpeed)) anim.movingSpeed = 0.0f;

                anim.boneKeyframes.resize(model.bones.size());
                for (size_t bi = 0; bi < model.bones.size(); bi++) {
                    uint32_t kfCount = 0;
                    f.read(reinterpret_cast<char*>(&kfCount), 4);
                    if (totalKeyframes >= kMaxTotalKeyframes) {
                        // Skip the keyframe payload by seeking past it so we
                        // don't desync the remaining anims/bones.
                        f.seekg(static_cast<std::streamoff>(kfCount) * 44, std::ios::cur);
                        continue;
                    }
                    for (uint32_t ki = 0; ki < kfCount && ki < 10000; ki++) {
                        if (totalKeyframes++ >= kMaxTotalKeyframes) break;
                        WoweeModel::AnimKeyframe kf;
                        f.read(reinterpret_cast<char*>(&kf.timeMs), 4);
                        f.read(reinterpret_cast<char*>(&kf.translation), 12);
                        f.read(reinterpret_cast<char*>(&kf.rotation), 16);
                        f.read(reinterpret_cast<char*>(&kf.scale), 12);
                        // Sanitize keyframe floats - bone interp returns NaN
                        // for any NaN input and corrupts the whole skeleton.
                        auto fixVec = [](glm::vec3& v, float def) {
                            if (!std::isfinite(v.x)) v.x = def;
                            if (!std::isfinite(v.y)) v.y = def;
                            if (!std::isfinite(v.z)) v.z = def;
                        };
                        fixVec(kf.translation, 0.0f);
                        fixVec(kf.scale, 1.0f);
                        if (!std::isfinite(kf.rotation.x)) kf.rotation.x = 0.0f;
                        if (!std::isfinite(kf.rotation.y)) kf.rotation.y = 0.0f;
                        if (!std::isfinite(kf.rotation.z)) kf.rotation.z = 0.0f;
                        if (!std::isfinite(kf.rotation.w)) kf.rotation.w = 1.0f;
                        anim.boneKeyframes[bi].push_back(kf);
                    }
                }
            }
        }
    }

    // WOM3: read batches (multi-material support).
    // Validate each batch references a real slice of the index buffer and a
    // real texture so a corrupted file can't crash the renderer.
    if (isV3) {
        uint32_t batchCount = 0;
        if (f.read(reinterpret_cast<char*>(&batchCount), 4) && batchCount > 0 && batchCount <= 4096) {
            model.batches.reserve(batchCount);
            for (uint32_t i = 0; i < batchCount; i++) {
                WoweeModel::Batch b;
                f.read(reinterpret_cast<char*>(&b.indexStart), 4);
                f.read(reinterpret_cast<char*>(&b.indexCount), 4);
                f.read(reinterpret_cast<char*>(&b.textureIndex), 4);
                f.read(reinterpret_cast<char*>(&b.blendMode), 2);
                f.read(reinterpret_cast<char*>(&b.flags), 2);
                if (b.indexCount == 0 ||
                    static_cast<uint64_t>(b.indexStart) + b.indexCount > model.indices.size() ||
                    (b.textureIndex >= model.texturePaths.size() && !model.texturePaths.empty())) {
                    LOG_WARNING("WOM3 batch ", i, " out of range (start=", b.indexStart,
                                " count=", b.indexCount, " tex=", b.textureIndex,
                                " maxIdx=", model.indices.size(),
                                " maxTex=", model.texturePaths.size(), ") - dropping");
                    continue;
                }
                model.batches.push_back(b);
            }
        }
    }

    LOG_INFO("WOM", (isV3 ? "3" : (isV2 ? "2" : "1")), " loaded: ", basePath, " (",
             vertCount, " verts, ", model.bones.size(), " bones, ",
             model.animations.size(), " anims, ", model.batches.size(), " batches)");
    return model;
}

bool WoweeModelLoader::save(const WoweeModel& model, const std::string& basePath) {
    namespace fs = std::filesystem;
    ensureParentDirectory(basePath);

    std::string womPath = basePath + ".wom";
    std::ofstream f(womPath, std::ios::binary);
    if (!f) return false;

    bool hasAnim = model.hasAnimation();
    bool hasBatches = model.hasBatches();
    // WOM3 implies WOM2 layout (vertex format with bones), so we only emit
    // WOM3 if the model also has animation data - pure-batch static meshes
    // still go to WOM1/WOM2 with batches written via the WOM3 trailing block
    // when present alongside animation. For static-only with batches, write
    // as WOM3 anyway (decoder handles missing bones).
    uint32_t magic = hasBatches ? WOM3_MAGIC : (hasAnim ? WOM2_MAGIC : WOM_MAGIC);
    f.write(reinterpret_cast<const char*>(&magic), 4);

    // Cap top-level counts at the load-side limits so a pathological
    // model can't write a file the loader rejects whole.
    uint32_t vertCount = static_cast<uint32_t>(
        std::min<size_t>(model.vertices.size(), 1'000'000));
    uint32_t indexCount = static_cast<uint32_t>(
        std::min<size_t>(model.indices.size(), 4'000'000));
    uint32_t texCount = static_cast<uint32_t>(
        std::min<size_t>(model.texturePaths.size(), 1024));
    f.write(reinterpret_cast<const char*>(&vertCount), 4);
    f.write(reinterpret_cast<const char*>(&indexCount), 4);
    f.write(reinterpret_cast<const char*>(&texCount), 4);
    // Sanitize bound floats at save time; matches the load-time guard so a
    // partial-write or in-memory corruption can't poison the file.
    float boundRadius = std::isfinite(model.boundRadius) && model.boundRadius >= 0.0f
                            ? model.boundRadius : 1.0f;
    glm::vec3 boundMin = model.boundMin, boundMax = model.boundMax;
    auto sanVec3 = [](glm::vec3& v) {
        if (!std::isfinite(v.x)) v.x = 0.0f;
        if (!std::isfinite(v.y)) v.y = 0.0f;
        if (!std::isfinite(v.z)) v.z = 0.0f;
    };
    sanVec3(boundMin);
    sanVec3(boundMax);
    f.write(reinterpret_cast<const char*>(&boundRadius), 4);
    f.write(reinterpret_cast<const char*>(&boundMin), 12);
    f.write(reinterpret_cast<const char*>(&boundMax), 12);

    // Same writeStr pattern as WoB: truncate to 1KB so the u16 length doesn't
    // wrap and corrupt the rest of the file.
    auto writeStr = [&](const std::string& s, size_t maxLen = 1024) {
        uint16_t len = static_cast<uint16_t>(std::min<size_t>(s.size(), maxLen));
        f.write(reinterpret_cast<const char*>(&len), 2);
        f.write(s.data(), len);
    };
    writeStr(model.name);

    // WOM2/WOM3 write full vertex with bone data; WOM1 writes 32-byte vertex
    if (hasAnim || hasBatches) {
        f.write(reinterpret_cast<const char*>(model.vertices.data()),
                vertCount * sizeof(WoweeModel::Vertex));
    } else {
        // WOM1: write only the capped vertCount entries so the body
        // matches the header.
        for (uint32_t vi = 0; vi < vertCount; vi++) {
            const auto& v = model.vertices[vi];
            f.write(reinterpret_cast<const char*>(&v.position), 12);
            f.write(reinterpret_cast<const char*>(&v.normal), 12);
            f.write(reinterpret_cast<const char*>(&v.texCoord), 8);
        }
    }

    // Clamp out-of-range indices on save too - symmetric with the load
    // guard. Avoids writing index values that the renderer would refuse
    // and that the load-time guard would have to clean up later.
    {
        const uint32_t vMax = vertCount > 0 ? vertCount - 1 : 0;
        std::vector<uint32_t> sanIdx = model.indices;
        for (auto& idx : sanIdx) if (idx > vMax) idx = 0;
        f.write(reinterpret_cast<const char*>(sanIdx.data()), indexCount * 4);
    }

    // Match header texCount on round-trip.
    for (uint32_t ti = 0; ti < texCount; ti++) writeStr(model.texturePaths[ti]);

    // WOM2/WOM3: write bones and animations (always, even if empty for WOM3)
    if (hasAnim || hasBatches) {
        // Cap counts at the load-side limits (512 bones, 1024 anims). Raw
        // size() would let a pathological in-memory model write a file the
        // loader silently rejects, leaving the post-truncation bytes to be
        // misread as the next section.
        uint32_t boneCount = static_cast<uint32_t>(
            std::min<size_t>(model.bones.size(), 512));
        f.write(reinterpret_cast<const char*>(&boneCount), 4);
        for (uint32_t bi = 0; bi < boneCount; bi++) {
            const auto& bone = model.bones[bi];
            // Symmetric scrub with load - pivot NaN propagates through
            // skeleton matrices to every child bone; parent indices outside
            // bone array would walk off the end during matrix evaluation.
            glm::vec3 pivot = bone.pivot;
            if (!std::isfinite(pivot.x)) pivot.x = 0.0f;
            if (!std::isfinite(pivot.y)) pivot.y = 0.0f;
            if (!std::isfinite(pivot.z)) pivot.z = 0.0f;
            int16_t parent = bone.parentBone;
            if (parent >= 0 && static_cast<uint32_t>(parent) >= boneCount)
                parent = -1;
            f.write(reinterpret_cast<const char*>(&bone.keyBoneId), 4);
            f.write(reinterpret_cast<const char*>(&parent), 2);
            f.write(reinterpret_cast<const char*>(&pivot), 12);
            f.write(reinterpret_cast<const char*>(&bone.flags), 4);
        }

        uint32_t animCount = static_cast<uint32_t>(
            std::min<size_t>(model.animations.size(), 1024));
        f.write(reinterpret_cast<const char*>(&animCount), 4);
        // Same NaN scrub as load - keyframes can carry corrupt source data
        // straight through fromM2 without ever round-tripping a load, so the
        // save side has to defend independently.
        auto sanV3 = [](glm::vec3 v, float def) {
            if (!std::isfinite(v.x)) v.x = def;
            if (!std::isfinite(v.y)) v.y = def;
            if (!std::isfinite(v.z)) v.z = def;
            return v;
        };
        for (uint32_t ai = 0; ai < animCount; ai++) {
            const auto& anim = model.animations[ai];
            f.write(reinterpret_cast<const char*>(&anim.id), 4);
            f.write(reinterpret_cast<const char*>(&anim.durationMs), 4);
            float movingSpeed = std::isfinite(anim.movingSpeed) ? anim.movingSpeed : 0.0f;
            f.write(reinterpret_cast<const char*>(&movingSpeed), 4);

            // Iterate bones using the *capped* boneCount so the per-bone
            // keyframe block stays aligned with what load expects to read.
            for (size_t bi = 0; bi < boneCount; bi++) {
                uint32_t kfCount = (bi < anim.boneKeyframes.size())
                    ? static_cast<uint32_t>(anim.boneKeyframes[bi].size()) : 0;
                f.write(reinterpret_cast<const char*>(&kfCount), 4);
                for (uint32_t ki = 0; ki < kfCount; ki++) {
                    const auto& kf = anim.boneKeyframes[bi][ki];
                    glm::vec3 t = sanV3(kf.translation, 0.0f);
                    glm::vec3 s = sanV3(kf.scale, 1.0f);
                    glm::quat q = kf.rotation;
                    if (!std::isfinite(q.x)) q.x = 0.0f;
                    if (!std::isfinite(q.y)) q.y = 0.0f;
                    if (!std::isfinite(q.z)) q.z = 0.0f;
                    if (!std::isfinite(q.w)) q.w = 1.0f;
                    f.write(reinterpret_cast<const char*>(&kf.timeMs), 4);
                    f.write(reinterpret_cast<const char*>(&t), 12);
                    f.write(reinterpret_cast<const char*>(&q), 16);
                    f.write(reinterpret_cast<const char*>(&s), 12);
                }
            }
        }
    }

    // WOM3: write batches. Drop batches that reference invalid index ranges
    // or texture slots - load would do the same drop and log a warning, but
    // skipping at save time keeps the file small and deterministic.
    if (hasBatches) {
        const uint32_t totalIdx = static_cast<uint32_t>(model.indices.size());
        const uint32_t totalTex = static_cast<uint32_t>(model.texturePaths.size());
        std::vector<WoweeModel::Batch> validBatches;
        validBatches.reserve(model.batches.size());
        for (const auto& b : model.batches) {
            if (b.indexCount == 0) continue;
            if (b.indexStart > totalIdx) continue;
            if (b.indexStart + b.indexCount > totalIdx) continue;
            if (totalTex > 0 && b.textureIndex >= totalTex) continue;
            validBatches.push_back(b);
        }
        // Cap batches at the load limit (4096). validBatches has already
        // dropped invalid entries; this trims the head if the model has
        // more than the loader will accept.
        uint32_t batchCount = static_cast<uint32_t>(
            std::min<size_t>(validBatches.size(), 4096));
        f.write(reinterpret_cast<const char*>(&batchCount), 4);
        for (uint32_t bi = 0; bi < batchCount; bi++) {
            const auto& b = validBatches[bi];
            f.write(reinterpret_cast<const char*>(&b.indexStart), 4);
            f.write(reinterpret_cast<const char*>(&b.indexCount), 4);
            f.write(reinterpret_cast<const char*>(&b.textureIndex), 4);
            f.write(reinterpret_cast<const char*>(&b.blendMode), 2);
            f.write(reinterpret_cast<const char*>(&b.flags), 2);
        }
    }

    LOG_INFO("WOM", (hasBatches ? "3" : (hasAnim ? "2" : "1")), " saved: ", womPath,
             " (", vertCount, " verts, ", model.bones.size(), " bones, ",
             model.animations.size(), " anims, ", model.batches.size(), " batches)");
    return true;
}

// Internal helper: convert a parsed M2Model (already merged with skin if
// applicable) into a WoweeModel. Shared by fromM2 (AssetManager path)
// and fromM2Bytes (extractor path).
static WoweeModel convertM2ToWom(const M2Model& m2);

// fromM2(path, am) lives in wowee_model_fromm2.cpp so the asset extractor
// can link wowee_model.cpp without pulling in the AssetManager dependency.
// convertM2ToWom() (defined below) is the shared conversion body.
WoweeModel convertM2ToWomShared(const M2Model& m2) { return convertM2ToWom(m2); }

WoweeModel WoweeModelLoader::fromM2Bytes(const std::vector<uint8_t>& m2Data,
                                          const std::vector<uint8_t>& skinData) {
    WoweeModel model;
    if (m2Data.empty()) return model;
    auto m2 = M2Loader::load(m2Data);
    if (!skinData.empty()) M2Loader::loadSkin(skinData, m2);
    if (!m2.isValid()) return model;
    return convertM2ToWom(m2);
}

static WoweeModel convertM2ToWom(const M2Model& m2) {
    WoweeModel model;
    model.name = m2.name;
    model.boundRadius = m2.boundRadius;

    // Convert vertices with bone data. Sanitize at conversion time so a
    // corrupt source M2 (mangled MPQ block, partial extraction) doesn't
    // silently produce a NaN-laced WOM that the load-time guard then has
    // to clean up on every load.
    model.vertices.reserve(m2.vertices.size());
    for (const auto& v : m2.vertices) {
        WoweeModel::Vertex wv;
        wv.position = v.position;
        wv.normal = v.normal;
        wv.texCoord = v.texCoords[0];
        std::memcpy(wv.boneWeights, v.boneWeights, 4);
        std::memcpy(wv.boneIndices, v.boneIndices, 4);
        if (!std::isfinite(wv.position.x)) wv.position.x = 0.0f;
        if (!std::isfinite(wv.position.y)) wv.position.y = 0.0f;
        if (!std::isfinite(wv.position.z)) wv.position.z = 0.0f;
        if (!std::isfinite(wv.normal.x)) wv.normal.x = 0.0f;
        if (!std::isfinite(wv.normal.y)) wv.normal.y = 0.0f;
        if (!std::isfinite(wv.normal.z)) wv.normal.z = 1.0f;
        if (!std::isfinite(wv.texCoord.x)) wv.texCoord.x = 0.0f;
        if (!std::isfinite(wv.texCoord.y)) wv.texCoord.y = 0.0f;
        model.vertices.push_back(wv);

        model.boundMin = glm::min(model.boundMin, wv.position);
        model.boundMax = glm::max(model.boundMax, wv.position);
    }

    model.indices.reserve(m2.indices.size());
    for (uint16_t idx : m2.indices)
        model.indices.push_back(static_cast<uint32_t>(idx));

    for (const auto& tex : m2.textures) {
        std::string path = tex.filename;
        auto dot = path.rfind('.');
        if (dot != std::string::npos)
            path = path.substr(0, dot) + ".png";
        model.texturePaths.push_back(path);
    }

    // Convert bones
    for (const auto& b : m2.bones) {
        WoweeModel::Bone wb;
        wb.keyBoneId = b.keyBoneId;
        wb.parentBone = b.parentBone;
        wb.pivot = b.pivot;
        wb.flags = b.flags;
        model.bones.push_back(wb);
    }

    // Convert batches with material/blend mode info (WOM3 feature).
    // Each M2 batch maps to a WOM batch - preserves multi-submesh material structure.
    for (const auto& mb : m2.batches) {
        WoweeModel::Batch wb;
        wb.indexStart = mb.indexStart;
        wb.indexCount = mb.indexCount;
        // Resolve textureLookup -> texture index
        uint32_t lookupIdx = mb.textureIndex;
        wb.textureIndex = (lookupIdx < m2.textureLookup.size())
            ? static_cast<uint32_t>(std::max<int16_t>(0, m2.textureLookup[lookupIdx]))
            : 0;
        if (mb.materialIndex < m2.materials.size()) {
            const auto& mat = m2.materials[mb.materialIndex];
            wb.blendMode = mat.blendMode;
            wb.flags = mat.flags;
        }
        model.batches.push_back(wb);
    }

    // Convert animations (first keyframe per bone per sequence)
    for (const auto& seq : m2.sequences) {
        WoweeModel::Animation anim;
        anim.id = seq.id;
        anim.durationMs = seq.duration;
        anim.movingSpeed = seq.movingSpeed;
        anim.boneKeyframes.resize(model.bones.size());

        for (size_t bi = 0; bi < m2.bones.size() && bi < model.bones.size(); bi++) {
            const auto& bone = m2.bones[bi];
            // Find keyframes for this sequence index
            uint32_t seqIdx = static_cast<uint32_t>(&seq - m2.sequences.data());

            auto extractKeys = [&](const M2AnimationTrack& track, size_t boneIdx) {
                if (seqIdx >= track.sequences.size()) return;
                const auto& sk = track.sequences[seqIdx];
                for (size_t ki = 0; ki < sk.timestamps.size(); ki++) {
                    // Check if we already have this timestamp
                    bool found = false;
                    for (auto& existing : anim.boneKeyframes[boneIdx]) {
                        if (existing.timeMs == sk.timestamps[ki]) {
                            if (!sk.vec3Values.empty() && ki < sk.vec3Values.size()) {
                                if (&track == &bone.translation)
                                    existing.translation = sk.vec3Values[ki];
                                else
                                    existing.scale = sk.vec3Values[ki];
                            }
                            if (!sk.quatValues.empty() && ki < sk.quatValues.size())
                                existing.rotation = sk.quatValues[ki];
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        WoweeModel::AnimKeyframe kf;
                        kf.timeMs = sk.timestamps[ki];
                        kf.translation = glm::vec3(0);
                        kf.rotation = glm::quat(1, 0, 0, 0);
                        kf.scale = glm::vec3(1);
                        if (!sk.vec3Values.empty() && ki < sk.vec3Values.size()) {
                            if (&track == &bone.translation)
                                kf.translation = sk.vec3Values[ki];
                            else
                                kf.scale = sk.vec3Values[ki];
                        }
                        if (!sk.quatValues.empty() && ki < sk.quatValues.size())
                            kf.rotation = sk.quatValues[ki];
                        anim.boneKeyframes[boneIdx].push_back(kf);
                    }
                }
            };

            extractKeys(bone.translation, bi);
            extractKeys(bone.rotation, bi);
            extractKeys(bone.scale, bi);
        }

        if (anim.durationMs > 0) model.animations.push_back(anim);
    }

    // Version reflects highest feature in use: WOM3 if multi-batch, WOM2 if
    // animated, WOM1 if just static geometry. The save() function picks magic
    // off this same hierarchy.
    model.version = model.hasBatches() ? 3 : (model.hasAnimation() ? 2 : 1);
    return model;
}

M2Model WoweeModelLoader::toM2(const WoweeModel& wom) {
    M2Model m;
    if (!wom.isValid()) return m;

    m.name = wom.name;
    m.boundRadius = wom.boundRadius;

    m.vertices.reserve(wom.vertices.size());
    for (const auto& v : wom.vertices) {
        M2Vertex mv;
        mv.position = v.position;
        mv.normal = v.normal;
        mv.texCoords[0] = v.texCoord;
        std::memcpy(mv.boneWeights, v.boneWeights, 4);
        std::memcpy(mv.boneIndices, v.boneIndices, 4);
        m.vertices.push_back(mv);
    }

    m.indices.reserve(wom.indices.size());
    for (uint32_t idx : wom.indices)
        m.indices.push_back(static_cast<uint16_t>(idx));

    // Convert .png paths back to .blp so the M2 renderer's PNG override path
    // engages - it's keyed on .blp extension, not .png. fromM2 stored .png to
    // signal intent; toM2 has to undo that for the runtime to find textures.
    for (const auto& tp : wom.texturePaths) {
        M2Texture tex;
        tex.type = 0;
        tex.flags = 0;
        tex.filename = tp;
        if (tex.filename.size() >= 4) {
            std::string ext = tex.filename.substr(tex.filename.size() - 4);
            if (ext == ".png" || ext == ".PNG")
                tex.filename = tex.filename.substr(0, tex.filename.size() - 4) + ".blp";
        }
        m.textures.push_back(tex);
    }
    m.textureLookup.clear();
    for (uint32_t i = 0; i < wom.texturePaths.size(); i++)
        m.textureLookup.push_back(static_cast<int16_t>(i));
    if (m.textureLookup.empty()) m.textureLookup.push_back(0);

    if (wom.hasBatches()) {
        for (const auto& wb : wom.batches) {
            M2Batch batch{};
            batch.indexStart = wb.indexStart;
            batch.indexCount = wb.indexCount;
            batch.vertexCount = static_cast<uint32_t>(m.vertices.size());
            batch.textureCount = 1;
            // textureLookup may be empty when the WOM has no textures at all;
            // in that case the renderer falls back to its white default.
            uint16_t safeTexIdx = m.textureLookup.empty()
                ? 0
                : static_cast<uint16_t>(std::min<size_t>(static_cast<size_t>(wb.textureIndex), m.textureLookup.size() - 1));
            batch.textureIndex = safeTexIdx;
            batch.materialIndex = static_cast<uint16_t>(m.materials.size());
            m.batches.push_back(batch);
            M2Material mat;
            mat.flags = wb.flags;
            mat.blendMode = wb.blendMode;
            m.materials.push_back(mat);
        }
    } else {
        M2Batch batch{};
        batch.textureCount = std::min(1u, static_cast<uint32_t>(wom.texturePaths.size()));
        batch.indexCount = static_cast<uint32_t>(m.indices.size());
        batch.vertexCount = static_cast<uint32_t>(m.vertices.size());
        m.batches.push_back(batch);
        M2Material mat;
        mat.flags = 0;
        mat.blendMode = 0;
        m.materials.push_back(mat);
    }

    // Copy bones (WOM2/WOM3) - pivot/parent only, animation tracks are filled
    // from the WoM animation block below.
    for (const auto& wb : wom.bones) {
        M2Bone bone;
        bone.keyBoneId = wb.keyBoneId;
        bone.parentBone = wb.parentBone;
        bone.pivot = wb.pivot;
        bone.flags = wb.flags;
        m.bones.push_back(bone);
    }

    // Copy animation sequence headers (id/duration/movingSpeed). Per-bone
    // keyframes inside WoM are richer than M2Sequence captures so a future
    // animator may want a deeper conversion; this is enough for length-based
    // selection in the renderer.
    for (const auto& wa : wom.animations) {
        M2Sequence seq;
        seq.id = wa.id;
        seq.duration = wa.durationMs;
        seq.movingSpeed = wa.movingSpeed;
        m.sequences.push_back(seq);
    }

    return m;
}

WoweeModel WoweeModelLoader::tryLoadByGamePath(
    const std::string& gamePath,
    const std::vector<std::string>& extraPrefixes) {
    std::string base = gamePath;
    auto dot = base.rfind('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    std::replace(base.begin(), base.end(), '\\', '/');
    auto tryPrefix = [&](const std::string& prefix) -> WoweeModel {
        std::string full = prefix + base;
        if (exists(full)) {
            auto wom = load(full);
            if (wom.isValid()) return wom;
        }
        return {};
    };
    for (const auto& p : extraPrefixes) {
        if (auto w = tryPrefix(p); w.isValid()) return w;
    }
    for (const char* p : {"custom_zones/models/", "output/models/"}) {
        if (auto w = tryPrefix(p); w.isValid()) return w;
    }
    return {};
}

} // namespace pipeline
} // namespace wowee
