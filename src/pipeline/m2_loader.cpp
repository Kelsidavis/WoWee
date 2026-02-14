/**
 * M2 Model Loader — Binary parser for WoW's M2 model format (WotLK 3.3.5a)
 *
 * M2 files contain skeletal-animated meshes used for characters, creatures,
 * and doodads. The format stores geometry, bones with animation tracks,
 * textures, and material batches. A companion .skin file holds the rendering
 * batches and submesh definitions.
 *
 * Key format details:
 *  - On-disk bone struct is 88 bytes (includes 3 animation track headers).
 *  - Animation tracks use an "array-of-arrays" indirection: the header points
 *    to N sub-array headers, each being {uint32 count, uint32 offset}.
 *  - Rotation tracks store compressed quaternions as int16[4], decoded with
 *    an offset mapping (not simple division).
 *  - Skin file indices use two-level indirection: triangle → vertex lookup
 *    table → global vertex index.
 *  - Skin batch struct is 24 bytes on disk — the geosetIndex field at offset 10
 *    is easily missed, causing a 2-byte alignment shift on all subsequent fields.
 *
 * Reference: https://wowdev.wiki/M2
 */
#include "pipeline/m2_loader.hpp"
#include "core/logger.hpp"
#include <cstring>
#include <algorithm>

namespace wowee {
namespace pipeline {

namespace {

// M2 file header structure (version 260+ for WotLK 3.3.5a)
struct M2Header {
    char magic[4];              // 'MD20'
    uint32_t version;
    uint32_t nameLength;
    uint32_t nameOffset;
    uint32_t globalFlags;

    uint32_t nGlobalSequences;
    uint32_t ofsGlobalSequences;
    uint32_t nAnimations;
    uint32_t ofsAnimations;
    uint32_t nAnimationLookup;
    uint32_t ofsAnimationLookup;

    uint32_t nBones;
    uint32_t ofsBones;
    uint32_t nKeyBoneLookup;
    uint32_t ofsKeyBoneLookup;

    uint32_t nVertices;
    uint32_t ofsVertices;
    uint32_t nViews;            // Number of skin files

    uint32_t nColors;
    uint32_t ofsColors;
    uint32_t nTextures;
    uint32_t ofsTextures;

    uint32_t nTransparency;
    uint32_t ofsTransparency;
    uint32_t nUVAnimation;
    uint32_t ofsUVAnimation;
    uint32_t nTexReplace;
    uint32_t ofsTexReplace;

    uint32_t nRenderFlags;
    uint32_t ofsRenderFlags;
    uint32_t nBoneLookupTable;
    uint32_t ofsBoneLookupTable;
    uint32_t nTexLookup;
    uint32_t ofsTexLookup;

    uint32_t nTexUnits;
    uint32_t ofsTexUnits;
    uint32_t nTransLookup;
    uint32_t ofsTransLookup;
    uint32_t nUVAnimLookup;
    uint32_t ofsUVAnimLookup;

    float vertexBox[6];         // Bounding box
    float vertexRadius;
    float boundingBox[6];
    float boundingRadius;

    uint32_t nBoundingTriangles;
    uint32_t ofsBoundingTriangles;
    uint32_t nBoundingVertices;
    uint32_t ofsBoundingVertices;
    uint32_t nBoundingNormals;
    uint32_t ofsBoundingNormals;

    uint32_t nAttachments;
    uint32_t ofsAttachments;
    uint32_t nAttachmentLookup;
    uint32_t ofsAttachmentLookup;

    uint32_t nEvents;
    uint32_t ofsEvents;
    uint32_t nLights;
    uint32_t ofsLights;
    uint32_t nCameras;
    uint32_t ofsCameras;
    uint32_t nCameraLookup;
    uint32_t ofsCameraLookup;
    uint32_t nRibbonEmitters;
    uint32_t ofsRibbonEmitters;
    uint32_t nParticleEmitters;
    uint32_t ofsParticleEmitters;
};

// M2 vertex structure (on-disk format)
struct M2VertexDisk {
    float pos[3];
    uint8_t boneWeights[4];
    uint8_t boneIndices[4];
    float normal[3];
    float texCoords[2][2];
};

// M2 animation track header (on-disk, 20 bytes)
struct M2TrackDisk {
    uint16_t interpolationType;
    int16_t globalSequence;
    uint32_t nTimestamps;
    uint32_t ofsTimestamps;
    uint32_t nKeys;
    uint32_t ofsKeys;
};

// FBlock header (on-disk, 16 bytes) — particle lifetime curves
// Like M2TrackDisk but WITHOUT interpolationType/globalSequence prefix
struct FBlockDisk {
    uint32_t nTimestamps;
    uint32_t ofsTimestamps;
    uint32_t nKeys;
    uint32_t ofsKeys;
};

// Full M2 bone structure (on-disk, 88 bytes for WotLK)
struct M2BoneDisk {
    int32_t keyBoneId;          // 4
    uint32_t flags;             // 4
    int16_t parentBone;         // 2
    uint16_t submeshId;         // 2
    uint32_t boneNameCRC;       // 4
    M2TrackDisk translation;    // 20
    M2TrackDisk rotation;       // 20
    M2TrackDisk scale;          // 20
    float pivot[3];             // 12
};                              // Total: 88

// Vanilla M2 animation track header (on-disk, 28 bytes — has extra ranges M2Array)
struct M2TrackDiskVanilla {
    uint16_t interpolationType; // 2
    int16_t globalSequence;     // 2
    uint32_t nRanges;           // 4 — extra in vanilla (animation sequence ranges)
    uint32_t ofsRanges;         // 4 — extra in vanilla
    uint32_t nTimestamps;       // 4
    uint32_t ofsTimestamps;     // 4
    uint32_t nKeys;             // 4
    uint32_t ofsKeys;           // 4
};                              // Total: 28

// Vanilla M2 bone structure (on-disk, 108 bytes — no boneNameCRC, 28-byte tracks)
struct M2BoneDiskVanilla {
    int32_t keyBoneId;              // 4
    uint32_t flags;                 // 4
    int16_t parentBone;             // 2
    uint16_t submeshId;             // 2
    M2TrackDiskVanilla translation; // 28
    M2TrackDiskVanilla rotation;    // 28
    M2TrackDiskVanilla scale;       // 28
    float pivot[3];                 // 12
};                                  // Total: 108

// M2 animation sequence structure (WotLK, 64 bytes)
struct M2SequenceDisk {
    uint16_t id;
    uint16_t variationIndex;
    uint32_t duration;
    float movingSpeed;
    uint32_t flags;
    int16_t frequency;
    uint16_t padding;
    uint32_t replayMin;
    uint32_t replayMax;
    uint32_t blendTime;
    float bounds[6];
    float boundRadius;
    int16_t nextAnimation;
    uint16_t aliasNext;
};

// Vanilla M2 animation sequence (68 bytes — has start_timestamp before duration)
struct M2SequenceDiskVanilla {
    uint16_t id;
    uint16_t variationIndex;
    uint32_t startTimestamp;    // Extra field in vanilla (removed in WotLK)
    uint32_t endTimestamp;      // Becomes 'duration' in WotLK
    float movingSpeed;
    uint32_t flags;
    int16_t frequency;
    uint16_t padding;
    uint32_t replayMin;
    uint32_t replayMax;
    uint32_t blendTime;
    float bounds[6];
    float boundRadius;
    int16_t nextAnimation;
    uint16_t aliasNext;
};

// M2 texture definition
struct M2TextureDisk {
    uint32_t type;
    uint32_t flags;
    uint32_t nameLength;
    uint32_t nameOffset;
};

// Skin file header (contains rendering batches)
struct M2SkinHeader {
    char magic[4];              // 'SKIN'
    uint32_t nIndices;
    uint32_t ofsIndices;
    uint32_t nTriangles;
    uint32_t ofsTriangles;
    uint32_t nVertexProperties;
    uint32_t ofsVertexProperties;
    uint32_t nSubmeshes;
    uint32_t ofsSubmeshes;
    uint32_t nBatches;
    uint32_t ofsBatches;
    uint32_t nBones;
};

// Skin submesh structure (48 bytes for WotLK)
struct M2SkinSubmesh {
    uint16_t id;
    uint16_t level;
    uint16_t vertexStart;
    uint16_t vertexCount;
    uint16_t indexStart;
    uint16_t indexCount;
    uint16_t boneCount;
    uint16_t boneStart;
    uint16_t boneInfluences;
    uint16_t centerBoneIndex;
    float centerPosition[3];
    float sortCenterPosition[3];
    float sortRadius;
};

// Vanilla M2 skin submesh (32 bytes, version < 264 — no sortCenter/sortRadius)
struct M2SkinSubmeshVanilla {
    uint16_t id;
    uint16_t level;
    uint16_t vertexStart;
    uint16_t vertexCount;
    uint16_t indexStart;
    uint16_t indexCount;
    uint16_t boneCount;
    uint16_t boneStart;
    uint16_t boneInfluences;
    uint16_t centerBoneIndex;
    float centerPosition[3];
};

// Embedded skin profile for vanilla M2 (no 'SKIN' magic, offsets are M2-file-relative)
struct M2SkinProfileEmbedded {
    uint32_t nIndices;
    uint32_t ofsIndices;
    uint32_t nTriangles;
    uint32_t ofsTriangles;
    uint32_t nVertexProperties;
    uint32_t ofsVertexProperties;
    uint32_t nSubmeshes;
    uint32_t ofsSubmeshes;
    uint32_t nBatches;
    uint32_t ofsBatches;
    uint32_t nBones;
};

// Skin batch structure (24 bytes on disk)
struct M2BatchDisk {
    uint8_t flags;
    int8_t priorityPlane;
    uint16_t shader;
    uint16_t skinSectionIndex;
    uint16_t geosetIndex;           // Geoset index (not same as submesh ID)
    uint16_t colorIndex;
    uint16_t materialIndex;
    uint16_t materialLayer;
    uint16_t textureCount;
    uint16_t textureComboIndex;     // Index into texture lookup table
    uint16_t textureCoordIndex;     // Texture coordinate combo index
    uint16_t textureWeightIndex;    // Transparency lookup index
    uint16_t textureTransformIndex; // Texture animation lookup index
};

// Compressed quaternion (on-disk) for rotation tracks
struct CompressedQuat {
    int16_t x, y, z, w;
};

// M2 texture transform (on-disk, 3 × M2TrackDisk = 60 bytes)
struct M2TextureTransformDisk {
    M2TrackDisk translation;    // 20
    M2TrackDisk rotation;       // 20
    M2TrackDisk scaling;        // 20
};

// Vanilla M2 texture transform (3 × 28-byte tracks = 84 bytes)
struct M2TextureTransformDiskVanilla {
    M2TrackDiskVanilla translation; // 28
    M2TrackDiskVanilla rotation;    // 28
    M2TrackDiskVanilla scaling;     // 28
};

// M2 attachment point (on-disk, WotLK — 40 bytes)
struct M2AttachmentDisk {
    uint32_t id;
    uint16_t bone;
    uint16_t unknown;
    float position[3];
    uint8_t trackData[20]; // M2TrackDisk (20 bytes)
};

// M2 attachment point (on-disk, vanilla — 48 bytes, track is 28 bytes)
struct M2AttachmentDiskVanilla {
    uint32_t id;
    uint16_t bone;
    uint16_t unknown;
    float position[3];
    uint8_t trackData[28]; // M2TrackDiskVanilla (28 bytes)
};

template<typename T>
T readValue(const std::vector<uint8_t>& data, uint32_t offset) {
    if (offset + sizeof(T) > data.size()) {
        return T{};
    }
    T value;
    std::memcpy(&value, &data[offset], sizeof(T));
    return value;
}

template<typename T>
std::vector<T> readArray(const std::vector<uint8_t>& data, uint32_t offset, uint32_t count) {
    std::vector<T> result;
    if (count == 0) return result;
    // Overflow-safe bounds check: avoid uint32 wrap on count * sizeof(T)
    size_t totalBytes = static_cast<size_t>(count) * sizeof(T);
    if (totalBytes / sizeof(T) != count) return result;  // multiplication overflowed
    if (static_cast<size_t>(offset) + totalBytes > data.size()) return result;
    // Sanity cap: refuse allocations > 64MB to prevent garbage counts from OOMing
    if (totalBytes > 64 * 1024 * 1024) return result;

    result.resize(count);
    std::memcpy(result.data(), &data[offset], totalBytes);
    return result;
}

std::string readString(const std::vector<uint8_t>& data, uint32_t offset, uint32_t length) {
    if (offset + length > data.size()) {
        return "";
    }

    // Strip trailing null bytes (M2 nameLength includes \0)
    while (length > 0 && data[offset + length - 1] == 0) {
        length--;
    }

    return std::string(reinterpret_cast<const char*>(&data[offset]), length);
}

enum class TrackType { VEC3, QUAT_COMPRESSED, FLOAT };

// Parse an M2 animation track from the binary data.
// The track uses an "array of arrays" layout: nTimestamps pairs of {count, offset}.
// sequenceFlags: per-sequence flags; sequences WITHOUT flag 0x20 store their keyframe
// data in external .anim files, so their sub-array offsets are .anim-relative and must
// be skipped when reading from the M2 file.
void parseAnimTrack(const std::vector<uint8_t>& data,
                    const M2TrackDisk& disk,
                    M2AnimationTrack& track,
                    TrackType type,
                    const std::vector<uint32_t>& sequenceFlags = {}) {
    track.interpolationType = disk.interpolationType;
    track.globalSequence = disk.globalSequence;

    if (disk.nTimestamps == 0 || disk.nKeys == 0) return;

    uint32_t numSubArrays = disk.nTimestamps;
    // Sanity cap: no model has >4096 animation sequences; garbage counts cause OOM
    if (numSubArrays > 4096) return;
    track.sequences.resize(numSubArrays);

    for (uint32_t i = 0; i < numSubArrays; i++) {
        // Sequences without flag 0x20 have their animation data in external .anim files.
        // Their sub-array offsets are .anim-file-relative, not M2-relative, so reading
        // from the M2 file would produce garbage data.
        if (i < sequenceFlags.size() && !(sequenceFlags[i] & 0x20)) continue;
        // Each sub-array header is {uint32_t count, uint32_t offset} = 8 bytes
        uint32_t tsHeaderOfs = disk.ofsTimestamps + i * 8;
        uint32_t keyHeaderOfs = disk.ofsKeys + i * 8;

        if (tsHeaderOfs + 8 > data.size() || keyHeaderOfs + 8 > data.size()) continue;

        uint32_t tsCount = readValue<uint32_t>(data, tsHeaderOfs);
        uint32_t tsOffset = readValue<uint32_t>(data, tsHeaderOfs + 4);
        uint32_t keyCount = readValue<uint32_t>(data, keyHeaderOfs);
        uint32_t keyOffset = readValue<uint32_t>(data, keyHeaderOfs + 4);

        if (tsCount == 0 || keyCount == 0) continue;

        // Validate offsets are within file data (external .anim files have out-of-range offsets)
        if (tsOffset + tsCount * sizeof(uint32_t) > data.size()) continue;

        // Read timestamps
        auto timestamps = readArray<uint32_t>(data, tsOffset, tsCount);
        track.sequences[i].timestamps = std::move(timestamps);

        // Validate key data offset
        size_t keyElementSize;
        if (type == TrackType::FLOAT) keyElementSize = sizeof(float);
        else if (type == TrackType::VEC3) keyElementSize = sizeof(float) * 3;
        else keyElementSize = sizeof(int16_t) * 4;
        if (keyOffset + keyCount * keyElementSize > data.size()) {
            track.sequences[i].timestamps.clear();
            continue;
        }

        // Read key values
        if (type == TrackType::FLOAT) {
            auto values = readArray<float>(data, keyOffset, keyCount);
            track.sequences[i].floatValues = std::move(values);
        } else if (type == TrackType::VEC3) {
            // Translation/scale: float[3] per key
            struct Vec3Disk { float x, y, z; };
            auto values = readArray<Vec3Disk>(data, keyOffset, keyCount);
            track.sequences[i].vec3Values.reserve(values.size());
            for (const auto& v : values) {
                track.sequences[i].vec3Values.emplace_back(v.x, v.y, v.z);
            }
        } else {
            // Rotation: compressed quaternion int16[4] per key
            auto compressed = readArray<CompressedQuat>(data, keyOffset, keyCount);
            track.sequences[i].quatValues.reserve(compressed.size());
            for (const auto& cq : compressed) {
                // M2 compressed quaternion: offset mapping, NOT simple division
                // int16 range [-32768..32767] maps to float [-1..1] with offset
                float fx = (cq.x < 0) ? (cq.x + 32768) / 32767.0f : (cq.x - 32767) / 32767.0f;
                float fy = (cq.y < 0) ? (cq.y + 32768) / 32767.0f : (cq.y - 32767) / 32767.0f;
                float fz = (cq.z < 0) ? (cq.z + 32768) / 32767.0f : (cq.z - 32767) / 32767.0f;
                float fw = (cq.w < 0) ? (cq.w + 32768) / 32767.0f : (cq.w - 32767) / 32767.0f;
                // M2 on-disk: (x,y,z,w), GLM quat constructor: (w,x,y,z)
                glm::quat q(fw, fx, fy, fz);
                float len = glm::length(q);
                if (len > 0.001f) {
                    q = q / len;
                } else {
                    q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // identity
                }
                track.sequences[i].quatValues.push_back(q);
            }
        }
    }
}

// Vanilla M2 range: indices into flat timestamp/key arrays for a given sequence
struct M2Range { uint32_t start; uint32_t end; };

// Parse a vanilla M2 animation track (version < 264).
// Vanilla uses flat arrays with per-sequence M2Range indices, unlike WotLK's array-of-arrays.
// Vanilla also uses Quaternion16 (simple x/32767) instead of WotLK's CompressedQuaternion.
void parseAnimTrackVanilla(const std::vector<uint8_t>& data,
                           const M2TrackDiskVanilla& disk,
                           M2AnimationTrack& track,
                           TrackType type) {
    track.interpolationType = disk.interpolationType;
    track.globalSequence = disk.globalSequence;

    if (disk.nTimestamps == 0 || disk.nKeys == 0) return;
    // Sanity caps
    if (disk.nTimestamps > 100000 || disk.nKeys > 100000) return;

    // Validate flat timestamp array
    if (disk.ofsTimestamps + disk.nTimestamps * sizeof(uint32_t) > data.size()) return;
    auto allTimestamps = readArray<uint32_t>(data, disk.ofsTimestamps, disk.nTimestamps);

    // Validate flat key array
    // Vanilla stores rotations as full float quaternions (16 bytes), NOT compressed int16 (8 bytes)
    size_t keySize;
    if (type == TrackType::FLOAT) keySize = sizeof(float);
    else if (type == TrackType::VEC3) keySize = sizeof(float) * 3;
    else keySize = sizeof(float) * 4; // C4Quaternion (float[4]) in vanilla
    if (disk.ofsKeys + disk.nKeys * keySize > data.size()) return;

    // Read per-sequence ranges
    std::vector<M2Range> ranges;
    if (disk.nRanges > 0 && disk.ofsRanges > 0 &&
        disk.nRanges < 4096 &&
        disk.ofsRanges + disk.nRanges * sizeof(M2Range) <= data.size()) {
        ranges = readArray<M2Range>(data, disk.ofsRanges, disk.nRanges);
    }

    // If no ranges, treat entire array as one sequence
    if (ranges.empty()) {
        ranges.push_back({0, disk.nTimestamps});
    }

    track.sequences.resize(ranges.size());

    for (size_t i = 0; i < ranges.size(); i++) {
        uint32_t start = ranges[i].start;
        uint32_t end = ranges[i].end;
        if (start >= end || start >= disk.nTimestamps) continue;
        end = std::min(end, disk.nTimestamps);

        // Copy timestamps for this sequence, normalized to start at 0
        // (vanilla stores absolute timestamps in the flat array, but the
        // renderer expects 0-relative times matching sequence duration)
        track.sequences[i].timestamps.assign(
            allTimestamps.begin() + start, allTimestamps.begin() + end);
        if (!track.sequences[i].timestamps.empty()) {
            uint32_t firstTime = track.sequences[i].timestamps[0];
            for (auto& ts : track.sequences[i].timestamps) {
                ts -= firstTime;
            }
        }

        // Copy key values for this sequence
        if (start >= disk.nKeys) continue;
        uint32_t keyEnd = std::min(end, disk.nKeys);
        uint32_t keyCount = keyEnd - start;

        if (type == TrackType::FLOAT) {
            auto allValues = readArray<float>(data, disk.ofsKeys, disk.nKeys);
            track.sequences[i].floatValues.assign(
                allValues.begin() + start, allValues.begin() + start + keyCount);
        } else if (type == TrackType::VEC3) {
            struct Vec3Disk { float x, y, z; };
            auto allValues = readArray<Vec3Disk>(data, disk.ofsKeys, disk.nKeys);
            track.sequences[i].vec3Values.reserve(keyCount);
            for (uint32_t k = start; k < start + keyCount; k++) {
                track.sequences[i].vec3Values.emplace_back(
                    allValues[k].x, allValues[k].y, allValues[k].z);
            }
        } else {
            // Vanilla: C4Quaternion — full float[4] per key (XYZW on disk)
            // NOT compressed int16 like WotLK
            struct C4Quaternion { float x, y, z, w; };
            auto allQ = readArray<C4Quaternion>(data, disk.ofsKeys, disk.nKeys);
            track.sequences[i].quatValues.reserve(keyCount);
            for (uint32_t k = start; k < start + keyCount; k++) {
                const auto& fq = allQ[k];
                // Disk order: XYZW, glm::quat constructor: (w, x, y, z)
                glm::quat q(fq.w, fq.x, fq.y, fq.z);
                float len = glm::length(q);
                if (len > 0.001f) q = q / len;
                else q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                track.sequences[i].quatValues.push_back(q);
            }
        }
    }
}

// Parse an FBlock (particle lifetime curve) from a 16-byte on-disk header.
// FBlocks are like M2Track but WITHOUT the interpolationType/globalSequence prefix.
void parseFBlock(const std::vector<uint8_t>& data, uint32_t offset,
                 M2FBlock& fb, int valueType) {
    // valueType: 0 = color (3 bytes RGB), 1 = alpha (uint16), 2 = scale (float pair)
    if (offset + sizeof(FBlockDisk) > data.size()) return;

    FBlockDisk disk = readValue<FBlockDisk>(data, offset);
    if (disk.nTimestamps == 0 || disk.nKeys == 0) return;
    // Sanity cap: particle FBlocks typically have 3 keyframes
    if (disk.nTimestamps > 1024 || disk.nKeys > 1024) return;

    // FBlock timestamps are uint16 (not sub-arrays), stored directly
    if (disk.ofsTimestamps + disk.nTimestamps * sizeof(uint16_t) > data.size()) return;

    auto rawTs = readArray<uint16_t>(data, disk.ofsTimestamps, disk.nTimestamps);
    uint16_t maxTs = 1;
    for (auto t : rawTs) { if (t > maxTs) maxTs = t; }
    fb.timestamps.reserve(rawTs.size());
    for (auto t : rawTs) {
        fb.timestamps.push_back(static_cast<float>(t) / static_cast<float>(maxTs));
    }

    uint32_t nKeys = disk.nKeys;
    uint32_t ofsKeys = disk.ofsKeys;

    if (valueType == 0) {
        // Color: 3 bytes per key.
        // WotLK particle FBlock color keys are stored as BGR in practice for many assets
        // (notably water/waterfall emitters). Decode to RGB explicitly.
        if (ofsKeys + nKeys * 3 > data.size()) return;
        fb.vec3Values.reserve(nKeys);
        for (uint32_t i = 0; i < nKeys; i++) {
            uint8_t b = data[ofsKeys + i * 3 + 0];
            uint8_t g = data[ofsKeys + i * 3 + 1];
            uint8_t r = data[ofsKeys + i * 3 + 2];
            fb.vec3Values.emplace_back(r / 255.0f, g / 255.0f, b / 255.0f);
        }
    } else if (valueType == 1) {
        // Alpha: uint16 per key
        if (ofsKeys + nKeys * sizeof(uint16_t) > data.size()) return;
        auto rawAlpha = readArray<uint16_t>(data, ofsKeys, nKeys);
        fb.floatValues.reserve(nKeys);
        for (auto a : rawAlpha) {
            fb.floatValues.push_back(static_cast<float>(a) / 32767.0f);
        }
    } else if (valueType == 2) {
        // Scale: float pair {x, y} per key, store x
        if (ofsKeys + nKeys * 8 > data.size()) return;
        fb.floatValues.reserve(nKeys);
        for (uint32_t i = 0; i < nKeys; i++) {
            float x = readValue<float>(data, ofsKeys + i * 8);
            fb.floatValues.push_back(x);
        }
    }
}

} // anonymous namespace

M2Model M2Loader::load(const std::vector<uint8_t>& m2Data) {
    M2Model model;

    // Read header with version-aware field parsing.
    // Vanilla M2 (version < 264) has 3 extra fields totaling +20 bytes:
    //   +8: playableAnimLookup M2Array (after animationLookup)
    //   +4: ofsViews (after nViews, making it a full M2Array)
    //   +8: unknown extra M2Array (after texReplace, before renderFlags)
    // Also: vanilla bones are 84 bytes (no boneNameCRC), sequences are 68 bytes.
    constexpr size_t COMMON_PREFIX_SIZE = 0x2C; // magic through ofsAnimationLookup

    if (m2Data.size() < COMMON_PREFIX_SIZE + 16) { // Need at least some fields after prefix
        core::Logger::getInstance().error("M2 data too small");
        return model;
    }

    M2Header header;
    std::memset(&header, 0, sizeof(header));
    // Read common prefix (magic through ofsAnimationLookup) — same for all versions
    std::memcpy(&header, m2Data.data(), COMMON_PREFIX_SIZE);

    // Verify magic
    if (std::strncmp(header.magic, "MD20", 4) != 0) {
        core::Logger::getInstance().error("Invalid M2 magic: expected MD20");
        return model;
    }

    uint32_t ofsViews = 0;

    if (header.version < 264) {
        // Vanilla M2: read remaining header fields using cursor, skipping extra fields
        size_t c = COMMON_PREFIX_SIZE;

        auto r32 = [&]() -> uint32_t {
            if (c + 4 > m2Data.size()) return 0;
            uint32_t v;
            std::memcpy(&v, m2Data.data() + c, 4);
            c += 4;
            return v;
        };

        // Skip playableAnimLookup M2Array (8 bytes)
        c += 8;

        // Bones through ofsVertices (same field order as WotLK, just shifted)
        header.nBones = r32();
        header.ofsBones = r32();
        header.nKeyBoneLookup = r32();
        header.ofsKeyBoneLookup = r32();
        header.nVertices = r32();
        header.ofsVertices = r32();

        // nViews + ofsViews (vanilla has both, WotLK has only nViews)
        header.nViews = r32();
        ofsViews = r32();

        // nColors through ofsTexReplace
        header.nColors = r32();
        header.ofsColors = r32();
        header.nTextures = r32();
        header.ofsTextures = r32();
        header.nTransparency = r32();
        header.ofsTransparency = r32();
        header.nUVAnimation = r32();
        header.ofsUVAnimation = r32();
        header.nTexReplace = r32();
        header.ofsTexReplace = r32();

        // Skip unknown extra M2Array (8 bytes)
        c += 8;

        // nRenderFlags through ofsUVAnimLookup
        header.nRenderFlags = r32();
        header.ofsRenderFlags = r32();
        header.nBoneLookupTable = r32();
        header.ofsBoneLookupTable = r32();
        header.nTexLookup = r32();
        header.ofsTexLookup = r32();
        header.nTexUnits = r32();
        header.ofsTexUnits = r32();
        header.nTransLookup = r32();
        header.ofsTransLookup = r32();
        header.nUVAnimLookup = r32();
        header.ofsUVAnimLookup = r32();

        // Float sections (vertexBox, vertexRadius, boundingBox, boundingRadius)
        if (c + 56 <= m2Data.size()) {
            std::memcpy(header.vertexBox, m2Data.data() + c, 24); c += 24;
            std::memcpy(&header.vertexRadius, m2Data.data() + c, 4); c += 4;
            std::memcpy(header.boundingBox, m2Data.data() + c, 24); c += 24;
            std::memcpy(&header.boundingRadius, m2Data.data() + c, 4); c += 4;
        } else { c += 56; }

        // Remaining M2Array pairs
        header.nBoundingTriangles = r32();
        header.ofsBoundingTriangles = r32();
        header.nBoundingVertices = r32();
        header.ofsBoundingVertices = r32();
        header.nBoundingNormals = r32();
        header.ofsBoundingNormals = r32();
        header.nAttachments = r32();
        header.ofsAttachments = r32();
        header.nAttachmentLookup = r32();
        header.ofsAttachmentLookup = r32();
        header.nEvents = r32();
        header.ofsEvents = r32();
        header.nLights = r32();
        header.ofsLights = r32();
        header.nCameras = r32();
        header.ofsCameras = r32();
        header.nCameraLookup = r32();
        header.ofsCameraLookup = r32();
        header.nRibbonEmitters = r32();
        header.ofsRibbonEmitters = r32();
        header.nParticleEmitters = r32();
        header.ofsParticleEmitters = r32();

        core::Logger::getInstance().info("Vanilla M2 (version ", header.version,
            "): nVerts=", header.nVertices, " nViews=", header.nViews,
            " ofsViews=", ofsViews, " nTex=", header.nTextures);
    } else {
        // WotLK: read remaining header with simple memcpy (no extra fields)
        size_t wotlkSize = sizeof(M2Header) - COMMON_PREFIX_SIZE;
        if (m2Data.size() < COMMON_PREFIX_SIZE + wotlkSize) {
            core::Logger::getInstance().error("M2 data too small for WotLK header");
            return model;
        }
        std::memcpy(reinterpret_cast<uint8_t*>(&header) + COMMON_PREFIX_SIZE,
                    m2Data.data() + COMMON_PREFIX_SIZE, wotlkSize);
    }

    core::Logger::getInstance().debug("Loading M2 model (version ", header.version, ")");

    // Read model name
    if (header.nameLength > 0 && header.nameOffset > 0) {
        model.name = readString(m2Data, header.nameOffset, header.nameLength);
    }

    model.version = header.version;
    model.globalFlags = header.globalFlags;

    // Bounding box
    model.boundMin = glm::vec3(header.boundingBox[0], header.boundingBox[1], header.boundingBox[2]);
    model.boundMax = glm::vec3(header.boundingBox[3], header.boundingBox[4], header.boundingBox[5]);
    model.boundRadius = header.boundingRadius;

    // Read vertices
    if (header.nVertices > 0 && header.ofsVertices > 0) {
        auto diskVerts = readArray<M2VertexDisk>(m2Data, header.ofsVertices, header.nVertices);
        model.vertices.reserve(diskVerts.size());

        for (const auto& dv : diskVerts) {
            M2Vertex v;
            v.position = glm::vec3(dv.pos[0], dv.pos[1], dv.pos[2]);
            std::memcpy(v.boneWeights, dv.boneWeights, 4);
            std::memcpy(v.boneIndices, dv.boneIndices, 4);
            v.normal = glm::vec3(dv.normal[0], dv.normal[1], dv.normal[2]);
            v.texCoords[0] = glm::vec2(dv.texCoords[0][0], dv.texCoords[0][1]);
            v.texCoords[1] = glm::vec2(dv.texCoords[1][0], dv.texCoords[1][1]);
            model.vertices.push_back(v);
        }

        core::Logger::getInstance().debug("  Vertices: ", model.vertices.size());
    }

    // Read animation sequences (needed before bones to know sequence count)
    if (header.nAnimations > 0 && header.ofsAnimations > 0) {
        model.sequences.reserve(header.nAnimations);

        if (header.version < 264) {
            // Vanilla: 68-byte sequence struct (has startTimestamp + endTimestamp)
            auto diskSeqs = readArray<M2SequenceDiskVanilla>(m2Data, header.ofsAnimations, header.nAnimations);
            for (const auto& ds : diskSeqs) {
                M2Sequence seq;
                seq.id = ds.id;
                seq.variationIndex = ds.variationIndex;
                seq.duration = (ds.endTimestamp > ds.startTimestamp)
                    ? (ds.endTimestamp - ds.startTimestamp) : ds.endTimestamp;
                seq.movingSpeed = ds.movingSpeed;
                seq.flags = ds.flags;
                seq.frequency = ds.frequency;
                seq.replayMin = ds.replayMin;
                seq.replayMax = ds.replayMax;
                seq.blendTime = ds.blendTime;
                seq.boundMin = glm::vec3(ds.bounds[0], ds.bounds[1], ds.bounds[2]);
                seq.boundMax = glm::vec3(ds.bounds[3], ds.bounds[4], ds.bounds[5]);
                seq.boundRadius = ds.boundRadius;
                seq.nextAnimation = ds.nextAnimation;
                seq.aliasNext = ds.aliasNext;
                model.sequences.push_back(seq);
            }
        } else {
            // WotLK: 64-byte sequence struct
            auto diskSeqs = readArray<M2SequenceDisk>(m2Data, header.ofsAnimations, header.nAnimations);
            for (const auto& ds : diskSeqs) {
                M2Sequence seq;
                seq.id = ds.id;
                seq.variationIndex = ds.variationIndex;
                seq.duration = ds.duration;
                seq.movingSpeed = ds.movingSpeed;
                seq.flags = ds.flags;
                seq.frequency = ds.frequency;
                seq.replayMin = ds.replayMin;
                seq.replayMax = ds.replayMax;
                seq.blendTime = ds.blendTime;
                seq.boundMin = glm::vec3(ds.bounds[0], ds.bounds[1], ds.bounds[2]);
                seq.boundMax = glm::vec3(ds.bounds[3], ds.bounds[4], ds.bounds[5]);
                seq.boundRadius = ds.boundRadius;
                seq.nextAnimation = ds.nextAnimation;
                seq.aliasNext = ds.aliasNext;
                model.sequences.push_back(seq);
            }
        }

        core::Logger::getInstance().debug("  Animation sequences: ", model.sequences.size());
    }

    // Read global sequence durations (used by environmental animations: smoke, fire, etc.)
    if (header.nGlobalSequences > 0 && header.ofsGlobalSequences > 0) {
        model.globalSequenceDurations = readArray<uint32_t>(m2Data,
            header.ofsGlobalSequences, header.nGlobalSequences);
        core::Logger::getInstance().debug("  Global sequences: ", model.globalSequenceDurations.size());
    }

    // Read bones with full animation track data
    if (header.nBones > 0 && header.ofsBones > 0) {
        size_t boneStructSize = (header.version < 264) ? sizeof(M2BoneDiskVanilla) : sizeof(M2BoneDisk);
        uint64_t expectedBoneSize = static_cast<uint64_t>(header.nBones) * boneStructSize;
        if (header.ofsBones + expectedBoneSize > m2Data.size()) {
            core::Logger::getInstance().warning("M2 bone data extends beyond file, loading with fallback");
        }

        model.bones.reserve(header.nBones);
        int bonesWithKeyframes = 0;

        // Build per-sequence flags to skip external-data sequences during M2 parse
        std::vector<uint32_t> seqFlags;
        seqFlags.reserve(model.sequences.size());
        for (const auto& seq : model.sequences) {
            seqFlags.push_back(seq.flags);
        }

        for (uint32_t boneIdx = 0; boneIdx < header.nBones; boneIdx++) {
            uint32_t boneOffset = header.ofsBones + boneIdx * boneStructSize;
            if (boneOffset + boneStructSize > m2Data.size()) {
                // Fallback: create identity bone
                M2Bone bone;
                bone.keyBoneId = -1;
                bone.flags = 0;
                bone.parentBone = -1;
                bone.submeshId = 0;
                bone.pivot = glm::vec3(0.0f);
                model.bones.push_back(bone);
                continue;
            }

            M2Bone bone;
            M2TrackDisk translation, rotation, scale;

            if (header.version < 264) {
                // Vanilla: 108-byte bone (no boneNameCRC, 28-byte tracks with ranges)
                M2BoneDiskVanilla db = readValue<M2BoneDiskVanilla>(m2Data, boneOffset);
                bone.keyBoneId = db.keyBoneId;
                bone.flags = db.flags;
                bone.parentBone = db.parentBone;
                bone.submeshId = db.submeshId;
                bone.pivot = glm::vec3(db.pivot[0], db.pivot[1], db.pivot[2]);
                // Convert vanilla 28-byte tracks to WotLK 20-byte format (drop ranges)
                translation = {db.translation.interpolationType, db.translation.globalSequence,
                               db.translation.nTimestamps, db.translation.ofsTimestamps,
                               db.translation.nKeys, db.translation.ofsKeys};
                rotation = {db.rotation.interpolationType, db.rotation.globalSequence,
                            db.rotation.nTimestamps, db.rotation.ofsTimestamps,
                            db.rotation.nKeys, db.rotation.ofsKeys};
                scale = {db.scale.interpolationType, db.scale.globalSequence,
                         db.scale.nTimestamps, db.scale.ofsTimestamps,
                         db.scale.nKeys, db.scale.ofsKeys};
            } else {
                // WotLK: 88-byte bone
                M2BoneDisk db = readValue<M2BoneDisk>(m2Data, boneOffset);
                bone.keyBoneId = db.keyBoneId;
                bone.flags = db.flags;
                bone.parentBone = db.parentBone;
                bone.submeshId = db.submeshId;
                bone.pivot = glm::vec3(db.pivot[0], db.pivot[1], db.pivot[2]);
                translation = db.translation;
                rotation = db.rotation;
                scale = db.scale;
            }

            // Parse animation tracks
            if (header.version >= 264) {
                parseAnimTrack(m2Data, translation, bone.translation, TrackType::VEC3, seqFlags);
                parseAnimTrack(m2Data, rotation, bone.rotation, TrackType::QUAT_COMPRESSED, seqFlags);
                parseAnimTrack(m2Data, scale, bone.scale, TrackType::VEC3, seqFlags);
            } else {
                // Vanilla: flat array format with per-sequence ranges + Quaternion16
                M2BoneDiskVanilla dbv = readValue<M2BoneDiskVanilla>(m2Data, boneOffset);
                parseAnimTrackVanilla(m2Data, dbv.translation, bone.translation, TrackType::VEC3);
                parseAnimTrackVanilla(m2Data, dbv.rotation, bone.rotation, TrackType::QUAT_COMPRESSED);
                parseAnimTrackVanilla(m2Data, dbv.scale, bone.scale, TrackType::VEC3);
            }

            if (bone.translation.hasData() || bone.rotation.hasData() || bone.scale.hasData()) {
                bonesWithKeyframes++;
            }

            model.bones.push_back(bone);
        }

        core::Logger::getInstance().debug("  Bones: ", model.bones.size(),
            " (", bonesWithKeyframes, " with keyframes)");
    }

    // Read textures
    if (header.nTextures > 0 && header.ofsTextures > 0) {
        auto diskTextures = readArray<M2TextureDisk>(m2Data, header.ofsTextures, header.nTextures);
        model.textures.reserve(diskTextures.size());

        for (const auto& dt : diskTextures) {
            M2Texture tex;
            tex.type = dt.type;
            tex.flags = dt.flags;

            if (dt.nameLength > 0 && dt.nameOffset > 0) {
                tex.filename = readString(m2Data, dt.nameOffset, dt.nameLength);
            }

            model.textures.push_back(tex);
        }

        core::Logger::getInstance().debug("  Textures: ", model.textures.size());
    }

    // Read texture lookup
    if (header.nTexLookup > 0 && header.ofsTexLookup > 0) {
        model.textureLookup = readArray<uint16_t>(m2Data, header.ofsTexLookup, header.nTexLookup);
    }

    // Read render flags / materials (blend modes)
    if (header.nRenderFlags > 0 && header.ofsRenderFlags > 0) {
        struct M2MaterialDisk { uint16_t flags; uint16_t blendMode; };
        auto diskMats = readArray<M2MaterialDisk>(m2Data, header.ofsRenderFlags, header.nRenderFlags);
        model.materials.reserve(diskMats.size());
        for (const auto& dm : diskMats) {
            M2Material mat;
            mat.flags = dm.flags;
            mat.blendMode = dm.blendMode;
            model.materials.push_back(mat);
        }
        core::Logger::getInstance().debug("  Materials: ", model.materials.size());
    }

    // Read texture transforms (UV animation data)
    if (header.nUVAnimation > 0 && header.ofsUVAnimation > 0) {
        // Build per-sequence flags for skipping external .anim data
        std::vector<uint32_t> seqFlags;
        seqFlags.reserve(model.sequences.size());
        for (const auto& seq : model.sequences) {
            seqFlags.push_back(seq.flags);
        }

        size_t uvStructSize = (header.version >= 264)
            ? sizeof(M2TextureTransformDisk)
            : sizeof(M2TextureTransformDiskVanilla);

        model.textureTransforms.reserve(header.nUVAnimation);
        for (uint32_t i = 0; i < header.nUVAnimation; i++) {
            uint32_t ofs = header.ofsUVAnimation + i * uvStructSize;
            if (ofs + uvStructSize > m2Data.size()) break;

            M2TextureTransform tt;
            if (header.version >= 264) {
                M2TextureTransformDisk dt = readValue<M2TextureTransformDisk>(m2Data, ofs);
                parseAnimTrack(m2Data, dt.translation, tt.translation, TrackType::VEC3, seqFlags);
                parseAnimTrack(m2Data, dt.rotation, tt.rotation, TrackType::QUAT_COMPRESSED, seqFlags);
                parseAnimTrack(m2Data, dt.scaling, tt.scale, TrackType::VEC3, seqFlags);
            } else {
                M2TextureTransformDiskVanilla dt = readValue<M2TextureTransformDiskVanilla>(m2Data, ofs);
                parseAnimTrackVanilla(m2Data, dt.translation, tt.translation, TrackType::VEC3);
                parseAnimTrackVanilla(m2Data, dt.rotation, tt.rotation, TrackType::QUAT_COMPRESSED);
                parseAnimTrackVanilla(m2Data, dt.scaling, tt.scale, TrackType::VEC3);
            }
            model.textureTransforms.push_back(std::move(tt));
        }
        core::Logger::getInstance().debug("  Texture transforms: ", model.textureTransforms.size());
    }

    // Read texture transform lookup (nTransLookup)
    if (header.nTransLookup > 0 && header.ofsTransLookup > 0) {
        model.textureTransformLookup = readArray<uint16_t>(m2Data, header.ofsTransLookup, header.nTransLookup);
    }

    // Read attachment points (vanilla uses 48-byte struct, WotLK uses 40-byte)
    if (header.nAttachments > 0 && header.ofsAttachments > 0) {
        model.attachments.reserve(header.nAttachments);
        if (header.version < 264) {
            auto diskAttachments = readArray<M2AttachmentDiskVanilla>(m2Data, header.ofsAttachments, header.nAttachments);
            for (const auto& da : diskAttachments) {
                M2Attachment att;
                att.id = da.id;
                att.bone = da.bone;
                att.position = glm::vec3(da.position[0], da.position[1], da.position[2]);
                model.attachments.push_back(att);
            }
        } else {
            auto diskAttachments = readArray<M2AttachmentDisk>(m2Data, header.ofsAttachments, header.nAttachments);
            for (const auto& da : diskAttachments) {
                M2Attachment att;
                att.id = da.id;
                att.bone = da.bone;
                att.position = glm::vec3(da.position[0], da.position[1], da.position[2]);
                model.attachments.push_back(att);
            }
        }
        core::Logger::getInstance().debug("  Attachments: ", model.attachments.size());
    }

    // Read attachment lookup
    if (header.nAttachmentLookup > 0 && header.ofsAttachmentLookup > 0) {
        model.attachmentLookup = readArray<uint16_t>(m2Data, header.ofsAttachmentLookup, header.nAttachmentLookup);
    }

    // Parse particle emitters (WotLK M2ParticleOld: 0x1DC = 476 bytes per emitter)
    // Skip for vanilla — emitter struct size differs
    static constexpr uint32_t EMITTER_STRUCT_SIZE = 0x1DC;
    if (header.version >= 264 &&
        header.nParticleEmitters > 0 && header.ofsParticleEmitters > 0 &&
        header.nParticleEmitters < 256 &&
        static_cast<size_t>(header.ofsParticleEmitters) +
            static_cast<size_t>(header.nParticleEmitters) * EMITTER_STRUCT_SIZE <= m2Data.size()) {

        // Build sequence flags for parseAnimTrack
        std::vector<uint32_t> emSeqFlags;
        emSeqFlags.reserve(model.sequences.size());
        for (const auto& seq : model.sequences) {
            emSeqFlags.push_back(seq.flags);
        }

        for (uint32_t ei = 0; ei < header.nParticleEmitters; ei++) {
            uint32_t base = header.ofsParticleEmitters + ei * EMITTER_STRUCT_SIZE;

            M2ParticleEmitter em;
            em.particleId = readValue<int32_t>(m2Data, base + 0x00);
            em.flags      = readValue<uint32_t>(m2Data, base + 0x04);
            em.position.x = readValue<float>(m2Data, base + 0x08);
            em.position.y = readValue<float>(m2Data, base + 0x0C);
            em.position.z = readValue<float>(m2Data, base + 0x10);
            em.bone       = readValue<uint16_t>(m2Data, base + 0x14);
            em.texture    = readValue<uint16_t>(m2Data, base + 0x16);
            em.blendingType = readValue<uint8_t>(m2Data, base + 0x28);
            em.emitterType  = readValue<uint8_t>(m2Data, base + 0x29);
            em.textureTileRotation = readValue<int16_t>(m2Data, base + 0x2E);
            em.textureRows = readValue<uint16_t>(m2Data, base + 0x30);
            em.textureCols = readValue<uint16_t>(m2Data, base + 0x32);
            if (em.textureRows == 0) em.textureRows = 1;
            if (em.textureCols == 0) em.textureCols = 1;

            // Parse animated tracks (M2TrackDisk at known offsets)
            auto parseTrack = [&](uint32_t off, M2AnimationTrack& track) {
                if (base + off + sizeof(M2TrackDisk) <= m2Data.size()) {
                    M2TrackDisk disk = readValue<M2TrackDisk>(m2Data, base + off);
                    parseAnimTrack(m2Data, disk, track, TrackType::FLOAT, emSeqFlags);
                }
            };
            parseTrack(0x34, em.emissionSpeed);
            parseTrack(0x48, em.speedVariation);
            parseTrack(0x5C, em.verticalRange);
            parseTrack(0x70, em.horizontalRange);
            parseTrack(0x84, em.gravity);
            parseTrack(0x98, em.lifespan);
            parseTrack(0xB0, em.emissionRate);
            parseTrack(0xC8, em.emissionAreaLength);
            parseTrack(0xDC, em.emissionAreaWidth);
            parseTrack(0xF0, em.deceleration);

            // Parse FBlocks (color, alpha, scale) — FBlocks are 16 bytes each
            parseFBlock(m2Data, base + 0x104, em.particleColor, 0);
            parseFBlock(m2Data, base + 0x114, em.particleAlpha, 1);
            parseFBlock(m2Data, base + 0x124, em.particleScale, 2);

            model.particleEmitters.push_back(std::move(em));
        }
        core::Logger::getInstance().debug("  Particle emitters: ", model.particleEmitters.size());
    }

    // Read collision mesh (bounding triangles/vertices/normals)
    if (header.nBoundingVertices > 0 && header.ofsBoundingVertices > 0) {
        struct Vec3Disk { float x, y, z; };
        auto diskVerts = readArray<Vec3Disk>(m2Data, header.ofsBoundingVertices, header.nBoundingVertices);
        model.collisionVertices.reserve(diskVerts.size());
        for (const auto& v : diskVerts) {
            model.collisionVertices.emplace_back(v.x, v.y, v.z);
        }
    }
    if (header.nBoundingTriangles > 0 && header.ofsBoundingTriangles > 0) {
        model.collisionIndices = readArray<uint16_t>(m2Data, header.ofsBoundingTriangles, header.nBoundingTriangles);
    }
    if (header.nBoundingNormals > 0 && header.ofsBoundingNormals > 0) {
        struct Vec4Disk { float x, y, z, w; };
        auto diskNormals = readArray<Vec4Disk>(m2Data, header.ofsBoundingNormals, header.nBoundingNormals);
        model.collisionNormals.reserve(diskNormals.size());
        for (const auto& n : diskNormals) {
            model.collisionNormals.emplace_back(n.x, n.y, n.z, n.w);
        }
    }
    if (!model.collisionVertices.empty()) {
        core::Logger::getInstance().debug("  Collision mesh: ", model.collisionVertices.size(),
            " verts, ", model.collisionIndices.size() / 3, " tris, ",
            model.collisionNormals.size(), " normals");
    }

    // Load embedded skin for vanilla M2 (version < 264)
    // Vanilla M2 files contain skin profiles directly, no external .skin files.
    if (header.version < 264 && header.nViews > 0 && ofsViews > 0 &&
        ofsViews + sizeof(M2SkinProfileEmbedded) <= m2Data.size()) {

        M2SkinProfileEmbedded skinProfile;
        std::memcpy(&skinProfile, m2Data.data() + ofsViews, sizeof(skinProfile));

        // Read vertex lookup table (maps skin-local indices to global vertex indices)
        std::vector<uint16_t> vertexLookup;
        if (skinProfile.nIndices > 0 && skinProfile.ofsIndices > 0) {
            vertexLookup = readArray<uint16_t>(m2Data, skinProfile.ofsIndices, skinProfile.nIndices);
        }

        // Read triangle indices (indices into the vertex lookup table)
        std::vector<uint16_t> triangles;
        if (skinProfile.nTriangles > 0 && skinProfile.ofsTriangles > 0) {
            triangles = readArray<uint16_t>(m2Data, skinProfile.ofsTriangles, skinProfile.nTriangles);
        }

        // Resolve two-level indirection: triangle index -> lookup table -> global vertex
        model.indices.clear();
        model.indices.reserve(triangles.size());
        for (uint16_t triIdx : triangles) {
            if (triIdx < vertexLookup.size()) {
                uint16_t globalIdx = vertexLookup[triIdx];
                if (globalIdx < model.vertices.size()) {
                    model.indices.push_back(globalIdx);
                } else {
                    model.indices.push_back(0);
                }
            } else {
                model.indices.push_back(0);
            }
        }

        // Read submeshes (vanilla: 32 bytes each, no sortCenter/sortRadius)
        std::vector<M2SkinSubmesh> submeshes;
        if (skinProfile.nSubmeshes > 0 && skinProfile.ofsSubmeshes > 0) {
            auto vanillaSubmeshes = readArray<M2SkinSubmeshVanilla>(m2Data,
                skinProfile.ofsSubmeshes, skinProfile.nSubmeshes);
            submeshes.reserve(vanillaSubmeshes.size());
            for (const auto& vs : vanillaSubmeshes) {
                M2SkinSubmesh sm;
                sm.id = vs.id;
                sm.level = vs.level;
                sm.vertexStart = vs.vertexStart;
                sm.vertexCount = vs.vertexCount;
                sm.indexStart = vs.indexStart;
                sm.indexCount = vs.indexCount;
                sm.boneCount = vs.boneCount;
                sm.boneStart = vs.boneStart;
                sm.boneInfluences = vs.boneInfluences;
                sm.centerBoneIndex = vs.centerBoneIndex;
                std::memcpy(sm.centerPosition, vs.centerPosition, 12);
                std::memset(sm.sortCenterPosition, 0, 12);
                sm.sortRadius = 0;
                submeshes.push_back(sm);
            }
        }

        // Read batches
        if (skinProfile.nBatches > 0 && skinProfile.ofsBatches > 0) {
            auto diskBatches = readArray<M2BatchDisk>(m2Data,
                skinProfile.ofsBatches, skinProfile.nBatches);
            model.batches.clear();
            model.batches.reserve(diskBatches.size());

            for (size_t i = 0; i < diskBatches.size(); i++) {
                const auto& db = diskBatches[i];
                M2Batch batch;
                batch.flags = db.flags;
                batch.priorityPlane = db.priorityPlane;
                batch.shader = db.shader;
                batch.skinSectionIndex = db.skinSectionIndex;
                batch.colorIndex = db.colorIndex;
                batch.materialIndex = db.materialIndex;
                batch.materialLayer = db.materialLayer;
                batch.textureCount = db.textureCount;
                batch.textureIndex = db.textureComboIndex;
                batch.textureUnit = db.textureCoordIndex;
                batch.transparencyIndex = db.textureWeightIndex;
                batch.textureAnimIndex = db.textureTransformIndex;

                if (db.skinSectionIndex < submeshes.size()) {
                    const auto& sm = submeshes[db.skinSectionIndex];
                    batch.indexStart = sm.indexStart;
                    batch.indexCount = sm.indexCount;
                    batch.vertexStart = sm.vertexStart;
                    batch.vertexCount = sm.vertexCount;
                    batch.submeshId = sm.id;
                    batch.submeshLevel = sm.level;
                } else {
                    batch.indexStart = 0;
                    batch.indexCount = model.indices.size();
                    batch.vertexStart = 0;
                    batch.vertexCount = model.vertices.size();
                }

                model.batches.push_back(batch);
            }
        }

        core::Logger::getInstance().info("Vanilla M2: embedded skin loaded — ",
            model.indices.size(), " indices, ", model.batches.size(), " batches");
    }

    static int m2LoadLogBudget = 200;
    if (m2LoadLogBudget-- > 0) {
        core::Logger::getInstance().debug("M2 model loaded: ", model.name);
    }
    return model;
}

bool M2Loader::loadSkin(const std::vector<uint8_t>& skinData, M2Model& model) {
    if (skinData.size() < sizeof(M2SkinHeader)) {
        core::Logger::getInstance().error("Skin data too small");
        return false;
    }

    // Read skin header
    M2SkinHeader header;
    std::memcpy(&header, skinData.data(), sizeof(M2SkinHeader));

    // Verify magic
    if (std::strncmp(header.magic, "SKIN", 4) != 0) {
        core::Logger::getInstance().error("Invalid skin magic: expected SKIN");
        return false;
    }

    core::Logger::getInstance().debug("Loading M2 skin file");

    // Read vertex lookup table (maps skin-local indices to global vertex indices)
    std::vector<uint16_t> vertexLookup;
    if (header.nIndices > 0 && header.ofsIndices > 0) {
        vertexLookup = readArray<uint16_t>(skinData, header.ofsIndices, header.nIndices);
    }

    // Read triangle indices (indices into the vertex lookup table)
    std::vector<uint16_t> triangles;
    if (header.nTriangles > 0 && header.ofsTriangles > 0) {
        triangles = readArray<uint16_t>(skinData, header.ofsTriangles, header.nTriangles);
    }

    // Resolve two-level indirection: triangle index -> lookup table -> global vertex
    model.indices.clear();
    model.indices.reserve(triangles.size());
    uint32_t outOfBounds = 0;
    for (uint16_t triIdx : triangles) {
        if (triIdx < vertexLookup.size()) {
            uint16_t globalIdx = vertexLookup[triIdx];
            if (globalIdx < model.vertices.size()) {
                model.indices.push_back(globalIdx);
            } else {
                model.indices.push_back(0);
                outOfBounds++;
            }
        } else {
            model.indices.push_back(0);
            outOfBounds++;
        }
    }
    core::Logger::getInstance().debug("  Resolved ", model.indices.size(), " final indices");
    if (outOfBounds > 0) {
        core::Logger::getInstance().warning("  ", outOfBounds, " out-of-bounds indices clamped to 0");
    }

    // Read submeshes (proper vertex/index ranges)
    std::vector<M2SkinSubmesh> submeshes;
    if (header.nSubmeshes > 0 && header.ofsSubmeshes > 0) {
        submeshes = readArray<M2SkinSubmesh>(skinData, header.ofsSubmeshes, header.nSubmeshes);
        core::Logger::getInstance().debug("  Submeshes: ", submeshes.size());
        for (size_t i = 0; i < submeshes.size(); i++) {
            const auto& sm = submeshes[i];
        }
    }

    // Read batches with proper submesh references
    if (header.nBatches > 0 && header.ofsBatches > 0) {
        auto diskBatches = readArray<M2BatchDisk>(skinData, header.ofsBatches, header.nBatches);
        model.batches.clear();
        model.batches.reserve(diskBatches.size());

        for (size_t i = 0; i < diskBatches.size(); i++) {
            const auto& db = diskBatches[i];
            M2Batch batch;

            batch.flags = db.flags;
            batch.priorityPlane = db.priorityPlane;
            batch.shader = db.shader;
            batch.skinSectionIndex = db.skinSectionIndex;
            batch.colorIndex = db.colorIndex;
            batch.materialIndex = db.materialIndex;
            batch.materialLayer = db.materialLayer;
            batch.textureCount = db.textureCount;
            batch.textureIndex = db.textureComboIndex;
            batch.textureUnit = db.textureCoordIndex;
            batch.transparencyIndex = db.textureWeightIndex;
            batch.textureAnimIndex = db.textureTransformIndex;

            // Look up proper vertex/index ranges from submesh
            if (db.skinSectionIndex < submeshes.size()) {
                const auto& sm = submeshes[db.skinSectionIndex];
                batch.indexStart = sm.indexStart;
                batch.indexCount = sm.indexCount;
                batch.vertexStart = sm.vertexStart;
                batch.vertexCount = sm.vertexCount;
                batch.submeshId = sm.id;
                batch.submeshLevel = sm.level;
            } else {
                // Fallback: render entire model as one batch
                batch.indexStart = 0;
                batch.indexCount = model.indices.size();
                batch.vertexStart = 0;
                batch.vertexCount = model.vertices.size();
            }

            model.batches.push_back(batch);
        }

        core::Logger::getInstance().debug("  Batches: ", model.batches.size());
    }

    return true;
}

void M2Loader::loadAnimFile(const std::vector<uint8_t>& m2Data,
                            const std::vector<uint8_t>& animData,
                            uint32_t sequenceIndex,
                            M2Model& model) {
    if (m2Data.size() < sizeof(M2Header) || animData.empty()) return;

    M2Header header;
    std::memcpy(&header, m2Data.data(), sizeof(M2Header));

    if (header.nBones == 0 || header.ofsBones == 0) return;
    if (sequenceIndex >= model.sequences.size()) return;

    int patchedTracks = 0;

    for (uint32_t boneIdx = 0; boneIdx < header.nBones && boneIdx < model.bones.size(); boneIdx++) {
        uint32_t boneOffset = header.ofsBones + boneIdx * sizeof(M2BoneDisk);
        if (boneOffset + sizeof(M2BoneDisk) > m2Data.size()) continue;

        M2BoneDisk db = readValue<M2BoneDisk>(m2Data, boneOffset);
        auto& bone = model.bones[boneIdx];

        // Helper to patch one track for this sequence index
        auto patchTrack = [&](const M2TrackDisk& disk, M2AnimationTrack& track, TrackType type) {
            if (disk.nTimestamps == 0 || disk.nKeys == 0) return;
            if (sequenceIndex >= disk.nTimestamps) return;

            // Ensure track.sequences is large enough
            if (track.sequences.size() <= sequenceIndex) {
                track.sequences.resize(sequenceIndex + 1);
            }

            auto& seqKeys = track.sequences[sequenceIndex];

            // Already has data (loaded from main M2 file)
            if (!seqKeys.timestamps.empty()) return;

            // Read sub-array header for this sequence from the M2 file
            uint32_t tsHeaderOfs = disk.ofsTimestamps + sequenceIndex * 8;
            uint32_t keyHeaderOfs = disk.ofsKeys + sequenceIndex * 8;
            if (tsHeaderOfs + 8 > m2Data.size() || keyHeaderOfs + 8 > m2Data.size()) return;

            uint32_t tsCount = readValue<uint32_t>(m2Data, tsHeaderOfs);
            uint32_t tsOffset = readValue<uint32_t>(m2Data, tsHeaderOfs + 4);
            uint32_t keyCount = readValue<uint32_t>(m2Data, keyHeaderOfs);
            uint32_t keyOffset = readValue<uint32_t>(m2Data, keyHeaderOfs + 4);

            if (tsCount == 0 || keyCount == 0) return;

            // These offsets point into the .anim file data
            if (tsOffset + tsCount * sizeof(uint32_t) > animData.size()) return;

            size_t keyElementSize = (type == TrackType::VEC3) ? sizeof(float) * 3 : sizeof(int16_t) * 4;
            if (keyOffset + keyCount * keyElementSize > animData.size()) return;

            // Read timestamps from .anim data
            auto timestamps = readArray<uint32_t>(animData, tsOffset, tsCount);
            seqKeys.timestamps = std::move(timestamps);

            // Read key values from .anim data
            if (type == TrackType::VEC3) {
                struct Vec3Disk { float x, y, z; };
                auto values = readArray<Vec3Disk>(animData, keyOffset, keyCount);
                seqKeys.vec3Values.reserve(values.size());
                for (const auto& v : values) {
                    seqKeys.vec3Values.emplace_back(v.x, v.y, v.z);
                }
            } else {
                auto compressed = readArray<CompressedQuat>(animData, keyOffset, keyCount);
                seqKeys.quatValues.reserve(compressed.size());
                for (const auto& cq : compressed) {
                    float fx = (cq.x < 0) ? (cq.x + 32768) / 32767.0f : (cq.x - 32767) / 32767.0f;
                    float fy = (cq.y < 0) ? (cq.y + 32768) / 32767.0f : (cq.y - 32767) / 32767.0f;
                    float fz = (cq.z < 0) ? (cq.z + 32768) / 32767.0f : (cq.z - 32767) / 32767.0f;
                    float fw = (cq.w < 0) ? (cq.w + 32768) / 32767.0f : (cq.w - 32767) / 32767.0f;
                    glm::quat q(fw, fx, fy, fz);
                    float len = glm::length(q);
                    if (len > 0.001f) {
                        q = q / len;
                    } else {
                        q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                    }
                    seqKeys.quatValues.push_back(q);
                }
            }
            patchedTracks++;
        };

        patchTrack(db.translation, bone.translation, TrackType::VEC3);
        patchTrack(db.rotation, bone.rotation, TrackType::QUAT_COMPRESSED);
        patchTrack(db.scale, bone.scale, TrackType::VEC3);
    }

    core::Logger::getInstance().debug("Loaded .anim for sequence ", sequenceIndex,
        " (id=", model.sequences[sequenceIndex].id, "): patched ", patchedTracks, " bone tracks");
}

} // namespace pipeline
} // namespace wowee
