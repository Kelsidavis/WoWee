/**
 * M2 Model Loader - Binary parser for WoW's M2 model format (WotLK 3.3.5a)
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
 *  - Skin batch struct is 24 bytes on disk - the geosetIndex field at offset 10
 *    is easily missed, causing a 2-byte alignment shift on all subsequent fields.
 *
 * Reference: https://wowdev.wiki/M2
 */
#include "pipeline/m2_loader.hpp"
#include "pipeline/m2_color_track.hpp"
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
static_assert(sizeof(M2Header) == 304,
              "M2Header is read straight from the file: 304 bytes, no padding");

// M2 vertex structure (on-disk format)
struct M2VertexDisk {
    float pos[3];
    uint8_t boneWeights[4];
    uint8_t boneIndices[4];
    float normal[3];
    float texCoords[2][2];
};
static_assert(sizeof(M2VertexDisk) == 48,
              "M2VertexDisk is read straight from the file: 48 bytes, no padding");

// M2 animation track header (on-disk, 20 bytes)
struct M2TrackDisk {
    uint16_t interpolationType;
    int16_t globalSequence;
    uint32_t nTimestamps;
    uint32_t ofsTimestamps;
    uint32_t nKeys;
    uint32_t ofsKeys;
};
static_assert(sizeof(M2TrackDisk) == 20,
              "M2TrackDisk is read straight from the file: 20 bytes, no padding");

// FBlock header (on-disk, 16 bytes) - particle lifetime curves
// Like M2TrackDisk but WITHOUT interpolationType/globalSequence prefix
struct FBlockDisk {
    uint32_t nTimestamps;
    uint32_t ofsTimestamps;
    uint32_t nKeys;
    uint32_t ofsKeys;
};
static_assert(sizeof(FBlockDisk) == 16,
              "FBlockDisk is read straight from the file: 16 bytes, no padding");

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
};
static_assert(sizeof(M2BoneDisk) == 88,
              "M2BoneDisk is read straight from the file: 88 bytes, no padding");                              // Total: 88

// Vanilla M2 animation track header (on-disk, 28 bytes - has extra ranges M2Array)
struct M2TrackDiskVanilla {
    uint16_t interpolationType; // 2
    int16_t globalSequence;     // 2
    uint32_t nRanges;           // 4 - extra in vanilla (animation sequence ranges)
    uint32_t ofsRanges;         // 4 - extra in vanilla
    uint32_t nTimestamps;       // 4
    uint32_t ofsTimestamps;     // 4
    uint32_t nKeys;             // 4
    uint32_t ofsKeys;           // 4
};
static_assert(sizeof(M2TrackDiskVanilla) == 28,
              "M2TrackDiskVanilla is read straight from the file: 28 bytes, no padding");                              // Total: 28

// Vanilla M2 bone structure (on-disk, 108 bytes - no boneNameCRC, 28-byte tracks)
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

// TBC M2 bone structure (on-disk, 112 bytes). Like vanilla (28-byte tracks WITH
// interpolation ranges) but with the boneNameCRC field that BC introduced (and
// WotLK kept). TBC rotations are compressed int16[4] quaternions, NOT vanilla's
// float[4] - see parseAnimTrackVanilla's compressedQuat flag.
struct M2BoneDiskTBC {
    int32_t keyBoneId;              // 4
    uint32_t flags;                 // 4
    int16_t parentBone;             // 2
    uint16_t submeshId;             // 2
    uint32_t boneNameCRC;           // 4 - added in BC
    M2TrackDiskVanilla translation; // 28
    M2TrackDiskVanilla rotation;    // 28
    M2TrackDiskVanilla scale;       // 28
    float pivot[3];                 // 12
};                                  // Total: 112
static_assert(sizeof(M2BoneDiskVanilla) == 108, "M2BoneDiskVanilla must be 108 bytes");
static_assert(sizeof(M2BoneDiskTBC) == 112, "M2BoneDiskTBC must be 112 bytes");

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
static_assert(sizeof(M2SequenceDisk) == 64,
              "M2SequenceDisk is read straight from the file: 64 bytes, no padding");

// Vanilla M2 animation sequence (68 bytes - has start_timestamp before duration)
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
static_assert(sizeof(M2SequenceDiskVanilla) == 68,
              "M2SequenceDiskVanilla is read straight from the file: 68 bytes, no padding");

// M2 texture definition
struct M2TextureDisk {
    uint32_t type;
    uint32_t flags;
    uint32_t nameLength;
    uint32_t nameOffset;
};
static_assert(sizeof(M2TextureDisk) == 16,
              "M2TextureDisk is read straight from the file: 16 bytes, no padding");

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
static_assert(sizeof(M2SkinHeader) == 48,
              "M2SkinHeader is read straight from the file: 48 bytes, no padding");

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

// Vanilla M2 skin submesh (32 bytes, version < 264 - no sortCenter/sortRadius)
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
static_assert(sizeof(M2SkinSubmesh) == 48, "M2SkinSubmesh must be 48 bytes");
static_assert(sizeof(M2SkinSubmeshVanilla) == 32, "M2SkinSubmeshVanilla must be 32 bytes");

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
static_assert(sizeof(M2SkinProfileEmbedded) == 44,
              "M2SkinProfileEmbedded is read straight from the file: 44 bytes, no padding");

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
static_assert(sizeof(M2BatchDisk) == 24,
              "M2BatchDisk is read straight from the file: 24 bytes, no padding");

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
static_assert(sizeof(M2TextureTransformDisk) == 60,
              "M2TextureTransformDisk is read straight from the file: 60 bytes, no padding");

// Vanilla M2 texture transform (3 × 28-byte tracks = 84 bytes)
struct M2TextureTransformDiskVanilla {
    M2TrackDiskVanilla translation; // 28
    M2TrackDiskVanilla rotation;    // 28
    M2TrackDiskVanilla scaling;     // 28
};
static_assert(sizeof(M2TextureTransformDiskVanilla) == 84,
              "M2TextureTransformDiskVanilla is read straight from the file: 84 bytes, no padding");

// M2 attachment point (on-disk, WotLK - 40 bytes)
struct M2AttachmentDisk {
    uint32_t id;
    uint16_t bone;
    uint16_t unknown;
    float position[3];
    uint8_t trackData[20]; // M2TrackDisk (20 bytes)
};
static_assert(sizeof(M2AttachmentDisk) == 40,
              "M2AttachmentDisk is read straight from the file: 40 bytes, no padding");

// M2 attachment point (on-disk, vanilla - 48 bytes, track is 28 bytes)
struct M2AttachmentDiskVanilla {
    uint32_t id;
    uint16_t bone;
    uint16_t unknown;
    float position[3];
    uint8_t trackData[28]; // M2TrackDiskVanilla (28 bytes)
};
static_assert(sizeof(M2AttachmentDiskVanilla) == 48,
              "M2AttachmentDiskVanilla is read straight from the file: 48 bytes, no padding");

// M2 camera (on-disk, WotLK - 100 bytes; tracks are 20 bytes)
struct M2CameraDisk {
    uint32_t type;
    float fov;
    float farClip;
    float nearClip;
    uint8_t positionTrack[20];
    float positionBase[3];
    uint8_t targetTrack[20];
    float targetBase[3];
    uint8_t rollTrack[20];
};

// M2 camera (on-disk, vanilla - 124 bytes; tracks are 28 bytes)
struct M2CameraDiskVanilla {
    uint32_t type;
    float fov;
    float farClip;
    float nearClip;
    uint8_t positionTrack[28];
    float positionBase[3];
    uint8_t targetTrack[28];
    float targetBase[3];
    uint8_t rollTrack[28];
};

// A wrong stride here reads garbage positions and would fling the backdrop
// somewhere arbitrary rather than fail.
static_assert(sizeof(M2CameraDisk) == 100, "M2CameraDisk must match the WotLK on-disk layout");
static_assert(sizeof(M2CameraDiskVanilla) == 124, "M2CameraDiskVanilla must match the vanilla on-disk layout");

template<typename T>
T readValue(const std::vector<uint8_t>& data, uint32_t offset) {
    if (offset + sizeof(T) > data.size()) {
        return T{};
    }
    T value;
    std::memcpy(&value, &data[offset], sizeof(T));
    return value;
}

// Sanity caps for header-supplied counts. Real-game M2 maximums are
// far below these (Blizzard's largest bones-per-model is ~256, largest
// animations-per-model is ~600). The caps exist purely so a hostile
// or corrupted M2 can't trigger a huge `reserve()` allocation or run
// a multi-billion iteration loop before hitting the per-element bounds
// check inside the loop.
constexpr uint32_t kMaxM2Bones        = 65536;
constexpr uint32_t kMaxM2Animations   = 65536;
constexpr uint32_t kMaxM2UVAnims      = 16384;
constexpr uint32_t kMaxM2Attachments  = 4096;
constexpr uint32_t kMaxM2Cameras      = 64;

inline uint32_t capCount(uint32_t value, uint32_t maxValue, const char* what) {
    if (value > maxValue) {
        core::Logger::getInstance().warning(
            "M2: header field ", what, "=", value,
            " exceeds sanity cap ", maxValue, ", clamping");
        return maxValue;
    }
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
    // Use size_t arithmetic to prevent uint32 wraparound (same fix as readArray).
    // A crafted M2 with offset=0xFFFFFFFF, length=2 would wrap to 1 in uint32,
    // passing the check and reading out of bounds.
    if (static_cast<size_t>(offset) + static_cast<size_t>(length) > data.size()) {
        return "";
    }

    // M2 string blocks are C-strings. Some extracted files have a valid
    // string terminated early with embedded NUL and garbage bytes after it.
    // Respect first NUL within the declared length.
    uint32_t actualLen = 0;
    while (actualLen < length && data[offset + actualLen] != 0) {
        actualLen++;
    }

    return std::string(reinterpret_cast<const char*>(&data[offset]), actualLen);
}

enum class TrackType { VEC3, QUAT_COMPRESSED, FLOAT, FIXED16, BYTE_BOOL };

// M2 sequence flag: when set, keyframe data is embedded in the M2 file.
// When clear, data lives in an external .anim file and the M2 offsets are
// .anim-relative - reading them from the M2 produces garbage.
constexpr uint32_t kM2SeqFlagEmbeddedData = 0x20;

// Parse an M2 animation track from the binary data.
// The track uses an "array of arrays" layout: nTimestamps pairs of {count, offset}.
// sequenceFlags: per-sequence flags; sequences without kM2SeqFlagEmbeddedData store
// their keyframe data in external .anim files, so their sub-array offsets must be skipped.
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
        if (i < sequenceFlags.size() && !(sequenceFlags[i] & kM2SeqFlagEmbeddedData)) continue;
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
        else if (type == TrackType::FIXED16) keyElementSize = sizeof(int16_t);
        else if (type == TrackType::BYTE_BOOL) keyElementSize = sizeof(uint8_t);
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
        } else if (type == TrackType::FIXED16) {
            // fixed16: int16 where 0x7FFF = 1.0 (color/transparency alpha tracks)
            auto raw = readArray<int16_t>(data, keyOffset, keyCount);
            track.sequences[i].floatValues.reserve(raw.size());
            for (int16_t v : raw) {
                track.sequences[i].floatValues.push_back(
                    std::clamp(static_cast<float>(v) / 32767.0f, 0.0f, 1.0f));
            }
        } else if (type == TrackType::BYTE_BOOL) {
            // One byte per key, and only ever 0 or 1. Reading these as floats
            // turns a 1 into a denormal of about 1.4e-45, which passes for
            // zero at every threshold downstream and hides the ribbon.
            track.sequences[i].floatValues.reserve(keyCount);
            for (uint32_t k = 0; k < keyCount; k++) {
                track.sequences[i].floatValues.push_back(
                    readValue<uint8_t>(data, keyOffset + k) != 0 ? 1.0f : 0.0f);
            }
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

// Parse a vanilla/TBC M2 animation track (version < 264).
// Both use flat arrays with per-sequence M2Range indices, unlike WotLK's array-of-arrays.
// Rotation key format differs by version:
//   - Vanilla (v256): C4Quaternion, full float[4] (16 bytes/key)
//   - TBC (v260-263): compressed int16[4] quaternion (8 bytes/key), like WotLK
// Set compressedQuat=true for TBC rotation tracks.
void parseAnimTrackVanilla(const std::vector<uint8_t>& data,
                           const M2TrackDiskVanilla& disk,
                           M2AnimationTrack& track,
                           TrackType type,
                           bool compressedQuat = false) {
    track.interpolationType = disk.interpolationType;
    track.globalSequence = disk.globalSequence;

    if (disk.nTimestamps == 0 || disk.nKeys == 0) return;
    // Sanity caps
    if (disk.nTimestamps > 100000 || disk.nKeys > 100000) return;

    // Validate flat timestamp array
    if (disk.ofsTimestamps + disk.nTimestamps * sizeof(uint32_t) > data.size()) return;
    auto allTimestamps = readArray<uint32_t>(data, disk.ofsTimestamps, disk.nTimestamps);

    // Validate flat key array. Rotation key stride depends on version:
    // vanilla = float[4] (16 bytes), TBC = compressed int16[4] (8 bytes).
    size_t keySize;
    if (type == TrackType::FLOAT) keySize = sizeof(float);
    else if (type == TrackType::FIXED16) keySize = sizeof(int16_t);
    else if (type == TrackType::VEC3) keySize = sizeof(float) * 3;
    else keySize = compressedQuat ? sizeof(int16_t) * 4 : sizeof(float) * 4;
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

    // Read the flat key array ONCE before the per-sequence loop. Previously
    // readArray was called inside the loop on every iteration, re-parsing and
    // copying the entire array (O(sequences × keys) redundant memcpy).
    struct Vec3Disk { float x, y, z; };
    struct C4Quaternion { float x, y, z, w; };
    std::vector<float> allFloatKeys;
    std::vector<Vec3Disk> allVec3Keys;
    std::vector<C4Quaternion> allQuatKeys;
    std::vector<CompressedQuat> allCompQuatKeys;
    if (type == TrackType::FLOAT) {
        allFloatKeys = readArray<float>(data, disk.ofsKeys, disk.nKeys);
    } else if (type == TrackType::FIXED16) {
        auto raw = readArray<int16_t>(data, disk.ofsKeys, disk.nKeys);
        allFloatKeys.reserve(raw.size());
        for (int16_t v : raw) {
            allFloatKeys.push_back(std::clamp(static_cast<float>(v) / 32767.0f, 0.0f, 1.0f));
        }
    } else if (type == TrackType::VEC3) {
        allVec3Keys = readArray<Vec3Disk>(data, disk.ofsKeys, disk.nKeys);
    } else if (compressedQuat) {
        allCompQuatKeys = readArray<CompressedQuat>(data, disk.ofsKeys, disk.nKeys);
    } else {
        allQuatKeys = readArray<C4Quaternion>(data, disk.ofsKeys, disk.nKeys);
    }

    track.sequences.resize(ranges.size());

    for (size_t i = 0; i < ranges.size(); i++) {
        uint32_t start = ranges[i].start;
        uint32_t end = ranges[i].end;
        if (start >= end || start >= disk.nTimestamps) continue;
        end = std::min(end, disk.nTimestamps);

        track.sequences[i].timestamps.assign(
            allTimestamps.begin() + start, allTimestamps.begin() + end);
        if (!track.sequences[i].timestamps.empty()) {
            uint32_t firstTime = track.sequences[i].timestamps[0];
            for (auto& ts : track.sequences[i].timestamps) {
                ts -= firstTime;
            }
        }

        if (start >= disk.nKeys) continue;
        uint32_t keyEnd = std::min(end, disk.nKeys);
        uint32_t keyCount = keyEnd - start;

        if (type == TrackType::FLOAT || type == TrackType::FIXED16) {
            track.sequences[i].floatValues.assign(
                allFloatKeys.begin() + start, allFloatKeys.begin() + start + keyCount);
        } else if (type == TrackType::VEC3) {
            track.sequences[i].vec3Values.reserve(keyCount);
            for (uint32_t k = start; k < start + keyCount; k++) {
                track.sequences[i].vec3Values.emplace_back(
                    allVec3Keys[k].x, allVec3Keys[k].y, allVec3Keys[k].z);
            }
        } else if (compressedQuat) {
            // TBC: compressed int16[4] quaternion - same offset mapping as WotLK
            track.sequences[i].quatValues.reserve(keyCount);
            for (uint32_t k = start; k < start + keyCount; k++) {
                const auto& cq = allCompQuatKeys[k];
                float fx = (cq.x < 0) ? (cq.x + 32768) / 32767.0f : (cq.x - 32767) / 32767.0f;
                float fy = (cq.y < 0) ? (cq.y + 32768) / 32767.0f : (cq.y - 32767) / 32767.0f;
                float fz = (cq.z < 0) ? (cq.z + 32768) / 32767.0f : (cq.z - 32767) / 32767.0f;
                float fw = (cq.w < 0) ? (cq.w + 32768) / 32767.0f : (cq.w - 32767) / 32767.0f;
                // M2 on-disk: (x,y,z,w), GLM quat constructor: (w,x,y,z)
                glm::quat q(fw, fx, fy, fz);
                float len = glm::length(q);
                if (len > 0.001f) q = q / len;
                else q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                track.sequences[i].quatValues.push_back(q);
            }
        } else {
            // Vanilla: C4Quaternion - full float[4] per key (XYZW on disk)
            track.sequences[i].quatValues.reserve(keyCount);
            for (uint32_t k = start; k < start + keyCount; k++) {
                const auto& fq = allQuatKeys[k];
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
    // valueType: 0 = color (C3Vector, 3 floats in 0-255 range), 1 = alpha (uint16), 2 = scale (float pair)
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
        // Color: C3Vector (3 × float per key, values in 0-255 range)
        if (ofsKeys + nKeys * 12 > data.size()) return;
        fb.vec3Values.reserve(nKeys);
        for (uint32_t i = 0; i < nKeys; i++) {
            float r = readValue<float>(data, ofsKeys + i * 12 + 0);
            float g = readValue<float>(data, ofsKeys + i * 12 + 4);
            float b = readValue<float>(data, ofsKeys + i * 12 + 8);
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
    // Read common prefix (magic through ofsAnimationLookup) - same for all versions
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

        core::Logger::getInstance().debug("Vanilla M2 (version ", header.version,
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

    // M2 binary layout splits three ways, not two. TBC (v260-263) shares the
    // extended header / 68-byte sequences / 48-byte attachments / 504-byte
    // particle emitters with vanilla, but introduced the boneNameCRC field
    // (112-byte bones vs vanilla's 108), compressed int16[4] rotation
    // quaternions (vs vanilla's float[4]), and 48-byte embedded skin submeshes
    // (vs vanilla's 32). Treating TBC as plain vanilla misaligns every bone
    // after the first, producing garbage pivots - the "vertex explosion" bug.
    const bool isTBC = (header.version >= 260 && header.version < 264);

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
    // Vanilla/TBC sequences live on one shared global timeline; events and
    // tracks reference it via absolute timestamps, so keep each sequence's
    // [start, end) window for mapping those back to sequence-local time.
    std::vector<std::pair<uint32_t, uint32_t>> vanillaSeqWindows;
    if (header.nAnimations > 0 && header.ofsAnimations > 0) {
        header.nAnimations = capCount(header.nAnimations, kMaxM2Animations, "nAnimations");
        model.sequences.reserve(header.nAnimations);

        if (header.version < 264) {
            // Vanilla: 68-byte sequence struct (has startTimestamp + endTimestamp)
            auto diskSeqs = readArray<M2SequenceDiskVanilla>(m2Data, header.ofsAnimations, header.nAnimations);
            vanillaSeqWindows.reserve(diskSeqs.size());
            for (const auto& ds : diskSeqs) {
                vanillaSeqWindows.emplace_back(ds.startTimestamp, ds.endTimestamp);
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

    // Read animation events, keeping the $FSD (footfall) timestamps that sync
    // footstep audio to the frames where feet actually strike the ground.
    // WotLK events are 36 bytes with per-sequence timestamp arrays; vanilla/TBC
    // events are 44 bytes with a flat global-timeline array bucketed here by
    // each sequence's [start, end) window.
    if (header.nEvents > 0 && header.nEvents < 512 && header.ofsEvents > 0 &&
        !model.sequences.empty()) {
        const bool wotlkEvents = header.version >= 264;
        const size_t eventSize = wotlkEvents ? 36 : 44;
        const size_t trackOfs = 24;  // identifier(4) + data(4) + bone(4) + position(12)
        model.footstepEventTimes.resize(model.sequences.size());

        auto rd32 = [&](size_t off) -> uint32_t {
            if (off + 4 > m2Data.size()) return 0;
            uint32_t v;
            std::memcpy(&v, m2Data.data() + off, 4);
            return v;
        };

        size_t footfallStamps = 0;
        for (uint32_t e = 0; e < header.nEvents; e++) {
            size_t base = header.ofsEvents + e * eventSize;
            if (base + eventSize > m2Data.size()) break;
            if (std::memcmp(m2Data.data() + base, "$FSD", 4) != 0) continue;

            if (wotlkEvents) {
                uint32_t nArrays = rd32(base + trackOfs + 4);
                uint32_t ofsArrays = rd32(base + trackOfs + 8);
                uint32_t count = std::min<uint32_t>(nArrays, model.sequences.size());
                for (uint32_t s = 0; s < count; s++) {
                    uint32_t nTs = rd32(ofsArrays + s * 8);
                    uint32_t ofsTs = rd32(ofsArrays + s * 8 + 4);
                    if (nTs == 0 || nTs > 1024) continue;
                    if (ofsTs + nTs * sizeof(uint32_t) > m2Data.size()) continue;
                    auto stamps = readArray<uint32_t>(m2Data, ofsTs, nTs);
                    auto& out = model.footstepEventTimes[s];
                    out.insert(out.end(), stamps.begin(), stamps.end());
                    footfallStamps += stamps.size();
                }
            } else {
                uint32_t nTs = rd32(base + trackOfs + 12);
                uint32_t ofsTs = rd32(base + trackOfs + 16);
                if (nTs == 0 || nTs > 100000) continue;
                if (ofsTs + nTs * sizeof(uint32_t) > m2Data.size()) continue;
                auto stamps = readArray<uint32_t>(m2Data, ofsTs, nTs);
                for (size_t s = 0; s < vanillaSeqWindows.size() &&
                                   s < model.footstepEventTimes.size(); s++) {
                    uint32_t start = vanillaSeqWindows[s].first;
                    uint32_t end = vanillaSeqWindows[s].second;
                    if (start >= end) continue;
                    auto& out = model.footstepEventTimes[s];
                    for (uint32_t ts : stamps) {
                        if (ts >= start && ts < end) {
                            out.push_back(ts - start);
                            footfallStamps++;
                        }
                    }
                }
            }
        }

        for (auto& list : model.footstepEventTimes) {
            std::sort(list.begin(), list.end());
        }
        if (footfallStamps > 0) {
            core::Logger::getInstance().debug("  Footfall events: ", footfallStamps, " timestamps");
        }
    }

    // Read bones with full animation track data
    if (header.nBones > 0 && header.ofsBones > 0) {
        header.nBones = capCount(header.nBones, kMaxM2Bones, "nBones");
        size_t boneStructSize = (header.version >= 264) ? sizeof(M2BoneDisk)
                              : isTBC                    ? sizeof(M2BoneDiskTBC)
                                                         : sizeof(M2BoneDiskVanilla);
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

            if (header.version >= 264) {
                // WotLK: 88-byte bone, array-of-arrays tracks, compressed quats
                M2BoneDisk db = readValue<M2BoneDisk>(m2Data, boneOffset);
                bone.keyBoneId = db.keyBoneId;
                bone.flags = db.flags;
                bone.parentBone = db.parentBone;
                bone.submeshId = db.submeshId;
                bone.pivot = glm::vec3(db.pivot[0], db.pivot[1], db.pivot[2]);
                parseAnimTrack(m2Data, db.translation, bone.translation, TrackType::VEC3, seqFlags);
                parseAnimTrack(m2Data, db.rotation, bone.rotation, TrackType::QUAT_COMPRESSED, seqFlags);
                parseAnimTrack(m2Data, db.scale, bone.scale, TrackType::VEC3, seqFlags);
            } else if (isTBC) {
                // TBC: 112-byte bone (boneNameCRC), flat tracks with ranges,
                // compressed int16[4] rotation quaternions.
                M2BoneDiskTBC db = readValue<M2BoneDiskTBC>(m2Data, boneOffset);
                bone.keyBoneId = db.keyBoneId;
                bone.flags = db.flags;
                bone.parentBone = db.parentBone;
                bone.submeshId = db.submeshId;
                bone.pivot = glm::vec3(db.pivot[0], db.pivot[1], db.pivot[2]);
                parseAnimTrackVanilla(m2Data, db.translation, bone.translation, TrackType::VEC3);
                parseAnimTrackVanilla(m2Data, db.rotation, bone.rotation, TrackType::QUAT_COMPRESSED, /*compressedQuat=*/true);
                parseAnimTrackVanilla(m2Data, db.scale, bone.scale, TrackType::VEC3);
            } else {
                // Vanilla: 108-byte bone (no boneNameCRC), flat tracks with
                // ranges, full float[4] rotation quaternions.
                M2BoneDiskVanilla db = readValue<M2BoneDiskVanilla>(m2Data, boneOffset);
                bone.keyBoneId = db.keyBoneId;
                bone.flags = db.flags;
                bone.parentBone = db.parentBone;
                bone.submeshId = db.submeshId;
                bone.pivot = glm::vec3(db.pivot[0], db.pivot[1], db.pivot[2]);
                parseAnimTrackVanilla(m2Data, db.translation, bone.translation, TrackType::VEC3);
                parseAnimTrackVanilla(m2Data, db.rotation, bone.rotation, TrackType::QUAT_COMPRESSED);
                parseAnimTrackVanilla(m2Data, db.scale, bone.scale, TrackType::VEC3);
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

    // Parse color animation alpha tracks (M2Color: vec3 color track + fixed16 alpha track).
    // WotLK: two 20-byte M2TrackDisk headers (40 bytes/color).
    // Vanilla/TBC (<264): two 28-byte M2TrackDiskVanilla headers (56 bytes/color).
    // The alpha track gates per-batch visibility per animation - e.g. the peasant
    // lumberjack's carry model has two wood-bundle submeshes, and only one is
    // alpha-1 in any given animation.
    if (header.nColors > 0 && header.ofsColors > 0 && header.nColors < 4096) {
        std::vector<uint32_t> seqFlags;
        seqFlags.reserve(model.sequences.size());
        for (const auto& seq : model.sequences) seqFlags.push_back(seq.flags);

        const bool wotlk = header.version >= 264;
        const uint32_t colorSize = wotlk ? 40u : 56u;   // 2 track headers per color
        const uint32_t alphaOfs  = wotlk ? 20u : 28u;   // skip the vec3 color track
        model.colorAlphas.reserve(header.nColors);
        model.colorRGB.reserve(header.nColors);
        model.colorAlphaTracks.resize(header.nColors);
        for (uint32_t ci = 0; ci < header.nColors; ci++) {
            uint32_t alphaTrackOfs = header.ofsColors + ci * colorSize + alphaOfs;
            M2AnimationTrack& track = model.colorAlphaTracks[ci];
            if (wotlk) {
                if (alphaTrackOfs + sizeof(M2TrackDisk) <= m2Data.size()) {
                    M2TrackDisk td = readValue<M2TrackDisk>(m2Data, alphaTrackOfs);
                    parseAnimTrack(m2Data, td, track, TrackType::FIXED16, seqFlags);
                }
            } else {
                if (alphaTrackOfs + sizeof(M2TrackDiskVanilla) <= m2Data.size()) {
                    M2TrackDiskVanilla td = readValue<M2TrackDiskVanilla>(m2Data, alphaTrackOfs);
                    parseAnimTrackVanilla(m2Data, td, track, TrackType::FIXED16);
                }
            }
            // At-rest alpha (first key of the first sequence with data) - used by
            // renderers that don't evaluate the track per frame.
            float alpha = 1.0f;
            for (const auto& seq : track.sequences) {
                if (!seq.floatValues.empty()) {
                    alpha = seq.floatValues[0];
                    break;
                }
            }
            model.colorAlphas.push_back(alpha);

            // The vec3 colour track sits at the front of the same record, and
            // its at-rest value is what tints the batch. Skipping it left every
            // tinted card the colour of its texture: Orgrimmar's bonfire glow
            // is authored white and carries (1.0, 0.329, 0.0) here, so the
            // fire rendered as a white blob.
            // The vec3 colour track sits at the front of the same record and
            // its at-rest value tints the batch. The two versions store the
            // keys differently and reading one as the other gives a plausible
            // colour rather than a failure: WotLK nests an array per sequence
            // behind the track's key array, vanilla points straight at the
            // values. See m2ColorTrackFirstKeyOffset.
            glm::vec3 rgb(1.0f);
            const uint32_t colorTrackOfs = header.ofsColors + ci * colorSize;
            uint32_t rgbOfs = 0;
            if (m2ColorTrackFirstKeyOffset(m2Data.size(), colorTrackOfs, wotlk,
                                           [&](uint32_t at) {
                                               return readValue<uint32_t>(m2Data, at);
                                           },
                                           rgbOfs)) {
                rgb.x = readValue<float>(m2Data, rgbOfs + 0);
                rgb.y = readValue<float>(m2Data, rgbOfs + 4);
                rgb.z = readValue<float>(m2Data, rgbOfs + 8);
                if (!std::isfinite(rgb.x) || !std::isfinite(rgb.y) ||
                    !std::isfinite(rgb.z)) {
                    rgb = glm::vec3(1.0f);
                }
                rgb = glm::clamp(rgb, glm::vec3(0.0f), glm::vec3(1.0f));
            }
            model.colorRGB.push_back(rgb);
        }
    }

    // Read bone lookup table (vertex bone indices reference this to get actual bone index)
    if (header.nBoneLookupTable > 0 && header.ofsBoneLookupTable > 0) {
        model.boneLookupTable = readArray<uint16_t>(m2Data, header.ofsBoneLookupTable, header.nBoneLookupTable);
        core::Logger::getInstance().debug("  BoneLookupTable: ", model.boneLookupTable.size(), " entries");
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

        header.nUVAnimation = capCount(header.nUVAnimation, kMaxM2UVAnims, "nUVAnimation");
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
                parseAnimTrackVanilla(m2Data, dt.rotation, tt.rotation, TrackType::QUAT_COMPRESSED, /*compressedQuat=*/isTBC);
                parseAnimTrackVanilla(m2Data, dt.scaling, tt.scale, TrackType::VEC3);
            }
            model.textureTransforms.push_back(std::move(tt));
        }
        core::Logger::getInstance().debug("  Texture transforms: ", model.textureTransforms.size());
    }

    // Read transparency lookup (indexed by batch.transparencyIndex).
    if (header.nTransLookup > 0 && header.ofsTransLookup > 0) {
        model.textureWeightLookup = readArray<uint16_t>(m2Data, header.ofsTransLookup, header.nTransLookup);
    }

    // Read texture-animation lookup (indexed by batch.textureAnimIndex).
    if (header.nUVAnimLookup > 0 && header.ofsUVAnimLookup > 0) {
        model.textureTransformLookup = readArray<uint16_t>(m2Data, header.ofsUVAnimLookup, header.nUVAnimLookup);
    }

    // Parse transparency tracks (M2Track<fixed16>) - controls per-batch opacity.
    // fixed16 = uint16_t / 32767.0f, range 0 (transparent) to 1 (opaque).
    // Keep the full track for runtime evaluation; using only its first key makes
    // pulsing weapon glints such as Sparkle_A stay at full intensity continuously.
    if (header.nTransparency > 0 && header.ofsTransparency > 0 &&
        header.nTransparency < 4096) {
        std::vector<uint32_t> seqFlags;
        seqFlags.reserve(model.sequences.size());
        for (const auto& seq : model.sequences) seqFlags.push_back(seq.flags);

        model.textureWeights.reserve(header.nTransparency);
        model.textureWeightTracks.resize(header.nTransparency);
        const bool wotlk = header.version >= 264;
        const uint32_t trackSize = wotlk ? sizeof(M2TrackDisk) : sizeof(M2TrackDiskVanilla);
        for (uint32_t ti = 0; ti < header.nTransparency; ti++) {
            uint32_t trackOfs = header.ofsTransparency + ti * trackSize;
            if (trackOfs + trackSize > m2Data.size()) {
                model.textureWeights.push_back(1.0f);
                continue;
            }
            M2AnimationTrack& track = model.textureWeightTracks[ti];
            if (wotlk) {
                M2TrackDisk td = readValue<M2TrackDisk>(m2Data, trackOfs);
                parseAnimTrack(m2Data, td, track, TrackType::FIXED16, seqFlags);
            } else {
                M2TrackDiskVanilla td = readValue<M2TrackDiskVanilla>(m2Data, trackOfs);
                parseAnimTrackVanilla(m2Data, td, track, TrackType::FIXED16);
            }
            float opacity = 1.0f;
            for (const auto& seq : track.sequences) {
                if (!seq.floatValues.empty()) {
                    opacity = seq.floatValues[0];
                    break;
                }
            }
            model.textureWeights.push_back(opacity);
        }
    }

    // Read attachment points (vanilla uses 48-byte struct, WotLK uses 40-byte)
    if (header.nAttachments > 0 && header.ofsAttachments > 0) {
        header.nAttachments = capCount(header.nAttachments, kMaxM2Attachments, "nAttachments");
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

    // Cameras - only the static base position/target are used; scene models keep
    // their framing here, and animated camera tracks are not needed for a backdrop.
    if (header.nCameras > 0 && header.ofsCameras > 0) {
        const uint32_t cameraCount = capCount(header.nCameras, kMaxM2Cameras, "nCameras");
        model.cameras.reserve(cameraCount);
        auto pushCamera = [&model](uint32_t type, float fov, float farClip, float nearClip,
                                   const float* posBase, const float* tgtBase) {
            // type 0xFFFFFFFF marks the model's own portrait/scene camera.
            (void)type;
            M2Camera cam;
            cam.fov = fov;
            cam.farClip = farClip;
            cam.nearClip = nearClip;
            cam.positionBase = glm::vec3(posBase[0], posBase[1], posBase[2]);
            cam.targetBase = glm::vec3(tgtBase[0], tgtBase[1], tgtBase[2]);
            model.cameras.push_back(cam);
        };
        if (header.version < 264) {
            auto disk = readArray<M2CameraDiskVanilla>(m2Data, header.ofsCameras, cameraCount);
            for (const auto& dc : disk)
                pushCamera(dc.type, dc.fov, dc.farClip, dc.nearClip, dc.positionBase, dc.targetBase);
        } else {
            auto disk = readArray<M2CameraDisk>(m2Data, header.ofsCameras, cameraCount);
            for (const auto& dc : disk)
                pushCamera(dc.type, dc.fov, dc.farClip, dc.nearClip, dc.positionBase, dc.targetBase);
        }
        core::Logger::getInstance().debug("  Cameras: ", model.cameras.size());
    }

    // Parse particle emitters - struct size differs between versions:
    //   WotLK (version >= 264): M2ParticleOld = 0x1DC (476) bytes, M2TrackDisk (20 bytes), FBlocks
    //   Vanilla (version < 264): 0x1F8 (504) bytes, M2TrackDiskVanilla (28 bytes), static lifecycle arrays
    if (header.nParticleEmitters > 0 && header.ofsParticleEmitters > 0 &&
        header.nParticleEmitters < 256) {

        const bool isVanilla = (header.version < 264);
        static constexpr uint32_t EMITTER_SIZE_WOTLK  = 0x1DC; // 476
        static constexpr uint32_t EMITTER_SIZE_VANILLA = 0x1F8; // 504
        const uint32_t emitterSize = isVanilla ? EMITTER_SIZE_VANILLA : EMITTER_SIZE_WOTLK;

        if (static_cast<size_t>(header.ofsParticleEmitters) +
                static_cast<size_t>(header.nParticleEmitters) * emitterSize <= m2Data.size()) {

        // Build sequence flags for parseAnimTrack (WotLK only)
        std::vector<uint32_t> emSeqFlags;
        if (!isVanilla) {
            emSeqFlags.reserve(model.sequences.size());
            for (const auto& seq : model.sequences) {
                emSeqFlags.push_back(seq.flags);
            }
        }

        for (uint32_t ei = 0; ei < header.nParticleEmitters; ei++) {
            uint32_t base = header.ofsParticleEmitters + ei * emitterSize;

            M2ParticleEmitter em;
            // Header fields (0x00-0x33) are the same for both versions
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

            if (isVanilla) {
                // Vanilla: 10 contiguous M2TrackDiskVanilla tracks (28 bytes each) at 0x34
                auto parseTrackV = [&](uint32_t off, M2AnimationTrack& track) {
                    if (base + off + sizeof(M2TrackDiskVanilla) <= m2Data.size()) {
                        M2TrackDiskVanilla disk = readValue<M2TrackDiskVanilla>(m2Data, base + off);
                        parseAnimTrackVanilla(m2Data, disk, track, TrackType::FLOAT);
                    }
                };
                parseTrackV(0x34, em.emissionSpeed);       // +28 = 0x50
                parseTrackV(0x50, em.speedVariation);      // +28 = 0x6C
                parseTrackV(0x6C, em.verticalRange);       // +28 = 0x88
                parseTrackV(0x88, em.horizontalRange);     // +28 = 0xA4
                parseTrackV(0xA4, em.gravity);             // +28 = 0xC0
                parseTrackV(0xC0, em.lifespan);            // +28 = 0xDC
                parseTrackV(0xDC, em.emissionRate);        // +28 = 0xF8
                parseTrackV(0xF8, em.emissionAreaLength);  // +28 = 0x114
                parseTrackV(0x114, em.emissionAreaWidth);  // +28 = 0x130
                parseTrackV(0x130, em.deceleration);       // +28 = 0x14C

                // Vanilla: NO FBlocks - color/alpha/scale are static inline values
                // Layout (empirically confirmed from real vanilla M2 files):
                //   +0x14C: float midpoint (lifecycle split: 0→mid→1)
                //   +0x150: uint32 colorValues[3] (BGRA, A channel = opacity)
                //   +0x15C: float scaleValues[3] (1D particle scale)
                float midpoint = readValue<float>(m2Data, base + 0x14C);
                if (midpoint < 0.0f || midpoint > 1.0f) midpoint = 0.5f;

                // Synthesize color FBlock from static BGRA values
                // Vanilla M2 stores 3× uint32 as BGRA (little-endian: byte0=B, byte1=G, byte2=R, byte3=A)
                {
                    em.particleColor.timestamps = {0.0f, midpoint, 1.0f};
                    em.particleColor.vec3Values.resize(3);
                    em.particleAlpha.timestamps = {0.0f, midpoint, 1.0f};
                    em.particleAlpha.floatValues.resize(3);
                    for (int c = 0; c < 3; c++) {
                        uint32_t bgra = readValue<uint32_t>(m2Data, base + 0x150 + c * 4);
                        float b = ((bgra >>  0) & 0xFF) / 255.0f;
                        float g = ((bgra >>  8) & 0xFF) / 255.0f;
                        float r = ((bgra >> 16) & 0xFF) / 255.0f;
                        float a = ((bgra >> 24) & 0xFF) / 255.0f;
                        em.particleColor.vec3Values[c] = glm::vec3(r, g, b);
                        em.particleAlpha.floatValues[c] = a;
                    }
                    // If all alpha zero, use sensible default (fade out)
                    bool allZero = true;
                    for (auto v : em.particleAlpha.floatValues) {
                        if (v > 0.01f) { allZero = false; break; }
                    }
                    if (allZero) {
                        em.particleAlpha.floatValues = {1.0f, 1.0f, 0.0f};
                    }
                }

                // Synthesize scale FBlock from static float values
                {
                    em.particleScale.timestamps = {0.0f, midpoint, 1.0f};
                    em.particleScale.floatValues.resize(3);
                    for (int s = 0; s < 3; s++) {
                        float scale = readValue<float>(m2Data, base + 0x15C + s * 4);
                        if (scale < 0.001f || scale > 100.0f) scale = 1.0f;
                        em.particleScale.floatValues[s] = scale;
                    }
                }
            } else {
                // WotLK: M2TrackDisk (20 bytes) at known offsets with vary floats interspersed
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

                // Parse FBlocks (color, alpha, scale) - FBlocks are 16 bytes each
                parseFBlock(m2Data, base + 0x104, em.particleColor, 0);
                parseFBlock(m2Data, base + 0x114, em.particleAlpha, 1);
                parseFBlock(m2Data, base + 0x124, em.particleScale, 2);
            }

            model.particleEmitters.push_back(std::move(em));
        }
        core::Logger::getInstance().debug("  Particle emitters: ", model.particleEmitters.size());
        } // end size check
    }

    // Parse ribbon emitters (WotLK only; vanilla format TBD).
    // WotLK M2RibbonEmitter = 0xAC (172) bytes per entry.
    static constexpr uint32_t RIBBON_SIZE_WOTLK = 0xAC;
    if (header.nRibbonEmitters > 0 && header.ofsRibbonEmitters > 0 &&
        header.nRibbonEmitters < 64 && header.version >= 264) {

        if (static_cast<size_t>(header.ofsRibbonEmitters) +
                static_cast<size_t>(header.nRibbonEmitters) * RIBBON_SIZE_WOTLK <= m2Data.size()) {

            // Build sequence flags for parseAnimTrack
            std::vector<uint32_t> ribSeqFlags;
            ribSeqFlags.reserve(model.sequences.size());
            for (const auto& seq : model.sequences) {
                ribSeqFlags.push_back(seq.flags);
            }

            for (uint32_t ri = 0; ri < header.nRibbonEmitters; ri++) {
                uint32_t base = header.ofsRibbonEmitters + ri * RIBBON_SIZE_WOTLK;

                M2RibbonEmitter rib;
                rib.ribbonId     = readValue<int32_t>(m2Data, base + 0x00);
                rib.bone         = readValue<uint32_t>(m2Data, base + 0x04);
                rib.position.x   = readValue<float>(m2Data, base + 0x08);
                rib.position.y   = readValue<float>(m2Data, base + 0x0C);
                rib.position.z   = readValue<float>(m2Data, base + 0x10);

                // textureIndices M2Array (0x14): count + offset → first element = texture lookup index
                {
                    uint32_t nTex = readValue<uint32_t>(m2Data, base + 0x14);
                    uint32_t ofsTex = readValue<uint32_t>(m2Data, base + 0x18);
                    if (nTex > 0 && ofsTex + sizeof(uint16_t) <= m2Data.size()) {
                        rib.textureIndex = readValue<uint16_t>(m2Data, ofsTex);
                    }
                }

                // materialIndices M2Array (0x1C): count + offset → first element = material index
                {
                    uint32_t nMat = readValue<uint32_t>(m2Data, base + 0x1C);
                    uint32_t ofsMat = readValue<uint32_t>(m2Data, base + 0x20);
                    if (nMat > 0 && ofsMat + sizeof(uint16_t) <= m2Data.size()) {
                        rib.materialIndex = readValue<uint16_t>(m2Data, ofsMat);
                    }
                }

                // colorTrack M2TrackDisk at 0x24 (vec3 RGB 0..1)
                if (base + 0x24 + sizeof(M2TrackDisk) <= m2Data.size()) {
                    M2TrackDisk disk = readValue<M2TrackDisk>(m2Data, base + 0x24);
                    parseAnimTrack(m2Data, disk, rib.colorTrack, TrackType::VEC3, ribSeqFlags);
                }

                // alphaTrack M2TrackDisk at 0x38 (fixed16: int16/32767)
                // Same nested-array layout as parseAnimTrack but keys are int16.
                if (base + 0x38 + sizeof(M2TrackDisk) <= m2Data.size()) {
                    parseAnimTrack(m2Data, readValue<M2TrackDisk>(m2Data, base + 0x38),
                                   rib.alphaTrack, TrackType::FIXED16, ribSeqFlags);
                }

                // heightAboveTrack M2TrackDisk at 0x4C (float)
                if (base + 0x4C + sizeof(M2TrackDisk) <= m2Data.size()) {
                    M2TrackDisk disk = readValue<M2TrackDisk>(m2Data, base + 0x4C);
                    parseAnimTrack(m2Data, disk, rib.heightAboveTrack, TrackType::FLOAT, ribSeqFlags);
                }

                // heightBelowTrack M2TrackDisk at 0x60 (float)
                if (base + 0x60 + sizeof(M2TrackDisk) <= m2Data.size()) {
                    M2TrackDisk disk = readValue<M2TrackDisk>(m2Data, base + 0x60);
                    parseAnimTrack(m2Data, disk, rib.heightBelowTrack, TrackType::FLOAT, ribSeqFlags);
                }

                rib.edgesPerSecond = readValue<float>(m2Data, base + 0x74);
                rib.edgeLifetime   = readValue<float>(m2Data, base + 0x78);
                rib.gravity        = readValue<float>(m2Data, base + 0x7C);
                rib.textureRows    = readValue<uint16_t>(m2Data, base + 0x80);
                rib.textureCols    = readValue<uint16_t>(m2Data, base + 0x82);
                if (rib.textureRows == 0) rib.textureRows = 1;
                if (rib.textureCols == 0) rib.textureCols = 1;

                // Clamp to sane values
                if (rib.edgesPerSecond < 1.0f  || rib.edgesPerSecond > 200.0f) rib.edgesPerSecond = 15.0f;
                if (rib.edgeLifetime   < 0.05f || rib.edgeLifetime   > 10.0f)  rib.edgeLifetime   = 0.5f;

                // visibilityTrack M2TrackDisk at 0x98 - keys are uint8 (0/1), NOT float.
                // Must read as uint8 and convert to float, else 0x01 reads as
                // float ~1.4e-45 which fails the visibility > 0.5 check.
                if (base + 0x98 + sizeof(M2TrackDisk) <= m2Data.size()) {
                    parseAnimTrack(m2Data, readValue<M2TrackDisk>(m2Data, base + 0x98),
                                   rib.visibilityTrack, TrackType::BYTE_BOOL, ribSeqFlags);
                }

                // Skip garbage emitters (common M2 artifact: alternating emitters
                // have bone=UINT_MAX or other invalid state)
                if (rib.bone == 0xFFFFFFFF) {
                    continue;
                }

                model.ribbonEmitters.push_back(std::move(rib));
            }
            core::Logger::getInstance().debug("  Ribbon emitters: ", model.ribbonEmitters.size());
        }
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

        // Read submeshes. Stride differs by version: vanilla (v256) = 32 bytes
        // (no sortCenter/sortRadius), TBC (v260-263) = 48 bytes (full M2SkinSubmesh,
        // same as WotLK's external .skin). Wrong stride scrambles the submesh→batch
        // vertex/index ranges.
        std::vector<M2SkinSubmesh> submeshes;
        if (skinProfile.nSubmeshes > 0 && skinProfile.ofsSubmeshes > 0) {
            if (isTBC) {
                submeshes = readArray<M2SkinSubmesh>(m2Data,
                    skinProfile.ofsSubmeshes, skinProfile.nSubmeshes);
            } else {
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
        }

        // Read batches
        if (skinProfile.nBatches > 0 && skinProfile.ofsBatches > 0) {
            auto diskBatches = readArray<M2BatchDisk>(m2Data,
                skinProfile.ofsBatches, skinProfile.nBatches);
            model.batches.clear();
            model.batches.reserve(diskBatches.size());

            for (const auto& db : diskBatches) {
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

        core::Logger::getInstance().debug("Vanilla M2: embedded skin loaded - ",
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
        (void)submeshes;
    }

    // Read batches with proper submesh references
    if (header.nBatches > 0 && header.ofsBatches > 0) {
        auto diskBatches = readArray<M2BatchDisk>(skinData, header.ofsBatches, header.nBatches);
        model.batches.clear();
        model.batches.reserve(diskBatches.size());

        for (const auto& db : diskBatches) {
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

    // A batch that reaches past the model's own indices is emptied here, after
    // the last of them is built, where every renderer that draws this model
    // shares the check.
    //
    // Placed after the loop rather than before it, which is where this first
    // went: the guard ran between the two batch-building paths and so checked
    // batches that did not exist yet, leaving the skin path - the one character
    // models take - unguarded.
    //
    // A submesh
    // read from the wrong offset gives a start that no buffer could satisfy,
    // and vkCmdDrawIndexed does not check. One such batch - start 6,290,784 in
    // a 768-index model - was read 25 MB past the end forty-six times and took
    // the GPU with it. Emptied rather than dropped so nothing downstream has to
    // cope with a batch list that changed length.
    {
        const uint32_t total = static_cast<uint32_t>(model.indices.size());
        uint32_t emptied = 0;
        for (auto& batch : model.batches) {
            const uint64_t end = static_cast<uint64_t>(batch.indexStart) + batch.indexCount;
            if (end <= total) continue;
            batch.indexStart = 0;
            batch.indexCount = 0;
            ++emptied;
        }
        if (emptied > 0) {
            core::Logger::getInstance().warning(
                "  ", emptied, " batch(es) index past the model's ", total,
                " indices - emptied; this model's submeshes are being read from "
                "the wrong offset");
        }
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


std::string skinPathForM2(const std::string& m2Path) {
    const size_t dot = m2Path.rfind('.');
    const size_t slash = m2Path.find_last_of("\\/");
    // Only an extension in the last component counts; a dot in a folder name is
    // part of the folder.
    const bool hasExt = dot != std::string::npos &&
                        (slash == std::string::npos || dot > slash);
    return (hasExt ? m2Path.substr(0, dot) : m2Path) + "00.skin";
}


std::string modelPathToM2(const std::string& modelPath) {
    if (modelPath.size() < 4) return modelPath;
    std::string ext = modelPath.substr(modelPath.size() - 4);
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (ext == ".mdx" || ext == ".mdl") {
        return modelPath.substr(0, modelPath.size() - 4) + ".m2";
    }
    return modelPath;
}

} // namespace pipeline
} // namespace wowee
