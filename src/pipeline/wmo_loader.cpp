#include "pipeline/wmo_loader.hpp"
#include "core/logger.hpp"
#include <cstring>
#include <glm/gtc/quaternion.hpp>

namespace wowee {
namespace pipeline {

namespace {

// WMO chunk identifiers
constexpr uint32_t MVER = 0x4D564552;  // Version
constexpr uint32_t MOHD = 0x4D4F4844;  // Header
constexpr uint32_t MOTX = 0x4D4F5458;  // Textures
constexpr uint32_t MOMT = 0x4D4F4D54;  // Materials
constexpr uint32_t MOGN = 0x4D4F474E;  // Group names
constexpr uint32_t MOGI = 0x4D4F4749;  // Group info
constexpr uint32_t MOLT = 0x4D4F4C54;  // Lights
constexpr uint32_t MODN = 0x4D4F444E;  // Doodad names
constexpr uint32_t MODD = 0x4D4F4444;  // Doodad definitions
constexpr uint32_t MODS = 0x4D4F4453;  // Doodad sets
constexpr uint32_t MOPV = 0x4D4F5056;  // Portal vertices
constexpr uint32_t MOPT = 0x4D4F5054;  // Portal info
constexpr uint32_t MOPR = 0x4D4F5052;  // Portal references
constexpr uint32_t MFOG = 0x4D464F47;  // Fog

// WMO group chunk identifiers
constexpr uint32_t MOGP = 0x4D4F4750;  // Group header
constexpr uint32_t MOVV = 0x4D4F5656;  // Vertices
constexpr uint32_t MOVI = 0x4D4F5649;  // Indices
constexpr uint32_t MOBA = 0x4D4F4241;  // Batches
constexpr uint32_t MOCV = 0x4D4F4356;  // Vertex colors
constexpr uint32_t MONR = 0x4D4F4E52;  // Normals
constexpr uint32_t MOTV = 0x4D4F5456;  // Texture coords
constexpr uint32_t MLIQ = 0x4D4C4951;  // Liquid

// Read utilities
template<typename T>
T read(const std::vector<uint8_t>& data, uint32_t& offset) {
    if (offset + sizeof(T) > data.size()) {
        return T{};
    }
    T value;
    std::memcpy(&value, &data[offset], sizeof(T));
    offset += sizeof(T);
    return value;
}

template<typename T>
std::vector<T> readArray(const std::vector<uint8_t>& data, uint32_t offset, uint32_t count) {
    std::vector<T> result;
    if (offset + count * sizeof(T) > data.size()) {
        return result;
    }
    result.resize(count);
    std::memcpy(result.data(), &data[offset], count * sizeof(T));
    return result;
}

std::string readString(const std::vector<uint8_t>& data, uint32_t offset) {
    std::string result;
    while (offset < data.size() && data[offset] != 0) {
        result += static_cast<char>(data[offset++]);
    }
    return result;
}

} // anonymous namespace

WMOModel WMOLoader::load(const std::vector<uint8_t>& wmoData) {
    WMOModel model;

    if (wmoData.size() < 8) {
        core::Logger::getInstance().error("WMO data too small");
        return model;
    }

    // WMO loader logs disabled

    uint32_t offset = 0;

    // Parse chunks
    while (offset + 8 <= wmoData.size()) {
        uint32_t chunkId = read<uint32_t>(wmoData, offset);
        uint32_t chunkSize = read<uint32_t>(wmoData, offset);

        if (offset + chunkSize > wmoData.size()) {
            core::Logger::getInstance().warning("Chunk extends beyond file");
            break;
        }

        uint32_t chunkStart = offset;
        uint32_t chunkEnd = offset + chunkSize;

        switch (chunkId) {
            case MVER: {
                model.version = read<uint32_t>(wmoData, offset);
                // WMO version log disabled
                break;
            }

            case MOHD: {
                // Header - SMOHeader structure (WotLK 3.3.5a)
                model.nTextures = read<uint32_t>(wmoData, offset);   // Was missing!
                model.nGroups = read<uint32_t>(wmoData, offset);
                model.nPortals = read<uint32_t>(wmoData, offset);
                model.nLights = read<uint32_t>(wmoData, offset);
                model.nDoodadNames = read<uint32_t>(wmoData, offset);
                model.nDoodadDefs = read<uint32_t>(wmoData, offset);
                model.nDoodadSets = read<uint32_t>(wmoData, offset);

                [[maybe_unused]] uint32_t ambColor = read<uint32_t>(wmoData, offset);  // Ambient color (BGRA)
                [[maybe_unused]] uint32_t wmoID = read<uint32_t>(wmoData, offset);

                model.boundingBoxMin.x = read<float>(wmoData, offset);
                model.boundingBoxMin.y = read<float>(wmoData, offset);
                model.boundingBoxMin.z = read<float>(wmoData, offset);

                model.boundingBoxMax.x = read<float>(wmoData, offset);
                model.boundingBoxMax.y = read<float>(wmoData, offset);
                model.boundingBoxMax.z = read<float>(wmoData, offset);

                // flags and numLod (uint16 each) - skip for now
                offset += 4;

                core::Logger::getInstance().info("WMO header: nTextures=", model.nTextures, " nGroups=", model.nGroups);
                break;
            }

            case MOTX: {
                // Textures - raw block of null-terminated strings
                // Material texture1/texture2/texture3 are byte offsets into this chunk.
                // We must map every offset to its texture index.
                uint32_t texOffset = chunkStart;
                uint32_t texIndex = 0;
                core::Logger::getInstance().info("MOTX chunk: ", chunkSize, " bytes");
                while (texOffset < chunkEnd) {
                    uint32_t relativeOffset = texOffset - chunkStart;

                    std::string texName = readString(wmoData, texOffset);
                    if (texName.empty()) {
                        // Skip null bytes (empty entries or padding)
                        texOffset++;
                        continue;
                    }

                    // Store mapping from byte offset to texture index
                    model.textureOffsetToIndex[relativeOffset] = texIndex;
                    model.textures.push_back(texName);
                    // MOTX texture log disabled
                    texOffset += texName.length() + 1;
                    texIndex++;
                }
                // WMO textures log disabled
                break;
            }

            case MOMT: {
                // Materials - dump raw fields to find correct layout
                uint32_t nMaterials = chunkSize / 64;  // Each material is 64 bytes
                for (uint32_t i = 0; i < nMaterials; i++) {
                    // Read all 16 uint32 fields (64 bytes)
                    uint32_t fields[16];
                    for (int j = 0; j < 16; j++) {
                        fields[j] = read<uint32_t>(wmoData, offset);
                    }

                    // SMOMaterial layout (wowdev.wiki):
                    // 0: flags, 1: shader, 2: blendMode
                    // 3: texture_1 (MOTX offset)
                    // 4: sidnColor (emissive), 5: frameSidnColor
                    // 6: texture_2 (MOTX offset)
                    // 7: diffColor, 8: ground_type
                    // 9: texture_3 (MOTX offset)
                    // 10: color_2, 11: flags2
                    // 12-15: runtime
                    WMOMaterial mat;
                    mat.flags = fields[0];
                    mat.shader = fields[1];
                    mat.blendMode = fields[2];
                    mat.texture1 = fields[3];
                    mat.color1 = fields[4];
                    mat.texture2 = fields[6];  // Skip frameSidnColor at [5]
                    mat.color2 = fields[7];
                    mat.texture3 = fields[9];  // Skip ground_type at [8]
                    mat.color3 = fields[10];

                    model.materials.push_back(mat);
                }
                core::Logger::getInstance().info("WMO materials: ", model.materials.size());
                break;
            }

            case MOGN: {
                // Group names
                uint32_t nameOffset = chunkStart;
                while (nameOffset < chunkEnd) {
                    std::string name = readString(wmoData, nameOffset);
                    if (name.empty()) break;
                    model.groupNames.push_back(name);
                    nameOffset += name.length() + 1;
                }
                // WMO group names log disabled
                break;
            }

            case MOGI: {
                // Group info
                uint32_t nGroupInfo = chunkSize / 32;  // Each group info is 32 bytes
                for (uint32_t i = 0; i < nGroupInfo; i++) {
                    WMOGroupInfo info;
                    info.flags = read<uint32_t>(wmoData, offset);
                    info.boundingBoxMin.x = read<float>(wmoData, offset);
                    info.boundingBoxMin.y = read<float>(wmoData, offset);
                    info.boundingBoxMin.z = read<float>(wmoData, offset);
                    info.boundingBoxMax.x = read<float>(wmoData, offset);
                    info.boundingBoxMax.y = read<float>(wmoData, offset);
                    info.boundingBoxMax.z = read<float>(wmoData, offset);
                    info.nameOffset = read<int32_t>(wmoData, offset);

                    model.groupInfo.push_back(info);
                }
                core::Logger::getInstance().info("WMO group info: ", model.groupInfo.size());
                break;
            }

            case MOLT: {
                // Lights
                uint32_t nLights = chunkSize / 48;  // Approximate size
                for (uint32_t i = 0; i < nLights && offset < chunkEnd; i++) {
                    WMOLight light;
                    light.type = read<uint32_t>(wmoData, offset);
                    light.useAttenuation = read<uint8_t>(wmoData, offset);
                    light.pad[0] = read<uint8_t>(wmoData, offset);
                    light.pad[1] = read<uint8_t>(wmoData, offset);
                    light.pad[2] = read<uint8_t>(wmoData, offset);

                    light.color.r = read<float>(wmoData, offset);
                    light.color.g = read<float>(wmoData, offset);
                    light.color.b = read<float>(wmoData, offset);
                    light.color.a = read<float>(wmoData, offset);

                    light.position.x = read<float>(wmoData, offset);
                    light.position.y = read<float>(wmoData, offset);
                    light.position.z = read<float>(wmoData, offset);

                    light.intensity = read<float>(wmoData, offset);
                    light.attenuationStart = read<float>(wmoData, offset);
                    light.attenuationEnd = read<float>(wmoData, offset);

                    for (int j = 0; j < 4; j++) {
                        light.unknown[j] = read<float>(wmoData, offset);
                    }

                    model.lights.push_back(light);
                }
                core::Logger::getInstance().info("WMO lights: ", model.lights.size());
                break;
            }

            case MODN: {
                // Doodad names — stored by byte offset into the MODN chunk
                // (MODD nameIndex is a byte offset, not a vector index)
                uint32_t nameOffset = 0;  // Offset relative to chunk start
                while (chunkStart + nameOffset < chunkEnd) {
                    std::string name = readString(wmoData, chunkStart + nameOffset);
                    if (!name.empty()) {
                        model.doodadNames[nameOffset] = name;
                    }
                    nameOffset += name.length() + 1;
                }
                core::Logger::getInstance().debug("Loaded ", model.doodadNames.size(), " doodad names");
                break;
            }

            case MODD: {
                // Doodad definitions
                uint32_t nDoodads = chunkSize / 40;  // Each doodad is 40 bytes
                for (uint32_t i = 0; i < nDoodads; i++) {
                    WMODoodad doodad;

                    // Name index (3 bytes) + flags (1 byte)
                    uint32_t nameAndFlags = read<uint32_t>(wmoData, offset);
                    doodad.nameIndex = nameAndFlags & 0x00FFFFFF;

                    doodad.position.x = read<float>(wmoData, offset);
                    doodad.position.y = read<float>(wmoData, offset);
                    doodad.position.z = read<float>(wmoData, offset);

                    // C4Quaternion in file: x, y, z, w
                    doodad.rotation.x = read<float>(wmoData, offset);
                    doodad.rotation.y = read<float>(wmoData, offset);
                    doodad.rotation.z = read<float>(wmoData, offset);
                    doodad.rotation.w = read<float>(wmoData, offset);

                    doodad.scale = read<float>(wmoData, offset);

                    uint32_t color = read<uint32_t>(wmoData, offset);
                    doodad.color.b = ((color >> 0) & 0xFF) / 255.0f;
                    doodad.color.g = ((color >> 8) & 0xFF) / 255.0f;
                    doodad.color.r = ((color >> 16) & 0xFF) / 255.0f;
                    doodad.color.a = ((color >> 24) & 0xFF) / 255.0f;

                    model.doodads.push_back(doodad);
                }
                core::Logger::getInstance().info("WMO doodads: ", model.doodads.size());
                break;
            }

            case MODS: {
                // Doodad sets
                uint32_t nSets = chunkSize / 32;  // Each set is 32 bytes
                for (uint32_t i = 0; i < nSets; i++) {
                    WMODoodadSet set;
                    std::memcpy(set.name, &wmoData[offset], 20);
                    offset += 20;
                    set.startIndex = read<uint32_t>(wmoData, offset);
                    set.count = read<uint32_t>(wmoData, offset);
                    set.padding = read<uint32_t>(wmoData, offset);

                    model.doodadSets.push_back(set);
                }
                core::Logger::getInstance().info("WMO doodad sets: ", model.doodadSets.size());
                break;
            }

            case MOPV: {
                // Portal vertices
                uint32_t nVerts = chunkSize / 12;  // Each vertex is 3 floats
                for (uint32_t i = 0; i < nVerts; i++) {
                    glm::vec3 vert;
                    vert.x = read<float>(wmoData, offset);
                    vert.y = read<float>(wmoData, offset);
                    vert.z = read<float>(wmoData, offset);
                    model.portalVertices.push_back(vert);
                }
                break;
            }

            case MOPT: {
                // Portal info
                uint32_t nPortals = chunkSize / 20;  // Each portal reference is 20 bytes
                for (uint32_t i = 0; i < nPortals; i++) {
                    WMOPortal portal;
                    portal.startVertex = read<uint16_t>(wmoData, offset);
                    portal.vertexCount = read<uint16_t>(wmoData, offset);
                    portal.planeIndex = read<uint16_t>(wmoData, offset);
                    portal.padding = read<uint16_t>(wmoData, offset);

                    // Skip additional data (12 bytes)
                    offset += 12;

                    model.portals.push_back(portal);
                }
                core::Logger::getInstance().info("WMO portals: ", model.portals.size());
                break;
            }

            case MOPR: {
                // Portal references - links groups via portals
                uint32_t nRefs = chunkSize / 8;  // Each reference is 8 bytes
                for (uint32_t i = 0; i < nRefs; i++) {
                    WMOPortalRef ref;
                    ref.portalIndex = read<uint16_t>(wmoData, offset);
                    ref.groupIndex = read<uint16_t>(wmoData, offset);
                    ref.side = read<int16_t>(wmoData, offset);
                    ref.padding = read<uint16_t>(wmoData, offset);
                    model.portalRefs.push_back(ref);
                }
                core::Logger::getInstance().info("WMO portal refs: ", model.portalRefs.size());
                break;
            }

            default:
                // Unknown chunk, skip it
                break;
        }

        offset = chunkEnd;
    }

    // Initialize groups array
    model.groups.resize(model.nGroups);

    // WMO loaded log disabled
    return model;
}

bool WMOLoader::loadGroup(const std::vector<uint8_t>& groupData,
                          WMOModel& model,
                          uint32_t groupIndex) {
    if (groupIndex >= model.groups.size()) {
        core::Logger::getInstance().error("Invalid group index: ", groupIndex);
        return false;
    }

    if (groupData.size() < 20) {
        core::Logger::getInstance().error("WMO group file too small");
        return false;
    }

    auto& group = model.groups[groupIndex];
    group.groupId = groupIndex;

    uint32_t offset = 0;

    // Parse chunks in group file
    while (offset + 8 < groupData.size()) {
        uint32_t chunkId = read<uint32_t>(groupData, offset);
        uint32_t chunkSize = read<uint32_t>(groupData, offset);
        uint32_t chunkEnd = offset + chunkSize;

        if (chunkEnd > groupData.size()) {
            break;
        }

        if (chunkId == MVER) {
            // Version - skip
        }
        else if (chunkId == MOGP) {
            // Group header - parse sub-chunks
            // MOGP header is 68 bytes, followed by sub-chunks
            if (chunkSize < 68) {
                offset = chunkEnd;
                continue;
            }

            // Read MOGP header
            uint32_t mogpOffset = offset;
            group.flags = read<uint32_t>(groupData, mogpOffset);
            bool isInterior = (group.flags & 0x2000) != 0;
            core::Logger::getInstance().info("  Group flags: 0x", std::hex, group.flags, std::dec,
                                             (isInterior ? " (INTERIOR)" : " (exterior)"));
            group.boundingBoxMin.x = read<float>(groupData, mogpOffset);
            group.boundingBoxMin.y = read<float>(groupData, mogpOffset);
            group.boundingBoxMin.z = read<float>(groupData, mogpOffset);
            group.boundingBoxMax.x = read<float>(groupData, mogpOffset);
            group.boundingBoxMax.y = read<float>(groupData, mogpOffset);
            group.boundingBoxMax.z = read<float>(groupData, mogpOffset);
            mogpOffset += 4; // nameOffset
            group.portalStart = read<uint16_t>(groupData, mogpOffset);
            group.portalCount = read<uint16_t>(groupData, mogpOffset);
            mogpOffset += 8; // transBatchCount, intBatchCount, extBatchCount, padding
            group.fogIndices[0] = read<uint32_t>(groupData, mogpOffset);
            group.fogIndices[1] = read<uint32_t>(groupData, mogpOffset);
            group.fogIndices[2] = read<uint32_t>(groupData, mogpOffset);
            group.fogIndices[3] = read<uint32_t>(groupData, mogpOffset);
            group.liquidType = read<uint32_t>(groupData, mogpOffset);
            // Skip to end of 68-byte header
            mogpOffset = offset + 68;

            // Parse sub-chunks within MOGP
            while (mogpOffset + 8 < chunkEnd) {
                uint32_t subChunkId = read<uint32_t>(groupData, mogpOffset);
                uint32_t subChunkSize = read<uint32_t>(groupData, mogpOffset);
                uint32_t subChunkEnd = mogpOffset + subChunkSize;

                if (subChunkEnd > chunkEnd) {
                    break;
                }

                // Debug: log chunk magic as string
                char magic[5] = {0};
                magic[0] = (subChunkId >> 0) & 0xFF;
                magic[1] = (subChunkId >> 8) & 0xFF;
                magic[2] = (subChunkId >> 16) & 0xFF;
                magic[3] = (subChunkId >> 24) & 0xFF;
                static int logCount = 0;
                if (logCount < 30) {
                    core::Logger::getInstance().debug("  WMO sub-chunk: ", magic, " (0x", std::hex, subChunkId, std::dec, ") size=", subChunkSize);
                    logCount++;
                }

                if (subChunkId == 0x4D4F5654) { // MOVT - Vertices
                    uint32_t vertexCount = subChunkSize / 12; // 3 floats per vertex
                    for (uint32_t i = 0; i < vertexCount; i++) {
                        WMOVertex vertex;
                        // Keep vertices in WoW model-local coords - coordinate swap done in model matrix
                        vertex.position.x = read<float>(groupData, mogpOffset);
                        vertex.position.y = read<float>(groupData, mogpOffset);
                        vertex.position.z = read<float>(groupData, mogpOffset);
                        vertex.normal = glm::vec3(0, 0, 1);
                        vertex.texCoord = glm::vec2(0, 0);
                        vertex.color = glm::vec4(1, 1, 1, 1);
                        group.vertices.push_back(vertex);
                    }
                }
                else if (subChunkId == 0x4D4F5649) { // MOVI - Indices
                    uint32_t indexCount = subChunkSize / 2; // uint16_t per index
                    for (uint32_t i = 0; i < indexCount; i++) {
                        group.indices.push_back(read<uint16_t>(groupData, mogpOffset));
                    }
                }
                else if (subChunkId == 0x4D4F4E52) { // MONR - Normals
                    uint32_t normalCount = subChunkSize / 12;
                    core::Logger::getInstance().info("  MONR: ", normalCount, " normals for ", group.vertices.size(), " vertices");
                    for (uint32_t i = 0; i < normalCount && i < group.vertices.size(); i++) {
                        group.vertices[i].normal.x = read<float>(groupData, mogpOffset);
                        group.vertices[i].normal.y = read<float>(groupData, mogpOffset);
                        group.vertices[i].normal.z = read<float>(groupData, mogpOffset);
                    }
                    if (normalCount > 0 && !group.vertices.empty()) {
                        const auto& n = group.vertices[0].normal;
                        core::Logger::getInstance().debug("    First normal: (", n.x, ", ", n.y, ", ", n.z, ")");
                    }
                }
                else if (subChunkId == 0x4D4F5456) { // MOTV - Texture coords
                    // Update texture coords for existing vertices
                    uint32_t texCoordCount = subChunkSize / 8;
                    core::Logger::getInstance().info("  MOTV: ", texCoordCount, " tex coords for ", group.vertices.size(), " vertices");
                    for (uint32_t i = 0; i < texCoordCount && i < group.vertices.size(); i++) {
                        group.vertices[i].texCoord.x = read<float>(groupData, mogpOffset);
                        group.vertices[i].texCoord.y = read<float>(groupData, mogpOffset);
                    }
                    if (texCoordCount > 0 && !group.vertices.empty()) {
                        core::Logger::getInstance().debug("    First UV: (", group.vertices[0].texCoord.x, ", ", group.vertices[0].texCoord.y, ")");
                    }
                }
                else if (subChunkId == 0x4D4F4356) { // MOCV - Vertex colors
                    // Update vertex colors
                    uint32_t colorCount = subChunkSize / 4;
                    core::Logger::getInstance().info("  MOCV: ", colorCount, " vertex colors for ", group.vertices.size(), " vertices");
                    for (uint32_t i = 0; i < colorCount && i < group.vertices.size(); i++) {
                        uint8_t b = read<uint8_t>(groupData, mogpOffset);
                        uint8_t g = read<uint8_t>(groupData, mogpOffset);
                        uint8_t r = read<uint8_t>(groupData, mogpOffset);
                        uint8_t a = read<uint8_t>(groupData, mogpOffset);
                        group.vertices[i].color = glm::vec4(r/255.0f, g/255.0f, b/255.0f, a/255.0f);
                    }
                    if (colorCount > 0 && !group.vertices.empty()) {
                        const auto& c = group.vertices[0].color;
                        core::Logger::getInstance().debug("    First color: (", c.r, ", ", c.g, ", ", c.b, ", ", c.a, ")");
                    }
                }
                else if (subChunkId == 0x4D4F4241) { // MOBA - Batches
                    // SMOBatch structure (24 bytes):
                    // - 6 x int16 bounding box (12 bytes)
                    // - uint32 startIndex (4 bytes)
                    // - uint16 count (2 bytes)
                    // - uint16 minIndex (2 bytes)
                    // - uint16 maxIndex (2 bytes)
                    // - uint8 flags (1 byte)
                    // - uint8 material_id (1 byte)
                    uint32_t batchCount = subChunkSize / 24;
                    for (uint32_t i = 0; i < batchCount; i++) {
                        WMOBatch batch;
                        mogpOffset += 12; // Skip bounding box (6 x int16 = 12 bytes)
                        batch.startIndex = read<uint32_t>(groupData, mogpOffset);
                        batch.indexCount = read<uint16_t>(groupData, mogpOffset);
                        batch.startVertex = read<uint16_t>(groupData, mogpOffset);
                        batch.lastVertex = read<uint16_t>(groupData, mogpOffset);
                        batch.flags = read<uint8_t>(groupData, mogpOffset);
                        batch.materialId = read<uint8_t>(groupData, mogpOffset);
                        group.batches.push_back(batch);

                        static int batchLogCount = 0;
                        if (batchLogCount < 15) {
                            core::Logger::getInstance().info("  Batch[", i, "]: start=", batch.startIndex,
                                " count=", batch.indexCount, " verts=[", batch.startVertex, "-",
                                batch.lastVertex, "] mat=", (int)batch.materialId, " flags=", (int)batch.flags);
                            batchLogCount++;
                        }
                    }
                }
                else if (subChunkId == MLIQ) { // MLIQ - WMO liquid data
                    // Basic WotLK layout:
                    // uint32 xVerts, yVerts, xTiles, yTiles
                    // float  baseX, baseY, baseZ
                    // uint16 materialId
                    // (optional pad/unknown bytes)
                    // followed by vertex/tile payload
                    uint32_t parseOffset = mogpOffset;
                    if (parseOffset + 30 <= subChunkEnd) {
                        group.liquid.xVerts = read<uint32_t>(groupData, parseOffset);
                        group.liquid.yVerts = read<uint32_t>(groupData, parseOffset);
                        group.liquid.xTiles = read<uint32_t>(groupData, parseOffset);
                        group.liquid.yTiles = read<uint32_t>(groupData, parseOffset);
                        group.liquid.basePosition.x = read<float>(groupData, parseOffset);
                        group.liquid.basePosition.y = read<float>(groupData, parseOffset);
                        group.liquid.basePosition.z = read<float>(groupData, parseOffset);
                        group.liquid.materialId = read<uint16_t>(groupData, parseOffset);
                        if (parseOffset + sizeof(uint16_t) <= subChunkEnd) {
                            // Reserved/flags in some WMO liquid variants.
                            parseOffset += sizeof(uint16_t);
                        }

                        // Keep parser resilient across minor format variants:
                        // prefer explicit per-vertex floats, otherwise fall back to flat.
                        const size_t vertexCount =
                            static_cast<size_t>(group.liquid.xVerts) * static_cast<size_t>(group.liquid.yVerts);
                        const size_t tileCount =
                            static_cast<size_t>(group.liquid.xTiles) * static_cast<size_t>(group.liquid.yTiles);
                        const size_t bytesRemaining = (subChunkEnd > parseOffset) ? (subChunkEnd - parseOffset) : 0;

                        group.liquid.heights.clear();
                        group.liquid.flags.clear();

                        if (vertexCount > 0 && bytesRemaining >= vertexCount * sizeof(float)) {
                            group.liquid.heights.resize(vertexCount);
                            for (size_t i = 0; i < vertexCount; i++) {
                                group.liquid.heights[i] = read<float>(groupData, parseOffset);
                            }
                        } else if (vertexCount > 0) {
                            group.liquid.heights.resize(vertexCount, group.liquid.basePosition.z);
                        }

                        if (tileCount > 0 && parseOffset + tileCount <= subChunkEnd) {
                            group.liquid.flags.resize(tileCount);
                            std::memcpy(group.liquid.flags.data(), &groupData[parseOffset], tileCount);
                        } else if (tileCount > 0) {
                            group.liquid.flags.resize(tileCount, 0);
                        }

                        if (group.liquid.materialId == 0) {
                            group.liquid.materialId = static_cast<uint16_t>(group.liquidType);
                        }
                    }
                }

                mogpOffset = subChunkEnd;
            }
        }

        offset = chunkEnd;
    }

    // Create a default batch if none were loaded
    if (group.batches.empty() && !group.indices.empty()) {
        WMOBatch batch;
        batch.startIndex = 0;
        batch.indexCount = static_cast<uint16_t>(group.indices.size());
        batch.materialId = 0;
        group.batches.push_back(batch);
    }

    core::Logger::getInstance().info("WMO group ", groupIndex, " loaded: ",
                                     group.vertices.size(), " vertices, ",
                                     group.indices.size(), " indices, ",
                                     group.batches.size(), " batches");
    return !group.vertices.empty() && !group.indices.empty();
}

} // namespace pipeline
} // namespace wowee
