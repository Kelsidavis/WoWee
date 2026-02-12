#include "pipeline/adt_loader.hpp"
#include "core/logger.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace wowee {
namespace pipeline {

// HeightMap implementation
float HeightMap::getHeight(int x, int y) const {
    if (x < 0 || x > 8 || y < 0 || y > 8) {
        return 0.0f;
    }

    // WoW uses 9x9 outer + 8x8 inner vertex layout
    // Outer vertices: 0-80 (9x9 grid)
    // Inner vertices: 81-144 (8x8 grid between outer vertices)

    // Calculate index based on vertex type
    int index;
    if (x < 9 && y < 9) {
        // Outer vertex
        index = y * 9 + x;
    } else {
        // Inner vertex (between outer vertices)
        int innerX = x - 1;
        int innerY = y - 1;
        if (innerX >= 0 && innerX < 8 && innerY >= 0 && innerY < 8) {
            index = 81 + innerY * 8 + innerX;
        } else {
            return 0.0f;
        }
    }

    return heights[index];
}

// ADTLoader implementation
ADTTerrain ADTLoader::load(const std::vector<uint8_t>& adtData) {
    ADTTerrain terrain;

    if (adtData.empty()) {
        LOG_ERROR("Empty ADT data");
        return terrain;
    }

    LOG_DEBUG("Loading ADT terrain (", adtData.size(), " bytes)");

    size_t offset = 0;
    int chunkIndex = 0;

    // Parse chunks
    int totalChunks = 0;
    while (offset < adtData.size()) {
        ChunkHeader header;
        if (!readChunkHeader(adtData.data(), offset, adtData.size(), header)) {
            break;
        }

        const uint8_t* chunkData = adtData.data() + offset + 8;
        size_t chunkSize = header.size;

        totalChunks++;
        if (totalChunks <= 5) {
            // Log first few chunks for debugging
            char magic[5] = {0};
            std::memcpy(magic, &header.magic, 4);
        }

        // Parse based on chunk type
        if (header.magic == MVER) {
            parseMVER(chunkData, chunkSize, terrain);
        }
        else if (header.magic == MTEX) {
            parseMTEX(chunkData, chunkSize, terrain);
        }
        else if (header.magic == MMDX) {
            parseMMDX(chunkData, chunkSize, terrain);
        }
        else if (header.magic == MWMO) {
            parseMWMO(chunkData, chunkSize, terrain);
        }
        else if (header.magic == MDDF) {
            parseMDDF(chunkData, chunkSize, terrain);
        }
        else if (header.magic == MODF) {
            parseMODF(chunkData, chunkSize, terrain);
        }
        else if (header.magic == MH2O) {
            LOG_DEBUG("Found MH2O chunk (", chunkSize, " bytes)");
            parseMH2O(chunkData, chunkSize, terrain);
        }
        else if (header.magic == MCNK) {
            parseMCNK(chunkData, chunkSize, chunkIndex++, terrain);
        }

        // Move to next chunk
        offset += 8 + chunkSize;
    }

    terrain.loaded = true;

    return terrain;
}

bool ADTLoader::readChunkHeader(const uint8_t* data, size_t offset, size_t dataSize, ChunkHeader& header) {
    if (offset + 8 > dataSize) {
        return false;
    }

    header.magic = readUInt32(data, offset);
    header.size = readUInt32(data, offset + 4);

    // Validate chunk size
    if (offset + 8 + header.size > dataSize) {
        LOG_WARNING("Chunk extends beyond file: magic=0x", std::hex, header.magic,
                    ", size=", std::dec, header.size);
        return false;
    }

    return true;
}

uint32_t ADTLoader::readUInt32(const uint8_t* data, size_t offset) {
    uint32_t value;
    std::memcpy(&value, data + offset, sizeof(uint32_t));
    return value;
}

float ADTLoader::readFloat(const uint8_t* data, size_t offset) {
    float value;
    std::memcpy(&value, data + offset, sizeof(float));
    return value;
}

uint16_t ADTLoader::readUInt16(const uint8_t* data, size_t offset) {
    uint16_t value;
    std::memcpy(&value, data + offset, sizeof(uint16_t));
    return value;
}

void ADTLoader::parseMVER(const uint8_t* data, size_t size, ADTTerrain& terrain) {
    if (size < 4) {
        LOG_WARNING("MVER chunk too small");
        return;
    }

    terrain.version = readUInt32(data, 0);
    LOG_DEBUG("ADT version: ", terrain.version);
}

void ADTLoader::parseMTEX(const uint8_t* data, size_t size, ADTTerrain& terrain) {
    // MTEX contains null-terminated texture filenames
    size_t offset = 0;

    while (offset < size) {
        const char* textureName = reinterpret_cast<const char*>(data + offset);
        size_t nameLen = std::strlen(textureName);

        if (nameLen == 0) {
            break;
        }

        terrain.textures.push_back(std::string(textureName, nameLen));
        offset += nameLen + 1;  // +1 for null terminator
    }

    LOG_DEBUG("Loaded ", terrain.textures.size(), " texture names");
}

void ADTLoader::parseMMDX(const uint8_t* data, size_t size, ADTTerrain& terrain) {
    // MMDX contains null-terminated M2 model filenames
    size_t offset = 0;

    while (offset < size) {
        const char* modelName = reinterpret_cast<const char*>(data + offset);
        size_t nameLen = std::strlen(modelName);

        if (nameLen == 0) {
            break;
        }

        terrain.doodadNames.push_back(std::string(modelName, nameLen));
        offset += nameLen + 1;
    }

    LOG_DEBUG("Loaded ", terrain.doodadNames.size(), " doodad names");
}

void ADTLoader::parseMWMO(const uint8_t* data, size_t size, ADTTerrain& terrain) {
    // MWMO contains null-terminated WMO filenames
    size_t offset = 0;

    while (offset < size) {
        const char* wmoName = reinterpret_cast<const char*>(data + offset);
        size_t nameLen = std::strlen(wmoName);

        if (nameLen == 0) {
            break;
        }

        terrain.wmoNames.push_back(std::string(wmoName, nameLen));
        offset += nameLen + 1;
    }

    LOG_DEBUG("Loaded ", terrain.wmoNames.size(), " WMO names from MWMO chunk");
    for (size_t i = 0; i < terrain.wmoNames.size(); i++) {
        LOG_DEBUG("  WMO[", i, "]: ", terrain.wmoNames[i]);
        // Flag potential duplicate cathedral models
        if (terrain.wmoNames[i].find("cathedral") != std::string::npos ||
            terrain.wmoNames[i].find("Cathedral") != std::string::npos) {
            LOG_DEBUG("*** CATHEDRAL WMO FOUND: ", terrain.wmoNames[i]);
        }
    }
}

void ADTLoader::parseMDDF(const uint8_t* data, size_t size, ADTTerrain& terrain) {
    // MDDF contains doodad placements (36 bytes each)
    const size_t entrySize = 36;
    size_t count = size / entrySize;

    for (size_t i = 0; i < count; i++) {
        size_t offset = i * entrySize;

        ADTTerrain::DoodadPlacement placement;
        placement.nameId = readUInt32(data, offset);
        placement.uniqueId = readUInt32(data, offset + 4);
        placement.position[0] = readFloat(data, offset + 8);
        placement.position[1] = readFloat(data, offset + 12);
        placement.position[2] = readFloat(data, offset + 16);
        placement.rotation[0] = readFloat(data, offset + 20);
        placement.rotation[1] = readFloat(data, offset + 24);
        placement.rotation[2] = readFloat(data, offset + 28);
        placement.scale = readUInt16(data, offset + 32);
        placement.flags = readUInt16(data, offset + 34);

        terrain.doodadPlacements.push_back(placement);
    }

    LOG_DEBUG("Loaded ", terrain.doodadPlacements.size(), " doodad placements");
}

void ADTLoader::parseMODF(const uint8_t* data, size_t size, ADTTerrain& terrain) {
    // MODF contains WMO placements (64 bytes each)
    const size_t entrySize = 64;
    size_t count = size / entrySize;

    for (size_t i = 0; i < count; i++) {
        size_t offset = i * entrySize;

        ADTTerrain::WMOPlacement placement;
        placement.nameId = readUInt32(data, offset);
        placement.uniqueId = readUInt32(data, offset + 4);
        placement.position[0] = readFloat(data, offset + 8);
        placement.position[1] = readFloat(data, offset + 12);
        placement.position[2] = readFloat(data, offset + 16);
        placement.rotation[0] = readFloat(data, offset + 20);
        placement.rotation[1] = readFloat(data, offset + 24);
        placement.rotation[2] = readFloat(data, offset + 28);
        placement.extentLower[0] = readFloat(data, offset + 32);
        placement.extentLower[1] = readFloat(data, offset + 36);
        placement.extentLower[2] = readFloat(data, offset + 40);
        placement.extentUpper[0] = readFloat(data, offset + 44);
        placement.extentUpper[1] = readFloat(data, offset + 48);
        placement.extentUpper[2] = readFloat(data, offset + 52);
        placement.flags = readUInt16(data, offset + 56);
        placement.doodadSet = readUInt16(data, offset + 58);

        terrain.wmoPlacements.push_back(placement);

        // Log STORMWIND.WMO placements to detect duplicates at different Z heights
        if (placement.nameId < terrain.wmoNames.size()) {
            const std::string& wmoName = terrain.wmoNames[placement.nameId];
            std::string upperName = wmoName;
            std::transform(upperName.begin(), upperName.end(), upperName.begin(), ::toupper);

            if (upperName.find("STORMWIND.WMO") != std::string::npos) {
                LOG_DEBUG("*** STORMWIND.WMO PLACEMENT:",
                         " uniqueId=", placement.uniqueId,
                         " pos=(", placement.position[0], ", ", placement.position[1], ", ", placement.position[2], ")",
                         " rot=(", placement.rotation[0], ", ", placement.rotation[1], ", ", placement.rotation[2], ")",
                         " doodadSet=", placement.doodadSet,
                         " flags=0x", std::hex, placement.flags, std::dec);
            }
        }
    }

    LOG_DEBUG("Loaded ", terrain.wmoPlacements.size(), " WMO placements");
}

void ADTLoader::parseMCNK(const uint8_t* data, size_t size, int chunkIndex, ADTTerrain& terrain) {
    if (chunkIndex < 0 || chunkIndex >= 256) {
        LOG_WARNING("Invalid chunk index: ", chunkIndex);
        return;
    }

    MapChunk& chunk = terrain.chunks[chunkIndex];

    // Read MCNK header (128 bytes)
    if (size < 128) {
        LOG_WARNING("MCNK chunk too small");
        return;
    }

    chunk.flags = readUInt32(data, 0);
    chunk.indexX = readUInt32(data, 4);
    chunk.indexY = readUInt32(data, 8);

    // Read holes mask (at offset 0x3C = 60 in MCNK header)
    // Each bit represents a 2x2 block of the 8x8 quad grid
    chunk.holes = readUInt16(data, 60);

    // Read layer count and offsets from MCNK header
    uint32_t nLayers = readUInt32(data, 12);
    uint32_t ofsHeight = readUInt32(data, 20);   // MCVT offset
    uint32_t ofsNormal = readUInt32(data, 24);   // MCNR offset
    uint32_t ofsLayer = readUInt32(data, 28);    // MCLY offset
    uint32_t ofsAlpha = readUInt32(data, 36);    // MCAL offset
    uint32_t sizeAlpha = readUInt32(data, 40);

    // Debug first chunk only
    if (chunkIndex == 0) {
        LOG_DEBUG("MCNK[0] offsets: nLayers=", nLayers,
                 " height=", ofsHeight, " normal=", ofsNormal,
                 " layer=", ofsLayer, " alpha=", ofsAlpha,
                 " sizeAlpha=", sizeAlpha, " size=", size,
                 " holes=0x", std::hex, chunk.holes, std::dec);
    }

    // Position (stored at offset 0x68 = 104 in MCNK header)
    chunk.position[0] = readFloat(data, 104);  // X
    chunk.position[1] = readFloat(data, 108);  // Y
    chunk.position[2] = readFloat(data, 112);  // Z

    // Parse sub-chunks using offsets from MCNK header
    // WoW ADT sub-chunks may have their own 8-byte headers (magic+size)
    // Check by inspecting the first 4 bytes at the offset

    // Height map (MCVT) - 145 floats = 580 bytes
    if (ofsHeight > 0 && ofsHeight + 580 <= size) {
        // Check if this points to a sub-chunk header (magic "MCVT" = 0x4D435654)
        uint32_t possibleMagic = readUInt32(data, ofsHeight);
        uint32_t headerSkip = 0;
        if (possibleMagic == MCVT) {
            headerSkip = 8;  // Skip magic + size
            if (chunkIndex == 0) {
                LOG_DEBUG("MCNK sub-chunks have headers (MCVT magic found at offset ", ofsHeight, ")");
            }
        }
        parseMCVT(data + ofsHeight + headerSkip, 580, chunk);
    }

    // Normals (MCNR) - 145 normals (3 bytes each) + 13 padding = 448 bytes
    if (ofsNormal > 0 && ofsNormal + 448 <= size) {
        uint32_t possibleMagic = readUInt32(data, ofsNormal);
        uint32_t skip = (possibleMagic == MCNR) ? 8 : 0;
        parseMCNR(data + ofsNormal + skip, 448, chunk);
    }

    // Texture layers (MCLY) - 16 bytes per layer
    if (ofsLayer > 0 && nLayers > 0) {
        size_t layerSize = nLayers * 16;
        uint32_t possibleMagic = readUInt32(data, ofsLayer);
        uint32_t skip = (possibleMagic == MCLY) ? 8 : 0;
        if (ofsLayer + skip + layerSize <= size) {
            parseMCLY(data + ofsLayer + skip, layerSize, chunk);
        }
    }

    // Alpha maps (MCAL) - variable size from header
    if (ofsAlpha > 0 && sizeAlpha > 0 && ofsAlpha + sizeAlpha <= size) {
        uint32_t possibleMagic = readUInt32(data, ofsAlpha);
        uint32_t skip = (possibleMagic == MCAL) ? 8 : 0;
        parseMCAL(data + ofsAlpha + skip, sizeAlpha - skip, chunk);
    }
}

void ADTLoader::parseMCVT(const uint8_t* data, size_t size, MapChunk& chunk) {
    // MCVT contains 145 height values (floats)
    if (size < 145 * sizeof(float)) {
        LOG_WARNING("MCVT chunk too small: ", size, " bytes");
        return;
    }

    float minHeight = 999999.0f;
    float maxHeight = -999999.0f;

    for (int i = 0; i < 145; i++) {
        float height = readFloat(data, i * sizeof(float));
        chunk.heightMap.heights[i] = height;

        if (height < minHeight) minHeight = height;
        if (height > maxHeight) maxHeight = height;
    }
    chunk.heightMap.loaded = true;

    // Log height range for first chunk only
    static bool logged = false;
    if (!logged) {
        LOG_DEBUG("MCVT height range: [", minHeight, ", ", maxHeight, "]");
        logged = true;
    }
}

void ADTLoader::parseMCNR(const uint8_t* data, size_t size, MapChunk& chunk) {
    // MCNR contains 145 normals (3 bytes each, signed)
    if (size < 145 * 3) {
        LOG_WARNING("MCNR chunk too small: ", size, " bytes");
        return;
    }

    for (int i = 0; i < 145 * 3; i++) {
        chunk.normals[i] = static_cast<int8_t>(data[i]);
    }
}

void ADTLoader::parseMCLY(const uint8_t* data, size_t size, MapChunk& chunk) {
    // MCLY contains texture layer definitions (16 bytes each)
    size_t layerCount = size / 16;

    if (layerCount > 4) {
        LOG_WARNING("More than 4 texture layers: ", layerCount);
        layerCount = 4;
    }

    static int layerLogCount = 0;
    for (size_t i = 0; i < layerCount; i++) {
        TextureLayer layer;

        layer.textureId = readUInt32(data, i * 16 + 0);
        layer.flags = readUInt32(data, i * 16 + 4);
        layer.offsetMCAL = readUInt32(data, i * 16 + 8);
        layer.effectId = readUInt32(data, i * 16 + 12);

        if (layerLogCount < 10) {
            LOG_DEBUG("  MCLY[", i, "]: texId=", layer.textureId,
                     " flags=0x", std::hex, layer.flags, std::dec,
                     " alphaOfs=", layer.offsetMCAL,
                     " useAlpha=", layer.useAlpha(),
                     " compressed=", layer.compressedAlpha());
            layerLogCount++;
        }

        chunk.layers.push_back(layer);
    }
}

void ADTLoader::parseMCAL(const uint8_t* data, size_t size, MapChunk& chunk) {
    // MCAL contains alpha maps for texture layers
    // Store raw data; decompression happens per-layer during mesh generation
    chunk.alphaMap.resize(size);
    std::memcpy(chunk.alphaMap.data(), data, size);
}

void ADTLoader::parseMH2O(const uint8_t* data, size_t size, ADTTerrain& terrain) {
    // MH2O contains water/liquid data for all 256 map chunks
    // Structure: 256 SMLiquidChunk headers followed by instance data

    // Each SMLiquidChunk header is 12 bytes (WotLK 3.3.5a):
    // - uint32_t offsetInstances (offset from MH2O chunk start)
    // - uint32_t layerCount
    // - uint32_t offsetAttributes (offset from MH2O chunk start)

    const size_t headerSize = 12;  // SMLiquidChunk size for WotLK
    const size_t totalHeaderSize = 256 * headerSize;

    if (size < totalHeaderSize) {
        LOG_WARNING("MH2O chunk too small for headers: ", size, " bytes");
        return;
    }

    int totalLayers = 0;

    for (int chunkIdx = 0; chunkIdx < 256; chunkIdx++) {
        size_t headerOffset = chunkIdx * headerSize;

        uint32_t offsetInstances = readUInt32(data, headerOffset);
        uint32_t layerCount = readUInt32(data, headerOffset + 4);
        // uint32_t offsetAttributes = readUInt32(data, headerOffset + 8);  // Not used

        if (layerCount == 0 || offsetInstances == 0) {
            continue;  // No water in this chunk
        }

        // Sanity checks
        if (offsetInstances >= size) {
            continue;
        }
        if (layerCount > 16) {
            // Sanity check - max 16 layers per chunk is reasonable
            LOG_WARNING("MH2O: Invalid layer count ", layerCount, " for chunk ", chunkIdx);
            continue;
        }

        // Parse each liquid layer (SMLiquidInstance - 24 bytes)
        for (uint32_t layerIdx = 0; layerIdx < layerCount; layerIdx++) {
            size_t instanceOffset = offsetInstances + layerIdx * 24;

            if (instanceOffset + 24 > size) {
                break;
            }

            ADTTerrain::WaterLayer layer;
            layer.liquidType = readUInt16(data, instanceOffset);
            uint16_t liquidObject = readUInt16(data, instanceOffset + 2);  // LVF format flags
            layer.minHeight = readFloat(data, instanceOffset + 4);
            layer.maxHeight = readFloat(data, instanceOffset + 8);
            layer.x = data[instanceOffset + 12];
            layer.y = data[instanceOffset + 13];
            layer.width = data[instanceOffset + 14];
            layer.height = data[instanceOffset + 15];
            uint32_t offsetExistsBitmap = readUInt32(data, instanceOffset + 16);
            uint32_t offsetVertexData = readUInt32(data, instanceOffset + 20);

            // Skip invalid layers
            if (layer.width == 0 || layer.height == 0) {
                continue;
            }

            // Clamp dimensions to valid range
            if (layer.width > 8) layer.width = 8;
            if (layer.height > 8) layer.height = 8;
            if (layer.x + layer.width > 8) layer.width = 8 - layer.x;
            if (layer.y + layer.height > 8) layer.height = 8 - layer.y;

            // Read exists bitmap (which tiles have water).
            // In WotLK MH2O this is chunk-wide 8x8 tile flags (64 bits = 8 bytes),
            // even when the layer covers a sub-rect.
            constexpr size_t bitmapBytes = 8;

            // Note: offsets in SMLiquidInstance are relative to MH2O chunk start
            if (offsetExistsBitmap > 0) {
                size_t bitmapOffset = offsetExistsBitmap;
                if (bitmapOffset + bitmapBytes <= size) {
                    layer.mask.resize(bitmapBytes);
                    std::memcpy(layer.mask.data(), data + bitmapOffset, bitmapBytes);
                }
            } else {
                // No bitmap means all tiles in chunk are valid for this layer.
                layer.mask.resize(bitmapBytes, 0xFF);
            }

            // Read vertex heights
            // Number of vertices is (width+1) * (height+1)
            size_t numVertices = (layer.width + 1) * (layer.height + 1);

            // Check liquid object flags (LVF) to determine vertex format
            bool hasHeightData = (liquidObject != 2);  // LVF_height_depth or LVF_height_texcoord

            if (hasHeightData && offsetVertexData > 0) {
                size_t vertexOffset = offsetVertexData;
                size_t vertexDataSize = numVertices * sizeof(float);

                if (vertexOffset + vertexDataSize <= size) {
                    layer.heights.resize(numVertices);
                    for (size_t i = 0; i < numVertices; i++) {
                        layer.heights[i] = readFloat(data, vertexOffset + i * sizeof(float));
                    }
                } else {
                    // Offset out of bounds - use flat water
                    layer.heights.resize(numVertices, layer.minHeight);
                }
            } else {
                // No height data - use flat surface at minHeight
                layer.heights.resize(numVertices, layer.minHeight);
            }

            // Default flags
            layer.flags = 0;

            terrain.waterData[chunkIdx].layers.push_back(layer);
            totalLayers++;
        }
    }

    LOG_DEBUG("Loaded MH2O water data: ", totalLayers, " liquid layers across ", size, " bytes");
}

} // namespace pipeline
} // namespace wowee
