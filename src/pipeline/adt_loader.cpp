#include "pipeline/adt_loader.hpp"

#include <span>
#include "core/logger.hpp"
#include "core/profiler.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <limits>

namespace wowee {
namespace pipeline {

// MCVT height grid: 9 outer + 8 inner vertices per row, 9 rows = 145 total.
// Each row is 17 entries: 9 outer corner vertices then 8 inner midpoints.
static constexpr int kMCVTVertexCount = 145;
static constexpr int kMCVTRowStride   = 17;  // 9 outer + 8 inner per row

// HeightMap implementation
float HeightMap::getHeight(int x, int y) const {
    if (x < 0 || x > 8 || y < 0 || y > 8) {
        return 0.0f;
    }

    // Outer vertex (x, y) in the interleaved grid
    int index = y * kMCVTRowStride + x;
    if (index < 0 || index >= kMCVTVertexCount) return 0.0f;

    return heights[index];
}

// ADTLoader implementation
ADTTerrain ADTLoader::load(const std::vector<uint8_t>& adtData) {
    ZoneScopedN("ADTLoader::load");
    ADTTerrain terrain;

    if (adtData.empty()) {
        LOG_ERROR("Empty ADT data");
        return terrain;
    }

    LOG_DEBUG("Loading ADT terrain (", adtData.size(), " bytes)");

    size_t offset = 0;
    int chunkIndex = 0;

    // Parse chunks
    while (offset < adtData.size()) {
        ChunkHeader header;
        if (!readChunkHeader(adtData, offset, header)) {
            break;
        }

        const size_t chunkSize = header.size;
        const std::span<const uint8_t> chunkData(adtData.data() + offset + 8, chunkSize);

        // Parse based on chunk type
        if (header.magic == MVER) {
            parseMVER(chunkData, terrain);
        }
        else if (header.magic == MTEX) {
            parseMTEX(chunkData, terrain);
        }
        else if (header.magic == MMDX) {
            parseMMDX(chunkData, terrain);
        }
        else if (header.magic == MWMO) {
            parseMWMO(chunkData, terrain);
        }
        else if (header.magic == MDDF) {
            parseMDDF(chunkData, terrain);
        }
        else if (header.magic == MODF) {
            parseMODF(chunkData, terrain);
        }
        else if (header.magic == MH2O) {
            LOG_DEBUG("Found MH2O chunk (", chunkSize, " bytes)");
            parseMH2O(chunkData, terrain);
        }
        else if (header.magic == MCNK) {
            parseMCNK(chunkData, chunkIndex++, terrain);
        }

        // Move to next chunk
        offset += 8 + chunkSize;
    }

    terrain.loaded = true;

    return terrain;
}

bool ADTLoader::readChunkHeader(std::span<const uint8_t> data, size_t offset, ChunkHeader& header) {
    if (offset + 8 > data.size()) {
        return false;
    }

    header.magic = readUInt32(data, offset);
    header.size = readUInt32(data, offset + 4);

    // Validate chunk size
    if (offset + 8 + header.size > data.size()) {
        LOG_WARNING("Chunk extends beyond file: magic=0x", std::hex, header.magic,
                    ", size=", std::dec, header.size);
        return false;
    }

    return true;
}

uint32_t ADTLoader::readUInt32(std::span<const uint8_t> data, size_t offset) {
    if (offset + sizeof(uint32_t) > data.size()) {
        return uint32_t{};
    }
    uint32_t value;
    std::memcpy(&value, data.data() + offset, sizeof(uint32_t));
    return value;
}

float ADTLoader::readFloat(std::span<const uint8_t> data, size_t offset) {
    if (offset + sizeof(float) > data.size()) {
        return float{};
    }
    float value;
    std::memcpy(&value, data.data() + offset, sizeof(float));
    return value;
}

uint16_t ADTLoader::readUInt16(std::span<const uint8_t> data, size_t offset) {
    if (offset + sizeof(uint16_t) > data.size()) {
        return uint16_t{};
    }
    uint16_t value;
    std::memcpy(&value, data.data() + offset, sizeof(uint16_t));
    return value;
}

void ADTLoader::parseMVER(std::span<const uint8_t> data, ADTTerrain& terrain) {
    if (data.size() < 4) {
        LOG_WARNING("MVER chunk too small");
        return;
    }

    terrain.version = readUInt32(data, 0);
    LOG_DEBUG("ADT version: ", terrain.version);
}

void ADTLoader::parseMTEX(std::span<const uint8_t> data, ADTTerrain& terrain) {
    // MTEX contains null-terminated texture filenames.
    // Use bounded scan instead of strlen to avoid reading past the chunk
    // boundary if the last string is not null-terminated (truncated file).
    size_t offset = 0;

    while (offset < data.size()) {
        const char* textureName = reinterpret_cast<const char*>(data.data() + offset);
        size_t maxLen = data.size() - offset;
        size_t nameLen = strnlen(textureName, maxLen);

        if (nameLen == 0) {
            break;
        }

        terrain.textures.emplace_back(textureName, nameLen);
        offset += nameLen + 1;  // +1 for null terminator
    }

    LOG_DEBUG("Loaded ", terrain.textures.size(), " texture names");
}

void ADTLoader::parseMMDX(std::span<const uint8_t> data, ADTTerrain& terrain) {
    // MMDX contains null-terminated M2 model filenames
    size_t offset = 0;

    while (offset < data.size()) {
        const char* modelName = reinterpret_cast<const char*>(data.data() + offset);
        size_t nameLen = strnlen(modelName, data.size() - offset);

        if (nameLen == 0) {
            break;
        }

        terrain.doodadNames.emplace_back(modelName, nameLen);
        offset += nameLen + 1;
    }

    LOG_DEBUG("Loaded ", terrain.doodadNames.size(), " doodad names");
}

void ADTLoader::parseMWMO(std::span<const uint8_t> data, ADTTerrain& terrain) {
    // MWMO contains null-terminated WMO filenames
    size_t offset = 0;

    while (offset < data.size()) {
        const char* wmoName = reinterpret_cast<const char*>(data.data() + offset);
        size_t nameLen = strnlen(wmoName, data.size() - offset);

        if (nameLen == 0) {
            break;
        }

        terrain.wmoNames.emplace_back(wmoName, nameLen);
        offset += nameLen + 1;
    }

    LOG_DEBUG("Loaded ", terrain.wmoNames.size(), " WMO names from MWMO chunk");
}

void ADTLoader::parseMDDF(std::span<const uint8_t> data, ADTTerrain& terrain) {
    // MDDF contains doodad placements (36 bytes each)
    const size_t entrySize = 36;
    size_t count = data.size() / entrySize;

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
        // Sanitize NaN/inf - corrupted MDDF entries would propagate bad
        // floats into the WMO/M2 instance transform and crash render.
        for (int k = 0; k < 3; k++) {
            if (!std::isfinite(placement.position[k])) placement.position[k] = 0.0f;
            if (!std::isfinite(placement.rotation[k])) placement.rotation[k] = 0.0f;
        }

        terrain.doodadPlacements.push_back(placement);
    }

    LOG_DEBUG("Loaded ", terrain.doodadPlacements.size(), " doodad placements");
}

void ADTLoader::parseMODF(std::span<const uint8_t> data, ADTTerrain& terrain) {
    // MODF contains WMO placements (64 bytes each)
    const size_t entrySize = 64;
    size_t count = data.size() / entrySize;

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
        // WotLK MODF entries include trailing nameSet + scale (4 bytes); older
        // expansions left them as padding.
        if (offset + 64 <= data.size()) {
            placement.nameSet = readUInt16(data, offset + 60);
            placement.scale = readUInt16(data, offset + 62);
            if (placement.scale == 0) placement.scale = 1024;
        }
        // Same NaN scrub as MDDF entries - corrupted MODF would crash WMO
        // instance transform.
        for (int k = 0; k < 3; k++) {
            if (!std::isfinite(placement.position[k])) placement.position[k] = 0.0f;
            if (!std::isfinite(placement.rotation[k])) placement.rotation[k] = 0.0f;
            if (!std::isfinite(placement.extentLower[k])) placement.extentLower[k] = 0.0f;
            if (!std::isfinite(placement.extentUpper[k])) placement.extentUpper[k] = 0.0f;
        }

        terrain.wmoPlacements.push_back(placement);
    }

    LOG_DEBUG("Loaded ", terrain.wmoPlacements.size(), " WMO placements");
}

void ADTLoader::parseMCNK(std::span<const uint8_t> data, int chunkIndex, ADTTerrain& terrain) {
    if (chunkIndex < 0 || chunkIndex >= 256) {
        LOG_WARNING("Invalid chunk index: ", chunkIndex);
        return;
    }

    MapChunk& chunk = terrain.chunks[chunkIndex];

    // Read MCNK header (128 bytes)
    if (data.size() < 128) {
        LOG_WARNING("MCNK chunk too small");
        return;
    }

    chunk.flags = readUInt32(data, 0);
    chunk.indexX = readUInt32(data, 4);
    chunk.indexY = readUInt32(data, 8);
    chunk.areaId = readUInt32(data, 52);

    // Read holes mask (at offset 0x3C = 60 in MCNK header)
    // Each bit represents a 2x2 block of the 8x8 quad grid
    chunk.holes = readUInt16(data, 60);

    // doodadMapping (at offset 0x40 = 64): two bits per quad of the 8x8 grid,
    // naming which of the four texture layers the quad takes its ground
    // effect from. Paint blends per texel but growth follows the quad's
    // dominant layer, so a few faint grassy texels bleeding over dirt do not
    // seed the dirt.
    for (size_t i = 0; i < chunk.doodadMapping.size(); ++i) {
        chunk.doodadMapping[i] = data[64 + i];
    }

    // noEffectDoodad (at offset 0x50 = 80): one bit per quad of the 8x8 grid,
    // set where the map forbids ground effect doodads. Tilled farm rows,
    // building footprints and WMO interior floors are painted with textures
    // whose effects otherwise grow, and this mask is the only thing in the
    // data that says nothing should.
    chunk.noEffectDoodad = static_cast<uint64_t>(readUInt32(data, 80)) |
                           (static_cast<uint64_t>(readUInt32(data, 84)) << 32);

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
                 " sizeAlpha=", sizeAlpha, " size=", data.size(),
                 " holes=0x", std::hex, chunk.holes, std::dec);
    }

    // MCNK position is in canonical WoW coordinates (NOT ADT placement space):
    //   offset 104: wowY (west axis, horizontal - unused, XY computed from tile indices)
    //   offset 108: wowX (north axis, horizontal - unused, XY computed from tile indices)
    //   offset 112: wowZ = HEIGHT BASE (MCVT heights are relative to this)
    chunk.position[0] = readFloat(data, 104);  // wowY (unused)
    chunk.position[1] = readFloat(data, 108);  // wowX (unused)
    chunk.position[2] = readFloat(data, 112);  // wowZ = height base


    // Parse sub-chunks using offsets from MCNK header
    // WoW ADT sub-chunks may have their own 8-byte headers (magic+size)
    // Check by inspecting the first 4 bytes at the offset

    // Height map (MCVT) - 145 floats = 580 bytes.
    // Guard must include the potential 8-byte sub-chunk header, otherwise the
    // parser reads up to 8 bytes past the validated range.
    if (ofsHeight > 0 && ofsHeight + 580 + 8 <= data.size()) {
        uint32_t possibleMagic = readUInt32(data, ofsHeight);
        uint32_t headerSkip = 0;
        if (possibleMagic == MCVT) {
            headerSkip = 8;
            if (chunkIndex == 0) {
                LOG_DEBUG("MCNK sub-chunks have headers (MCVT magic found at offset ", ofsHeight, ")");
            }
        }
        parseMCVT(data.subspan(ofsHeight + headerSkip, 580), chunk);
    }

    // Normals (MCNR) - 145 normals (3 bytes each) + 13 padding = 448 bytes.
    if (ofsNormal > 0 && ofsNormal + 448 + 8 <= data.size()) {
        uint32_t possibleMagic = readUInt32(data, ofsNormal);
        uint32_t skip = (possibleMagic == MCNR) ? 8 : 0;
        parseMCNR(data.subspan(ofsNormal + skip, 448), chunk);
    }

    // Texture layers (MCLY) - 16 bytes per layer
    if (ofsLayer > 0 && nLayers > 0) {
        size_t layerSize = nLayers * 16;
        uint32_t possibleMagic = readUInt32(data, ofsLayer);
        uint32_t skip = (possibleMagic == MCLY) ? 8 : 0;
        if (ofsLayer + skip + layerSize <= data.size()) {
            parseMCLY(data.subspan(ofsLayer + skip, layerSize), chunk);
        }
    }

    // Alpha maps (MCAL) - variable size from header.
    // sizeAlpha is only known to be >= 1 here, so a four-byte chunk whose
    // contents happen to spell MCAL would take skip to 8 and wrap
    // sizeAlpha - skip to ~1.8e19. Drop the sub-chunk rather than hand a
    // parser a length longer than the address space.
    if (ofsAlpha > 0 && sizeAlpha > 0 && ofsAlpha + sizeAlpha <= data.size()) {
        uint32_t possibleMagic = readUInt32(data, ofsAlpha);
        uint32_t skip = (possibleMagic == MCAL) ? 8 : 0;
        if (sizeAlpha > skip) {
            parseMCAL(data.subspan(ofsAlpha + skip, sizeAlpha - skip), chunk);
        }
    }

    // Liquid (MCLQ) - vanilla/TBC per-chunk water (no MH2O in these expansions)
    // ofsLiquid at MCNK header offset 0x60, sizeLiquid at 0x64
    uint32_t ofsLiquid = readUInt32(data, 0x60);
    uint32_t sizeLiquid = readUInt32(data, 0x64);
    if (ofsLiquid > 0 && sizeLiquid > 8 && ofsLiquid + sizeLiquid <= data.size()) {
        uint32_t possibleMagic = readUInt32(data, ofsLiquid);
        uint32_t skip = (possibleMagic == MCLQ) ? 8 : 0;
        if (sizeLiquid > skip) {
            parseMCLQ(data.subspan(ofsLiquid + skip, sizeLiquid - skip),
                      chunkIndex, chunk.flags, terrain);
        }
    }
}

void ADTLoader::parseMCVT(std::span<const uint8_t> data, MapChunk& chunk) {
    if (data.size() < kMCVTVertexCount * sizeof(float)) {
        LOG_WARNING("MCVT chunk too small: ", data.size(), " bytes");
        return;
    }

    float minHeight = std::numeric_limits<float>::max();
    float maxHeight = std::numeric_limits<float>::lowest();

    for (int i = 0; i < kMCVTVertexCount; i++) {
        float height = readFloat(data, i * sizeof(float));
        chunk.heightMap.heights[i] = height;

        if (height < minHeight) minHeight = height;
        if (height > maxHeight) maxHeight = height;
    }
    chunk.heightMap.loaded = true;

    // Log height range for first chunk only
    static bool logged = false;
    if (!logged) {
        LOG_INFO("MCVT height range: [", minHeight, ", ", maxHeight, "]",
                 " (heights[0]=", chunk.heightMap.heights[0],
                 " heights[8]=", chunk.heightMap.heights[8],
                 " heights[136]=", chunk.heightMap.heights[136],
                 " heights[144]=", chunk.heightMap.heights[144], ")");
        logged = true;
    }
}

void ADTLoader::parseMCNR(std::span<const uint8_t> data, MapChunk& chunk) {
    // MCNR: one signed XYZ normal per vertex (3 bytes each)
    if (data.size() < kMCVTVertexCount * 3) {
        LOG_WARNING("MCNR chunk too small: ", data.size(), " bytes");
        return;
    }

    for (int i = 0; i < kMCVTVertexCount * 3; i++) {
        chunk.normals[i] = static_cast<int8_t>(data[i]);
    }
}

void ADTLoader::parseMCLY(std::span<const uint8_t> data, MapChunk& chunk) {
    // MCLY contains texture layer definitions (16 bytes each)
    size_t layerCount = data.size() / 16;

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

void ADTLoader::parseMCAL(std::span<const uint8_t> data, MapChunk& chunk) {
    // MCAL contains alpha maps for texture layers
    // Store raw data; decompression happens per-layer during mesh generation
    chunk.alphaMap.resize(data.size());
    std::memcpy(chunk.alphaMap.data(), data.data(), data.size());
}

void ADTLoader::parseMCLQ(std::span<const uint8_t> data, int chunkIndex,
                          uint32_t mcnkFlags, ADTTerrain& terrain) {
    // MCLQ: Vanilla/TBC per-chunk liquid data (inside MCNK)
    // Layout:
    //   float minHeight, maxHeight  (8 bytes)
    //   SLiquidVertex[9*9]          (81 * 8 = 648 bytes)
    //     water: uint8 depth, flow0, flow1, filler, float height
    //     magma: uint16 s, uint16 t, float height
    //   uint8 tiles[8*8]            (64 bytes)
    // Total minimum: 720 bytes

    if (data.size() < 720) {
        return;  // Not enough data for a valid MCLQ
    }

    float minHeight = readFloat(data, 0);
    float maxHeight = readFloat(data, 4);

    // Determine liquid type from MCNK flags
    // 0x04 = has liquid (river/lake), 0x08 = ocean, 0x10 = magma, 0x20 = slime
    uint16_t liquidType = 0;  // water
    if (mcnkFlags & 0x08) liquidType = 1;       // ocean
    else if (mcnkFlags & 0x10) liquidType = 2;  // magma
    else if (mcnkFlags & 0x20) liquidType = 3;  // slime

    // Read 9x9 height values (skip depth/flow bytes, just read the float height)
    const std::span<const uint8_t> vertData = data.subspan(8);
    std::vector<float> heights(81);
    for (int i = 0; i < 81; i++) {
        heights[i] = readFloat(vertData, i * 8 + 4);  // float at offset 4 within each 8-byte vertex
    }

    // Read 8x8 tile flags
    const std::span<const uint8_t> tileData = data.subspan(8 + 648);
    std::vector<uint8_t> tileMask(64);
    bool anyVisible = false;
    for (int i = 0; i < 64; i++) {
        uint8_t tileFlag = tileData[i];
        // The low nibble stores the liquid type; 0x0F is the MCLQ sentinel
        // for a tile with no liquid.  Some files also mark dry tiles with
        // the legacy 0x80 hidden bit.
        bool hidden = (tileFlag & 0x0F) == 0x0F || (tileFlag & 0x80) != 0;
        tileMask[i] = hidden ? 0 : 1;
        if (!hidden) anyVisible = true;
    }

    if (!anyVisible) {
        return;  // All tiles hidden, no visible water
    }

    // Validate heights - if all heights are 0 or unreasonable, skip
    bool validHeights = false;
    for (float h : heights) {
        if (h != 0.0f && std::isfinite(h)) {
            validHeights = true;
            break;
        }
    }
    // If heights are all zero, use maxHeight as flat water level
    if (!validHeights) {
        for (float& h : heights) h = maxHeight;
    }

    // Build a WaterLayer matching the MH2O format
    ADTTerrain::WaterLayer layer;
    layer.liquidType = liquidType;
    layer.flags = 0;
    layer.minHeight = minHeight;
    layer.maxHeight = maxHeight;
    layer.x = 0;
    layer.y = 0;
    layer.width = 8;   // 8 tiles = 9 vertices per axis
    layer.height = 8;
    layer.heights = std::move(heights);
    layer.mask.resize(8);  // 8 bytes = 64 bits for 8x8 tiles
    for (int row = 0; row < 8; row++) {
        uint8_t rowBits = 0;
        for (int col = 0; col < 8; col++) {
            if (tileMask[row * 8 + col]) {
                rowBits |= (1 << col);
            }
        }
        layer.mask[row] = rowBits;
    }

    terrain.waterData[chunkIndex].layers.push_back(std::move(layer));

    static int mclqLogCount = 0;
    if (mclqLogCount < 5) {
        LOG_INFO("MCLQ[", chunkIndex, "]: type=", liquidType,
                 " height=[", minHeight, ",", maxHeight, "]");
        mclqLogCount++;
    }
}

void ADTLoader::parseMH2O(std::span<const uint8_t> data, ADTTerrain& terrain) {
    // MH2O contains water/liquid data for all 256 map chunks
    // Structure: 256 SMLiquidChunk headers followed by instance data

    // Each SMLiquidChunk header is 12 bytes (WotLK 3.3.5a):
    // - uint32_t offsetInstances (offset from MH2O chunk start)
    // - uint32_t layerCount
    // - uint32_t offsetAttributes (offset from MH2O chunk start)

    const size_t headerSize = 12;  // SMLiquidChunk data.size() for WotLK
    const size_t totalHeaderSize = 256 * headerSize;

    if (data.size() < totalHeaderSize) {
        LOG_WARNING("MH2O chunk too small for headers: ", data.size(), " bytes");
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
        if (offsetInstances >= data.size()) {
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

            if (instanceOffset + 24 > data.size()) {
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
            // The bitmap holds one bit per tile of the layer's width×height
            // sub-rectangle - row-major, LSB-first, packed into
            // (width*height+7)/8 bytes. It is NOT a chunk-wide 8-byte block.
            // Normalize into a canonical chunk-wide 8x8 mask
            // (bit = (y+row)*8 + (x+col), LSB-first) so MCLQ and MH2O
            // masks share one format downstream.
            // Note: offsets in SMLiquidInstance are relative to MH2O chunk start
            layer.mask.assign(8, 0x00);
            bool anyTile = false;
            size_t packedBytes = (static_cast<size_t>(layer.width) * layer.height + 7) / 8;
            bool haveBitmap = offsetExistsBitmap > 0 &&
                              offsetExistsBitmap + packedBytes <= data.size();
            const uint8_t* bits = haveBitmap ? data.data() + offsetExistsBitmap : nullptr;
            int bitPos = 0;
            for (int row = 0; row < layer.height; row++) {
                for (int col = 0; col < layer.width; col++, bitPos++) {
                    // No bitmap (or out-of-range offset) means every tile in
                    // the sub-rect exists.
                    bool exists = !bits || (bits[bitPos / 8] & (1 << (bitPos % 8))) != 0;
                    if (exists) {
                        int tileIdx = (layer.y + row) * 8 + (layer.x + col);
                        layer.mask[tileIdx / 8] |= static_cast<uint8_t>(1 << (tileIdx % 8));
                        anyTile = true;
                    }
                }
            }
            if (!anyTile) {
                continue;  // Bitmap masks out every tile - nothing to render
            }

            // Read vertex heights
            // Number of vertices is (width+1) * (height+1)
            size_t numVertices = (layer.width + 1) * (layer.height + 1);

            // Check liquid object flags (LVF) to determine vertex format
            bool hasHeightData = (liquidObject != 2);  // LVF_height_depth or LVF_height_texcoord

            if (hasHeightData && offsetVertexData > 0) {
                size_t vertexOffset = offsetVertexData;
                size_t vertexDataSize = numVertices * sizeof(float);

                if (vertexOffset + vertexDataSize <= data.size()) {
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

    LOG_DEBUG("Loaded MH2O water data: ", totalLayers, " liquid layers across ", data.size(), " bytes");
}

} // namespace pipeline
} // namespace wowee
