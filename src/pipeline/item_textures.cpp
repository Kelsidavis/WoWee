#include "pipeline/item_textures.hpp"

#include <algorithm>
#include <cctype>

#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"

namespace wowee {
namespace pipeline {



std::string resolveItemRegionTexture(AssetManager& assets, int region,
                                     const std::string& texName, bool isFemale) {
    if (texName.empty()) return {};
    const char* dir = itemComponentDir(region);
    if (dir[0] == '\0') return {};

    const std::string base = std::string("Item\\TextureComponents\\") + dir + "\\" + texName;
    std::string gendered = base + (isFemale ? "_F.blp" : "_M.blp");
    if (assets.fileExists(gendered)) return gendered;
    std::string unisex = base + "_U.blp";
    if (assets.fileExists(unisex)) return unisex;
    std::string plain = base + ".blp";
    if (assets.fileExists(plain)) return plain;
    return {};
}

ItemDisplayArt readItemDisplayArt(const DBCFile& itemDisplayInfo, uint32_t recordIndex) {
    const auto* layout = getActiveDBCLayout()
        ? getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
    const uint32_t modelLeft   = layout ? (*layout)["LeftModel"]         : 1u;
    const uint32_t modelRight  = layout ? (*layout)["RightModel"]        : 2u;
    const uint32_t texLeft     = layout ? (*layout)["LeftModelTexture"]  : 3u;
    const uint32_t texRight    = layout ? (*layout)["RightModelTexture"] : 4u;

    ItemDisplayArt art;
    art.modelFile   = itemDisplayInfo.getString(recordIndex, modelLeft);
    art.textureName = itemDisplayInfo.getString(recordIndex, texLeft);
    if (art.modelFile.empty()) {
        art.modelFile   = itemDisplayInfo.getString(recordIndex, modelRight);
        art.textureName = itemDisplayInfo.getString(recordIndex, texRight);
    }
    if (art.modelFile.empty()) return art;

    // The tables name .mdx, the format these models were in before they were
    // converted. Every caller renamed it, and one of them forgot the case where
    // the name has no extension at all.
    const size_t dot = art.modelFile.rfind('.');
    if (dot != std::string::npos) art.modelFile.resize(dot);
    art.modelFile += ".m2";
    return art;
}


}  // namespace pipeline
}  // namespace wowee
