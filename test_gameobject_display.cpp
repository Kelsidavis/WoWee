#include <iostream>
#include "pipeline/dbc.hpp"
#include "pipeline/asset_manager.hpp"

int main() {
    wowee::pipeline::AssetManager assetManager;
    assetManager.initialize("Data");

    auto godi = assetManager.loadDBC("GameObjectDisplayInfo.dbc");
    if (!godi || !godi->isLoaded()) {
        std::cerr << "Failed to load GameObjectDisplayInfo.dbc\n";
        return 1;
    }

    std::cout << "GameObjectDisplayInfo.dbc loaded with " << godi->getRecordCount() << " records\n\n";

    // Check displayIds 35 and 1287
    uint32_t targetIds[] = {35, 1287};
    for (uint32_t targetId : targetIds) {
        bool found = false;
        for (uint32_t i = 0; i < godi->getRecordCount(); i++) {
            uint32_t displayId = godi->getUInt32(i, 0);
            if (displayId == targetId) {
                std::string modelName = godi->getString(i, 1);
                std::cout << "DisplayId " << displayId << ": " << modelName << "\n";
                found = true;
                break;
            }
        }
        if (!found) {
            std::cout << "DisplayId " << targetId << ": NOT FOUND\n";
        }
    }

    return 0;
}
