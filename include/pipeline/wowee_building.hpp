#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <cstdint>

namespace wowee {
namespace pipeline {

struct WMOModel;

// Wowee Open Building format (.wob) - novel WMO replacement
// Buildings with multiple groups, portals, and doodad sets
struct WoweeBuilding {
    struct Vertex {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 texCoord;
        glm::vec4 color; // vertex color/lighting
    };

    struct Material {
        std::string texturePath;
        uint32_t flags = 0;
        uint32_t shader = 0;
        uint32_t blendMode = 0;
    };

    struct Group {
        std::string name;
        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
        std::vector<std::string> texturePaths;
        std::vector<Material> materials;
        glm::vec3 boundMin{0}, boundMax{0};
        bool isOutdoor = false;
    };

    struct Portal {
        int groupA = -1, groupB = -1;
        std::vector<glm::vec3> vertices; // portal polygon
    };

    struct DoodadPlacement {
        std::string modelPath; // .wom path
        glm::vec3 position;
        glm::vec3 rotation;
        float scale = 1.0f;
    };

    std::string name;
    std::vector<Group> groups;
    std::vector<Portal> portals;
    std::vector<DoodadPlacement> doodads;
    float boundRadius = 1.0f;

    [[nodiscard]] bool isValid() const { return !groups.empty(); }
};

class WoweeBuildingLoader {
public:
    static WoweeBuilding load(const std::string& basePath);
    static bool save(const WoweeBuilding& building, const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Convert WOB to WMOModel for the client's WMO renderer
    static bool toWMOModel(const WoweeBuilding& building, WMOModel& outModel);

    // Convert WMOModel to WOB (for editor export)
    static WoweeBuilding fromWMO(const WMOModel& wmo, const std::string& name = "");

    // Convenience: try loading <path-without-ext>.wob from the standard editor
    // search paths (custom_zones/buildings/, output/buildings/). `extraPrefixes`
    // are tried before the defaults - pass per-zone roots like
    // {"output/<map>/buildings/", "custom_zones/<map>/buildings/"} when the
    // caller knows the active zone. Returns valid building on hit.
    static WoweeBuilding tryLoadByGamePath(
        const std::string& gamePath,
        const std::vector<std::string>& extraPrefixes = {});
};

} // namespace pipeline
} // namespace wowee
