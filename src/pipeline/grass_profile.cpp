#include "pipeline/grass_profile.hpp"

#include <algorithm>
#include <cctype>

namespace wowee {
namespace pipeline {

namespace {

std::string basenameLower(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    std::string name = (slash == std::string::npos) ? path : path.substr(slash + 1);
    const size_t dot = name.find_last_of('.');
    if (dot != std::string::npos) name = name.substr(0, dot);
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return name;
}

bool has(const std::string& name, const char* needle) {
    return name.find(needle) != std::string::npos;
}

} // namespace

bool isRoadLikeTexture(const std::string& texturePath) {
    std::string t = texturePath;
    std::transform(t.begin(), t.end(), t.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return has(t, "road") || has(t, "cobble") || has(t, "path") ||
           has(t, "street") || has(t, "pavement") || has(t, "brick");
}

GrassCategory categoriseDoodad(const std::string& modelPath) {
    const std::string name = basenameLower(modelPath);

    // Order matters where codes overlap. "roc" is checked before the growing
    // things because a rock with a tuft beside it is still scree, and the
    // longer spellings are checked before the three-letter codes they contain.
    if (has(name, "roc") || has(name, "cor") || has(name, "shl") ||
        has(name, "she") || has(name, "mus")) {
        return GrassCategory::Barren;
    }
    if (has(name, "bon") || has(name, "con") || has(name, "ras")) {
        return GrassCategory::Dry;
    }
    if (has(name, "flo") || has(name, "ifl") || has(name, "lov")) {
        return GrassCategory::Flowers;
    }
    if (has(name, "bus") || has(name, "ibu") || has(name, "sap") ||
        has(name, "tho") || has(name, "bra") || has(name, "eaf") ||
        has(name, "fern")) {
        return GrassCategory::Scrub;
    }
    // Everything left over grows like grass, which is both the largest group
    // and the right guess for a name this does not recognise.
    return GrassCategory::Meadow;
}

GrassProfile profileFor(GrassCategory category) {
    GrassProfile p;
    switch (category) {
        case GrassCategory::Meadow:
            return p;  // the defaults are meadow, bloom 0.05 / seed 0.20
        case GrassCategory::Scrub:
            p.heightScale = 1.35f;
            p.widthScale = 1.5f;
            p.densityScale = 0.7f;
            p.rootColor = {0.06f, 0.13f, 0.04f};
            p.tipColor = {0.22f, 0.36f, 0.12f};
            p.colorVariation = 0.12f;
            p.stiffness = 1.6f;
            p.bloomChance = 0.03f;
            p.seedChance = 0.08f;
            return p;
        case GrassCategory::Flowers:
            p.heightScale = 0.85f;
            p.widthScale = 1.1f;
            p.densityScale = 1.0f;
            p.rootColor = {0.10f, 0.18f, 0.06f};
            p.tipColor = {0.55f, 0.52f, 0.24f};
            p.colorVariation = 0.30f;
            p.stiffness = 0.9f;
            p.bloomChance = 0.40f;
            p.seedChance = 0.06f;
            return p;
        case GrassCategory::Dry:
            p.heightScale = 0.75f;
            p.widthScale = 0.9f;
            p.densityScale = 0.55f;
            p.rootColor = {0.20f, 0.17f, 0.07f};
            p.tipColor = {0.55f, 0.47f, 0.22f};
            p.colorVariation = 0.20f;
            p.stiffness = 1.4f;
            p.bloomChance = 0.02f;
            p.seedChance = 0.50f;
            return p;
        case GrassCategory::Barren:
            p.heightScale = 0.6f;
            p.widthScale = 0.8f;
            p.densityScale = 0.25f;
            p.rootColor = {0.16f, 0.15f, 0.09f};
            p.tipColor = {0.38f, 0.35f, 0.20f};
            p.colorVariation = 0.18f;
            p.stiffness = 1.5f;
            p.bloomChance = 0.0f;
            p.seedChance = 0.12f;
            return p;
    }
    return p;
}

GrassProfile deriveProfile(const std::vector<std::string>& modelPaths,
                           const std::vector<uint32_t>& weights) {
    if (modelPaths.empty()) return profileFor(GrassCategory::Meadow);

    // Some effects weight every doodad zero. Treating that as "none of them"
    // would give an effect no profile at all; it means the four are equal.
    float totalWeight = 0.0f;
    for (size_t i = 0; i < modelPaths.size(); ++i) {
        totalWeight += (i < weights.size()) ? static_cast<float>(weights[i]) : 0.0f;
    }
    const bool equalWeights = totalWeight <= 0.0f;
    if (equalWeights) totalWeight = static_cast<float>(modelPaths.size());

    GrassProfile out;
    out.heightScale = 0.0f;
    out.widthScale = 0.0f;
    out.densityScale = 0.0f;
    out.rootColor = glm::vec3(0.0f);
    out.tipColor = glm::vec3(0.0f);
    out.colorVariation = 0.0f;
    out.stiffness = 0.0f;
    out.bloomChance = 0.0f;
    out.seedChance = 0.0f;
    out.bloomColorA = glm::vec3(0.0f);
    out.bloomColorB = glm::vec3(0.0f);
    out.headColorA = glm::vec3(0.0f);
    out.headColorB = glm::vec3(0.0f);

    for (size_t i = 0; i < modelPaths.size(); ++i) {
        const float w = equalWeights
                            ? 1.0f
                            : ((i < weights.size()) ? static_cast<float>(weights[i]) : 0.0f);
        if (w <= 0.0f) continue;
        const float k = w / totalWeight;
        const GrassProfile p = profileFor(categoriseDoodad(modelPaths[i]));
        out.heightScale += p.heightScale * k;
        out.widthScale += p.widthScale * k;
        out.densityScale += p.densityScale * k;
        out.rootColor += p.rootColor * k;
        out.tipColor += p.tipColor * k;
        out.colorVariation += p.colorVariation * k;
        out.stiffness += p.stiffness * k;
        out.bloomChance += p.bloomChance * k;
        out.seedChance += p.seedChance * k;
        out.bloomColorA += p.bloomColorA * k;
        out.bloomColorB += p.bloomColorB * k;
        out.headColorA += p.headColorA * k;
        out.headColorB += p.headColorB * k;
    }
    return out;
}

} // namespace pipeline
} // namespace wowee
