#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Addon Manifest catalog (.wmod) -
// novel replacement for the per-addon TOC (.toc) text
// files vanilla WoW scattered across Interface/AddOns/.
// Each entry is one addon manifest binding the addon
// to its display metadata (name / description /
// version / author), client-build gate
// (minClientBuild), persistence + lazy-load flags,
// and required + optional dependency lists.
//
// The variable-length dependency arrays give the
// validator something interesting to check: a DFS
// cycle detector flags require-A-which-requires-A
// loops that would deadlock the addon loader.
//
// Cross-references with previously-added formats:
//   None directly - addons are a client-side concept,
//   so WMOD does not reference WMS / WCDB / spell
//   data. Dependencies between WMOD entries are
//   internal addonId references.
//
// Binary layout (little-endian):
//   magic[4]            = "WMOD"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     addonId (uint32)
//     nameLen + name
//     descLen + description
//     versionLen + version (semver "1.2.3")
//     authorLen + author
//     minClientBuild (uint32)        - lowest
//                                       supported
//                                       client patch
//                                       number; 0 = no
//                                       gate
//     requiresSavedVariables (uint8) - 0/1 bool
//     loadOnDemand (uint8)           - 0/1 bool - LoD
//                                       addons skip
//                                       initial load
//     pad0 (uint16)
//     dependencyCount (uint32)
//     dependencies (uint32 × count)  - required addonIds
//     optionalDependencyCount (uint32)
//     optionalDependencies (uint32 × count)
struct WoweeAddonManifest {
    struct Entry {
        uint32_t addonId = 0;
        std::string name;
        std::string description;
        std::string version;
        std::string author;
        uint32_t minClientBuild = 0;
        uint8_t requiresSavedVariables = 0;
        uint8_t loadOnDemand = 0;
        uint16_t pad0 = 0;
        std::vector<uint32_t> dependencies;
        std::vector<uint32_t> optionalDependencies;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t addonId) const;
    [[nodiscard]] const Entry* findByName(const std::string& name) const;

    // Returns addons that depend on the given addonId
    // (reverse-lookup, used by the addon-disable UI to
    // warn "disabling this will also disable: X, Y").
    [[nodiscard]] std::vector<const Entry*> findDependents(uint32_t addonId) const;
};

class WoweeAddonManifestLoader {
public:
    static bool save(const WoweeAddonManifest& cat,
                     const std::string& basePath);
    static WoweeAddonManifest load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-mod* variants.
    //
    //   makeStandardAddons - 4 vanilla-era addons
    //                          (Recount / Atlas /
    //                          Auctioneer / Questie)
    //                          with realistic deps.
    //                          Recount has no deps;
    //                          Auctioneer optionally
    //                          depends on Atlas for
    //                          map links.
    //   makeUIReplacement  - 3 full-UI replacements
    //                          (Bartender4 / ElvUI /
    //                          SuperOrders) with a
    //                          chain dep where Super
    //                          Orders requires Elv.
    //   makeUtility        - 3 standalone utility
    //                          addons (XPerl /
    //                          Decursive / GearVendor)
    //                          with no inter-deps -
    //                          baseline for the
    //                          empty-deps path.
    static WoweeAddonManifest makeStandardAddons(const std::string& catalogName);
    static WoweeAddonManifest makeUIReplacement(const std::string& catalogName);
    static WoweeAddonManifest makeUtility(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
