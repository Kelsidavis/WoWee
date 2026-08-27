#pragma once

#include <string>

namespace wowee {
namespace pipeline {

/// What to do with a base asset tree offered as a fallback behind an
/// expansion's own.
///
/// The fallback is how a thin expansion tree works at all: an expansion
/// directory that holds only overrides answers a fraction of the lookups and
/// the base answers the rest. It is also how one client's assets come to stand
/// in for another's, because it resolves per file and a borrowed file is an
/// error nowhere. Cataclysm is the case that makes it visible rather than
/// merely wrong, its Azeroth being the sundered one: a tile the Cata tree does
/// not cover is drawn as the old world, and nothing says so.
///
/// Kept separate from AssetManager because it is a decision rather than a
/// mechanism, and AssetManager cannot be linked into a test without stb_image,
/// the profiler and the BLP and DBC loaders behind it.
enum class BaseFallbackDecision {
    Use,          ///< Same client. Nothing to say.
    UseUnlabelled,///< Written before provenance existed. Cannot be checked, so warn and use.
    Refuse,       ///< A different client's tree. Naming it beats drawing from it.
    UseForced     ///< A different client's tree, and the operator asked for it anyway.
};

/// @param baseExpansion   what the base tree's manifest records, empty if unrecorded
/// @param activeExpansion the expansion actually being run, empty if unknown
/// @param forced          WOWEE_ASSET_BASE_FALLBACK=1
///
/// An unknown active expansion cannot contradict anything, so it is not a
/// mismatch. Only two recorded ids that differ are.
[[nodiscard]] inline BaseFallbackDecision decideBaseFallback(
    const std::string& baseExpansion,
    const std::string& activeExpansion,
    bool forced) {
    if (baseExpansion.empty()) return BaseFallbackDecision::UseUnlabelled;
    if (activeExpansion.empty()) return BaseFallbackDecision::Use;
    if (baseExpansion == activeExpansion) return BaseFallbackDecision::Use;
    return forced ? BaseFallbackDecision::UseForced : BaseFallbackDecision::Refuse;
}

}  // namespace pipeline
}  // namespace wowee
