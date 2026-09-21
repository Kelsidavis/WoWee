#pragma once

/// Bringing models out of a Warlords-or-later installation and into the format
/// this client reads.
///
/// Legion still stores an MD20 the way Wrath did, field for field, and still
/// names most of its textures inline. What changed around it: the model is
/// wrapped in an MD21 chunk whose offsets are relative to the chunk rather than
/// to the file, the skins moved out into files of their own named by an SFID
/// chunk, and the version stamp moved on. Unwrap, restamp, fetch the skins, and
/// a great many models convert as they are.
///
/// What does not convert is refused rather than half-written. A model on disk
/// that looks complete and draws white is worse than one that is not there.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace wowee::assets {

class CascStorage;

/// Where the models being brought in are read from.
///
/// A later installation names a model's skins and animations by id, through
/// chunks inside the model; an earlier one names them by path, beside it. The
/// importer asks for both and writes whatever answers, so one pass covers
/// Legion's CASC and Cataclysm's MPQs without knowing which it is looking at.
class ModelSource {
public:
    virtual ~ModelSource() = default;
    /// `limit` of 0 means the whole file; anything else is a prefix, which is
    /// all that is needed to decide whether a model is worth taking.
    virtual std::vector<uint8_t> read(const std::string& path, std::size_t limit = 0) = 0;
    /// Empty when this source has no notion of file ids, as MPQs do not.
    virtual std::vector<uint8_t> readId(uint32_t /*fileId*/) { return {}; }
};

/// A later installation, read through CASC.
std::unique_ptr<ModelSource> cascSource(CascStorage& storage);

/// An earlier installation, read through its MPQ chain. Null, with `error`
/// filled, when no archive would open.
std::unique_ptr<ModelSource> mpqSource(const std::string& mpqDir,
                                       const std::string& expansion,
                                       std::string* error);

struct ImportCandidate {
    /// The model's FILE name without its extension - not the name recorded
    /// inside it, which is a different string often enough to matter:
    /// elementalearth.m2 calls itself ElementalEarth2.
    std::string name;
    std::string destination;   ///< game-relative path, and what to ask for
    uint32_t localVertices = 0;
    /// How many monster-skin slots the model already here asks to be filled.
    /// A CreatureDisplayInfo row names as many skins as the model it shipped
    /// with wanted, so this is what the data can dress.
    uint32_t localMonsterSkins = 0;
};

struct ImportResult {
    std::size_t written = 0;
    std::size_t refusedByGate = 0;   ///< a texture slot the client cannot fill
    std::size_t missingTextures = 0;  ///< a texture a batch draws, not in the install
    std::size_t unusedTexturesCleared = 0;  ///< named, never drawn, not in the install
    std::size_t missingSkin = 0;      ///< index data that did not come with it
    std::size_t needsMoreSkins = 0;   ///< more monster skins than the DBC can fill
    std::size_t hasEmitters = 0;     ///< particle or ribbon structs that grew
    std::size_t notBetter = 0;
};


/// Convert what is worth converting under one path prefix.
///
/// `betterRatio` is how much larger a later model must be to be worth taking;
/// 1.3 means thirty percent more geometry.
ImportResult importModels(ModelSource& source, const std::string& expansionDir,
                          const std::string& outputDir, const std::string& prefix,
                          float betterRatio,
                          const std::function<void(const std::string&)>& say,
                          const std::atomic<bool>& cancel);

}  // namespace wowee::assets
