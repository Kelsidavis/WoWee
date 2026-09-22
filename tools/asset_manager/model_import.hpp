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

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
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
    /// The file the client draws now: an import in the override directory
    /// when there is one, the extracted model otherwise.
    std::string localPath;
};

struct ImportResult {
    std::size_t written = 0;
    std::size_t refusedByGate = 0;   ///< a texture slot the client cannot fill
    std::size_t missingTextures = 0;  ///< a texture a batch draws, not in the install
    std::size_t unusedTexturesCleared = 0;  ///< named, never drawn, not in the install
    std::size_t missingSkin = 0;      ///< index data that did not come with it
    /// Dressed from CreatureDisplayInfo, and the later install has no
    /// repainted copy of a skin the rows name - see tableSkinsByModel.
    std::size_t dressedByTable = 0;
    std::size_t tableSkinsBrought = 0;   ///< display-table skins written for a later model
    std::size_t earlierImportsDressed = 0;   ///< an earlier run's import given its skins
    std::size_t earlierImportsRemoved = 0;   ///< an earlier import that cannot be dressed
    std::size_t hasEmitters = 0;     ///< particle or ribbon structs that grew
    std::size_t notBetter = 0;
};

/// The skins each CreatureDisplayInfo row names, by the model it dresses.
///
/// A creature's skins are not in its model. Its display row names up to three,
/// in columns that fill the model's MONSTER_1 to MONSTER_3 slots, and the files
/// sit in the model's own folder. The later client kept those rows: Legion's
/// display 19906, a Warmaul ogre, is still model 2503 dressed from
/// creature/ogre02/ogre02skinblue512.blp - but its ogre02.m2 is a new mesh, and
/// the file at that path was repainted for it. So a later model has to bring
/// the later copies of these files with it, or it is dressed in art painted for
/// the mesh it replaced.
///
/// Keyed by the model's path; each entry is one row's three skin paths, empty
/// where the row names none. Paths are lowercase with forward slashes, as
/// extraction writes them. Empty when the tables are not in `expansionDir`.
std::map<std::string, std::vector<std::array<std::string, 3>>>
tableSkinsByModel(const std::string& expansionDir);


struct RepairResult {
    std::size_t modelsLooked = 0;
    std::size_t modelsRepaired = 0;
    std::size_t namesCleared = 0;
    std::size_t leftDrawn = 0;   ///< a missing texture a batch draws: left as it is
};

/// Clear the names of textures that exist nowhere from models already in the
/// override directory, where no batch draws them.
///
/// For imports made before the importer refused half-arrived models - the
/// pipeline it replaced wrote a sixth of its models naming textures it never
/// fetched, mostly reflection maps. Nothing draws those slots, so nothing looks
/// wrong, but the client goes looking for each file whenever the model loads
/// and warns that it is not there. A missing texture a batch does draw is left
/// alone and counted: clearing it would hide the missing file rather than fix
/// it. (The other thing that pipeline left undone - creatures without the skins
/// their display rows name - needs the later install, so importModels does it.)
///
/// A texture resolves when a file is at its path, lowercased, under the
/// override directory or the extraction. Something only a fallback directory
/// holds reads as missing here, which costs nothing - only a slot nothing
/// draws is ever cleared.
RepairResult repairEarlierImports(const std::string& expansionDir);

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
