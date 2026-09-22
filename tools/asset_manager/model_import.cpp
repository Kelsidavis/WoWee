#include "model_import.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>

namespace wowee::assets {
namespace {

namespace fs = std::filesystem;

constexpr uint32_t kWotlkVersion = 264;
constexpr std::size_t kTexturesCount = 80;
constexpr std::size_t kTexturesOffset = 84;
constexpr std::size_t kTextureCombosCount = 128;   ///< 0x80, the texture lookup
constexpr std::size_t kTextureCombosOffset = 132;
constexpr std::size_t kSkinBatchesCount = 36;      ///< in the .skin, not the model
constexpr std::size_t kSkinBatchesOffset = 40;
constexpr std::size_t kSkinBatchStride = 24;
constexpr std::size_t kRibbonsCount = 288;
constexpr std::size_t kSequenceStride = 64;
constexpr std::size_t kHeadRead = 4096;   ///< enough for every field the gates read
constexpr uint32_t kSequenceEmbedded = 0x20;   ///< keyframes are inside the model
constexpr std::size_t kParticlesCount = 296;

uint32_t readLE32(const uint8_t* at) {
    return uint32_t(at[0]) | (uint32_t(at[1]) << 8) |
           (uint32_t(at[2]) << 16) | (uint32_t(at[3]) << 24);
}

uint16_t readLE16(const uint8_t* at) {
    return uint16_t(uint16_t(at[0]) | (uint16_t(at[1]) << 8));
}

void writeLE32(uint8_t* at, uint32_t value) {
    for (int i = 0; i < 4; ++i) at[i] = uint8_t((value >> (i * 8)) & 0xFF);
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

/// The MD20 inside a Legion M2, and where each chunk sits.
/// Where the MD20 starts inside whatever this is: at 0 for a bare model, or
/// eight bytes in for one a later client wrapped in an MD21 chunk. A file that
/// announces neither is not a model.
bool findBody(const std::vector<uint8_t>& blob, std::size_t* bodyAt) {
    if (blob.size() < 8) return false;
    if (std::memcmp(blob.data(), "MD20", 4) == 0) { *bodyAt = 0; return true; }
    if (std::memcmp(blob.data(), "MD21", 4) == 0) { *bodyAt = 8; return true; }
    return false;
}

bool md21Body(const std::vector<uint8_t>& blob, std::vector<uint8_t>& body,
              std::map<std::string, std::pair<std::size_t, uint32_t>>& chunks) {
    if (blob.size() < 8) return false;
    if (std::memcmp(blob.data(), "MD20", 4) == 0) {
        body = blob;
        return true;
    }
    if (std::memcmp(blob.data(), "MD21", 4) != 0) return false;

    std::size_t pos = 0;
    while (pos + 8 <= blob.size()) {
        const std::string tag(reinterpret_cast<const char*>(blob.data() + pos), 4);
        const uint32_t size = readLE32(blob.data() + pos + 4);
        chunks[tag] = {pos + 8, size};
        pos += 8 + size;
        if (size == 0) break;
    }
    auto found = chunks.find("MD21");
    if (found == chunks.end()) return false;
    const std::size_t start = found->second.first;
    const uint32_t size = found->second.second;
    if (start + size > blob.size()) return false;
    body.assign(blob.begin() + start, blob.begin() + start + size);
    return true;
}

/// The texture paths a model names, and how many slots of its own name none.
///
/// A slot's type says who fills it. Type 0 is the model naming its own file;
/// types 11 to 13 are a creature's skin, which the client composes from
/// CreatureDisplayInfo and which is empty here by design. So an empty name
/// means opposite things in the two cases, and only the first is a fault.
/// The slots a creature's CreatureDisplayInfo row fills.
///
/// Types 11, 12 and 13 are MONSTER_1 to MONSTER_3: the client dresses them with
/// the row's three texture columns, and those name art painted for the model
/// that shipped with the row - its UV layout, not a later one's.
std::vector<std::size_t> tableSkinSlots(const std::vector<uint8_t>& body) {
    std::vector<std::size_t> slots;
    if (body.size() < kTexturesOffset + 4) return slots;
    const uint32_t count = readLE32(body.data() + kTexturesCount);
    const uint32_t offset = readLE32(body.data() + kTexturesOffset);
    if (count > 512) return slots;

    for (uint32_t i = 0; i < count; ++i) {
        const std::size_t entry = offset + std::size_t(i) * 16;
        if (entry + 16 > body.size()) break;
        const uint32_t kind = readLE32(body.data() + entry);
        if (kind >= 11 && kind <= 13) slots.push_back(i);
    }
    return slots;
}

/// Whether any batch in this skin draws one of the named texture slots.
///
/// A batch names a run of the model's texture-combo list, and each entry there
/// is an index into the texture array. A slot no run reaches is carried by the
/// model and never sampled, so what is in it does not matter.
bool anyBatchSamples(const std::vector<uint8_t>& skin, const std::vector<uint8_t>& body,
                     const std::vector<std::size_t>& slots) {
    if (slots.empty()) return false;
    if (skin.size() < kSkinBatchesOffset + 4) return true;      // unreadable: assume drawn
    if (std::memcmp(skin.data(), "SKIN", 4) != 0) return true;
    if (body.size() < kTextureCombosOffset + 4) return true;

    const uint32_t comboCount = readLE32(body.data() + kTextureCombosCount);
    const uint32_t comboAt = readLE32(body.data() + kTextureCombosOffset);
    const uint32_t batchCount = readLE32(skin.data() + kSkinBatchesCount);
    const uint32_t batchAt = readLE32(skin.data() + kSkinBatchesOffset);

    for (uint32_t i = 0; i < batchCount; ++i) {
        const std::size_t entry = batchAt + std::size_t(i) * kSkinBatchStride;
        if (entry + kSkinBatchStride > skin.size()) break;
        const uint16_t used = readLE16(skin.data() + entry + 14);
        const uint16_t first = readLE16(skin.data() + entry + 16);
        for (uint32_t n = 0; n < used; ++n) {
            const uint32_t combo = first + n;
            if (combo >= comboCount) continue;
            const std::size_t at = comboAt + std::size_t(combo) * 2;
            if (at + 2 > body.size()) continue;
            const uint16_t slot = readLE16(body.data() + at);
            for (std::size_t want : slots) {
                if (slot == want) return true;
            }
        }
    }
    return false;
}

/// The texture files a model names for itself, the slot each is in, and which
/// of its slots it left unnamed while claiming to name them.
///
/// Type 0 only, for the names. Every other type is a slot the client fills -
/// the monster skins from CreatureDisplayInfo, the body from CharSections - and
/// it never loads a name found on one, so a leftover path there is neither a
/// file to fetch nor a reason to refuse the model.
///
/// Types 11 to 13 are the creature skin, filled from CreatureDisplayInfo at
/// draw time, so an empty name there is correct. Type 0 is the model naming its
/// own file, and an empty name there means it named it by id through a chunk
/// this does not read.
std::vector<std::string> texturePaths(const std::vector<uint8_t>& body,
                                      std::vector<std::size_t>* unnamedOwn,
                                      std::vector<std::size_t>* namedSlots) {
    std::vector<std::string> out;
    if (unnamedOwn) unnamedOwn->clear();
    if (namedSlots) namedSlots->clear();
    if (body.size() < kTexturesOffset + 4) return out;

    const uint32_t count = readLE32(body.data() + kTexturesCount);
    const uint32_t offset = readLE32(body.data() + kTexturesOffset);
    if (count > 512) return out;

    for (uint32_t i = 0; i < count; ++i) {
        const std::size_t entry = offset + std::size_t(i) * 16;
        if (entry + 16 > body.size()) break;
        const uint32_t kind = readLE32(body.data() + entry);
        const uint32_t length = readLE32(body.data() + entry + 8);
        const uint32_t at = readLE32(body.data() + entry + 12);

        std::string name;
        if (length > 0 && at + length <= body.size()) {
            name.assign(reinterpret_cast<const char*>(body.data() + at), length);
            const std::size_t nul = name.find('\0');
            if (nul != std::string::npos) name.resize(nul);
        }
        if (kind != 0) continue;
        if (!name.empty()) {
            out.push_back(name);
            if (namedSlots) namedSlots->push_back(i);
        } else if (unnamedOwn) {
            unnamedOwn->push_back(i);
        }
    }
    return out;
}

std::vector<uint8_t> readBytes(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return {};
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
}

/// The skin columns a batch of this model draws: 0 to 2 for MONSTER_1 to
/// MONSTER_3, which is also the display row's column for each.
std::vector<int> drawnTableColumns(const std::vector<uint8_t>& skin,
                                   const std::vector<uint8_t>& body) {
    std::vector<int> columns;
    const uint32_t textureAt = readLE32(body.data() + kTexturesOffset);
    for (std::size_t slot : tableSkinSlots(body)) {
        if (!anyBatchSamples(skin, body, {slot})) continue;
        const int column = static_cast<int>(readLE32(body.data() + textureAt + slot * 16)) - 11;
        if (std::find(columns.begin(), columns.end(), column) == columns.end())
            columns.push_back(column);
    }
    return columns;
}

bool writeFile(const fs::path& path, const uint8_t* data, std::size_t size) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    return static_cast<bool>(out);
}

}  // namespace

std::size_t indexLocalModels(const std::string& expansionDir,
                             std::vector<ImportCandidate>* out) {
    // name -> (vertices, game-relative path, came from override, the file itself)
    std::map<std::string, std::tuple<uint32_t, std::string, bool, std::string>> found;
    std::error_code ec;
    const fs::path root(expansionDir);

    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec)) {
        if (!it->is_regular_file(ec)) continue;
        const std::string filename = it->path().filename().string();
        if (filename.rfind("._", 0) == 0) continue;
        if (lower(it->path().extension().string()) != ".m2") continue;

        std::ifstream in(it->path(), std::ios::binary);
        uint8_t head[200] = {0};
        in.read(reinterpret_cast<char*>(head), sizeof(head));
        if (std::memcmp(head, "MD20", 4) != 0) continue;

        const uint32_t version = readLE32(head + 4);
        // Vanilla and TBC carry a playable-animation lookup Wrath dropped, so
        // every array after it sits eight bytes later.
        const std::size_t shift = version >= 264 ? 0 : 8;
        const uint32_t vertices = readLE32(head + 60 + shift);

        fs::path relative = fs::relative(it->path(), root, ec);
        std::string first = relative.begin() != relative.end()
                            ? lower(relative.begin()->string()) : std::string();
        const bool overridden = first == "override";
        if (overridden) {
            // The path recorded is the game-relative one, never the override
            // copy's, because that is where a new pack has to put its file.
            fs::path trimmed;
            bool skip = true;
            for (const fs::path& part : relative) {
                if (skip) { skip = false; continue; }
                trimmed /= part;
            }
            relative = trimmed;
        }

        const std::string key = lower(filename.substr(0, filename.size() - 3));
        auto existing = found.find(key);
        if (existing == found.end()) {
            found[key] = {vertices, relative.generic_string(), overridden, it->path().string()};
        } else {
            const bool wasOverride = std::get<2>(existing->second);
            const uint32_t wasVertices = std::get<0>(existing->second);
            // An override replaces the shipped file whatever its size: it is
            // what the client actually draws, so it is what a candidate has to
            // beat. Measured against the untouched file instead, a pack already
            // installed reads as an improvement worth making again - and a
            // better model gets replaced by a poorer one.
            if (overridden && !wasOverride) {
                existing->second = {vertices, relative.generic_string(), true, it->path().string()};
            } else if (overridden == wasOverride && vertices > wasVertices) {
                existing->second = {vertices, relative.generic_string(), overridden,
                                    it->path().string()};
            }
        }
    }

    if (out != nullptr) {
        out->clear();
        for (const auto& [key, record] : found) {
            ImportCandidate candidate;
            candidate.name = key;
            candidate.localVertices = std::get<0>(record);
            candidate.destination = std::get<1>(record);
            candidate.localPath = std::get<3>(record);
            out->push_back(std::move(candidate));
        }
    }
    return found.size();
}

std::map<std::string, std::vector<std::array<std::string, 3>>>
tableSkinsByModel(const std::string& expansionDir) {
    std::map<std::string, std::vector<std::array<std::string, 3>>> out;

    // WDBC: a 20-byte header, fixed-size records of 32-bit fields, then the
    // strings the records point into. The columns read here are where every
    // expansion this client knows keeps them - dbc_layouts.json agrees for
    // classic, tbc, wotlk and turtle: CreatureModelData's path is field 2,
    // CreatureDisplayInfo's model is field 1 and its skins fields 6 to 8.
    struct Table {
        std::vector<uint8_t> blob;
        uint32_t records = 0, fields = 0, recordSize = 0, strings = 0;
        bool read(const fs::path& path) {
            blob = readBytes(path);
            if (blob.size() < 20 || std::memcmp(blob.data(), "WDBC", 4) != 0) return false;
            records = readLE32(blob.data() + 4);
            fields = readLE32(blob.data() + 8);
            recordSize = readLE32(blob.data() + 12);
            strings = readLE32(blob.data() + 16);
            return blob.size() >= 20 + std::size_t(records) * recordSize + strings;
        }
        uint32_t u32(uint32_t row, uint32_t field) const {
            if (field >= fields) return 0;
            return readLE32(blob.data() + 20 + std::size_t(row) * recordSize + field * 4);
        }
        std::string text(uint32_t row, uint32_t field) const {
            const uint32_t at = u32(row, field);
            const std::size_t base = 20 + std::size_t(records) * recordSize;
            if (at >= strings) return {};
            const char* s = reinterpret_cast<const char*>(blob.data() + base + at);
            return std::string(s, std::find(s, s + (strings - at), '\0'));
        }
    };
    auto normalise = [](std::string path) {
        path = lower(path);
        std::replace(path.begin(), path.end(), '\\', '/');
        return path;
    };

    const fs::path tables = fs::path(expansionDir) / "dbfilesclient";
    Table models, displays;
    if (!models.read(tables / "creaturemodeldata.dbc") ||
        !displays.read(tables / "creaturedisplayinfo.dbc")) {
        return out;
    }

    // The model path as the table writes it - Creature\Ogre02\Ogre02.mdx - in
    // the spelling the extraction and the importer use: creature/ogre02/ogre02.m2.
    std::map<uint32_t, std::string> modelPath;
    for (uint32_t r = 0; r < models.records; ++r) {
        std::string path = normalise(models.text(r, 2));
        const std::size_t dot = path.rfind('.');
        if (dot == std::string::npos) continue;
        path = path.substr(0, dot) + ".m2";
        modelPath[models.u32(r, 0)] = path;
    }

    for (uint32_t r = 0; r < displays.records; ++r) {
        auto model = modelPath.find(displays.u32(r, 1));
        if (model == modelPath.end()) continue;
        const std::string folder = fs::path(model->second).parent_path().generic_string();
        std::array<std::string, 3> skins;
        for (int column = 0; column < 3; ++column) {
            const std::string name = displays.text(r, 6 + column);
            if (!name.empty()) skins[column] = normalise(folder + "/" + name + ".blp");
        }
        auto& rows = out[model->second];
        if (std::find(rows.begin(), rows.end(), skins) == rows.end()) rows.push_back(skins);
    }
    return out;
}

RepairResult repairEarlierImports(const std::string& expansionDir) {
    RepairResult result;
    const fs::path root(expansionDir);
    const fs::path overrides = root / "override";
    std::error_code ec;
    if (!fs::is_directory(overrides, ec)) return result;

    auto resolves = [&](const std::string& name) {
        std::string relative = lower(name);
        std::replace(relative.begin(), relative.end(), '\\', '/');
        std::error_code existsEc;
        return fs::is_regular_file(overrides / relative, existsEc) ||
               fs::is_regular_file(root / relative, existsEc);
    };

    for (fs::recursive_directory_iterator it(overrides, ec), end; it != end && !ec;
         it.increment(ec)) {
        std::error_code fileEc;
        if (!it->is_regular_file(fileEc)) continue;
        const fs::path path = it->path();
        if (path.filename().string().rfind("._", 0) == 0) continue;
        if (lower(path.extension().string()) != ".m2") continue;

        std::vector<uint8_t> body = readBytes(path);
        // Wrath's layout only: every offset below is where 264 keeps it, and
        // that is what the importer writes.
        if (body.size() < kTextureCombosOffset + 4 || std::memcmp(body.data(), "MD20", 4) != 0 ||
            readLE32(body.data() + 4) < kWotlkVersion) {
            continue;
        }
        ++result.modelsLooked;

        // The first skin, the one the importer's own tests read. A model with
        // no skin beside it is not something to reason about.
        const std::string stem = path.stem().string();
        const std::vector<uint8_t> skin = readBytes(path.parent_path() / (stem + "00.skin"));
        if (skin.empty()) continue;

        std::vector<std::size_t> namedSlots;
        const std::vector<std::string> names = texturePaths(body, nullptr, &namedSlots);
        std::vector<std::size_t> missing;
        for (std::size_t t = 0; t < names.size(); ++t) {
            if (!resolves(names[t])) missing.push_back(namedSlots[t]);
        }
        if (missing.empty()) continue;

        const uint32_t textureAt = readLE32(body.data() + kTexturesOffset);
        std::size_t cleared = 0;
        for (std::size_t slot : missing) {
            if (anyBatchSamples(skin, body, {slot})) { ++result.leftDrawn; continue; }
            writeLE32(body.data() + textureAt + slot * 16 + 8, 0);    // name length
            writeLE32(body.data() + textureAt + slot * 16 + 12, 0);   // name offset
            ++cleared;
        }
        if (cleared == 0) continue;
        if (!writeFile(path, body.data(), body.size())) continue;
        ++result.modelsRepaired;
        result.namesCleared += cleared;
    }
    return result;
}

ImportResult importModels(ModelSource& source, const std::string& expansionDir,
                          const std::string& outputDir, const std::string& prefix,
                          float betterRatio,
                          const std::function<void(const std::string&)>& say,
                          const std::atomic<bool>& cancel) {
    ImportResult result;

    std::vector<ImportCandidate> local;
    indexLocalModels(expansionDir, &local);
    if (say) say("    " + std::to_string(local.size()) + " models already here to compare against");

    const std::string wantPrefix = lower(prefix);
    std::set<std::string> fetched;
    std::set<std::string> failedTextures;

    // The skins a creature is dressed in, and where the later copies of them go.
    //
    // Into the override directory, never over the extracted file: the client
    // reads that directory first, and the extracted copy is what tells a
    // repainted skin from one the later client never touched.
    const auto tableSkins = tableSkinsByModel(expansionDir);
    const fs::path overrideRoot = fs::path(outputDir) / "override";

    // The later install's copies of every skin the display rows name for this
    // model, in the columns it draws - or why they cannot dress it. Only the
    // files not already in the override directory as the later copy are
    // returned to be written.
    //
    // Two answers refuse. A row with nothing in a column the model draws leaves
    // that part of it bare - Legion's boar draws a second skin, for its mane,
    // that no 3.3.5 row names. And a skin the later install holds byte for byte
    // as extracted was not repainted for the later mesh, so the later client
    // must dress it from files these rows never name.
    struct SkinFetch {
        bool ok = false;
        std::string why;
        std::vector<std::pair<std::string, std::vector<uint8_t>>> files;
    };
    auto fetchTableSkins = [&](const std::string& model,
                               const std::vector<int>& columns) -> SkinFetch {
        SkinFetch out;
        auto rows = tableSkins.find(lower(model));
        if (rows == tableSkins.end() || rows->second.empty()) {
            out.why = "no display row names its skins";
            return out;
        }
        std::set<std::string> wanted;
        for (const auto& row : rows->second) {
            for (int column : columns) {
                if (row[column].empty()) {
                    out.why = "it draws a skin a display row leaves empty";
                    return out;
                }
                wanted.insert(row[column]);
            }
        }
        for (const std::string& skin : wanted) {
            std::vector<uint8_t> theirs = source.read(skin);
            if (theirs.empty()) {
                out.why = "the later install has no " + skin;
                return out;
            }
            const std::vector<uint8_t> original = readBytes(fs::path(expansionDir) / skin);
            if (!original.empty() && original == theirs) {
                out.why = skin + " was not repainted for the later mesh";
                return out;
            }
            if (readBytes(overrideRoot / skin) != theirs) {
                out.files.emplace_back(skin, std::move(theirs));
            }
        }
        out.ok = true;
        return out;
    };
    auto writeSkins = [&](const SkinFetch& fetch) {
        for (const auto& [skin, bytes] : fetch.files) {
            if (writeFile(overrideRoot / skin, bytes.data(), bytes.size())) {
                ++result.tableSkinsBrought;
            }
        }
    };

    // Asked for by path, one local model at a time, rather than by sweeping the
    // whole installation and matching on the name inside each file.
    //
    // CASC stores no filenames, only a hash of the path - but a hash can be
    // computed, and the path of a model already here is a path already known.
    // So this asks for exactly the file it wants: twenty-two thousand lookups
    // instead of three quarters of a million reads.
    //
    // Matching on the internal name is what the sweep does and it does not
    // work: Legion's earth elemental calls itself "ElementalEarth2" while the
    // file it must replace is elementalearth.m2, so the two never meet and the
    // model is passed over. A path is the same on both sides by construction.
    for (const ImportCandidate& candidate : local) {
        if (cancel.load()) break;

        const std::string& where = candidate.destination;
        if (!wantPrefix.empty() && lower(where).rfind(wantPrefix, 0) != 0) continue;

        // The head first. Most models here are not improved by the later
        // client, and every count needed to decide that sits in the first few
        // hundred bytes of the model - so the ones turned away cost a short
        // read each rather than the megabytes of a whole character model.
        std::vector<uint8_t> head = source.read(where, kHeadRead);
        std::size_t bodyAt = 0;
        if (!findBody(head, &bodyAt)) continue;

        if (head.size() < bodyAt + kParticlesCount + 4) {
            // A header the short read did not reach the end of. Only worth
            // asking again when the read was actually cut short - a file that
            // came back smaller than the limit is already all of it, and a
            // second read would return the same bytes.
            if (head.size() < kHeadRead) continue;
            head = source.read(where);
            if (!findBody(head, &bodyAt)) continue;
            if (head.size() < bodyAt + kParticlesCount + 4) continue;
        }
        const uint8_t* at = head.data() + bodyAt;

        const uint32_t theirVertices = readLE32(at + 60);
        if (betterRatio > 0.0f &&
            (candidate.localVertices < 20 ||
             float(theirVertices) <= float(candidate.localVertices) * betterRatio)) {
            ++result.notBetter;
            // An earlier run's import may be sitting here, and the pipelines
            // before this one left two kinds of creature wearing skins that do
            // not fit it. See the two cases below.
            if (!candidate.localPath.empty()) {
                const fs::path localModel(candidate.localPath);
                const std::vector<uint8_t> localBody = readBytes(localModel);
                const std::vector<uint8_t> localSkin = readBytes(
                    localModel.parent_path() / (localModel.stem().string() + "00.skin"));
                const bool readable = localBody.size() >= kTextureCombosOffset + 4 &&
                                      std::memcmp(localBody.data(), "MD20", 4) == 0 &&
                                      readLE32(localBody.data() + 4) >= kWotlkVersion &&
                                      !localSkin.empty();
                const std::vector<int> columns =
                    readable ? drawnTableColumns(localSkin, localBody) : std::vector<int>{};
                // An import, and not the extracted original the later client
                // happens to share: only an import sits in the override
                // directory. The importer itself writes over the extracted
                // file, and there nothing tells the two apart.
                std::error_code relEc;
                const std::string underOverride =
                    fs::relative(localModel, overrideRoot, relEc).generic_string();
                const bool isEarlierImport = !relEc && !underOverride.empty() &&
                                             underOverride.rfind("..", 0) != 0;

                // Nothing can dress it, so it comes out, with what was written
                // beside it, and the extracted model under it is drawn again.
                // Textures stay: they may be shared.
                auto removeEarlierImport = [&](const std::string& why) {
                    const std::string stemLower = lower(localModel.stem().string());
                    std::error_code dirEc;
                    std::vector<fs::path> beside;
                    for (const auto& entry :
                         fs::directory_iterator(localModel.parent_path(), dirEc)) {
                        const std::string name = lower(entry.path().filename().string());
                        const std::string ext = lower(entry.path().extension().string());
                        if (name.rfind(stemLower, 0) != 0) continue;
                        if (ext != ".skin" && ext != ".anim") continue;
                        // "00.skin", "0060-00.anim": digits and a dash between
                        // the stem and the extension. ogre and ogrewarlord
                        // share a prefix and differ here.
                        const std::string middle = name.substr(
                            stemLower.size(), name.size() - stemLower.size() - ext.size());
                        if (!middle.empty() &&
                            middle.find_first_not_of("0123456789-") == std::string::npos) {
                            beside.push_back(entry.path());
                        }
                    }
                    std::error_code rmEc;
                    if (fs::remove(localModel, rmEc)) {
                        for (const fs::path& p : beside) fs::remove(p, rmEc);
                        ++result.earlierImportsRemoved;
                        if (say) say("    removed an earlier import of " + where + ": " + why);
                    }
                };

                if (!columns.empty() && isEarlierImport) {
                    if (theirVertices == candidate.localVertices) {
                        // The later client's own model, brought without the
                        // skins its display rows name.
                        const SkinFetch fetch = fetchTableSkins(where, columns);
                        if (fetch.ok) {
                            if (!fetch.files.empty()) {
                                writeSkins(fetch);
                                ++result.earlierImportsDressed;
                            }
                        } else {
                            removeEarlierImport(fetch.why);
                        }
                    } else if (float(candidate.localVertices) > float(theirVertices) * 1.1f ||
                               float(candidate.localVertices) * 1.1f < float(theirVertices)) {
                        // Not the later client's model at this path at all.
                        // An earlier pack filed models under the community
                        // listfile's names, which guess: what it installed as
                        // creature/ridinghorse/ridinghorse.m2 is Legion's
                        // HorseMultiSaddle, 3155 vertices, while Legion's own
                        // file at that path is the 673-vertex horse 3.3.5
                        // ships. The displays that name this path are dressed
                        // for that horse, in 3.3.5 and in Legion alike, so no
                        // skin they name can fit what is here.
                        removeEarlierImport(
                            "not the later client's model at this path (" +
                            std::to_string(candidate.localVertices) + " vertices here, " +
                            std::to_string(theirVertices) + " in the later install), so the "
                            "skins named for this path do not fit it");
                    }
                    // Within a tenth either way it is the same model exported
                    // again with small edits - Legion's banshee is 1141
                    // vertices, an earlier pack's 1132 - and the skins named
                    // for it still fit.
                }
            }
            continue;
        }

        // These two structs grew after Wrath. A model that emits nothing is the
        // common case; one that does needs more than a version stamp.
        if (readLE32(at + kRibbonsCount) != 0 || readLE32(at + kParticlesCount) != 0) {
            ++result.hasEmitters;
            continue;
        }

        const std::vector<uint8_t> blob = source.read(where);
        std::vector<uint8_t> body;
        std::map<std::string, std::pair<std::size_t, uint32_t>> chunks;
        if (!md21Body(blob, body, chunks)) continue;
        if (body.size() < kParticlesCount + 4) continue;

        // The skin and the animations are named after the model FILE, not the
        // name inside the model: the client works both paths out from the model
        // path, and a stem that differs even in case is a model with no index
        // data, which draws as spikes.
        const fs::path destination = fs::path(outputDir) / where;
        const std::string stem = destination.stem().string();
        std::vector<std::pair<fs::path, std::vector<uint8_t>>> sidecars;

        auto sfid = chunks.find("SFID");
        if (sfid != chunks.end()) {
            const std::size_t at = sfid->second.first;
            const uint32_t size = sfid->second.second;
            for (uint32_t lod = 0; lod * 4 < size; ++lod) {
                if (at + lod * 4 + 4 > blob.size()) break;
                const uint32_t skinId = readLE32(blob.data() + at + lod * 4);
                std::vector<uint8_t> skin = source.readId(skinId);
                if (skin.empty()) continue;
                char suffix[16];
                std::snprintf(suffix, sizeof(suffix), "%02u.skin", lod);
                sidecars.emplace_back(destination.parent_path() / (stem + suffix),
                                      std::move(skin));
            }
        }
        if (sidecars.empty()) {
            // No SFID means an earlier client, which keeps the skins beside the
            // model under the names the client already builds. Four is every
            // skin any model of that era has.
            const std::string base = where.substr(0, where.size() - 3);
            for (uint32_t lod = 0; lod < 4; ++lod) {
                char suffix[16];
                std::snprintf(suffix, sizeof(suffix), "%02u.skin", lod);
                std::vector<uint8_t> skin = source.read(base + suffix);
                if (skin.empty()) continue;
                sidecars.emplace_back(destination.parent_path() / (stem + suffix),
                                      std::move(skin));
            }
        }
        if (sidecars.empty()) {
            // A model whose index data did not arrive draws as a burst of
            // spikes from the origin - far more obviously wrong than the model
            // it would have replaced.
            ++result.missingSkin;
            continue;
        }

        // A creature dressed from its CreatureDisplayInfo row brings the later
        // copies of the skins that row names, or does not come at all. The
        // mesh is laid out for them: Legion's ogre wore 3.3.5's
        // Ogre02SkinBlue512 across a body UV-mapped for Legion's repaint of that
        // same file, and the Warmaul Reavers in Nagrand came out a patchwork.
        //
        // Only when a batch draws one of those slots. A model that carries
        // them unused is dressed by its own named textures, which the checks
        // below bring over or refuse.
        SkinFetch tableSkinFetch;
        if (const std::vector<int> columns = drawnTableColumns(sidecars.front().second, body);
            !columns.empty()) {
            tableSkinFetch = fetchTableSkins(where, columns);
            if (!tableSkinFetch.ok) {
                ++result.dressedByTable;
                if (say) say("    " + where + " not taken: " + tableSkinFetch.why);
                continue;
            }
        }

        std::vector<std::size_t> unnamedOwn;
        std::vector<std::size_t> namedSlots;
        const std::vector<std::string> textures = texturePaths(body, &unnamedOwn, &namedSlots);
        if (!unnamedOwn.empty() && anyBatchSamples(sidecars.front().second, body, unnamedOwn)) {
            // A type 0 slot with no name is a model naming its own texture by
            // FileDataID through a chunk this does not read. The client cannot
            // fill such a slot and draws it flat white - but only if something
            // draws it at all. Models carry slots no batch ever samples, and
            // refusing those turns away a model that would have been fine.
            ++result.refusedByGate;
            continue;
        }

        // Everything is resolved before anything is written: a model whose
        // textures did not arrive is the half-written model the refusals exist
        // to prevent.
        //
        // Half-written means a texture something draws. Later models name
        // files they never sample - reflection and environment maps in slots
        // no batch reaches, a sixth of one install's worth - and the rule that
        // excuses an unnamed slot no batch draws applies to a named one just
        // the same. Such a slot is kept and its name cleared, so the client
        // neither goes looking for the file nor warns that it is not there.
        std::vector<std::pair<std::string, std::vector<uint8_t>>> pending;
        std::vector<std::size_t> unfetched;
        for (std::size_t t = 0; t < textures.size(); ++t) {
            const std::string& texture = textures[t];
            std::string key = lower(texture);
            std::replace(key.begin(), key.end(), '\\', '/');
            if (fetched.count(key)) continue;
            if (failedTextures.count(key)) { unfetched.push_back(namedSlots[t]); continue; }
            std::vector<uint8_t> bytes = source.read(texture);
            if (bytes.empty()) {
                failedTextures.insert(key);
                unfetched.push_back(namedSlots[t]);
                if (say) say("    texture not in this install: " + texture);
                continue;
            }
            pending.emplace_back(texture, std::move(bytes));
        }
        if (!unfetched.empty() && anyBatchSamples(sidecars.front().second, body, unfetched)) {
            ++result.missingTextures;
            continue;
        }

        // Keyframes moved out of the model after Wrath, into files the AFID
        // chunk names by id. The client still looks for them under the Wrath
        // spelling, so that is what they are written as. Left behind, a
        // converted model finds the old client's .anim files instead - written
        // for a skeleton this model no longer has.
        const std::size_t skinCount = sidecars.size();
        auto afid = chunks.find("AFID");
        if (afid != chunks.end()) {
            const std::size_t at = afid->second.first;
            const uint32_t size = afid->second.second;
            for (uint32_t i = 0; (i + 1) * 8 <= size; ++i) {
                const std::size_t entry = at + i * 8;
                if (entry + 8 > blob.size()) break;
                const uint16_t animId = readLE16(blob.data() + entry);
                const uint16_t variation = readLE16(blob.data() + entry + 2);
                const uint32_t animFileId = readLE32(blob.data() + entry + 4);
                if (animFileId == 0) continue;
                std::vector<uint8_t> anim = source.readId(animFileId);
                if (anim.empty()) continue;
                char tail[32];
                std::snprintf(tail, sizeof(tail), "%04u-%02u.anim", animId, variation);
                sidecars.emplace_back(destination.parent_path() / (stem + tail),
                                      std::move(anim));
            }
        }
        if (sidecars.size() == skinCount) {
            // No AFID either, so the sequence array is what names them: every
            // sequence without the flag that says "keyframes are in here" has a
            // file of its own beside the model.
            const uint32_t sequences = readLE32(body.data() + 28);
            const uint32_t at = readLE32(body.data() + 32);
            const std::string base = where.substr(0, where.size() - 3);
            for (uint32_t i = 0; i < sequences && i < 4096; ++i) {
                const std::size_t entry = at + i * kSequenceStride;
                if (entry + kSequenceStride > body.size()) break;
                if (readLE32(body.data() + entry + 12) & kSequenceEmbedded) continue;
                const uint16_t animId = readLE16(body.data() + entry);
                const uint16_t variation = readLE16(body.data() + entry + 2);
                char tail[32];
                std::snprintf(tail, sizeof(tail), "%04u-%02u.anim", animId, variation);
                std::vector<uint8_t> anim = source.read(base + tail);
                if (anim.empty()) continue;
                sidecars.emplace_back(destination.parent_path() / (stem + tail),
                                      std::move(anim));
            }
        }

        std::vector<uint8_t> patched = body;
        writeLE32(patched.data() + 4, kWotlkVersion);
        if (!unfetched.empty()) {
            const uint32_t textureAt = readLE32(patched.data() + kTexturesOffset);
            for (std::size_t slot : unfetched) {
                const std::size_t entry = textureAt + slot * 16;
                writeLE32(patched.data() + entry + 8, 0);    // name length
                writeLE32(patched.data() + entry + 12, 0);   // name offset
            }
            result.unusedTexturesCleared += unfetched.size();
        }
        if (!writeFile(destination, patched.data(), patched.size())) continue;

        for (const auto& [path, bytes] : sidecars) {
            writeFile(path, bytes.data(), bytes.size());
        }
        writeSkins(tableSkinFetch);

        for (auto& [texture, bytes] : pending) {
            // Lowercased, because that is how extraction spells every path it
            // writes. Left at the model's own spelling this is a second copy on
            // a case-insensitive filesystem and a missing texture on a
            // case-sensitive one.
            std::string relative = lower(texture);
            std::replace(relative.begin(), relative.end(), '\\', '/');
            writeFile(fs::path(outputDir) / relative, bytes.data(), bytes.size());
            fetched.insert(relative);
        }

        ++result.written;
        if (say && result.written % 20 == 0) {
            say("    " + std::to_string(result.written) + " models converted");
        }
    }
    return result;
}

}  // namespace wowee::assets
