#include "model_import.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
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
/// How many monster skins a model expects the client to hand it.
///
/// Types 11, 12 and 13 are MONSTER_1 to MONSTER_3, filled from a
/// CreatureDisplayInfo row's three texture columns. A model asking for more of
/// them than the row has is a model the data cannot dress: the columns were
/// written for the model that shipped with them.
std::size_t monsterSkinSlots(const std::vector<uint8_t>& body) {
    if (body.size() < kTexturesOffset + 4) return 0;
    const uint32_t count = readLE32(body.data() + kTexturesCount);
    const uint32_t offset = readLE32(body.data() + kTexturesOffset);
    if (count > 512) return 0;

    std::size_t slots = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const std::size_t entry = offset + std::size_t(i) * 16;
        if (entry + 16 > body.size()) break;
        const uint32_t kind = readLE32(body.data() + entry);
        if (kind >= 11 && kind <= 13) ++slots;
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
        if (!name.empty()) {
            out.push_back(name);
            if (namedSlots) namedSlots->push_back(i);
        } else if (kind == 0 && unnamedOwn) {
            unnamedOwn->push_back(i);
        }
    }
    return out;
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
    // name -> (vertices, game-relative path, came from override, monster skins)
    std::map<std::string, std::tuple<uint32_t, std::string, bool, uint32_t>> found;
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

        // The monster-skin slots, read by seeking to the texture array rather
        // than reading the file: it sits wherever the model put it, which for
        // a character model is megabytes in.
        uint32_t monsterSkins = 0;
        {
            const uint32_t texCount = readLE32(head + kTexturesCount + shift);
            const uint32_t texOffset = readLE32(head + kTexturesOffset + shift);
            if (texCount > 0 && texCount <= 512) {
                std::vector<uint8_t> entries(std::size_t(texCount) * 16);
                in.seekg(texOffset);
                in.read(reinterpret_cast<char*>(entries.data()),
                        static_cast<std::streamsize>(entries.size()));
                if (in.gcount() == static_cast<std::streamsize>(entries.size())) {
                    for (uint32_t i = 0; i < texCount; ++i) {
                        const uint32_t kind = readLE32(entries.data() + std::size_t(i) * 16);
                        if (kind >= 11 && kind <= 13) ++monsterSkins;
                    }
                }
            }
            in.clear();
        }

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
            found[key] = {vertices, relative.generic_string(), overridden, monsterSkins};
        } else {
            const bool wasOverride = std::get<2>(existing->second);
            const uint32_t wasVertices = std::get<0>(existing->second);
            // An override replaces the shipped file whatever its size: it is
            // what the client actually draws, so it is what a candidate has to
            // beat. Measured against the untouched file instead, a pack already
            // installed reads as an improvement worth making again - and a
            // better model gets replaced by a poorer one.
            if (overridden && !wasOverride) {
                existing->second = {vertices, relative.generic_string(), true, monsterSkins};
            } else if (overridden == wasOverride && vertices > wasVertices) {
                existing->second = {vertices, relative.generic_string(), overridden, monsterSkins};
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
            candidate.localMonsterSkins = std::get<3>(record);
            out->push_back(std::move(candidate));
        }
    }
    return found.size();
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

        // A creature is dressed from its CreatureDisplayInfo row, and that row
        // was written for the model that shipped with it. A later model with
        // more monster-skin slots than the one it replaces has slots nothing
        // can fill - Legion's boar carries a second one for its mane, and
        // 3.3.5's row names a body skin and nothing else, so the mane draws
        // untextured however well the rest goes.
        // Only where the model here has monster skins of its own. With none,
        // there is no CreatureDisplayInfo row behind it to be outgrown - a
        // doodad that happens to carry one of these slots is not a creature
        // and is not dressed from that table.
        if (candidate.localMonsterSkins > 0 &&
            monsterSkinSlots(body) > candidate.localMonsterSkins) {
            ++result.needsMoreSkins;
            continue;
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
