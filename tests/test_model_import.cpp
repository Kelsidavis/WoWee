// Bringing a later client's models into the format this one reads.
//
// The importer was once matched on the name inside a model against the name of
// the file it would replace, and the two are not the same thing: Legion's earth
// elemental calls itself "ElementalEarth2" and lives at elementalearth.m2, so
// nothing ever matched and the pass converted nothing while reporting success.
// It matches on path now, which is the same on both sides by construction.
//
// The rest of these are the refusals. A model written without its index data
// draws as a burst of spikes and one written without its textures draws flat
// white - both worse than the model that was already there, so a candidate is
// resolved whole or not written at all.

#include <catch2/catch_amalgamated.hpp>

#include <array>
#include <atomic>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "model_import.hpp"

namespace fs = std::filesystem;
using namespace wowee::assets;

namespace {

void put32(std::vector<uint8_t>& out, std::size_t at, uint32_t value) {
    for (int i = 0; i < 4; ++i) out[at + i] = uint8_t((value >> (i * 8)) & 0xFF);
}
void put16(std::vector<uint8_t>& out, std::size_t at, uint16_t value) {
    out[at] = uint8_t(value & 0xFF);
    out[at + 1] = uint8_t(value >> 8);
}
uint32_t get32(const std::vector<uint8_t>& in, std::size_t at) {
    return uint32_t(in[at]) | (uint32_t(in[at + 1]) << 8) |
           (uint32_t(in[at + 2]) << 16) | (uint32_t(in[at + 3]) << 24);
}

constexpr std::size_t kHeader = 512;   ///< comfortably past every field read

struct TextureSlot {
    uint32_t type = 0;
    std::string path;    ///< empty means the model does not name it
};

/// An MD20 body: the header fields the importer reads, and a texture array.
std::vector<uint8_t> makeBody(uint32_t version, uint32_t vertices,
                              const std::vector<TextureSlot>& textures,
                              uint32_t particles = 0,
                              const std::string& internalName = "SomethingElse") {
    std::vector<uint8_t> body(kHeader, 0);
    std::memcpy(body.data(), "MD20", 4);
    put32(body, 4, version);

    body.insert(body.end(), internalName.begin(), internalName.end());
    body.push_back(0);
    put32(body, 8, uint32_t(internalName.size() + 1));
    put32(body, 12, kHeader);

    put32(body, 60, vertices);
    put32(body, 288, 0);          // ribbons
    put32(body, 296, particles);

    const std::size_t arrayAt = body.size();
    body.resize(arrayAt + textures.size() * 16, 0);
    put32(body, 80, uint32_t(textures.size()));
    put32(body, 84, uint32_t(arrayAt));
    for (std::size_t i = 0; i < textures.size(); ++i) {
        const std::size_t slot = arrayAt + i * 16;
        put32(body, slot, textures[i].type);
        if (textures[i].path.empty()) continue;
        const std::size_t nameAt = body.size();
        body.insert(body.end(), textures[i].path.begin(), textures[i].path.end());
        body.push_back(0);
        put32(body, slot + 8, uint32_t(textures[i].path.size() + 1));
        put32(body, slot + 12, uint32_t(nameAt));
    }

    // The texture-combo list a batch names a run of, one entry per slot so that
    // combo i is slot i.
    const std::size_t comboAt = body.size();
    body.resize(comboAt + textures.size() * 2, 0);
    put32(body, 128, uint32_t(textures.size()));
    put32(body, 132, uint32_t(comboAt));
    for (std::size_t i = 0; i < textures.size(); ++i) {
        put16(body, comboAt + i * 2, uint16_t(i));
    }
    return body;
}

/// A run of the combo list one batch draws.
struct BatchRun {
    uint16_t firstCombo = 0;
    uint16_t comboCount = 1;
};

/// A .skin holding nothing but the batches, which is all the gate reads.
std::vector<uint8_t> makeSkin(const std::vector<BatchRun>& batches) {
    constexpr std::size_t kSkinHeader = 48;
    std::vector<uint8_t> skin(kSkinHeader, 0);
    std::memcpy(skin.data(), "SKIN", 4);
    const std::size_t at = skin.size();
    skin.resize(at + batches.size() * 24, 0);
    put32(skin, 36, uint32_t(batches.size()));
    put32(skin, 40, uint32_t(at));
    for (std::size_t i = 0; i < batches.size(); ++i) {
        put16(skin, at + i * 24 + 14, batches[i].comboCount);
        put16(skin, at + i * 24 + 16, batches[i].firstCombo);
    }
    return skin;
}

/// The MD21 wrapper a later client puts around that body, with the chunks that
/// name its skins and its animations.
std::vector<uint8_t> wrap(const std::vector<uint8_t>& body,
                          const std::vector<uint32_t>& skinIds,
                          const std::vector<std::array<uint32_t, 3>>& anims) {
    std::vector<uint8_t> out;
    auto chunk = [&out](const char* tag, const std::vector<uint8_t>& payload) {
        const std::size_t at = out.size();
        out.resize(at + 8);
        std::memcpy(out.data() + at, tag, 4);
        put32(out, at + 4, uint32_t(payload.size()));
        out.insert(out.end(), payload.begin(), payload.end());
    };

    // MD21 first, as every real one is: the four bytes at the front of the file
    // are what says this is a model at all.
    chunk("MD21", body);
    if (!skinIds.empty()) {
        std::vector<uint8_t> payload(skinIds.size() * 4, 0);
        for (std::size_t i = 0; i < skinIds.size(); ++i) put32(payload, i * 4, skinIds[i]);
        chunk("SFID", payload);
    }
    if (!anims.empty()) {
        std::vector<uint8_t> payload(anims.size() * 8, 0);
        for (std::size_t i = 0; i < anims.size(); ++i) {
            put16(payload, i * 8, uint16_t(anims[i][0]));
            put16(payload, i * 8 + 2, uint16_t(anims[i][1]));
            put32(payload, i * 8 + 4, anims[i][2]);
        }
        chunk("AFID", payload);
    }
    return out;
}

/// A later installation held in memory.
class FakeSource final : public ModelSource {
public:
    std::map<std::string, std::vector<uint8_t>> files;
    std::map<uint32_t, std::vector<uint8_t>> ids;
    std::vector<std::string> asked;

    std::vector<uint8_t> read(const std::string& path, std::size_t limit) override {
        asked.push_back(path);
        auto found = files.find(path);
        if (found == files.end()) return {};
        if (limit > 0 && found->second.size() > limit) {
            return std::vector<uint8_t>(found->second.begin(),
                                        found->second.begin() + std::ptrdiff_t(limit));
        }
        return found->second;
    }
    std::vector<uint8_t> readId(uint32_t fileId) override {
        auto found = ids.find(fileId);
        return found == ids.end() ? std::vector<uint8_t>{} : found->second;
    }
};

/// An extraction to read from and a separate place to write to, so that a
/// model the importer refused can be told apart from the one already there.
struct Sandbox {
    fs::path root;
    fs::path out;
    Sandbox() {
        static std::atomic<int> counter{0};
        root = fs::temp_directory_path() /
               ("wowee_model_import_" + std::to_string(counter.fetch_add(1)) + "_" +
                std::to_string(reinterpret_cast<uintptr_t>(this)));
        out = root / "written";
        fs::create_directories(root);
        fs::create_directories(out);
    }
    ~Sandbox() { std::error_code ec; fs::remove_all(root, ec); }

    /// An extracted model already here, of the size given.
    void installed(const std::string& relative, uint32_t vertices,
                   const std::vector<TextureSlot>& textures = {}) {
        const fs::path at = root / relative;
        fs::create_directories(at.parent_path());
        const std::vector<uint8_t> body = makeBody(264, vertices, textures);
        std::ofstream out(at, std::ios::binary);
        out.write(reinterpret_cast<const char*>(body.data()), std::streamsize(body.size()));
    }
    [[nodiscard]] bool has(const std::string& relative) const {
        return fs::exists(out / relative);
    }
    [[nodiscard]] std::vector<uint8_t> readBack(const std::string& relative) const {
        std::ifstream in(out / relative, std::ios::binary);
        return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                                    std::istreambuf_iterator<char>());
    }
};

const std::atomic<bool> kNeverCancelled{false};

ImportResult run(FakeSource& source, Sandbox& box, const std::string& prefix = "creature") {
    return importModels(source, box.root.string(), box.out.string(), prefix, 1.3f,
                        nullptr, kNeverCancelled);
}

/// A texture the model names for itself, and its bytes in the later install -
/// the ordinary model, which is not dressed from anyone's table.
const TextureSlot kOwnTexture{0, "Creature\\Thing\\Own.blp"};
void offerOwnTexture(FakeSource& source) {
    source.files[kOwnTexture.path] = std::vector<uint8_t>(16, 0x11);
}

void writeLE(std::vector<uint8_t>& out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(uint8_t((v >> (i * 8)) & 0xFF));
}

/// A WDBC with `fields` columns. Each row gives its numbers; a string column
/// is given as an index into `strings`, and written as its offset.
void writeDbc(const fs::path& path, uint32_t fields,
              const std::vector<std::vector<uint32_t>>& rows,
              const std::vector<std::string>& strings) {
    std::string block(1, '\0');
    for (const std::string& str : strings) {
        block += str;
        block.push_back('\0');
    }
    std::vector<uint8_t> out = {'W', 'D', 'B', 'C'};
    writeLE(out, uint32_t(rows.size()));
    writeLE(out, fields);
    writeLE(out, fields * 4);
    writeLE(out, uint32_t(block.size()));
    for (const auto& row : rows) {
        for (uint32_t f = 0; f < fields; ++f) writeLE(out, f < row.size() ? row[f] : 0);
    }
    out.insert(out.end(), block.begin(), block.end());
    fs::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary)
        .write(reinterpret_cast<const char*>(out.data()), std::streamsize(out.size()));
}

constexpr uint32_t kNoString = 0xFFFFFFFFu;

/// CreatureModelData naming one model, and CreatureDisplayInfo rows dressing
/// it: each row's three skin names, "" for none. Where 3.3.5 keeps them:
/// the model path in field 2, the model id in field 1, skins in 6 to 8.
void writeDisplayTables(Sandbox& box, const std::string& modelPath,
                        const std::vector<std::array<std::string, 3>>& rows) {
    writeDbc(box.root / "dbfilesclient" / "creaturemodeldata.dbc", 5,
             {{7, 0, 1}}, {modelPath});
    std::vector<std::string> strings;
    std::vector<std::vector<uint32_t>> table;
    uint32_t id = 100;
    for (const auto& skins : rows) {
        std::vector<uint32_t> row(11, 0);
        row[0] = id++;
        row[1] = 7;
        for (int c = 0; c < 3; ++c) {
            if (skins[c].empty()) { row[6 + c] = kNoString; continue; }
            row[6 + c] = uint32_t(strings.size());
            strings.push_back(skins[c]);
        }
        table.push_back(row);
    }
    // String columns become offsets: 0 is the empty string at the front.
    std::vector<uint32_t> offsets;
    uint32_t at = 1;
    for (const std::string& str : strings) { offsets.push_back(at); at += uint32_t(str.size()) + 1; }
    for (auto& row : table) {
        for (int c = 6; c < 9; ++c) row[c] = (row[c] == kNoString) ? 0 : offsets[row[c]];
    }
    writeDbc(box.root / "dbfilesclient" / "creaturedisplayinfo.dbc", 11, table, strings);
}

/// A file as extraction left it, under the sandbox root.
void extracted(Sandbox& box, const std::string& relative, uint8_t fill) {
    const fs::path at = box.root / relative;
    fs::create_directories(at.parent_path());
    std::ofstream(at, std::ios::binary) << std::string(16, char(fill));
}

}  // namespace

TEST_CASE("a model is found by its path, not by the name inside it") {
    Sandbox box;
    box.installed("creature/elementalearth/elementalearth.m2", 1328);

    FakeSource source;
    // The name inside says ElementalEarth2 - as Legion's really does. Matching
    // on it finds nothing; matching on the path finds this.
    source.files["creature/elementalearth/elementalearth.m2"] =
        wrap(makeBody(274, 3830, {kOwnTexture}, 0, "ElementalEarth2"), {7001}, {});
    source.ids[7001] = makeSkin({{0, 1}});
    offerOwnTexture(source);

    const ImportResult result = run(source, box);

    CHECK(result.written == 1);
    CHECK(box.has("creature/elementalearth/elementalearth.m2"));
    CHECK(box.has("creature/elementalearth/elementalearth00.skin"));
}

TEST_CASE("the model is restamped to the version this client reads") {
    Sandbox box;
    box.installed("creature/murloc/murloc.m2", 100);

    FakeSource source;
    source.files["creature/murloc/murloc.m2"] =
        wrap(makeBody(274, 900, {kOwnTexture}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});
    offerOwnTexture(source);

    REQUIRE(run(source, box).written == 1);

    const std::vector<uint8_t> written = box.readBack("creature/murloc/murloc.m2");
    REQUIRE(written.size() > 8);
    CHECK(std::memcmp(written.data(), "MD20", 4) == 0);   // the wrapper is gone
    CHECK(get32(written, 4) == 264);
}

TEST_CASE("a model no bigger than the one already here is left alone") {
    Sandbox box;
    box.installed("creature/gazelle/gazelle.m2", 1025);

    FakeSource source;
    source.files["creature/gazelle/gazelle.m2"] =
        wrap(makeBody(274, 1025, {{11, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.notBetter == 1);
}

TEST_CASE("a model that names its own texture and does not say which is refused") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    // Type 0 is the model naming its own file. An empty name there means it is
    // named by id through a chunk this does not read, and the client draws the
    // slot flat white.
    source.files["creature/thing/thing.m2"] =
        wrap(makeBody(274, 900, {{11, ""}, {0, ""}}), {1}, {});
    source.ids[1] = makeSkin({{1, 1}});   // the batch draws slot 1, the bad one

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.refusedByGate == 1);
    CHECK_FALSE(box.has("creature/thing/thing.m2"));
}

TEST_CASE("an unnamed slot no batch draws is not a reason to refuse a model") {
    Sandbox box;
    box.installed("creature/compy/compy.m2", 100);

    FakeSource source;
    // Measured on a real one: compy carries a type 0 slot with no name, and its
    // only batch samples the creature-skin slot beside it. Nothing ever reads
    // the empty one, so refusing the model over it turns away a model that
    // draws correctly.
    source.files["creature/compy/compy.m2"] =
        wrap(makeBody(274, 900, {kOwnTexture, {0, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});   // the batch draws slot 0 only
    offerOwnTexture(source);

    const ImportResult result = run(source, box);
    CHECK(result.refusedByGate == 0);
    CHECK(result.written == 1);
}

TEST_CASE("a creature brings the skins its display rows name") {
    // The Warmaul Reaver: Legion kept display 19906 pointing at ogre02.m2 and
    // creature/ogre02/ogre02skinblue512.blp, and repainted that file for its
    // new mesh. Without the repaint the new mesh wore the old skin.
    Sandbox box;
    box.installed("creature/ogre02/ogre02.m2", 100);
    writeDisplayTables(box, "Creature\\Ogre02\\Ogre02.mdx",
                       {{"Ogre02SkinBlue512", "Ogre02ShoulderBlue", ""}});
    extracted(box, "creature/ogre02/ogre02skinblue512.blp", 0x01);
    extracted(box, "creature/ogre02/ogre02shoulderblue.blp", 0x02);

    FakeSource source;
    source.files["creature/ogre02/ogre02.m2"] =
        wrap(makeBody(274, 900, {{11, ""}, {12, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}, {1, 1}});
    source.files["creature/ogre02/ogre02skinblue512.blp"] = std::vector<uint8_t>(16, 0xA1);
    source.files["creature/ogre02/ogre02shoulderblue.blp"] = std::vector<uint8_t>(16, 0xA2);

    const ImportResult result = run(source, box);
    CHECK(result.written == 1);
    CHECK(result.tableSkinsBrought == 2);
    // Beside the extracted skins, not over them: the client reads the override
    // directory first, and the extracted copy is how the next run can tell.
    CHECK(box.has("override/creature/ogre02/ogre02skinblue512.blp"));
    CHECK(box.has("override/creature/ogre02/ogre02shoulderblue.blp"));
}

TEST_CASE("a creature whose skin was not repainted is refused") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);
    writeDisplayTables(box, "Creature\\Thing\\Thing.mdx", {{"ThingSkin", "", ""}});
    extracted(box, "creature/thing/thingskin.blp", 0x05);

    FakeSource source;
    source.files["creature/thing/thing.m2"] = wrap(makeBody(274, 900, {{11, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});
    // The same bytes in both: the later client dresses its mesh from other
    // files, which these rows never name.
    source.files["creature/thing/thingskin.blp"] = std::vector<uint8_t>(16, 0x05);

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.dressedByTable == 1);
    CHECK_FALSE(box.has("creature/thing/thing.m2"));
}

TEST_CASE("a creature drawing a skin its rows leave empty is refused") {
    // The boar: Legion's draws a second skin, for its mane, and 3.3.5's row
    // names one. The mane would draw bare however the first one arrived.
    Sandbox box;
    box.installed("creature/boar/boar.m2", 401);
    writeDisplayTables(box, "Creature\\Boar\\Boar.mdx", {{"BoarSkin", "", ""}});

    FakeSource source;
    source.files["creature/boar/boar.m2"] =
        wrap(makeBody(274, 1841, {{11, ""}, {12, "maehne"}}), {1}, {});
    source.ids[1] = makeSkin({{0, 2}});
    source.files["creature/boar/boarskin.blp"] = std::vector<uint8_t>(16, 0x33);

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.dressedByTable == 1);
}

TEST_CASE("a creature with no display row is refused") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    source.files["creature/thing/thing.m2"] = wrap(makeBody(274, 900, {{11, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.dressedByTable == 1);
}

TEST_CASE("an earlier import is given the skins it was missing") {
    // What the pipeline before this one left in override/: Legion's mesh, and
    // the extracted skins still under it.
    Sandbox box;
    box.installed("creature/ogre02/ogre02.m2", 900, {{11, ""}});
    box.installed("override/creature/ogre02/ogre02.m2", 900, {{11, ""}});
    std::ofstream(box.root / "override/creature/ogre02/ogre0200.skin", std::ios::binary)
        << std::string(reinterpret_cast<const char*>(makeSkin({{0, 1}}).data()),
                       makeSkin({{0, 1}}).size());
    writeDisplayTables(box, "Creature\\Ogre02\\Ogre02.mdx", {{"Ogre02SkinBlue512", "", ""}});
    extracted(box, "creature/ogre02/ogre02skinblue512.blp", 0x01);

    FakeSource source;
    source.files["creature/ogre02/ogre02.m2"] = wrap(makeBody(274, 900, {{11, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});
    source.files["creature/ogre02/ogre02skinblue512.blp"] = std::vector<uint8_t>(16, 0xA1);

    // Run where the override directory is: the sandbox root is both.
    const ImportResult result = importModels(source, box.root.string(), box.root.string(),
                                             "creature", 1.3f, nullptr, kNeverCancelled);
    CHECK(result.earlierImportsDressed == 1);
    CHECK(result.tableSkinsBrought == 1);
    CHECK(fs::exists(box.root / "override/creature/ogre02/ogre02skinblue512.blp"));

    // And a second run has nothing left to bring.
    const ImportResult again = importModels(source, box.root.string(), box.root.string(),
                                            "creature", 1.3f, nullptr, kNeverCancelled);
    CHECK(again.earlierImportsDressed == 0);
    CHECK(again.earlierImportsRemoved == 0);
}

TEST_CASE("an earlier import nothing can dress is removed") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 900, {{11, ""}});
    box.installed("override/creature/thing/thing.m2", 900, {{11, ""}});
    const std::vector<uint8_t> skin = makeSkin({{0, 1}});
    auto put = [&](const char* rel) {
        std::ofstream(box.root / rel, std::ios::binary)
            .write(reinterpret_cast<const char*>(skin.data()), std::streamsize(skin.size()));
    };
    put("override/creature/thing/thing00.skin");
    put("override/creature/thing/thing0060-00.anim");
    put("override/creature/thing/thingwarlord00.skin");   // another model's, kept
    writeDisplayTables(box, "Creature\\Thing\\Thing.mdx", {{"ThingSkin", "", ""}});
    extracted(box, "creature/thing/thingskin.blp", 0x05);

    FakeSource source;
    source.files["creature/thing/thing.m2"] = wrap(makeBody(274, 900, {{11, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});
    source.files["creature/thing/thingskin.blp"] = std::vector<uint8_t>(16, 0x05);

    const ImportResult result = importModels(source, box.root.string(), box.root.string(),
                                             "creature", 1.3f, nullptr, kNeverCancelled);
    CHECK(result.earlierImportsRemoved == 1);
    CHECK_FALSE(fs::exists(box.root / "override/creature/thing/thing.m2"));
    CHECK_FALSE(fs::exists(box.root / "override/creature/thing/thing00.skin"));
    CHECK_FALSE(fs::exists(box.root / "override/creature/thing/thing0060-00.anim"));
    CHECK(fs::exists(box.root / "override/creature/thing/thingwarlord00.skin"));
    CHECK(fs::exists(box.root / "creature/thing/thing.m2"));   // the extracted one stays
}

TEST_CASE("an earlier import that is not the later model at its path is removed") {
    // An earlier pack filed a different model under this name - Legion's
    // HorseMultiSaddle as creature/ridinghorse/ridinghorse.m2, while Legion's
    // own file there is the horse 3.3.5 ships. The skins its displays name are
    // for that horse, and cannot fit what was installed.
    Sandbox box;
    box.installed("creature/horse/horse.m2", 673, {{11, ""}});
    box.installed("override/creature/horse/horse.m2", 3155, {{11, ""}});
    const std::vector<uint8_t> skin = makeSkin({{0, 1}});
    std::ofstream(box.root / "override/creature/horse/horse00.skin", std::ios::binary)
        .write(reinterpret_cast<const char*>(skin.data()), std::streamsize(skin.size()));
    writeDisplayTables(box, "Creature\\Horse\\Horse.mdx", {{"HorseSkin", "", ""}});

    FakeSource source;
    source.files["creature/horse/horse.m2"] = wrap(makeBody(274, 673, {{11, ""}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});

    const ImportResult result = importModels(source, box.root.string(), box.root.string(),
                                             "creature", 1.3f, nullptr, kNeverCancelled);
    CHECK(result.earlierImportsRemoved == 1);
    CHECK(result.written == 0);
    CHECK_FALSE(fs::exists(box.root / "override/creature/horse/horse.m2"));
    CHECK_FALSE(fs::exists(box.root / "override/creature/horse/horse00.skin"));
    CHECK(fs::exists(box.root / "creature/horse/horse.m2"));
}

TEST_CASE("a model whose texture is not in this installation is refused") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    source.files["creature/thing/thing.m2"] =
        wrap(makeBody(274, 900, {{0, "Creature\\Thing\\Thing.blp"}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});
    // and no texture in source.files

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.missingTextures == 1);
    CHECK_FALSE(box.has("creature/thing/thing.m2"));
}

TEST_CASE("a missing texture no batch draws is cleared, not a refusal") {
    Sandbox box;
    box.installed("creature/frog/frog.m2", 100);

    FakeSource source;
    // Measured on real imports: a sixth of them name a reflection map in a
    // slot no batch reaches. The frog names Creature\Frog2\oldglass.blp and
    // draws only its creature skin.
    source.files["creature/frog/frog.m2"] =
        wrap(makeBody(274, 900, {kOwnTexture, {0, "Creature\\Frog2\\oldglass.blp"}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});   // the batch draws slot 0 only
    offerOwnTexture(source);

    const ImportResult result = run(source, box);
    CHECK(result.written == 1);
    CHECK(result.missingTextures == 0);
    CHECK(result.unusedTexturesCleared == 1);

    // The name is gone from the written model, so the client does not go
    // looking for a file that was never brought over.
    const std::vector<uint8_t> written = box.readBack("creature/frog/frog.m2");
    REQUIRE(written.size() > 88);
    const uint32_t textureAt = get32(written, 84);
    REQUIRE(written.size() >= textureAt + 32);
    CHECK(get32(written, textureAt + 16 + 8) == 0);
    CHECK(get32(written, textureAt + 16 + 12) == 0);
}

TEST_CASE("a missing texture drawn as a second layer still refuses the model") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    source.files["creature/thing/thing.m2"] =
        wrap(makeBody(274, 900, {kOwnTexture, {0, "Creature\\Thing\\Shine.blp"}}), {1}, {});
    source.ids[1] = makeSkin({{0, 2}});   // one batch, both layers
    offerOwnTexture(source);

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.missingTextures == 1);
    CHECK_FALSE(box.has("creature/thing/thing.m2"));
}

TEST_CASE("a leftover name on a creature-skin slot is not fetched or refused") {
    Sandbox box;
    box.installed("creature/boar/boar.m2", 100, {{11, ""}});

    FakeSource source;
    // The client fills type 11 from CreatureDisplayInfo and never loads a name
    // found there - Legion's boar carries one for a file this install lacks.
    source.files["creature/boar/boar.m2"] =
        wrap(makeBody(274, 900, {{11, "Creature\\Boar\\Maehne.blp"}}), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});
    writeDisplayTables(box, "Creature\\Boar\\Boar.mdx", {{"BoarSkin", "", ""}});
    source.files["creature/boar/boarskin.blp"] = std::vector<uint8_t>(16, 0x33);

    const ImportResult result = run(source, box);
    CHECK(result.written == 1);
    CHECK(result.missingTextures == 0);
    for (const std::string& asked : source.asked) CHECK(asked.find("Maehne") == std::string::npos);
}

TEST_CASE("a texture is written at the spelling extraction uses") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    source.files["creature/thing/thing.m2"] =
        wrap(makeBody(274, 900, {{0, "Creature\\Thing\\THING.BLP"}}), {1}, {});
    source.files["Creature\\Thing\\THING.BLP"] = std::vector<uint8_t>(16, 0x42);
    source.ids[1] = makeSkin({{0, 1}});

    REQUIRE(run(source, box).written == 1);
    // Extraction writes every path lowercased with forward slashes. Left at the
    // model's own spelling this is a second copy on a case-insensitive
    // filesystem and a texture the client cannot find on a case-sensitive one.
    CHECK(box.has("creature/thing/thing.blp"));
}

TEST_CASE("a model whose index data did not come with it is refused") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    source.files["creature/thing/thing.m2"] =
        wrap(makeBody(274, 900, {{11, ""}}), {4242}, {});
    // 4242 resolves to nothing, and there is no skin beside it either.

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.missingSkin == 1);
    CHECK_FALSE(box.has("creature/thing/thing.m2"));
}

TEST_CASE("animations are written under the names the client looks for") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    source.files["creature/thing/thing.m2"] =
        wrap(makeBody(274, 900, {kOwnTexture}), {1}, {{{0, 0, 90}}, {{61, 1, 91}}});
    source.ids[1] = makeSkin({{0, 1}});
    offerOwnTexture(source);
    source.ids[90] = std::vector<uint8_t>(8, 0);
    source.ids[91] = std::vector<uint8_t>(8, 0);

    REQUIRE(run(source, box).written == 1);
    CHECK(box.has("creature/thing/thing0000-00.anim"));
    CHECK(box.has("creature/thing/thing0061-01.anim"));
}

TEST_CASE("an earlier installation names its skins by path instead") {
    Sandbox box;
    box.installed("world/tree/tree.m2", 156);

    FakeSource source;
    // No MD21 wrapper and no SFID: a pre-Warlords client keeps the skins beside
    // the model, under the names the client builds from the model's own path.
    source.files["world/tree/tree.m2"] = makeBody(264, 897, {kOwnTexture});
    offerOwnTexture(source);
    source.files["world/tree/tree00.skin"] = makeSkin({{0, 1}});
    source.files["world/tree/tree01.skin"] = makeSkin({{0, 1}});

    const ImportResult result = run(source, box, "world");
    CHECK(result.written == 1);
    CHECK(box.has("world/tree/tree00.skin"));
    CHECK(box.has("world/tree/tree01.skin"));
}

TEST_CASE("a model that emits particles is left alone") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);

    FakeSource source;
    // The particle struct grew after Wrath, so a model carrying one cannot be
    // converted by restamping the version.
    source.files["creature/thing/thing.m2"] =
        wrap(makeBody(274, 900, {{11, ""}}, 3), {1}, {});
    source.ids[1] = makeSkin({{0, 1}});

    const ImportResult result = run(source, box);
    CHECK(result.written == 0);
    CHECK(result.hasEmitters == 1);
}

TEST_CASE("only the asked-for prefix is looked at") {
    Sandbox box;
    box.installed("creature/thing/thing.m2", 100);
    box.installed("world/tree/tree.m2", 100);

    FakeSource source;
    run(source, box, "world");

    for (const std::string& path : source.asked) {
        CHECK(path.rfind("world", 0) == 0);
    }
    CHECK_FALSE(source.asked.empty());
}

TEST_CASE("an earlier import's missing texture no batch draws is cleared") {
    // What the pipeline before this importer left behind: a model in override/
    // naming a reflection map that was never brought over.
    Sandbox box;
    const fs::path dir = box.root / "override" / "creature" / "frog";
    fs::create_directories(dir);
    const std::vector<uint8_t> body = makeBody(264, 900, {
        {11, ""},
        {0, "Creature\\Frog2\\oldglass.blp"},     // missing, drawn by nothing
        {0, "Creature\\Frog\\Shine.blp"},         // missing, and a batch draws it
        {0, "Creature\\Frog\\Here.blp"},          // present
    });
    std::ofstream(dir / "frog.m2", std::ios::binary)
        .write(reinterpret_cast<const char*>(body.data()), std::streamsize(body.size()));
    const std::vector<uint8_t> skin = makeSkin({{0, 1}, {2, 1}, {3, 1}});
    std::ofstream(dir / "frog00.skin", std::ios::binary)
        .write(reinterpret_cast<const char*>(skin.data()), std::streamsize(skin.size()));
    fs::create_directories(box.root / "creature" / "frog");
    std::ofstream(box.root / "creature" / "frog" / "here.blp") << "x";

    const RepairResult result = repairEarlierImports(box.root.string());
    CHECK(result.modelsLooked == 1);
    CHECK(result.modelsRepaired == 1);
    CHECK(result.namesCleared == 1);
    CHECK(result.leftDrawn == 1);

    std::ifstream in(dir / "frog.m2", std::ios::binary);
    const std::vector<uint8_t> after((std::istreambuf_iterator<char>(in)),
                                     std::istreambuf_iterator<char>());
    const uint32_t textureAt = get32(after, 84);
    CHECK(get32(after, textureAt + 1 * 16 + 8) == 0);   // cleared
    CHECK(get32(after, textureAt + 2 * 16 + 8) != 0);   // drawn: kept
    CHECK(get32(after, textureAt + 3 * 16 + 8) != 0);   // present: kept

    // Run again and there is nothing left to do.
    CHECK(repairEarlierImports(box.root.string()).modelsRepaired == 0);
}
