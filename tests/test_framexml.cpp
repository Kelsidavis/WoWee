#include <catch_amalgamated.hpp>

#include "ui/xml_parser.hpp"
#include "ui/framexml_emitter.hpp"

#include <fstream>
#include <iterator>
#include <string>

using namespace wowee::ui;

namespace {
bool has(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}
XmlNode parseOrFail(const std::string& src) {
    XmlNode root;
    std::string err;
    REQUIRE(parseXml(src, root, err));
    INFO(err);
    return root;
}
}

// ── Parser ──────────────────────────────────────────────────────────────────

TEST_CASE("Attributes, nesting and self-closing elements", "[framexml][xml]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name='A' hidden='true'><Size><AbsDimension x='10' y='20'/></Size></Frame></Ui>");
    REQUIRE(root.name == "Ui");
    REQUIRE(root.children.size() == 1);
    const XmlNode& f = root.children[0];
    REQUIRE(f.name == "Frame");
    REQUIRE(f.attrOr("name", "") == "A");
    REQUIRE(f.attrBool("hidden"));
    const XmlNode* dim = f.child("Size")->child("AbsDimension");
    REQUIRE(dim != nullptr);
    REQUIRE(dim->attrFloat("x") == Catch::Approx(10.0f));
    REQUIRE(dim->attrFloat("y") == Catch::Approx(20.0f));
}

TEST_CASE("The declaration and comments are skipped", "[framexml][xml]") {
    XmlNode root = parseOrFail(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<!-- a comment, possibly with <tags> inside -->\n"
        "<Ui><!-- another --><Frame name=\"A\"/></Ui>");
    REQUIRE(root.name == "Ui");
    REQUIRE(root.children.size() == 1);
    REQUIRE(root.children[0].attrOr("name", "") == "A");
}

TEST_CASE("CDATA is taken verbatim", "[framexml][xml]") {
    // The reason CDATA matters: this is Lua, and decoding entities inside it
    // would turn every comparison into something that will not compile.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"A\"><Scripts><OnLoad><![CDATA[\n"
        "if a < b and c > d then self:Show() end\n"
        "]]></OnLoad></Scripts></Frame></Ui>");
    const XmlNode* onLoad = root.children[0].child("Scripts")->child("OnLoad");
    REQUIRE(onLoad != nullptr);
    REQUIRE(has(onLoad->text, "a < b and c > d"));
}

TEST_CASE("Entities decode outside CDATA", "[framexml][xml]") {
    XmlNode root = parseOrFail("<Ui><Frame text=\"Fish &amp; Chips &lt;3\"/></Ui>");
    REQUIRE(root.children[0].attrOr("text", "") == "Fish & Chips <3");
}

TEST_CASE("Malformed input is reported, not thrown", "[framexml][xml]") {
    // One bad file among a hundred must not take the rest of the interface down.
    XmlNode root;
    std::string err;
    REQUIRE_FALSE(parseXml("<Ui><Frame></Ui>", root, err));
    REQUIRE_FALSE(err.empty());

    REQUIRE_FALSE(parseXml("", root, err));
    REQUIRE_FALSE(parseXml("<Ui><Frame name=unquoted/></Ui>", root, err));
}

// ── Emitter ─────────────────────────────────────────────────────────────────

TEST_CASE("A frame emits CreateFrame with its type and parent", "[framexml][emit]") {
    XmlNode root = parseOrFail("<Ui><Button name=\"MyButton\" parent=\"UIParent\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "CreateFrame(\"Button\", \"MyButton\", UIParent)"));
}

TEST_CASE("hidden=\"true\" is applied before the children exist", "[framexml][emit]") {
    // The attribute is the frame's state from the moment it exists, not
    // something done to it once built. Emitted last, a frame the interface
    // declares hidden was visible for the whole of its own construction - and
    // FrameXML guards on exactly that. SpellButton_UpdateButton opens with
    // `if ( not this:IsVisible() ) then return end`, and the spell buttons are
    // built and fired before SpellBookFrame_OnLoad has run: on the real client
    // the guard holds, here it did not, and the button ran on into a page
    // number table that the parent's OnLoad fills a step later. That nil
    // arithmetic took spellbookframe.xml down whole on a 1.12 interface.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Outer\" hidden=\"true\" parent=\"UIParent\">"
        "<Frames><Frame name=\"Inner\"/></Frames>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);

    const size_t hide = r.lua.find(":Hide()");
    const size_t inner = r.lua.find("\"Inner\"");
    REQUIRE(hide != std::string::npos);
    REQUIRE(inner != std::string::npos);
    CHECK(hide < inner);
}

TEST_CASE("a frame with no hidden attribute is not hidden", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Plain\" parent=\"UIParent\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    CHECK(r.lua.find(":Hide()") == std::string::npos);
}

// ── The real files ──────────────────────────────────────────────────────────
//
// The cases above emit XML written for them, which proves the emitter handles a
// shape and nothing about whether it handles Blizzard's. characterframe.xml has
// been reported three times as a window that will not open, and every link in
// ToggleCharacter's chain reads correct - so the question worth asking here is
// the one that can be answered without the game: does the frame get built at
// all, and does its first tab get the id ToggleCharacter reads?

namespace {
/// A FrameXML file as shipped, or an empty node when it is not there - the
/// tests using it skip rather than fail, since the interface is data and a
/// checkout without it is not a broken emitter.
XmlNode parseShippedFile(const std::string& name) {
    XmlNode root;
    // Anchored to the source tree rather than the working directory. ctest runs
    // from the build directory, where a relative path finds nothing and the
    // test passes by skipping - which is worse than not having it.
    std::ifstream in(std::string(WOWEE_SOURCE_DIR) +
                     "/Data/interface/framexml/" + name);
    if (!in) return root;
    std::string src((std::istreambuf_iterator<char>(in)),
                    std::istreambuf_iterator<char>());
    std::string err;
    parseXml(src, root, err);
    return root;
}
}  // namespace

// The achievement banner is what these two cost. AlertFrame_AnimateIn plays the
// frame's animIn, then the glow's and the shine's - which are declared on the
// *textures* - and then plays waitAndAnimOut, whose animOut it reaches through
// the group. Losing either meant the third and fourth lines never ran: the
// banner appeared, the fade that hides it was never started, and it sat there
// for the rest of the session. Nothing raised where anyone could see it.
TEST_CASE("a texture may animate itself", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name='Alert'><Layers><Layer level='OVERLAY'>"
        "<Texture name='$parentGlow' parentKey='glow'>"
        "<Animations><AnimationGroup name='$parentAnimIn' parentKey='animIn'>"
        "<Alpha change='1' duration='0.2' order='1'/>"
        "</AnimationGroup></Animations>"
        "</Texture></Layer></Layers></Frame></Ui>");
    const std::string lua = emitFrameXml(root).lua;
    INFO(lua);
    // The group is created on the texture, not on the frame that holds it.
    CHECK(has(lua, ":CreateAnimationGroup(\"AlertGlowAnimIn\")"));
    CHECK(has(lua, ".animIn = "));
    CHECK(has(lua, ":SetDuration(0.2)"));
}

TEST_CASE("an animation's parentKey goes on its group", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name='Alert'><Animations>"
        "<AnimationGroup name='$parentWaitAndAnimOut' parentKey='waitAndAnimOut'>"
        "<Alpha startDelay='4.05' change='-1' duration='1.5' parentKey='animOut'/>"
        "</AnimationGroup></Animations></Frame></Ui>");
    const std::string lua = emitFrameXml(root).lua;
    INFO(lua);
    // The group hangs off the frame and the animation off the group, which is
    // how alertframes.lua reaches it: frame.waitAndAnimOut.animOut.
    const size_t group = lua.find(" = __w[1]:CreateAnimationGroup(");
    REQUIRE(group != std::string::npos);
    const std::string gvar = lua.substr(lua.rfind('\n', group) + 1,
                                        group - lua.rfind('\n', group) - 1);
    CHECK(has(lua, "__w[1].waitAndAnimOut = " + gvar));
    CHECK(has(lua, gvar + ".animOut = "));
    CHECK_FALSE(has(lua, "__w[1].animOut = "));
    CHECK(has(lua, ":SetStartDelay(4.05)"));
}

// A virtual frame's name may be written as an element. Turtle's LootFrame.xml
// declares <Button name="LootButton" virtual="true"> and then writes
// <LootButton name="LootButton1"/>, which the real client accepts. This
// reported an unknown type and built nothing inside it, so the loot window came
// up with no buttons - reported as issue 132.
TEST_CASE("a template's name may be used as an element", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui>"
        "<Button name='LootButton' virtual='true'><Size><AbsDimension x='10' y='10'/></Size></Button>"
        "<LootButton name='LootButton1' id='1'/>"
        "</Ui>");
    const EmitResult r = emitFrameXml(root);
    INFO(r.lua);
    // The template records what kind of frame it makes...
    CHECK(has(r.lua, "__WoweeTemplateTypes[\"LootButton\"] = \"Button\""));
    // ...and the element is created as that kind and inherits it.
    CHECK(has(r.lua, "CreateFrame((__WoweeTemplateTypes[\"LootButton\"] or \"Frame\")"));
    CHECK(has(r.lua, "__WoweeTemplates[\"LootButton\"]("));
    CHECK(has(r.lua, ":SetID(1)"));
    // And with the template declared in the same file it is not reported: the
    // emitter can see it is a template rather than a typo.
    for (const std::string& w : r.warnings) {
        CHECK(w.find("LootButton") == std::string::npos);
    }
}

TEST_CASE("an unknown element with no name is still reported", "[framexml][emit]") {
    XmlNode root = parseOrFail("<Ui><Nonsense><Frame name='Inner'/></Nonsense></Ui>");
    const EmitResult r = emitFrameXml(root);
    bool said = false;
    for (const std::string& w : r.warnings) {
        if (w.find("Nonsense") != std::string::npos) said = true;
    }
    CHECK(said);
}

TEST_CASE("characterframe.xml builds the frame the C key opens",
          "[framexml][emit][shipped]") {
    XmlNode root = parseShippedFile("characterframe.xml");
    if (root.children.empty()) return;   // interface data not present
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "CreateFrame(\"Frame\", \"CharacterFrame\", UIParent)"));
    // Hidden in its XML, which is what makes ToggleCharacter take the show
    // branch rather than the hide one on the first press.
    REQUIRE(has(r.lua, ":Hide()"));
}

TEST_CASE("paperdollframe.xml builds the tab and gives it its id",
          "[framexml][emit][shipped]") {
    XmlNode root = parseShippedFile("paperdollframe.xml");
    if (root.children.empty()) return;
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "\"PaperDollFrame\""));
    // ToggleCharacter reads subFrame:GetID() and hands it to
    // PanelTemplates_SetTab. The XML says id="1"; without it the tab is zero
    // and the wrong one is selected.
    REQUIRE(has(r.lua, ":SetID(1)"));
}

TEST_CASE("A message frame keeps as many lines as it asks for",
          "[framexml][emit]") {
    // Twenty-two frames ask for three. Without this each kept the default of a
    // hundred and twenty-eight and drew far past the box drawn for it.
    XmlNode root = parseOrFail(
        "<Ui><MessageFrame name=\"M\" parent=\"UIParent\" maxLines=\"3\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetMaxLines(3)"));
}

TEST_CASE("A frame that says nothing about its lines is left alone",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><MessageFrame name=\"M\" parent=\"UIParent\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "SetMaxLines"));
}

TEST_CASE("A binding becomes a listed command and a callable body",
          "[framexml][emit][bindings]") {
    XmlNode root = parseOrFail(
        "<Bindings><Binding name=\"JUMP\" header=\"MOVEMENT\">"
        "DoJump();</Binding></Bindings>");
    const EmitResult r = emitFrameXml(root);
    // The header opens a row of its own, before the command that declared it.
    REQUIRE(has(r.lua, "\"HEADER_MOVEMENT\""));
    REQUIRE(has(r.lua, "\"JUMP\""));
    REQUIRE(r.lua.find("HEADER_MOVEMENT") < r.lua.find("+1] = \"JUMP\""));
    REQUIRE(has(r.lua, "__WoweeBindingScripts[\"JUMP\"] = function(keystate)"));
    REQUIRE(has(r.lua, "DoJump();"));
}

TEST_CASE("A binding without a header adds no row of its own",
          "[framexml][emit][bindings]") {
    XmlNode root = parseOrFail("<Bindings><Binding name=\"JUMP\"/></Bindings>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "HEADER"));
    REQUIRE(has(r.lua, "\"JUMP\""));
    // Nothing to run is not the same as a body that does nothing: RunBinding
    // has to be able to tell them apart.
    REQUIRE_FALSE(has(r.lua, "__WoweeBindingScripts[\"JUMP\"]"));
}

TEST_CASE("runOnUp is carried, because a release means something to some",
          "[framexml][emit][bindings]") {
    XmlNode root = parseOrFail(
        "<Bindings><Binding name=\"MOVEFORWARD\" runOnUp=\"true\">go()</Binding>"
        "<Binding name=\"SCREENSHOT\">snap()</Binding></Bindings>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "__WoweeBindingRunOnUp[\"MOVEFORWARD\"] = true"));
    REQUIRE_FALSE(has(r.lua, "__WoweeBindingRunOnUp[\"SCREENSHOT\"]"));
}

TEST_CASE("A bindings file is not reported as the wrong kind of document",
          "[framexml][emit][bindings]") {
    // <Bindings> is a root in its own right. Warning about it would say the
    // file is broken when it is exactly what it should be.
    XmlNode root = parseOrFail("<Bindings><Binding name=\"JUMP\"/></Bindings>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(r.warnings.empty());
}

TEST_CASE("Size and anchors become the same calls a script would make",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\">"
        "<Size><AbsDimension x=\"128\" y=\"64\"/></Size>"
        "<Anchors><Anchor point=\"TOPLEFT\" relativePoint=\"BOTTOMRIGHT\">"
        "<Offset><AbsDimension x=\"5\" y=\"-7\"/></Offset></Anchor></Anchors>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetSize(128"));
    REQUIRE(has(r.lua, ":SetPoint(\"TOPLEFT\""));
    REQUIRE(has(r.lua, "\"BOTTOMRIGHT\""));
    REQUIRE(has(r.lua, "5.000000, -7.000000"));
}

TEST_CASE("$parent expands against the frame that owns the region",
          "[framexml][emit]") {
    // Nearly every region in the original interface is named this way, and
    // getting it wrong means none of them are reachable by name.
    REQUIRE(substituteParent("$parentText", "FooFrame") == "FooFrameText");
    REQUIRE(substituteParent("PlainName", "FooFrame") == "PlainName");
    REQUIRE(substituteParent("", "FooFrame").empty());

    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"FooFrame\"><Layers><Layer level=\"BACKGROUND\">"
        "<Texture name=\"$parentBg\" file=\"Interface\\Foo\" setAllPoints=\"true\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "CreateTexture(\"FooFrameBg\", \"BACKGROUND\")"));
    REQUIRE(has(r.lua, "SetTexture(\"Interface\\\\Foo\")"));
    REQUIRE(has(r.lua, ":SetAllPoints("));
}

TEST_CASE("alphaMode reaches the texture as a blend mode", "[framexml][emit]") {
    // Art declared this way is a glow on black with no alpha channel of its
    // own. Dropping the attribute draws it as an opaque black shape over
    // whatever it was meant to light up.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"FooFrame\"><Layers><Layer level=\"OVERLAY\">"
        "<Texture name=\"$parentGlow\" file=\"Interface\\Glow\" alphaMode=\"ADD\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetBlendMode(\"ADD\")"));
}

TEST_CASE("A virtual frame becomes a template rather than a frame",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"MyTemplate\" virtual=\"true\">"
        "<Size><AbsDimension x=\"32\" y=\"32\"/></Size></Frame>"
        "<Frame name=\"Real\" inherits=\"MyTemplate\"/></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "__WoweeTemplates[\"MyTemplate\"] = function(self)"));
    // The template must not create a frame of its own; that is the whole
    // meaning of virtual.
    REQUIRE_FALSE(has(r.lua, "CreateFrame(\"Frame\", \"MyTemplate\""));
    REQUIRE(has(r.lua, "CreateFrame(\"Frame\", \"Real\""));
    REQUIRE(has(r.lua, "__WoweeTemplates[\"MyTemplate\"]("));
}

TEST_CASE("Inheriting several templates applies them in order", "[framexml][emit]") {
    XmlNode root = parseOrFail("<Ui><Frame name=\"R\" inherits=\"A, B\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    const size_t a = r.lua.find("__WoweeTemplates[\"A\"](");
    const size_t b = r.lua.find("__WoweeTemplates[\"B\"](");
    REQUIRE(a != std::string::npos);
    REQUIRE(b != std::string::npos);
    REQUIRE(a < b);
}

TEST_CASE("Scripts bind both inline bodies and named functions",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Scripts>"
        "<OnLoad><![CDATA[ self:SetAlpha(0.5) ]]></OnLoad>"
        "<OnClick function=\"MyHandler\"/>"
        "</Scripts></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetScript(\"OnLoad\", function(self, ...)"));
    REQUIRE(has(r.lua, "self:SetAlpha(0.5)"));
    REQUIRE(has(r.lua, ":SetScript(\"OnClick\", MyHandler)"));
    // OnLoad is expected to run once the frame is built, which is what every
    // handler in FrameXML assumes about itself. Through the engine, which is
    // what puts `this` in scope for an interface written before 3.0.
    REQUIRE(has(r.lua, "__WoweeFireOnLoad("));
}

TEST_CASE("Referenced files are reported rather than loaded", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Script file=\"Foo.lua\"/><Include file=\"Bar.xml\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(r.scriptFiles.size() == 1);
    REQUIRE(r.scriptFiles[0] == "Foo.lua");
    REQUIRE(r.includeFiles.size() == 1);
    REQUIRE(r.includeFiles[0] == "Bar.xml");
}

TEST_CASE("Nested frames are parented to the frame containing them",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Outer\"><Frames>"
        "<Button name=\"$parentBtn\"/>"
        "</Frames></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "CreateFrame(\"Button\", \"OuterBtn\", __w[1])"));
}

TEST_CASE("Strings that reach Lua are escaped", "[framexml][emit]") {
    // Texture paths are full of backslashes, and a quote in label text would
    // otherwise end the string and leave the rest as code.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Layers><Layer>"
        "<FontString text=\"He said &quot;hi&quot;\"/>"
        "<Texture file=\"Interface\\Icons\\Foo\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "\\\"hi\\\""));
    REQUIRE(has(r.lua, "Interface\\\\Icons\\\\Foo"));
}

TEST_CASE("$parent inside a template resolves to the frame that inherits it",
          "[framexml][emit]") {
    // The subtlety that makes templates work at all. A region named $parentBg in
    // a template must become FooFrameBg on the frame inheriting it, not
    // TemplateNameBg - the template's own name is never the answer, and every
    // frame sharing that template would collide on it if it were.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"MyTemplate\" virtual=\"true\"><Layers><Layer>"
        "<Texture name=\"$parentBg\" setAllPoints=\"true\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "GetName()"));
    REQUIRE(has(r.lua, "\"Bg\""));
    REQUIRE_FALSE(has(r.lua, "\"MyTemplateBg\""));
}

TEST_CASE("A template installs OnLoad but does not run it", "[framexml][emit]") {
    // A frame is loaded once, when it is finished - not once per template it
    // is built from. Running it per template fired ChatFrameEditBoxTemplate's
    // OnLoad before the edit box's own OnLoad had set self.chatFrame, which is
    // the first thing that handler indexes.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"T\" virtual=\"true\">"
        "<Scripts><OnLoad>DoThing(self)</OnLoad></Scripts>"
        "</Frame>"
        "<Frame name=\"Real\" inherits=\"T\"/></Ui>");
    const EmitResult r = emitFrameXml(root);

    // Once, for the real frame - not inside the template body.
    const std::string fire = "__WoweeFireOnLoad(";
    size_t count = 0;
    for (size_t at = r.lua.find(fire); at != std::string::npos;
         at = r.lua.find(fire, at + 1)) ++count;
    REQUIRE(count == 1);
    REQUIRE(has(r.lua, "SetScript(\"OnLoad\""));
}

TEST_CASE("$parent skips unnamed frames to the nearest named one",
          "[framexml][emit]") {
    // An unnamed frame has no name to lend, so $parent means the nearest
    // ancestor that has one. PartyMemberPetFrameTemplate buries its
    // $parentName two unnamed frames deep and expects
    // PartyMemberFrame1PetFrameName; asking the unnamed frame gave nil and
    // named the region "Name", which nothing was looking for.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"T\" virtual=\"true\"><Frames>"
        "<Frame><Frames><Frame>"
        "<Layers><Layer><FontString name=\"$parentName\"/></Layer></Layers>"
        "</Frame></Frames></Frame>"
        "</Frames></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);

    // Named from the template root, which is the only thing with a name -
    // and only if it has one, since an owner with no name lends none.
    REQUIRE(has(r.lua, "self:GetName() .. \"Name\""));
    REQUIRE(has(r.lua, "self:GetName() and"));
}

TEST_CASE("A frame can fill its parent instead of anchoring", "[framexml][emit]") {
    // Honoured for regions and ignored for frames, which is 139 declarations
    // across 53 files. An unanchored frame falls to the centre-on-parent
    // default with no size, so its centre is the screen's - PlayerFrame's name
    // sat in the middle of the world because two frames above it said
    // setAllPoints and nothing acted on it.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Outer\"><Frames>"
        "<Frame name=\"$parentInner\" setAllPoints=\"true\"/>"
        "</Frames></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetAllPoints(__w[1])"));
}

TEST_CASE("A Font element becomes a font object", "[framexml][emit]") {
    // Not a widget: a named set of type settings that font strings inherit by
    // name, and SetFontObject reads height and colour off it. FrameXML defines
    // 42 of them and every label inherits one.
    XmlNode root = parseOrFail(
        "<Ui>"
        "<Font name=\"Base\" font=\"Fonts\\\\FRIZQT__.TTF\" virtual=\"true\">"
        "<FontHeight><AbsValue val=\"10\"/></FontHeight>"
        "<Color r=\"1\" g=\"0.8\" b=\"0\"/>"
        "</Font>"
        "<Font name=\"Derived\" inherits=\"Base\" virtual=\"true\">"
        "<FontHeight><AbsValue val=\"16\"/></FontHeight>"
        "</Font>"
        "</Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "Base.height = 10"));
    REQUIRE(has(r.lua, "Base.r = 1"));
    // Copied first, so the height stated here wins over the inherited one.
    REQUIRE(has(r.lua, "for k, v in pairs(base) do Derived[k] = v end"));
    REQUIRE(r.lua.find("pairs(base) do Derived") < r.lua.find("Derived.height = 16"));
    // A font object is not a frame.
    REQUIRE_FALSE(has(r.lua, "CreateFrame(\"Font\""));
}

TEST_CASE("A slider carries its range, step and grip", "[framexml][emit]") {
    // The range is set before the value, or the value is clamped against a
    // default range it was never meant to sit in. The thumb is a region like
    // button art, handed to the setter afterwards.
    XmlNode root = parseOrFail(
        "<Ui><Slider name=\"S\" minValue=\"0\" maxValue=\"100\" valueStep=\"5\""
        " defaultValue=\"20\" orientation=\"VERTICAL\">"
        "<ThumbTexture name=\"$parentThumb\" file=\"Art\\\\Grip\"/>"
        "</Slider></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, ":SetMinMaxValues("));
    REQUIRE(has(r.lua, ":SetValueStep("));
    REQUIRE(has(r.lua, ":SetOrientation(\"VERTICAL\")"));
    REQUIRE(has(r.lua, ":SetThumbTexture("));
    REQUIRE(r.lua.find(":SetMinMaxValues(") < r.lua.find(":SetValue("));
}

TEST_CASE("A frame's id becomes SetID", "[framexml][emit]") {
    // How a frame in a numbered set knows which one it is. FrameXML builds
    // names out of it - PartyMemberFrame_RefreshPetDebuffs reaches for
    // _G["PartyMemberFrame" .. self:GetID() .. "PetFrame"] - and 848 of these
    // are declared across 57 files.
    XmlNode root = parseOrFail("<Ui><Frame name=\"F\" id=\"3\"/></Ui>");
    REQUIRE(has(emitFrameXml(root).lua, ":SetID(3)"));
}

TEST_CASE("parentKey binds a region to a field on its owner", "[framexml][emit]") {
    // How FrameXML's handlers reach their own pieces: QuestHonorFrameTemplate's
    // OnLoad opens with self.icon:SetTexture(...), and the icon is bound only
    // by parentKey="icon" on the texture. Ignoring the attribute left all 242
    // of those fields nil across 31 files.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Panel\">"
        "<Layers><Layer><Texture name=\"$parentIcon\" parentKey=\"icon\"/></Layer></Layers>"
        "<Frames><Frame name=\"$parentBar\" parentKey=\"bar\"/></Frames>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "[\"icon\"] = "));
    REQUIRE(has(r.lua, "[\"bar\"] = "));
}

TEST_CASE("A template that inherits another applies it too", "[framexml][emit]") {
    // Templates are built from other templates constantly - 217 of FrameXML's
    // virtual frames inherit one - and the virtual branch used to return before
    // inherits was ever emitted. InterfaceOptionsListButtonTemplate silently
    // dropped the OptionsListButtonTemplate it is built on, so it arrived with
    // no highlight texture and no size.
    XmlNode root = parseOrFail(
        "<Ui><Button name=\"Derived\" inherits=\"Base\" virtual=\"true\">"
        "<Scripts><OnClick function=\"Foo\"/></Scripts>"
        "</Button></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "__WoweeTemplates[\"Derived\"] = function(self)"));
    REQUIRE(has(r.lua, "__WoweeTemplates[\"Base\"](self)"));
    // Before the body, so the template's own settings win over the base's.
    REQUIRE(r.lua.find("__WoweeTemplates[\"Base\"](self)") <
            r.lua.find("SetScript(\"OnClick\""));
}

TEST_CASE("A frame's own $parent anchor means its parent, not itself",
          "[framexml][emit]") {
    // A sibling reference. VideoOptionsFrameCancel anchors to $parentApply,
    // meaning the Apply button beside it on the frame holding both - not a
    // child of the Cancel button. Resolving it against the button's own name
    // produced VideoOptionsFrameCancelApply, which nothing is called, so the
    // anchor silently fell back to the parent and the button sat in the wrong
    // place. Regions are the other way round and must keep working.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Panel\"><Frames>"
        "<Button name=\"$parentCancel\"><Anchors>"
        "<Anchor point=\"BOTTOMRIGHT\" relativeTo=\"$parentApply\"/>"
        "</Anchors></Button>"
        "</Frames><Layers><Layer>"
        "<Texture name=\"$parentBg\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "\"PanelApply\""));
    REQUIRE_FALSE(has(r.lua, "\"PanelCancelApply\""));
    // The region still names itself after the frame that owns it.
    REQUIRE(has(r.lua, "\"PanelBg\""));
}

TEST_CASE("Button art declared outside a Layer is still created",
          "[framexml][emit]") {
    // <NormalTexture> and <ButtonText> are regions like any other, just
    // declared as their own element with an implied layer and a setter. The
    // emitter ignored all of them, so the names they declare never existed -
    // _G["DropDownList1Button1NormalText"] among them, which is what stopped
    // UIDropDownMenu loading. The highlight belongs on its own layer, and the
    // label above the art rather than under it.
    XmlNode root = parseOrFail(
        "<Ui><Button name=\"MyButton\">"
        "<NormalTexture name=\"$parentNormalTexture\" file=\"Art\\\\Face\"/>"
        "<HighlightTexture name=\"$parentHighlight\"/>"
        "<ButtonText name=\"$parentNormalText\"/>"
        "</Button></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "\"MyButtonNormalTexture\""));
    REQUIRE(has(r.lua, "\"MyButtonNormalText\""));
    REQUIRE(has(r.lua, ":SetNormalTexture("));
    REQUIRE(has(r.lua, ":SetFontString("));
    REQUIRE(has(r.lua, "\"HIGHLIGHT\""));
    REQUIRE(has(r.lua, "\"OVERLAY\""));
    // A font string, not a texture - the element name does not say so.
    REQUIRE(has(r.lua, "CreateFontString(\"MyButtonNormalText\""));
}

TEST_CASE("A nested frame in a template also resolves $parent at replay time",
          "[framexml][emit]") {
    // The same rule as the region above, and it was the region that had it. A
    // child frame named $parentScrollBar was emitted with the template's own
    // name baked in, so every scroll frame in FrameXML created a global called
    // UIPanelScrollFrameTemplateScrollBar and overwrote the last one, while the
    // _G[self:GetName().."ScrollBar"] its handlers look up never existed. That
    // one line accounted for most of the files that would not load.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"MyTemplate\" virtual=\"true\"><Frames>"
        "<Frame name=\"$parentScrollBar\"/>"
        "</Frames></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "GetName()"));
    REQUIRE(has(r.lua, "\"ScrollBar\""));
    REQUIRE_FALSE(has(r.lua, "\"MyTemplateScrollBar\""));
}

TEST_CASE("Outside a template $parent is resolved when emitted",
          "[framexml][emit]") {
    // No reason to defer it where the owning frame is already known; a literal
    // keeps the generated code readable.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"FooFrame\"><Layers><Layer>"
        "<Texture name=\"$parentBg\"/></Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "\"FooFrameBg\""));
    REQUIRE_FALSE(has(r.lua, "GetName()"));
}

TEST_CASE("A nested frame anchors to its container, not the screen",
          "[framexml][emit]") {
    // FrameXML nests constantly, and an unqualified anchor means "my parent".
    // Reading it as UIParent puts anything inside a panel somewhere else on the
    // screen entirely.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Outer\"><Frames>"
        "<StatusBar name=\"$parentBar\"><Anchors>"
        "<Anchor point=\"BOTTOMLEFT\" relativePoint=\"BOTTOMLEFT\">"
        "<Offset><AbsDimension x=\"5\" y=\"5\"/></Offset></Anchor>"
        "</Anchors></StatusBar>"
        "</Frames></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "CreateFrame(\"StatusBar\", \"OuterBar\", __w[1])"));
    REQUIRE(has(r.lua, ":SetPoint(\"BOTTOMLEFT\", __w[1],"));
    REQUIRE_FALSE(has(r.lua, ":SetPoint(\"BOTTOMLEFT\", UIParent,"));
}

TEST_CASE("An anchor inside a template resolves its parent at replay time",
          "[framexml][emit]") {
    // The containing frame is not known while emitting a template - it is
    // whichever frame inherits it - so the parent has to be asked for then.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"T\" virtual=\"true\"><Anchors>"
        "<Anchor point=\"CENTER\"/></Anchors></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "self:GetParent()"));
}

TEST_CASE("A FontString's inherits names a font object, not a template",
          "[framexml][emit]") {
    // Frames inherit templates; FontStrings inherit shared font objects, and
    // that is where their size and colour come from. FrameXML does it more than
    // three thousand times, so treating it as a template would leave every
    // label the same size in the same colour.
    //
    // It is not either/or, though: FrameXML declares virtual FontStrings too,
    // and the name alone does not say which kind it is. So the emitted line
    // asks - but a font object must still reach SetFontObject, which is what
    // this checks.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Layers><Layer>"
        "<FontString name=\"$parentT\" inherits=\"GameFontNormalLarge\" text=\"Hi\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetFontObject(\"GameFontNormalLarge\")"));
    REQUIRE(has(r.lua, "else "));
}

TEST_CASE("A virtual Texture becomes a template a region can inherit",
          "[framexml][emit]") {
    // Twenty-two of these sit at the top level of FrameXML - the dialog
    // button's normal, pushed and highlight art among them. None was emitted,
    // so every button inheriting its art had none.
    XmlNode root = parseOrFail(
        "<Ui>"
        "<Texture name=\"DialogButtonNormalTexture\" file=\"Interface\\Up\" virtual=\"true\"/>"
        "<Frame name=\"F\"><Layers><Layer>"
        "<Texture name=\"$parentArt\" inherits=\"DialogButtonNormalTexture\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "__WoweeTemplates[\"DialogButtonNormalTexture\"] = function(self)"));
    REQUIRE(has(r.lua, "self:SetTexture(\"Interface\\\\Up\")"));
    REQUIRE(has(r.lua, "__WoweeTemplates[\"DialogButtonNormalTexture\"](__w[2])"));
}

TEST_CASE("A Texture's inherits is not treated as a font object",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Layers><Layer>"
        "<Texture name=\"$parentTex\" inherits=\"SomeTextureTemplate\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "SetFontObject"));
}

TEST_CASE("Handler bodies get their arguments by name", "[framexml][emit]") {
    // Blizzard's inline scripts use their argument names without declaring
    // them. Passed positionally instead, an OnUpdate body's `elapsed` is nil
    // and the first arithmetic on it fails - which is most of FrameXML's
    // OnUpdate handlers.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Scripts>"
        "<OnUpdate><![CDATA[ self.t = (self.t or 0) + elapsed ]]></OnUpdate>"
        "<OnClick><![CDATA[ if button == \"LeftButton\" then self:Hide() end ]]></OnClick>"
        "<OnEvent><![CDATA[ if event == \"PLAYER_LOGIN\" then self:Show() end ]]></OnEvent>"
        "<OnValueChanged><![CDATA[ self:SetAlpha(value) ]]></OnValueChanged>"
        "</Scripts></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);

    REQUIRE(has(r.lua, "function(self, elapsed, ...)"));
    REQUIRE(has(r.lua, "function(self, button, down, ...)"));
    REQUIRE(has(r.lua, "function(self, event,"));
    REQUIRE(has(r.lua, "function(self, value, ...)"));
    REQUIRE_FALSE(has(r.lua, "local arg1, arg2, arg3, arg4 = ..."));
}

TEST_CASE("A handler with no named arguments still takes self",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Scripts>"
        "<OnShow><![CDATA[ self:SetAlpha(1) ]]></OnShow>"
        "</Scripts></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "function(self, ...)"));
}

TEST_CASE("Every handler is vararg whatever its named arguments",
          "[framexml][emit]") {
    // A body is free to use `...` whatever handler it belongs to, and a
    // parameter list without it does not merely lose the values - it fails to
    // compile, taking the whole template with it.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Scripts>"
        "<OnEvent><![CDATA[ local a, b = ...; self:SetAlpha(1) ]]></OnEvent>"
        "<OnUpdate><![CDATA[ local x = select(1, ...) ]]></OnUpdate>"
        "</Scripts></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "function(self, elapsed, ...)"));
    REQUIRE(has(r.lua, "function(self, event,"));
    // Both signatures end in varargs.
    size_t at = 0, sigs = 0;
    while ((at = r.lua.find(", ...)", at)) != std::string::npos) { ++sigs; at += 5; }
    REQUIRE(sigs >= 2);
}

TEST_CASE("A $parent relativeTo becomes a name, not a bare symbol",
          "[framexml][emit]") {
    // $parentBg is not a Lua identifier. Pasted in as one it is a syntax error,
    // which does not lose the anchor - it loses the whole file.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"FooFrame\"><Layers><Layer>"
        "<Texture name=\"$parentBg\"/>"
        "<Texture name=\"$parentIcon\"><Anchors>"
        "<Anchor point=\"LEFT\" relativeTo=\"$parentBg\" relativePoint=\"RIGHT\"/>"
        "</Anchors></Texture>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "$parent"));
    REQUIRE(has(r.lua, "\"FooFrameBg\""));
}

TEST_CASE("Temporaries do not run into Lua's local-variable limit",
          "[framexml][emit]") {
    // Lua allows 200 locals per function. A large file declares far more
    // widgets than that, and going over does not degrade - the whole chunk
    // refuses to compile. FriendsFrame and InterfaceOptionsPanels both did.
    std::string xml = "<Ui><Frame name=\"Big\"><Layers><Layer>";
    for (int i = 0; i < 300; ++i) {
        xml += "<Texture name=\"$parentT" + std::to_string(i) + "\"/>";
    }
    xml += "</Layer></Layers></Frame></Ui>";
    XmlNode root = parseOrFail(xml);
    const EmitResult r = emitFrameXml(root);

    // One table, not three hundred locals.
    REQUIRE(has(r.lua, "local __w = {}"));
    size_t locals = 0, at = 0;
    while ((at = r.lua.find("local ", at)) != std::string::npos) { ++locals; at += 6; }
    REQUIRE(locals == 1);
}

TEST_CASE("An empty function attribute is not emitted as a handler name",
          "[framexml][emit]") {
    // SetScript("X", ) is a syntax error, so this loses the whole file rather
    // than the one handler.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Scripts>"
        "<OnMouseWheel function=\"\"/>"
        "<OnShow function=\"RealHandler\"/>"
        "</Scripts></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "SetScript(\"OnMouseWheel\", )"));
    REQUIRE(has(r.lua, "SetScript(\"OnShow\", RealHandler)"));
}

TEST_CASE("$parent follows the parent attribute, not the file structure",
          "[framexml][emit]") {
    // A frame written at the top level with parent="Something" belongs to that
    // frame, and its $parent means it. WorldMapTitleButton is declared this way
    // and anchors to $parentMiniBorderLeft - the world map's own border. Taken
    // from where it sits in the file instead, there is no containing frame to
    // name, so $parent collapsed to the bare suffix and SetPoint looked up a
    // global that nothing has.
    XmlNode root = parseOrFail(
        "<Ui>"
        "<Frame name=\"Panel\"><Layers><Layer>"
        "<Texture name=\"$parentEdge\" file=\"Interface\\Edge\"/>"
        "</Layer></Layers></Frame>"
        "<Button name=\"PanelTitle\" parent=\"Panel\">"
        "<Anchors><Anchor point=\"TOPLEFT\" relativeTo=\"$parentEdge\"/></Anchors>"
        "</Button></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "CreateTexture(\"PanelEdge\""));
    REQUIRE(has(r.lua, ":SetPoint(\"TOPLEFT\", \"PanelEdge\""));
    REQUIRE_FALSE(has(r.lua, "\"Edge\""));
}

TEST_CASE("A $parent name on an unnamed owner is no name at all",
          "[framexml][emit]") {
    // Inside a template the owner is not known until replay, so the name is
    // built then. If the frame inheriting the template has no name there is
    // nothing to build from, and WoW gives the region no name - where falling
    // back to an empty string publishes the bare suffix as a global.
    // ContainerFrameTemplate replayed onto an unnamed frame created a texture
    // called "Portrait", and the next frame to do the same overwrote it.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"T\" virtual=\"true\"><Layers><Layer>"
        "<Texture name=\"$parentPortrait\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "self:GetName() and"));
    REQUIRE(has(r.lua, "or nil)"));
    REQUIRE_FALSE(has(r.lua, "or \"\") .. \"Portrait\""));
}

TEST_CASE("A frame with hover handlers takes the mouse without saying so",
          "[framexml][emit]") {
    // Blizzard's StatFrameTemplate carries OnEnter and OnLeave and no
    // enableMouse, and the character sheet's stat tooltips depend on it.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Stat\">"
        "<Scripts><OnEnter>Tip(self)</OnEnter><OnLeave>Hide()</OnLeave></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "EnableMouse(true)"));
}

TEST_CASE("An explicit enableMouse=false is not overridden by a hover handler",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Quiet\" enableMouse=\"false\">"
        "<Scripts><OnEnter>Tip(self)</OnEnter></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "EnableMouse(false)"));
    REQUIRE_FALSE(has(r.lua, "EnableMouse(true)"));
}

TEST_CASE("A drag handler also asks for the mouse", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Draggable\">"
        "<Scripts><OnMouseDown>Grab(self)</OnMouseDown></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "EnableMouse(true)"));
}

TEST_CASE("The wheel is its own switch, not the mouse", "[framexml][emit]") {
    // A scrolling frame takes the wheel without taking clicks: enabling the
    // mouse as well would have it swallow clicks meant for what is under it.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Scroller\">"
        "<Scripts><OnMouseWheel>Scroll(self)</OnMouseWheel></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "EnableMouseWheel(true)"));
    REQUIRE_FALSE(has(r.lua, "EnableMouse(true)"));
}

TEST_CASE("An explicit enableMouseWheel is honoured", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"NoScroll\" enableMouseWheel=\"false\">"
        "<Scripts><OnMouseWheel>Scroll(self)</OnMouseWheel></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "EnableMouseWheel(false)"));
    REQUIRE_FALSE(has(r.lua, "EnableMouseWheel(true)"));
}

TEST_CASE("A frame with no hover handlers is left alone", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Plain\">"
        "<Scripts><OnShow>Nothing()</OnShow></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "EnableMouse"));
}

TEST_CASE("A Backdrop element becomes a SetBackdrop call", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Panel\">"
        "<Backdrop bgFile=\"bg.blp\" edgeFile=\"edge.blp\" tile=\"true\">"
        "<BackgroundInsets><AbsInset left=\"11\" right=\"12\" top=\"13\" bottom=\"14\"/></BackgroundInsets>"
        "<TileSize><AbsValue val=\"32\"/></TileSize>"
        "<EdgeSize><AbsValue val=\"16\"/></EdgeSize>"
        "</Backdrop></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetBackdrop("));
    REQUIRE(has(r.lua, "bgFile=\"bg.blp\""));
    REQUIRE(has(r.lua, "edgeFile=\"edge.blp\""));
    REQUIRE(has(r.lua, "tile=true"));
    REQUIRE(has(r.lua, "tileSize=32"));
    REQUIRE(has(r.lua, "edgeSize=16"));
    REQUIRE(has(r.lua, "insets={left=11, right=12, top=13, bottom=14}"));
}

TEST_CASE("A Backdrop with only an edge file still emits", "[framexml][emit]") {
    // 17 of the 77 in FrameXML have no bgFile: a border and nothing behind it.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Edged\">"
        "<Backdrop edgeFile=\"edge.blp\" tile=\"false\">"
        "<EdgeSize><AbsValue val=\"8\"/></EdgeSize>"
        "</Backdrop></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetBackdrop("));
    REQUIRE(has(r.lua, "edgeSize=8"));
    REQUIRE(has(r.lua, "tile=false"));
    REQUIRE_FALSE(has(r.lua, "bgFile="));
}

TEST_CASE("A frame with no Backdrop emits no SetBackdrop", "[framexml][emit]") {
    XmlNode root = parseOrFail("<Ui><Frame name=\"Bare\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "SetBackdrop"));
}

TEST_CASE("A declared alpha reaches SetAlpha", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Dim\" alpha=\"0.5\">"
        "<Layers><Layer><Texture name=\"$parentTex\" alpha=\"0.25\"/></Layer></Layers>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetAlpha(0.5)"));
    REQUIRE(has(r.lua, "SetAlpha(0.25)"));
}

TEST_CASE("justifyV reaches the font string", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Layers><Layer>"
        "<FontString name=\"$parentTop\" justifyV=\"TOP\" justifyH=\"LEFT\"/>"
        "</Layer></Layers></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetJustifyV(\"TOP\")"));
    REQUIRE(has(r.lua, "SetJustifyH(\"LEFT\")"));
}

TEST_CASE("A button's NormalFont reaches its label", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Button name=\"Btn\">"
        "<NormalFont style=\"GameFontNormal\"/>"
        "</Button></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetNormalFontObject(\"GameFontNormal\")"));
}

TEST_CASE("HitRectInsets become a SetHitRectInsets call", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Panel\">"
        "<HitRectInsets><AbsInset left=\"0\" right=\"30\" top=\"0\" bottom=\"45\"/></HitRectInsets>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetHitRectInsets(0, 30, 0, 45)"));
}

TEST_CASE("A font object's Shadow reaches the table", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Font name=\"Shadowed\" font=\"F.ttf\">"
        "<FontHeight><AbsValue val=\"12\"/></FontHeight>"
        "<Shadow><Offset><AbsDimension x=\"1\" y=\"-1\"/></Offset>"
        "<Color r=\"0\" g=\"0\" b=\"0\" a=\"1\"/></Shadow>"
        "</Font></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "Shadowed.shadowX = 1"));
    REQUIRE(has(r.lua, "Shadowed.shadowY = -1"));
    REQUIRE(has(r.lua, "Shadowed.shadowA = 1"));
}

TEST_CASE("A font object without a Shadow says nothing about one",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Font name=\"Plain\" font=\"F.ttf\">"
        "<FontHeight><AbsValue val=\"12\"/></FontHeight></Font></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "shadow"));
}

TEST_CASE("An Animations block becomes group and animation calls",
          "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Pulse\">"
        "<Animations><AnimationGroup name=\"$parentGroup\" looping=\"BOUNCE\">"
        "<Alpha change=\"-0.7\" duration=\"0.75\" order=\"1\" startDelay=\"0.2\"/>"
        "<Translation offsetX=\"10\" offsetY=\"-5\" duration=\"1\"/>"
        "</AnimationGroup></Animations></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "CreateAnimationGroup(\"PulseGroup\")"));
    REQUIRE(has(r.lua, "SetLooping(\"BOUNCE\")"));
    REQUIRE(has(r.lua, "CreateAnimation(\"Alpha\")"));
    REQUIRE(has(r.lua, "SetChange(-0.7)"));
    REQUIRE(has(r.lua, "SetStartDelay(0.2)"));
    REQUIRE(has(r.lua, "CreateAnimation(\"Translation\")"));
    REQUIRE(has(r.lua, "SetOffset(10, -5)"));
    // The vars are table slots, not names: "local __w[3]" would not parse.
    REQUIRE_FALSE(has(r.lua, "local __w["));
}

TEST_CASE("A TitleRegion makes a frame draggable", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Loot\"><TitleRegion setAllPoints=\"true\"/></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetMovable(true)"));
    REQUIRE(has(r.lua, "RegisterForDrag(\"LeftButton\")"));
    REQUIRE(has(r.lua, "StartMoving()"));
}

TEST_CASE("A frame with its own drag scripts keeps them", "[framexml][emit]") {
    // The chat frame has a title region and five OnDragStart handlers; the
    // generic pair must not replace behaviour that is deliberately specific.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Chat\">"
        "<TitleRegion setAllPoints=\"true\"/>"
        "<Scripts><OnDragStart>Special(self)</OnDragStart></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetMovable(true)"));
    REQUIRE_FALSE(has(r.lua, "StartMoving()"));
}

TEST_CASE("PushedTextOffset reaches the button", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Button name=\"Btn\">"
        "<PushedTextOffset><AbsDimension x=\"1\" y=\"-1\"/></PushedTextOffset>"
        "</Button></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetPushedTextOffset(1, -1)"));
}

TEST_CASE("Element names are matched without regard to case",
          "[framexml][emit]") {
    // Blizzard's own floatingchatframe.xml declares <Fontstring>, and WoW's
    // parser does not care. Addons are written less carefully still.
    XmlNode root = parseOrFail(
        "<Ui><frame name=\"Mixed\"><layers><Layer>"
        "<Fontstring name=\"$parentLabel\" text=\"hi\"/>"
        "</Layer></layers></frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "CreateFrame(\"Frame\", \"Mixed\""));
    REQUIRE(has(r.lua, "CreateFontString"));
    REQUIRE(r.warnings.empty());
}

TEST_CASE("An element nobody knows is still reported", "[framexml][emit]") {
    // Canonicalising must not quietly accept anything: an unknown element is
    // a frame, and everything inside it, that never gets built.
    XmlNode root = parseOrFail("<Ui><Wibble name=\"X\"/></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(r.warnings.empty());
}

TEST_CASE("A button's state fonts reach it", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Button name=\"Btn\">"
        "<NormalFont style=\"GameFontNormal\"/>"
        "<HighlightFont style=\"GameFontHighlight\"/>"
        "<DisabledFont style=\"GameFontDisable\"/>"
        "</Button></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "SetNormalFontObject(\"GameFontNormal\")"));
    REQUIRE(has(r.lua, "SetHighlightFontObject(\"GameFontHighlight\")"));
    REQUIRE(has(r.lua, "SetDisabledFontObject(\"GameFontDisable\")"));
}

TEST_CASE("A key handler asks for the keyboard", "[framexml][emit]") {
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Dialog\">"
        "<Scripts><OnKeyDown>Handle(self, key)</OnKeyDown></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, "EnableKeyboard(true)"));
}

TEST_CASE("A frame with no key handler does not take the keyboard",
          "[framexml][emit]") {
    // The safety property: nothing listens during ordinary play, so movement
    // keys are never swallowed by a frame that merely exists.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"Quiet\">"
        "<Scripts><OnShow>Nothing()</OnShow></Scripts>"
        "</Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE_FALSE(has(r.lua, "EnableKeyboard"));
}

TEST_CASE("An edit box is built as it was declared", "[framexml][emit]") {
    // Every one of these had a method and a field behind it and no way to
    // reach them from the XML, so a box came out with the defaults whatever
    // it said. SendMailBodyEditBox declares letters="500" multiLine="true",
    // and without them it was a single-line box with no limit - a letter
    // nobody could write a second line in.
    XmlNode root = parseOrFail(
        "<Ui><EditBox name=\"E\" letters=\"500\" multiLine=\"true\"/></Ui>");
    const std::string lua = emitFrameXml(root).lua;
    REQUIRE(has(lua, ":SetMaxLetters(500)"));
    REQUIRE(has(lua, ":SetMultiLine(true)"));
}

TEST_CASE("An edit box that says nothing about itself is left alone",
          "[framexml][emit]") {
    // The defaults belong to the widget, not to the emitter: writing
    // SetMultiLine(false) for a box that never mentioned it would override
    // whatever a template had already set.
    XmlNode root = parseOrFail("<Ui><EditBox name=\"E\"/></Ui>");
    const std::string lua = emitFrameXml(root).lua;
    REQUIRE_FALSE(has(lua, ":SetMaxLetters"));
    REQUIRE_FALSE(has(lua, ":SetMultiLine"));
    REQUIRE_FALSE(has(lua, ":SetAutoFocus"));
}

TEST_CASE("autoFocus false is carried, because it is why it is written",
          "[framexml][emit]") {
    // Declared false on nearly every box in FrameXML. A box that takes focus
    // when it appears swallows the keyboard from whatever the player was
    // doing, so the false is the whole point of the attribute.
    XmlNode root = parseOrFail(
        "<Ui><EditBox name=\"E\" autoFocus=\"false\" numeric=\"true\"/></Ui>");
    const std::string lua = emitFrameXml(root).lua;
    REQUIRE(has(lua, ":SetAutoFocus(false)"));
    REQUIRE(has(lua, ":SetNumeric(true)"));
}

TEST_CASE("text= names a global, not a caption", "[framexml][emit]") {
    // CharacterFrameTab1 says text="CHARACTER", and CHARACTER is a global
    // holding the word "Character". Emitting the literal put the key on screen
    // and - because the tab is sized from the width of its own label - made
    // every tab on the character sheet a sliver with the text clipped inside.
    XmlNode root = parseOrFail(
        "<Ui><Button name=\"B\" text=\"CHARACTER\"/></Ui>");
    REQUIRE(has(emitFrameXml(root).lua, ":SetText(_G[\"CHARACTER\"] or \"CHARACTER\")"));
}

TEST_CASE("A caption that is not a name is used as written",
          "[framexml][emit]") {
    // Only a bare key is looked up. Anything with a space or a lower-case
    // letter is the text itself, and _G would answer nil for it.
    XmlNode root = parseOrFail(
        "<Ui><Button name=\"B\" text=\"Click me\"/></Ui>");
    const std::string lua = emitFrameXml(root).lua;
    REQUIRE(has(lua, ":SetText(\"Click me\")"));
    REQUIRE_FALSE(has(lua, "_G["));
}


TEST_CASE("A button gets its font string before its text", "[framexml][emit]") {
    // The character sheet's tabs are the case this is about. The label comes
    // from text= on the tab, but the font string it lands on is declared by
    // the template the tab inherits - so two separate emissions have to happen
    // in the right order, and SetText only forwards to a font string that is
    // already attached. Set the text first and it goes into __text, where
    // nothing draws it and nothing measures it.
    //
    // Both of the tab's symptoms come out of that single ordering: no label,
    // and - because PanelTemplates_TabResize sizes the tab from
    // CharacterFrameTab1Text:GetWidth() - a sliver barely wider than its
    // borders.
    XmlNode root = parseOrFail(
        "<Ui>"
        "<Button name=\"TabTemplate\" virtual=\"true\">"
        "  <ButtonText name=\"$parentText\"/>"
        "</Button>"
        "<Button name=\"Tab1\" inherits=\"TabTemplate\" text=\"CHARACTER\"/>"
        "</Ui>");
    const std::string lua = emitFrameXml(root).lua;

    // The template runs from inherits=, so what has to be ordered here is the
    // inherits call against the tab's own SetText.
    const size_t inherit = lua.find("__WoweeTemplates[\"TabTemplate\"]");
    const size_t setText = lua.find(":SetText(_G[\"CHARACTER\"]");
    REQUIRE(inherit != std::string::npos);
    REQUIRE(setText != std::string::npos);
    REQUIRE(inherit < setText);

    // And the template itself must attach the font string rather than merely
    // create it, or the tab has a named region no button knows about.
    REQUIRE(has(lua, ":SetFontString("));
}

TEST_CASE("OnEvent takes its arguments through the varargs, not by name",
          "[framexml][emit]") {
    // Both spellings are in the interface and both have to work. A body that
    // says `arg1` needs the name; a body that hands `...` to a Lua function
    // needs the varargs - and fifty-three of them do, ContainerFrame among
    // them. Naming arg1..arg9 as parameters served only the first, because
    // every value that arrived was bound to a name and `...` came out empty:
    // ContainerFrame_OnEvent compared a nil arg1 against the bag's own id, so
    // no open bag ever redrew on BAG_UPDATE.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Scripts>"
        "<OnEvent><![CDATA[ Handler(self, event, ...) ]]></OnEvent>"
        "</Scripts></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetScript(\"OnEvent\", function(self, event, ...)"));
    // ...and the names still exist, bound off those same varargs.
    REQUIRE(has(r.lua, "local arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9 = ...;"));
    REQUIRE(has(r.lua, "Handler(self, event, ...)"));
}

TEST_CASE("every other handler keeps its named arguments", "[framexml][emit]") {
    // OnEvent is the exception, not the new rule. An OnUpdate body says
    // `elapsed` and does arithmetic with it, and a prelude would be dead
    // weight in front of every one of them.
    XmlNode root = parseOrFail(
        "<Ui><Frame name=\"F\"><Scripts>"
        "<OnUpdate><![CDATA[ self.t = self.t + elapsed ]]></OnUpdate>"
        "</Scripts></Frame></Ui>");
    const EmitResult r = emitFrameXml(root);
    REQUIRE(has(r.lua, ":SetScript(\"OnUpdate\", function(self, elapsed, ...)"));
    REQUIRE_FALSE(has(r.lua, "local arg1"));
}
