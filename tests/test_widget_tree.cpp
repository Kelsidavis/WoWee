#include <catch_amalgamated.hpp>

#include "ui/widget_tree.hpp"

#include <limits>
#include <string>

using namespace wowee::ui;

// The anchor solver is what every frame's position comes out of, and it is the
// part of a widget system that is wrong in ways nothing reports: a frame lands
// somewhere plausible and only looks wrong against the art it was meant to sit
// on. Pin the rules directly.
//
// Coordinates are WoW's: origin bottom-left, y upward.

namespace {
constexpr float kScreenW = 1024.0f;
constexpr float kScreenH = 768.0f;
}

TEST_CASE("Anchor point names resolve to rect fractions", "[widget][anchor]") {
    auto p = [](const char* n) { return resolveAnchorPoint(n); };

    REQUIRE(p("BOTTOMLEFT").fx == Catch::Approx(0.0f));
    REQUIRE(p("BOTTOMLEFT").fy == Catch::Approx(0.0f));
    REQUIRE(p("TOPRIGHT").fx == Catch::Approx(1.0f));
    REQUIRE(p("TOPRIGHT").fy == Catch::Approx(1.0f));
    REQUIRE(p("CENTER").fx == Catch::Approx(0.5f));
    REQUIRE(p("CENTER").fy == Catch::Approx(0.5f));

    // The combined names carry both halves; TOP must not be read as "not
    // bottom, therefore centred".
    REQUIRE(p("TOP").fx == Catch::Approx(0.5f));
    REQUIRE(p("TOP").fy == Catch::Approx(1.0f));
    REQUIRE(p("LEFT").fx == Catch::Approx(0.0f));
    REQUIRE(p("LEFT").fy == Catch::Approx(0.5f));

    REQUIRE(p("topleft").fx == Catch::Approx(0.0f));   // case-insensitive
    REQUIRE(p("NONSENSE").fx == Catch::Approx(0.5f));  // unknown falls to CENTER
}

TEST_CASE("UIParent fills the screen", "[widget][layout]") {
    WidgetTree tree;
    tree.layout(kScreenW, kScreenH);
    const Widget* root = tree.get(tree.root());
    REQUIRE(root != nullptr);
    REQUIRE(root->left == Catch::Approx(0.0f));
    REQUIRE(root->bottom == Catch::Approx(0.0f));
    REQUIRE(root->rectW == Catch::Approx(kScreenW));
    REQUIRE(root->rectH == Catch::Approx(kScreenH));
}

TEST_CASE("One anchor plus a size positions the frame", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
    Widget* w = tree.get(f);
    w->width = 100.0f;
    w->height = 50.0f;

    Anchor a;
    a.point = "BOTTOMLEFT";
    a.relativePoint = "BOTTOMLEFT";
    a.x = 10.0f;
    a.y = 20.0f;
    tree.addPoint(f, a);

    tree.layout(kScreenW, kScreenH);
    REQUIRE(w->left == Catch::Approx(10.0f));
    REQUIRE(w->bottom == Catch::Approx(20.0f));
    REQUIRE(w->rectW == Catch::Approx(100.0f));
    REQUIRE(w->rectH == Catch::Approx(50.0f));
}

TEST_CASE("Anchoring by CENTER offsets from the middle of the parent",
          "[widget][layout]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
    Widget* w = tree.get(f);
    w->width = 200.0f;
    w->height = 100.0f;
    Anchor a;   // defaults are CENTER to CENTER
    tree.addPoint(f, a);

    tree.layout(kScreenW, kScreenH);
    // Its centre lands on the screen centre, so its corner is half its size away.
    REQUIRE(w->left == Catch::Approx(kScreenW * 0.5f - 100.0f));
    REQUIRE(w->bottom == Catch::Approx(kScreenH * 0.5f - 50.0f));
}

TEST_CASE("Two opposing anchors derive the size", "[widget][layout]") {
    // This is what SetAllPoints relies on, and what most of FrameXML's
    // backgrounds and borders use instead of ever stating a size.
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
    Widget* w = tree.get(f);
    w->width = 1.0f;    // deliberately wrong; the anchors must win
    w->height = 1.0f;

    Anchor tl; tl.point = "TOPLEFT";     tl.relativePoint = "TOPLEFT";     tl.x =  40.0f; tl.y = -30.0f;
    Anchor br; br.point = "BOTTOMRIGHT"; br.relativePoint = "BOTTOMRIGHT"; br.x = -60.0f; br.y =  50.0f;
    tree.addPoint(f, tl);
    tree.addPoint(f, br);

    tree.layout(kScreenW, kScreenH);
    REQUIRE(w->left == Catch::Approx(40.0f));
    REQUIRE(w->bottom == Catch::Approx(50.0f));
    REQUIRE(w->rectW == Catch::Approx(kScreenW - 40.0f - 60.0f));
    REQUIRE(w->rectH == Catch::Approx(kScreenH - 30.0f - 50.0f));
}

TEST_CASE("SetAllPoints matches the target exactly", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t parent = tree.create(WidgetKind::Frame, 0, "P");
    Widget* p = tree.get(parent);
    p->width = 300.0f;
    p->height = 200.0f;
    Anchor pa; pa.point = "BOTTOMLEFT"; pa.relativePoint = "BOTTOMLEFT"; pa.x = 12.0f; pa.y = 34.0f;
    tree.addPoint(parent, pa);

    const uint32_t tex = tree.create(WidgetKind::Texture, parent, "");
    tree.setAllPoints(tex, parent);

    tree.layout(kScreenW, kScreenH);
    const Widget* t = tree.get(tex);
    REQUIRE(t->left == Catch::Approx(p->left));
    REQUIRE(t->bottom == Catch::Approx(p->bottom));
    REQUIRE(t->rectW == Catch::Approx(p->rectW));
    REQUIRE(t->rectH == Catch::Approx(p->rectH));
}

TEST_CASE("A frame can anchor to a sibling, not just its parent", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t a = tree.create(WidgetKind::Frame, 0, "A");
    tree.get(a)->width = 50.0f;
    tree.get(a)->height = 50.0f;
    Anchor aa; aa.point = "BOTTOMLEFT"; aa.relativePoint = "BOTTOMLEFT"; aa.x = 100.0f; aa.y = 100.0f;
    tree.addPoint(a, aa);

    const uint32_t b = tree.create(WidgetKind::Frame, 0, "B");
    tree.get(b)->width = 50.0f;
    tree.get(b)->height = 50.0f;
    Anchor ba;
    ba.point = "LEFT";
    ba.relativeTo = a;
    ba.relativePoint = "RIGHT";
    ba.x = 8.0f;
    tree.addPoint(b, ba);

    tree.layout(kScreenW, kScreenH);
    // Sits just right of A, vertically centred on it.
    REQUIRE(tree.get(b)->left == Catch::Approx(158.0f));
    REQUIRE(tree.get(b)->bottom == Catch::Approx(100.0f));
}

TEST_CASE("Hiding a frame hides everything under it", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t parent = tree.create(WidgetKind::Frame, 0, "P");
    tree.get(parent)->width = 100.0f;
    tree.get(parent)->height = 100.0f;
    tree.addPoint(parent, Anchor{});

    const uint32_t tex = tree.create(WidgetKind::Texture, parent, "");
    tree.get(tex)->texturePath = "Interface\\Buttons\\Button-Backpack-Up.blp";
    tree.setAllPoints(tex, parent);

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().size() == 1);

    tree.get(parent)->shown = false;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().empty());
}

TEST_CASE("Draw order runs strata, then level, then layer", "[widget][draworder]") {
    WidgetTree tree;
    auto addTexture = [&](uint32_t parent, DrawLayer layer, const char* name) {
        const uint32_t t = tree.create(WidgetKind::Texture, parent, name);
        Widget* w = tree.get(t);
        w->texturePath = "x.blp";
        w->layer = layer;
        tree.setAllPoints(t, parent);
        return t;
    };

    const uint32_t lowFrame = tree.create(WidgetKind::Frame, 0, "Low");
    tree.get(lowFrame)->strata = FrameStrata::Low;
    tree.get(lowFrame)->strataExplicit = true;
    tree.get(lowFrame)->width = 10.0f;
    tree.get(lowFrame)->height = 10.0f;
    tree.addPoint(lowFrame, Anchor{});

    const uint32_t highFrame = tree.create(WidgetKind::Frame, 0, "High");
    tree.get(highFrame)->strata = FrameStrata::High;
    tree.get(highFrame)->strataExplicit = true;
    tree.get(highFrame)->width = 10.0f;
    tree.get(highFrame)->height = 10.0f;
    tree.addPoint(highFrame, Anchor{});

    // Deliberately created in the wrong order: an OVERLAY in a low stratum must
    // still fall behind a BACKGROUND in a high one.
    const uint32_t highBackground = addTexture(highFrame, DrawLayer::Background, "highBg");
    const uint32_t lowOverlay     = addTexture(lowFrame,  DrawLayer::Overlay,    "lowOver");
    const uint32_t lowBackground  = addTexture(lowFrame,  DrawLayer::Background, "lowBg");

    tree.layout(kScreenW, kScreenH);
    const auto& order = tree.drawOrder();
    REQUIRE(order.size() == 3);

    auto positionOf = [&](uint32_t id) {
        for (size_t i = 0; i < order.size(); ++i) if (order[i]->id == id) return i;
        return order.size();
    };
    REQUIRE(positionOf(lowBackground) < positionOf(lowOverlay));
    REQUIRE(positionOf(lowOverlay) < positionOf(highBackground));
}

TEST_CASE("A child frame draws over its parent", "[widget][draworder]") {
    WidgetTree tree;
    const uint32_t parent = tree.create(WidgetKind::Frame, 0, "P");
    tree.get(parent)->width = 100.0f;
    tree.get(parent)->height = 100.0f;
    tree.addPoint(parent, Anchor{});
    const uint32_t parentArt = tree.create(WidgetKind::Texture, parent, "pa");
    tree.get(parentArt)->texturePath = "p.blp";
    tree.setAllPoints(parentArt, parent);

    const uint32_t child = tree.create(WidgetKind::Frame, parent, "C");
    tree.setAllPoints(child, parent);
    const uint32_t childArt = tree.create(WidgetKind::Texture, child, "ca");
    tree.get(childArt)->texturePath = "c.blp";
    tree.setAllPoints(childArt, child);

    tree.layout(kScreenW, kScreenH);
    const auto& order = tree.drawOrder();
    REQUIRE(order.size() == 2);
    REQUIRE(order[0]->id == parentArt);
    REQUIRE(order[1]->id == childArt);
}

TEST_CASE("Nothing to draw is not drawn", "[widget][draworder]") {
    WidgetTree tree;
    const uint32_t parent = tree.create(WidgetKind::Frame, 0, "P");
    tree.get(parent)->width = 100.0f;
    tree.get(parent)->height = 100.0f;
    tree.addPoint(parent, Anchor{});

    // A frame is a container and paints nothing itself.
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().empty());

    // A texture with no source, and a font string with no text, likewise.
    const uint32_t empty = tree.create(WidgetKind::Texture, parent, "");
    tree.setAllPoints(empty, parent);
    const uint32_t blank = tree.create(WidgetKind::FontString, parent, "");
    tree.setAllPoints(blank, parent);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().empty());

    // A solid colour counts as a source even with no file behind it.
    tree.get(empty)->solidColor = true;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().size() == 1);
}

TEST_CASE("A zero-sized region is skipped rather than drawn degenerate",
          "[widget][draworder]") {
    WidgetTree tree;
    const uint32_t parent = tree.create(WidgetKind::Frame, 0, "P");
    tree.get(parent)->width = 100.0f;
    tree.get(parent)->height = 100.0f;
    tree.addPoint(parent, Anchor{});

    const uint32_t tex = tree.create(WidgetKind::Texture, parent, "");
    tree.get(tex)->texturePath = "x.blp";
    tree.addPoint(tex, Anchor{});   // anchored, but never given a size

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().empty());
}

TEST_CASE("Two anchors at the same point do not blow the rect apart",
          "[widget][layout]") {
    // Anchoring a frame twice at the same point is redundant rather than a size
    // constraint. Solving it as one would divide by a near-zero spread and throw
    // the rect off the screen.
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
    tree.get(f)->width = 80.0f;
    tree.get(f)->height = 40.0f;

    Anchor a; a.point = "CENTER"; a.relativePoint = "CENTER";
    Anchor b; b.point = "CENTER"; b.relativePoint = "CENTER"; b.x = 5.0f;
    tree.addPoint(f, a);
    tree.addPoint(f, b);

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->rectW == Catch::Approx(80.0f));
    REQUIRE(tree.get(f)->rectH == Catch::Approx(40.0f));
    REQUIRE(std::abs(tree.get(f)->left) < kScreenW);
}

// ── Hit testing ─────────────────────────────────────────────────────────────

namespace {
uint32_t makeButton(WidgetTree& tree, float x, float y, float w, float h,
                    FrameStrata strata = FrameStrata::Medium) {
    const uint32_t id = tree.create(WidgetKind::Frame, 0, "");
    Widget* f = tree.get(id);
    f->width = w;
    f->height = h;
    f->mouseEnabled = true;
    f->strata = strata;
    f->strataExplicit = true;
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT"; a.x = x; a.y = y;
    tree.addPoint(id, a);
    return id;
}
}

TEST_CASE("A frame is only hit inside its rect", "[widget][hittest]") {
    WidgetTree tree;
    const uint32_t b = makeButton(tree, 100.0f, 100.0f, 50.0f, 40.0f);
    tree.layout(kScreenW, kScreenH);

    REQUIRE(tree.hitTest(125.0f, 120.0f) == b);   // middle
    REQUIRE(tree.hitTest(100.0f, 100.0f) == b);   // corner counts
    REQUIRE(tree.hitTest(99.0f, 120.0f) == 0);    // just left
    REQUIRE(tree.hitTest(125.0f, 141.0f) == 0);   // just above
}

TEST_CASE("A frame without the mouse enabled is transparent to clicks",
          "[widget][hittest]") {
    // WoW's default, and the reason a plain Frame used as a container does not
    // steal clicks from whatever is underneath it.
    WidgetTree tree;
    const uint32_t f = makeButton(tree, 0.0f, 0.0f, 100.0f, 100.0f);
    tree.get(f)->mouseEnabled = false;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(50.0f, 50.0f) == 0);
}

TEST_CASE("A hidden frame cannot be clicked", "[widget][hittest]") {
    WidgetTree tree;
    const uint32_t f = makeButton(tree, 0.0f, 0.0f, 100.0f, 100.0f);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(50.0f, 50.0f) == f);

    tree.get(f)->shown = false;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(50.0f, 50.0f) == 0);
}

TEST_CASE("The frame drawn on top is the frame that gets the click",
          "[widget][hittest]") {
    // The whole point: what the player can see is what they hit. Overlapping
    // frames must resolve the same way the draw order does, or a click lands on
    // something buried.
    WidgetTree tree;
    const uint32_t low  = makeButton(tree, 0.0f, 0.0f, 100.0f, 100.0f, FrameStrata::Low);
    const uint32_t high = makeButton(tree, 0.0f, 0.0f, 100.0f, 100.0f, FrameStrata::High);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(50.0f, 50.0f) == high);

    // With strata equal, the later frame is on top and takes it.
    tree.get(high)->strata = FrameStrata::Low;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(50.0f, 50.0f) == high);
    REQUIRE(low != high);
}

TEST_CASE("A child frame takes the click from its parent", "[widget][hittest]") {
    WidgetTree tree;
    const uint32_t parent = makeButton(tree, 0.0f, 0.0f, 200.0f, 200.0f);
    const uint32_t child = tree.create(WidgetKind::Frame, parent, "");
    tree.get(child)->width = 50.0f;
    tree.get(child)->height = 50.0f;
    tree.get(child)->mouseEnabled = true;
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT"; a.x = 10.0f; a.y = 10.0f;
    tree.addPoint(child, a);

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(30.0f, 30.0f) == child);    // over the child
    REQUIRE(tree.hitTest(150.0f, 150.0f) == parent); // parent elsewhere
}

TEST_CASE("A zero-sized frame is never hit", "[widget][hittest]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "");
    tree.get(f)->mouseEnabled = true;
    tree.addPoint(f, Anchor{});   // anchored but never sized
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(kScreenW * 0.5f, kScreenH * 0.5f) == 0);
}

// ── Backdrop and status bar geometry ────────────────────────────────────────

TEST_CASE("A frame with a backdrop draws; a bare frame does not",
          "[widget][backdrop]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
    tree.get(f)->width = 100.0f;
    tree.get(f)->height = 60.0f;
    tree.addPoint(f, Anchor{});

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().empty());          // a container paints nothing

    tree.get(f)->hasBackdrop = true;
    tree.get(f)->bgFile = "Interface\\Tooltips\\UI-Tooltip-Background";
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().size() == 1);
    REQUIRE(tree.drawOrder()[0]->id == f);
}

TEST_CASE("A frame's backdrop draws beneath its own regions",
          "[widget][backdrop][draworder]") {
    // The backdrop is the panel; anything the frame owns belongs on top of it.
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
    tree.get(f)->width = 100.0f;
    tree.get(f)->height = 60.0f;
    tree.get(f)->hasBackdrop = true;
    tree.addPoint(f, Anchor{});

    const uint32_t art = tree.create(WidgetKind::Texture, f, "");
    tree.get(art)->texturePath = "x.blp";
    tree.setAllPoints(art, f);

    tree.layout(kScreenW, kScreenH);
    const auto& order = tree.drawOrder();
    REQUIRE(order.size() == 2);
    REQUIRE(order[0]->id == f);
    REQUIRE(order[1]->id == art);
}

TEST_CASE("Status bar fill is clamped and survives a degenerate range",
          "[widget][statusbar]") {
    WidgetTree tree;
    const uint32_t b = tree.create(WidgetKind::Frame, 0, "B");
    Widget* w = tree.get(b);
    w->isStatusBar = true;
    w->barMin = 0.0f;
    w->barMax = 100.0f;

    w->barValue = 50.0f;
    REQUIRE(w->barFraction() == Catch::Approx(0.5f));
    w->barValue = 0.0f;
    REQUIRE(w->barFraction() == Catch::Approx(0.0f));
    w->barValue = 100.0f;
    REQUIRE(w->barFraction() == Catch::Approx(1.0f));

    // Out of range clamps rather than overflowing the bar.
    w->barValue = 250.0f;
    REQUIRE(w->barFraction() == Catch::Approx(1.0f));
    w->barValue = -10.0f;
    REQUIRE(w->barFraction() == Catch::Approx(0.0f));

    // A bar whose range was never set, or set backwards, reads empty instead of
    // dividing by nothing - health frames are created before their values are
    // known and would otherwise flash full or NaN on the first frame.
    w->barMin = 0.0f; w->barMax = 0.0f; w->barValue = 5.0f;
    REQUIRE(w->barFraction() == Catch::Approx(0.0f));
    w->barMin = 100.0f; w->barMax = 0.0f;
    REQUIRE(w->barFraction() == Catch::Approx(0.0f));
}

TEST_CASE("A status bar with no texture and no backdrop is not drawn",
          "[widget][statusbar]") {
    WidgetTree tree;
    const uint32_t b = tree.create(WidgetKind::Frame, 0, "B");
    tree.get(b)->isStatusBar = true;
    tree.get(b)->width = 80.0f;
    tree.get(b)->height = 10.0f;
    tree.addPoint(b, Anchor{});
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().empty());

    tree.get(b)->barTexture = "bar.blp";
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.drawOrder().size() == 1);
}

TEST_CASE("The interface is laid out in units, whatever the display is",
          "[widget][layout]") {
    // FrameXML is authored against a virtual screen 768 units tall, so a frame
    // of a given size looks the same on every display. The tree works in those
    // units and the renderer multiplies once. Getting this wrong is not a
    // rounding error: treating units as pixels drew the whole interface at
    // half size on a 1528-tall window.
    //
    // Only the height sets the scale. The width follows from it, which is why
    // a wide display shows more of the world beside the same-sized frames
    // rather than larger ones - and why a portrait display, where the height
    // is the long side, draws the interface bigger. That is what the original
    // client does too.
    WidgetTree tree;

    SECTION("a 768-tall display is one unit per pixel") {
        tree.layout(1024.0f, 768.0f);
        REQUIRE(tree.uiScale() == Catch::Approx(1.0f));
        REQUIRE(tree.get(tree.root())->rectW == Catch::Approx(1024.0f));
        REQUIRE(tree.get(tree.root())->rectH == Catch::Approx(768.0f));
    }

    SECTION("a 1440p ultrawide is wider in units, not taller") {
        tree.layout(3440.0f, 1440.0f);
        REQUIRE(tree.uiScale() == Catch::Approx(1440.0f / 768.0f));
        REQUIRE(tree.get(tree.root())->rectH == Catch::Approx(768.0f));
        REQUIRE(tree.get(tree.root())->rectW == Catch::Approx(3440.0f * 768.0f / 1440.0f));
    }

    SECTION("a portrait display is narrow in units and scaled up") {
        // 1080x1920 rotated. The virtual screen is 432 units across, so the
        // interface is drawn at two and a half times size against very little
        // width - which is the original client's behaviour on the same
        // monitor, not a fault to correct here.
        tree.layout(1080.0f, 1920.0f);
        REQUIRE(tree.uiScale() == Catch::Approx(2.5f));
        REQUIRE(tree.get(tree.root())->rectW == Catch::Approx(432.0f));
        REQUIRE(tree.get(tree.root())->rectH == Catch::Approx(768.0f));
    }

    SECTION("a frame keeps its authored size in units on every display") {
        const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "F");
        tree.get(f)->width = 232.0f;
        tree.get(f)->height = 100.0f;
        tree.addPoint(f, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});
        for (auto wh : {std::pair<float, float>{1024.0f, 768.0f},
                        {2560.0f, 1440.0f},
                        {1080.0f, 1920.0f}}) {
            tree.layout(wh.first, wh.second);
            REQUIRE(tree.get(f)->rectW == Catch::Approx(232.0f));
            REQUIRE(tree.get(f)->rectH == Catch::Approx(100.0f));
        }
    }
}

TEST_CASE("A frame's anchors can be read back as they were set",
          "[widget][anchor]") {
    // FrameXML reads a point and puts it straight back to move something - a
    // dragged chat window, a frame the panel manager shifts aside. Anything
    // less than the anchor it was given means that round trip moves the frame,
    // and a constant means it moves to the same place every time.
    WidgetTree tree;
    const uint32_t a = tree.create(WidgetKind::Frame, tree.root(), "A");
    const uint32_t b = tree.create(WidgetKind::Frame, tree.root(), "B");

    tree.addPoint(b, Anchor{"TOPLEFT", a, "BOTTOMRIGHT", 7.0f, -3.0f});
    tree.addPoint(b, Anchor{"BOTTOMRIGHT", 0, "BOTTOMRIGHT", -4.0f, 5.0f});

    const Widget* w = tree.get(b);
    REQUIRE(w->anchors.size() == 2);
    REQUIRE(w->anchors[0].point == "TOPLEFT");
    REQUIRE(w->anchors[0].relativeTo == a);
    REQUIRE(w->anchors[0].relativePoint == "BOTTOMRIGHT");
    REQUIRE(w->anchors[0].x == Catch::Approx(7.0f));
    REQUIRE(w->anchors[0].y == Catch::Approx(-3.0f));
    // Zero means "my parent" rather than a frame that was never named, which
    // is what SetPoint's own default means too.
    REQUIRE(w->anchors[1].relativeTo == 0);

    // Clearing and re-applying the first anchor must land the frame where it
    // already was, which is the round trip the interface actually performs.
    tree.get(a)->width = 40.0f;
    tree.get(a)->height = 20.0f;
    tree.addPoint(a, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});
    tree.get(b)->width = 10.0f;
    tree.get(b)->height = 10.0f;
    tree.clearPoints(b);
    tree.addPoint(b, Anchor{"TOPLEFT", a, "BOTTOMRIGHT", 7.0f, -3.0f});
    tree.layout(1024.0f, 768.0f);
    const float left = tree.get(b)->left;
    const float bottom = tree.get(b)->bottom;

    const Anchor readBack = tree.get(b)->anchors[0];
    tree.clearPoints(b);
    tree.addPoint(b, readBack);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(b)->left == Catch::Approx(left));
    REQUIRE(tree.get(b)->bottom == Catch::Approx(bottom));
}

TEST_CASE("A widget knows what type it is", "[widget]") {
    // FrameXML branches on this: whether to treat something as a region it can
    // position, whether a frame is a Button worth clicking. Answering "Frame"
    // for everything makes each of those branches take the wrong side, and
    // silently, because the answer is a plausible string rather than nothing.
    WidgetTree tree;
    const uint32_t button = tree.create(WidgetKind::Frame, tree.root(), "B");
    tree.get(button)->objectType = "Button";
    const uint32_t tex = tree.create(WidgetKind::Texture, button, "T");
    tree.get(tex)->objectType = "Texture";

    REQUIRE(tree.get(button)->objectType == "Button");
    REQUIRE(tree.get(tex)->objectType == "Texture");
    // A widget created without being told keeps the type a plain frame has,
    // which is what CreateFrame's own default argument means.
    REQUIRE(tree.get(tree.root())->objectType == "Frame");
}

TEST_CASE("A scroll frame is a window onto a taller child", "[widget][scroll]") {
    // Scrolling down means seeing content further down the child, which is the
    // child moving up - and up is a larger bottom in these coordinates. The
    // whole feature was absent: SetVerticalScroll did nothing and every getter
    // answered zero, so a scroll bar had nothing to report and nothing to move.
    WidgetTree tree;
    const uint32_t frame = tree.create(WidgetKind::Frame, tree.root(), "S");
    tree.get(frame)->isScrollFrame = true;
    tree.get(frame)->width = 200.0f;
    tree.get(frame)->height = 100.0f;
    tree.addPoint(frame, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});

    const uint32_t child = tree.create(WidgetKind::Frame, frame, "SChild");
    tree.get(child)->width = 200.0f;
    tree.get(child)->height = 300.0f;
    // Anchored so its top sits at the frame's top, as a scroll child is.
    tree.addPoint(child, Anchor{"TOPLEFT", frame, "TOPLEFT", 0.0f, 0.0f});
    tree.get(frame)->scrollChild = child;

    tree.layout(1024.0f, 768.0f);
    const float unscrolled = tree.get(child)->bottom;

    tree.get(frame)->scrollY = 50.0f;
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(child)->bottom == Catch::Approx(unscrolled + 50.0f));

    // Everything under the frame is clipped to it, however deep - a scroll
    // child holds frames of its own.
    const uint32_t grandchild = tree.create(WidgetKind::Frame, child, "SGrand");
    tree.get(grandchild)->width = 10.0f;
    tree.get(grandchild)->height = 10.0f;
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(child)->clipTo == frame);
    REQUIRE(tree.get(grandchild)->clipTo == frame);
    // The frame itself is not clipped by itself.
    REQUIRE(tree.get(frame)->clipTo == 0);
}

TEST_CASE("Only a frame that asked for the wheel takes it", "[widget][scroll]") {
    // As in WoW, where a frame ignores the wheel until EnableMouseWheel is
    // called. It matters in both directions: a frame that did not ask must not
    // swallow the wheel, or the camera stops zooming wherever the interface
    // happens to be; and the frame that did ask is usually not the one under
    // the cursor, since a scroll child fills its parent and takes the hit.
    WidgetTree tree;
    const uint32_t scroll = tree.create(WidgetKind::Frame, tree.root(), "S");
    tree.get(scroll)->isScrollFrame = true;
    tree.get(scroll)->wheelEnabled = true;
    tree.get(scroll)->mouseEnabled = true;
    tree.get(scroll)->width = 100.0f;
    tree.get(scroll)->height = 100.0f;
    tree.addPoint(scroll, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});

    const uint32_t child = tree.create(WidgetKind::Frame, scroll, "SChild");
    tree.get(child)->mouseEnabled = true;
    tree.get(child)->width = 100.0f;
    tree.get(child)->height = 300.0f;
    tree.addPoint(child, Anchor{"TOPLEFT", scroll, "TOPLEFT", 0.0f, 0.0f});

    tree.layout(1024.0f, 768.0f);

    // The child is what the cursor lands on; the scroll frame above it is what
    // should handle the wheel, which is the walk up the dispatch performs.
    // hitTest measures upward from the bottom, as the tree does; the caller
    // is what flips the cursor into it.
    const uint32_t hit = tree.hitTest(50.0f, 50.0f);
    REQUIRE(hit == child);
    REQUIRE_FALSE(tree.get(child)->wheelEnabled);
    REQUIRE(tree.get(tree.get(child)->parent)->wheelEnabled);
}

TEST_CASE("Scrolled out of sight is out of reach", "[widget][scroll][hittest]") {
    // Clipping without this is only half the feature: the part of a scroll
    // child above or below the window is not drawn, so it must not answer
    // clicks either - a quest log that reacts to entries nobody can see is
    // worse than one that does not scroll at all.
    WidgetTree tree;
    const uint32_t scroll = tree.create(WidgetKind::Frame, tree.root(), "S");
    tree.get(scroll)->isScrollFrame = true;
    tree.get(scroll)->width = 100.0f;
    tree.get(scroll)->height = 100.0f;
    tree.addPoint(scroll, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 200.0f});

    // An entry inside the child, positioned below the window rather than in it.
    const uint32_t child = tree.create(WidgetKind::Frame, scroll, "SChild");
    tree.get(child)->width = 100.0f;
    tree.get(child)->height = 300.0f;
    tree.addPoint(child, Anchor{"TOPLEFT", scroll, "TOPLEFT", 0.0f, 0.0f});
    tree.get(scroll)->scrollChild = child;

    const uint32_t entry = tree.create(WidgetKind::Frame, child, "SEntry");
    tree.get(entry)->mouseEnabled = true;
    tree.get(entry)->width = 100.0f;
    tree.get(entry)->height = 20.0f;
    tree.addPoint(entry, Anchor{"BOTTOMLEFT", child, "BOTTOMLEFT", 0.0f, 0.0f});

    tree.layout(1024.0f, 768.0f);
    // The entry sits at the bottom of a 300-tall child hanging below a 100-tall
    // window, so it is well outside it.
    const Widget* e = tree.get(entry);
    REQUIRE(e->clipTo == scroll);
    REQUIRE(tree.hitTest(50.0f, e->bottom + 10.0f) == 0);

    // Scrolled far enough, the same entry comes into the window and answers.
    tree.get(scroll)->scrollY = 200.0f;
    tree.layout(1024.0f, 768.0f);
    const Widget* moved = tree.get(entry);
    REQUIRE(tree.hitTest(50.0f, moved->bottom + 10.0f) == entry);
}

TEST_CASE("Scroll frames are tracked as they are marked", "[widget][scroll]") {
    // The range has to be re-checked every frame, and walking every widget to
    // find a handful of scroll frames is the kind of cost that does not show
    // up until the interface is large - which, with FrameXML loaded, it is.
    WidgetTree tree;
    const uint32_t a = tree.create(WidgetKind::Frame, tree.root(), "A");
    const uint32_t b = tree.create(WidgetKind::Frame, tree.root(), "B");
    tree.create(WidgetKind::Frame, tree.root(), "C");

    tree.markScrollFrame(a);
    tree.markScrollFrame(b);
    // Marking twice must not list it twice: SetScrollChild and CreateFrame
    // both mark, and the same frame goes through both.
    tree.markScrollFrame(a);

    REQUIRE(tree.scrollFrames().size() == 2);
    REQUIRE(tree.get(a)->isScrollFrame);
    REQUIRE(tree.get(b)->isScrollFrame);
}

TEST_CASE("Visibility is a state to be noticed, not an event to be sent",
          "[widget][layout]") {
    // Hiding a frame hides everything under it, and none of those had Hide
    // called on them - so anything watching for a frame to go away has to
    // compare what layout resolved rather than listen at the point something
    // was hidden three levels up. This is the property that makes it possible.
    WidgetTree tree;
    const uint32_t panel = tree.create(WidgetKind::Frame, tree.root(), "P");
    tree.get(panel)->width = 100.0f;
    tree.get(panel)->height = 100.0f;
    tree.addPoint(panel, Anchor{"CENTER", 0, "CENTER", 0.0f, 0.0f});

    const uint32_t inner = tree.create(WidgetKind::Frame, panel, "PInner");
    tree.get(inner)->width = 50.0f;
    tree.get(inner)->height = 50.0f;
    tree.addPoint(inner, Anchor{"CENTER", panel, "CENTER", 0.0f, 0.0f});

    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(panel)->visible);
    REQUIRE(tree.get(inner)->visible);

    // Only the panel is hidden; the child is still shown in its own right.
    tree.get(panel)->shown = false;
    tree.layout(1024.0f, 768.0f);
    REQUIRE_FALSE(tree.get(panel)->visible);
    REQUIRE_FALSE(tree.get(inner)->visible);
    REQUIRE(tree.get(inner)->shown);

    tree.get(panel)->shown = true;
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(inner)->visible);
}

TEST_CASE("A button shows one state texture, not all of them",
          "[widget][draworder]") {
    // A button carries art for each state and shows one. Drawing all of them
    // puts the disabled art over the normal art with the highlight permanently
    // on top, which is not a subtle fault - every button in the interface
    // looks hovered and wrong at once.
    WidgetTree tree;
    const uint32_t button = tree.create(WidgetKind::Frame, tree.root(), "B");
    tree.get(button)->objectType = "Button";
    tree.get(button)->width = 100.0f;
    tree.get(button)->height = 20.0f;
    tree.addPoint(button, Anchor{"CENTER", 0, "CENTER", 0.0f, 0.0f});

    auto art = [&](const char* name, ButtonArt kind) {
        const uint32_t t = tree.create(WidgetKind::Texture, button, name);
        tree.get(t)->texturePath = "Interface\\Art";
        tree.get(t)->buttonArt = kind;
        tree.setAllPoints(t, button);
        return t;
    };
    const uint32_t normal    = art("BN", ButtonArt::Normal);
    const uint32_t pushed    = art("BP", ButtonArt::Pushed);
    const uint32_t highlight = art("BH", ButtonArt::Highlight);
    const uint32_t disabled  = art("BD", ButtonArt::Disabled);

    auto drawn = [&](uint32_t id) {
        for (const Widget* w : tree.drawOrder()) if (w->id == id) return true;
        return false;
    };

    // Idle: normal only.
    tree.setInteraction(0, 0);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(drawn(normal));
    REQUIRE_FALSE(drawn(pushed));
    REQUIRE_FALSE(drawn(highlight));
    REQUIRE_FALSE(drawn(disabled));

    // Hovered: the highlight joins it. The cursor lands on the button's own
    // art rather than the button, so hovering a child must count.
    tree.setInteraction(normal, 0);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(drawn(highlight));
    REQUIRE(drawn(normal));

    // Held: pushed replaces normal.
    tree.setInteraction(button, button);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(drawn(pushed));
    REQUIRE_FALSE(drawn(normal));

    // Disabled: only the disabled art, and no highlight however hovered.
    tree.get(button)->enabled = false;
    tree.setInteraction(button, 0);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(drawn(disabled));
    REQUIRE_FALSE(drawn(normal));
    REQUIRE_FALSE(drawn(highlight));
}

TEST_CASE("A state the interface asked for outlasts the cursor",
          "[widget][draworder]") {
    // ActionButton_UpdateState holds down the button for an ability that is
    // toggled on, and moving the mouse must not let it back up. A selected tab
    // locks its highlight the same way.
    WidgetTree tree;
    const uint32_t button = tree.create(WidgetKind::Frame, tree.root(), "B");
    tree.get(button)->width = 40.0f;
    tree.get(button)->height = 40.0f;
    tree.addPoint(button, Anchor{"CENTER", 0, "CENTER", 0.0f, 0.0f});

    auto art = [&](const char* name, ButtonArt kind) {
        const uint32_t t = tree.create(WidgetKind::Texture, button, name);
        tree.get(t)->texturePath = "Interface\\Art";
        tree.get(t)->buttonArt = kind;
        tree.setAllPoints(t, button);
        return t;
    };
    const uint32_t normal    = art("BN", ButtonArt::Normal);
    const uint32_t pushed    = art("BP", ButtonArt::Pushed);
    const uint32_t highlight = art("BH", ButtonArt::Highlight);

    auto drawn = [&](uint32_t id) {
        for (const Widget* w : tree.drawOrder()) if (w->id == id) return true;
        return false;
    };

    // Held down by the interface, with the cursor nowhere near it.
    tree.get(button)->forcedState = Widget::Forced::Pushed;
    tree.setInteraction(0, 0);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(drawn(pushed));
    REQUIRE_FALSE(drawn(normal));

    // Released again, and the cursor still elsewhere.
    tree.get(button)->forcedState = Widget::Forced::Normal;
    tree.layout(1024.0f, 768.0f);
    REQUIRE(drawn(normal));
    REQUIRE_FALSE(drawn(pushed));

    // A locked highlight stays lit without the cursor; a forced-disabled
    // button puts it out again, since disabled art replaces the lot.
    tree.get(button)->highlightLocked = true;
    tree.layout(1024.0f, 768.0f);
    REQUIRE(drawn(highlight));
    tree.get(button)->forcedState = Widget::Forced::Disabled;
    tree.layout(1024.0f, 768.0f);
    REQUIRE_FALSE(drawn(highlight));
}

TEST_CASE("A region with neither size nor anchors fills its parent",
          "[widget][layout]") {
    // WoW's default, and not a rare shorthand: PlayerFrameTexture is the whole
    // of the player frame's art and MinimapBorder is the ring around the
    // minimap, and both are declared with nothing but a file. Laid out to
    // nothing they never reach the draw order, so they are never uploaded
    // either - which reads as missing art rather than as a layout fault.
    WidgetTree tree;
    const uint32_t frame = tree.create(WidgetKind::Frame, tree.root(), "F");
    tree.get(frame)->width = 232.0f;
    tree.get(frame)->height = 100.0f;
    tree.addPoint(frame, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 10.0f, 20.0f});

    const uint32_t art = tree.create(WidgetKind::Texture, frame, "FArt");
    tree.get(art)->texturePath = "Interface\\Art";

    tree.layout(1024.0f, 768.0f);
    const Widget* a = tree.get(art);
    REQUIRE(a->rectW == Catch::Approx(232.0f));
    REQUIRE(a->rectH == Catch::Approx(100.0f));
    REQUIRE(a->left == Catch::Approx(10.0f));
    REQUIRE(a->bottom == Catch::Approx(20.0f));

    // A stated size still wins: only a region that says nothing gets the
    // parent's rect, or every deliberately small unanchored region would
    // suddenly cover its frame.
    const uint32_t sized = tree.create(WidgetKind::Texture, frame, "FSized");
    tree.get(sized)->texturePath = "Interface\\Art";
    tree.get(sized)->width = 16.0f;
    tree.get(sized)->height = 16.0f;
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(sized)->rectW == Catch::Approx(16.0f));
}

TEST_CASE("A frame knows its level before it is ever laid out",
          "[widget][layout]") {
    // GetFrameLevel answers with this, and FrameXML asks during OnLoad:
    // RaiseFrameLevel is frame:SetFrameLevel(frame:GetFrameLevel() + 1). A
    // frame that had never been laid out answered zero, so the adjustment was
    // computed against nothing and the frame ended up below its own parent -
    // which for the action bar meant the bar took every click aimed at a
    // button sitting on it.
    WidgetTree tree;
    const uint32_t bar = tree.create(WidgetKind::Frame, tree.root(), "Bar");
    const uint32_t art = tree.create(WidgetKind::Frame, bar, "BarArt");
    const uint32_t button = tree.create(WidgetKind::Frame, art, "BarButton");

    // Before any layout at all.
    REQUIRE(tree.get(art)->effLevel == tree.get(bar)->effLevel + 1);
    REQUIRE(tree.get(button)->effLevel == tree.get(art)->effLevel + 1);

    // And the relative adjustment FrameXML performs now lands above the
    // parent rather than at one.
    const int raised = tree.get(art)->effLevel + 1;
    tree.get(art)->level = raised;
    tree.get(art)->levelExplicit = true;
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(art)->effLevel > tree.get(bar)->effLevel);
}

TEST_CASE("A message frame keeps its lines and drops the oldest",
          "[widget][chat]") {
    // Chat is a list that grows at one end and falls off the other, not a
    // single string. AddMessage was a name in the method list and nothing
    // else, so every line the interface handed it went nowhere: the frame
    // received the events, formatted the text, and dropped it.
    WidgetTree tree;
    const uint32_t chat = tree.create(WidgetKind::Frame, tree.root(), "Chat");
    Widget* w = tree.get(chat);
    w->isMessageFrame = true;
    w->maxMessages = 3;
    w->width = 200.0f;
    w->height = 60.0f;
    tree.addPoint(chat, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});

    for (const char* line : {"one", "two", "three", "four"}) {
        w->messages.push_back({line, {1.0f, 1.0f, 1.0f, 1.0f}});
        while (w->messages.size() > w->maxMessages) w->messages.pop_front();
    }
    REQUIRE(w->messages.size() == 3);
    REQUIRE(w->messages.front().text == "two");
    REQUIRE(w->messages.back().text == "four");

    // A frame with lines paints, even though a bare frame does not.
    tree.layout(1024.0f, 768.0f);
    bool drawn = false;
    for (const Widget* d : tree.drawOrder()) if (d->id == chat) drawn = true;
    REQUIRE(drawn);

    // Emptied, it goes back to painting nothing.
    w->messages.clear();
    tree.layout(1024.0f, 768.0f);
    drawn = false;
    for (const Widget* d : tree.drawOrder()) if (d->id == chat) drawn = true;
    REQUIRE_FALSE(drawn);
}

TEST_CASE("A tooltip with lines paints; an empty one does not",
          "[widget][tooltip]") {
    // AddLine was a name in the method list and nothing else, so every tooltip
    // in the interface was shown, positioned, sized - and empty. A tooltip is
    // also a frame, and a frame paints nothing on its own, so having lines has
    // to be what makes it draw.
    WidgetTree tree;
    const uint32_t tip = tree.create(WidgetKind::Frame, tree.root(), "Tip");
    Widget* w = tree.get(tip);
    w->isTooltip = true;
    w->width = 120.0f;
    w->height = 40.0f;
    tree.addPoint(tip, Anchor{"CENTER", 0, "CENTER", 0.0f, 0.0f});

    tree.layout(1024.0f, 768.0f);
    bool drawn = false;
    for (const Widget* d : tree.drawOrder()) if (d->id == tip) drawn = true;
    REQUIRE_FALSE(drawn);

    w->tooltipLines.push_back({"Thunderfury", "", {1,1,1,1}, {1,1,1,1}});
    tree.layout(1024.0f, 768.0f);
    for (const Widget* d : tree.drawOrder()) if (d->id == tip) drawn = true;
    REQUIRE(drawn);
}

TEST_CASE("A tooltip anchored to its owner sits beside it", "[widget][tooltip]") {
    // SetOwner is what every tooltip in the interface calls before filling
    // itself, and it decided nothing - so a tooltip with lines would still
    // have appeared wherever its XML left it rather than beside the button it
    // describes. ANCHOR_RIGHT means the tooltip's left edge meets the owner's
    // right, which is the whole of the mapping.
    WidgetTree tree;
    const uint32_t button = tree.create(WidgetKind::Frame, tree.root(), "B");
    tree.get(button)->width = 40.0f;
    tree.get(button)->height = 40.0f;
    tree.addPoint(button, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 100.0f, 100.0f});

    const uint32_t tip = tree.create(WidgetKind::Frame, tree.root(), "Tip");
    Widget* w = tree.get(tip);
    w->isTooltip = true;
    w->width = 120.0f;
    w->height = 40.0f;
    w->tooltipLines.push_back({"Fireball", "", {1,1,1,1}, {1,1,1,1}});
    tree.addPoint(tip, Anchor{"LEFT", button, "RIGHT", 0.0f, 0.0f});

    tree.layout(1024.0f, 768.0f);
    // The owner spans x 100..140; the tooltip starts where it ends.
    REQUIRE(tree.get(tip)->left == Catch::Approx(140.0f));
}

TEST_CASE("A frame with no anchor points is not displayed", "[widget][layout]") {
    // WoW's rule, and the reason for it is visible the moment it is missing:
    // FrameXML declares plenty of frames with no anchors - a money frame, a
    // dropdown, a quest reward panel - and every one of them fell to the
    // centre-on-parent default and sat in the middle of the screen looking
    // like a fault in something else.
    WidgetTree tree;
    const uint32_t stray = tree.create(WidgetKind::Frame, tree.root(), "Stray");
    tree.get(stray)->width = 285.0f;
    tree.get(stray)->height = 28.0f;

    tree.layout(1024.0f, 768.0f);
    REQUIRE_FALSE(tree.get(stray)->visible);

    // Anchored, it appears - and so does everything under it.
    const uint32_t child = tree.create(WidgetKind::Frame, stray, "StrayChild");
    tree.get(child)->width = 10.0f;
    tree.get(child)->height = 10.0f;
    tree.addPoint(child, Anchor{"CENTER", stray, "CENTER", 0.0f, 0.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE_FALSE(tree.get(child)->visible);

    tree.addPoint(stray, Anchor{"CENTER", 0, "CENTER", 0.0f, 0.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(stray)->visible);
    REQUIRE(tree.get(child)->visible);

    // A region is different: with no anchors it fills its parent rather than
    // vanishing, which is what the art declared that way relies on.
    const uint32_t art = tree.create(WidgetKind::Texture, stray, "StrayArt");
    tree.get(art)->texturePath = "Interface\\Art";
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(art)->visible);
    REQUIRE(tree.get(art)->rectW == Catch::Approx(285.0f));
}

TEST_CASE("Setting a point that is already set replaces it", "[widget][layout]") {
    // FrameXML repositions frames with a bare SetPoint and no ClearAllPoints,
    // and relies on the second call displacing the first. The durability frame
    // is the case that showed it: the XML anchors it forty units right of the
    // minimap's bottom-right corner, and UIParentManageFramePositions then
    // anchors the same TOPRIGHT point back inside the screen. Keeping both left
    // two constraints on one edge, the first won, and the frame stayed forty
    // units past the right edge of the screen.
    WidgetTree tree;
    const uint32_t cluster = tree.create(WidgetKind::Frame, tree.root(), "Cluster");
    tree.get(cluster)->width = 192.0f;
    tree.get(cluster)->height = 192.0f;
    tree.addPoint(cluster, Anchor{"TOPRIGHT", 0, "TOPRIGHT", 0.0f, 0.0f});

    const uint32_t dur = tree.create(WidgetKind::Frame, tree.root(), "Durability");
    tree.get(dur)->width = 60.0f;
    tree.get(dur)->height = 65.0f;
    tree.addPoint(dur, Anchor{"TOPRIGHT", cluster, "BOTTOMRIGHT", 40.0f, 15.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(dur)->anchors.size() == 1u);
    REQUIRE(tree.get(dur)->left + tree.get(dur)->rectW == Catch::Approx(1064.0f));

    // The reposition, with the same point named again.
    tree.addPoint(dur, Anchor{"TOPRIGHT", cluster, "BOTTOMRIGHT", -20.0f, 15.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(dur)->anchors.size() == 1u);
    REQUIRE(tree.get(dur)->left + tree.get(dur)->rectW == Catch::Approx(1004.0f));

    // A different point still adds, because that is how a frame gets sized from
    // two opposing corners.
    tree.addPoint(dur, Anchor{"BOTTOMLEFT", cluster, "BOTTOMLEFT", 0.0f, 0.0f});
    REQUIRE(tree.get(dur)->anchors.size() == 2u);
}

TEST_CASE("A size just set reads back before the next layout", "[widget][layout]") {
    // FrameXML sizes things and measures them in the same breath. The container
    // frames set the height of each piece of their background art and then add
    // those heights up to size the frame itself:
    //
    //     frame:SetHeight(bgTop:GetHeight() + bgBottom:GetHeight() + middle)
    //
    // Answering GetHeight from the last laid-out rect makes that sum the
    // previous frame's numbers, and the art and the buttons anchored inside it
    // then describe two different frames - which is what put the item slots of
    // an opened bag below the art drawn for it.
    WidgetTree tree;
    const uint32_t art = tree.create(WidgetKind::Texture, tree.root(), "Art");
    tree.addPoint(art, Anchor{"TOPLEFT", 0, "TOPLEFT", 0.0f, 0.0f});
    tree.setHeight(art, 40.0f);
    tree.setWidth(art, 100.0f);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(art)->rectH == Catch::Approx(40.0f));

    // Resized, and read back at once - no layout in between, exactly as
    // ContainerFrame_GenerateFrame does it.
    tree.setHeight(art, 94.0f);
    REQUIRE(tree.get(art)->rectH == Catch::Approx(94.0f));
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(art)->rectH == Catch::Approx(94.0f));

    // Where anchors decide the size, the layout still has the last word.
    const uint32_t stretched = tree.create(WidgetKind::Frame, tree.root(), "Stretched");
    tree.setHeight(stretched, 10.0f);
    tree.addPoint(stretched, Anchor{"TOPLEFT", 0, "TOPLEFT", 0.0f, 0.0f});
    tree.addPoint(stretched, Anchor{"BOTTOMRIGHT", 0, "BOTTOMRIGHT", 0.0f, 0.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(stretched)->rectH == Catch::Approx(768.0f));
}

TEST_CASE("A moved frame keeps its size and lands where it was dropped",
          "[widget][layout]") {
    // StartMoving has to detach a frame from whatever it was anchored to,
    // otherwise the drag and the anchor fight and the frame springs back. A bag
    // window is anchored to the one beside it, so this is every bag but the
    // first.
    WidgetTree tree;
    const uint32_t anchorFrame = tree.create(WidgetKind::Frame, tree.root(), "Neighbour");
    tree.get(anchorFrame)->width = 100.0f;
    tree.get(anchorFrame)->height = 50.0f;
    tree.addPoint(anchorFrame, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});

    const uint32_t bag = tree.create(WidgetKind::Frame, tree.root(), "Bag");
    tree.get(bag)->width = 200.0f;
    tree.get(bag)->height = 300.0f;
    tree.addPoint(bag, Anchor{"BOTTOMRIGHT", anchorFrame, "BOTTOMLEFT", 0.0f, 0.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(bag)->left == Catch::Approx(-200.0f));

    // Picked up: pinned where it stands, with its size intact.
    tree.pinToCurrentPosition(bag);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(bag)->anchors.size() == 1u);
    REQUIRE(tree.get(bag)->left == Catch::Approx(-200.0f));
    REQUIRE(tree.get(bag)->rectW == Catch::Approx(200.0f));
    REQUIRE(tree.get(bag)->rectH == Catch::Approx(300.0f));

    // Dragged, and it stays dragged rather than springing back to the
    // neighbour it used to hang off.
    tree.nudge(bag, 250.0f, 60.0f);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(bag)->left == Catch::Approx(50.0f));
    REQUIRE(tree.get(bag)->bottom == Catch::Approx(60.0f));
    REQUIRE(tree.get(bag)->rectW == Catch::Approx(200.0f));

    // Moving the neighbour no longer drags it along.
    tree.nudge(anchorFrame, 500.0f, 0.0f);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(bag)->left == Catch::Approx(50.0f));
}

TEST_CASE("A moved frame gives way when the interface positions it again",
          "[widget][layout]") {
    // The anchor a move leaves behind is on whichever point the frame was
    // picked up by, and the interface re-anchors on its own points without
    // clearing first - updateContainerFrameAnchors sets BOTTOMRIGHT on every
    // bag each time one opens. Keeping both left two constraints on one axis,
    // and the bag opened with no width.
    WidgetTree tree;
    const uint32_t bag = tree.create(WidgetKind::Frame, tree.root(), "Bag");
    tree.get(bag)->width = 192.0f;
    tree.get(bag)->height = 240.0f;
    tree.addPoint(bag, Anchor{"BOTTOMRIGHT", 0, "BOTTOMRIGHT", -10.0f, 20.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(bag)->rectW == Catch::Approx(192.0f));

    // Dragged somewhere else.
    tree.pinToCurrentPosition(bag);
    tree.nudge(bag, -300.0f, 100.0f);
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(bag)->anchors.size() == 1u);
    REQUIRE(tree.get(bag)->left == Catch::Approx(522.0f));
    REQUIRE(tree.get(bag)->rectW == Catch::Approx(192.0f));

    // The interface positions it again, on a different point and without
    // clearing: the move's anchor gives way rather than fighting it.
    tree.addPoint(bag, Anchor{"BOTTOMRIGHT", 0, "BOTTOMRIGHT", -10.0f, 20.0f});
    tree.layout(1024.0f, 768.0f);
    REQUIRE(tree.get(bag)->anchors.size() == 1u);
    REQUIRE(tree.get(bag)->rectW == Catch::Approx(192.0f));
    REQUIRE(tree.get(bag)->left == Catch::Approx(1024.0f - 10.0f - 192.0f));

    // And an ordinary re-anchor after that still replaces by point, not by
    // clearing everything.
    tree.addPoint(bag, Anchor{"TOPLEFT", 0, "TOPLEFT", 5.0f, -5.0f});
    REQUIRE(tree.get(bag)->anchors.size() == 2u);
}

TEST_CASE("A clamped frame stops at the screen edge", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Clamped");
    tree.setWidth(f, 200.0f);
    tree.setHeight(f, 100.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    a.x = 10.0f; a.y = 10.0f;
    tree.addPoint(f, a);
    tree.get(f)->clampedToScreen = true;
    tree.layout(kScreenW, kScreenH);

    // Dragged hard toward the bottom-left: it stops at the corner rather than
    // leaving the screen.
    tree.nudge(f, -500.0f, -500.0f);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->left == Catch::Approx(0.0f));
    REQUIRE(tree.get(f)->bottom == Catch::Approx(0.0f));

    // And toward the top-right, where the limit is the far edge less its size.
    tree.nudge(f, 5000.0f, 5000.0f);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->left == Catch::Approx(kScreenW - 200.0f));
    REQUIRE(tree.get(f)->bottom == Catch::Approx(kScreenH - 100.0f));
}

TEST_CASE("An unclamped frame is free to leave the screen", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Free");
    tree.setWidth(f, 200.0f);
    tree.setHeight(f, 100.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    a.x = 10.0f; a.y = 10.0f;
    tree.addPoint(f, a);
    tree.layout(kScreenW, kScreenH);

    tree.nudge(f, -500.0f, 0.0f);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->left == Catch::Approx(-490.0f));
}

TEST_CASE("A clamped frame is brought on-screen wherever it was placed",
          "[widget][layout]") {
    // Saved positions from a larger resolution land frames outside; pinning
    // them where they are would make them unrecoverable, which is the very
    // thing clamping exists to prevent.
    //
    // This used to assert that the layout left it stranded and only a *drag*
    // pulled it back. That was too narrow, and tooltips are what it cost:
    // GameTooltipTemplate is clamped and every tooltip inherits it, but a
    // tooltip is anchored beside its owner and never dragged, so one owned by
    // a frame near an edge ran straight off the screen. A clamped frame is
    // clamped however it got where it is.
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Stranded");
    tree.setWidth(f, 200.0f);
    tree.setHeight(f, 100.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    a.x = -300.0f; a.y = 10.0f;
    tree.addPoint(f, a);
    tree.get(f)->clampedToScreen = true;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->left == Catch::Approx(0.0f));

    // And it stays there: dragging further left cannot push it back out.
    tree.nudge(f, -50.0f, 0.0f);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->left == Catch::Approx(0.0f));
}

TEST_CASE("A clamped tooltip anchored past the right edge is pulled back in",
          "[widget][layout]") {
    // The calendar button sits at the right of the minimap and anchors its
    // tooltip ANCHOR_RIGHT, so the tooltip's left edge starts at the button's
    // right edge - a few pixels from the screen edge, with the whole width of
    // the tooltip still to come.
    WidgetTree tree;
    const uint32_t owner = tree.create(WidgetKind::Frame, tree.root(), "Button");
    tree.setWidth(owner, 32.0f);
    tree.setHeight(owner, 32.0f);
    Anchor oa; oa.point = "BOTTOMRIGHT"; oa.relativePoint = "BOTTOMRIGHT";
    oa.x = 0.0f; oa.y = 400.0f;
    tree.addPoint(owner, oa);

    const uint32_t tip = tree.create(WidgetKind::Frame, tree.root(), "Tip");
    tree.setWidth(tip, 250.0f);
    tree.setHeight(tip, 60.0f);
    Anchor ta; ta.point = "LEFT"; ta.relativeTo = owner; ta.relativePoint = "RIGHT";
    tree.addPoint(tip, ta);
    tree.get(tip)->clampedToScreen = true;
    tree.layout(kScreenW, kScreenH);

    // Fully on screen, and hard against the edge it would otherwise have
    // crossed rather than merely nearer to it.
    REQUIRE(tree.get(tip)->left + tree.get(tip)->rectW <= Catch::Approx(kScreenW));
    REQUIRE(tree.get(tip)->left == Catch::Approx(kScreenW - 250.0f));
}

TEST_CASE("A frame wider than the screen is not snapped to a nonsense edge",
          "[widget][layout]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Huge");
    tree.setWidth(f, kScreenW + 400.0f);
    tree.setHeight(f, 100.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    a.x = -100.0f; a.y = 10.0f;
    tree.addPoint(f, a);
    tree.get(f)->clampedToScreen = true;
    tree.layout(kScreenW, kScreenH);

    tree.nudge(f, -50.0f, 0.0f);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->left == Catch::Approx(-150.0f));  // moved, not pinned
}

TEST_CASE("Raise puts a frame in front of its strata", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t a = tree.create(WidgetKind::Frame, tree.root(), "A");
    const uint32_t b = tree.create(WidgetKind::Frame, tree.root(), "B");
    Anchor p; p.point = "CENTER"; p.relativePoint = "CENTER";
    tree.addPoint(a, p);
    tree.addPoint(b, p);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(a)->effLevel == tree.get(b)->effLevel);

    tree.raise(a);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(a)->effLevel > tree.get(b)->effLevel);

    // And it survives the next layout, which would otherwise recompute the
    // level from the parent and undo it.
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(a)->effLevel > tree.get(b)->effLevel);

    tree.raise(b);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(b)->effLevel > tree.get(a)->effLevel);
}

TEST_CASE("Raising does not cross a strata boundary", "[widget][layout]") {
    // Strata outrank levels: a raised DIALOG frame still sits under a TOOLTIP
    // one, which is the whole point of having strata.
    WidgetTree tree;
    const uint32_t low  = tree.create(WidgetKind::Frame, tree.root(), "Low");
    const uint32_t high = tree.create(WidgetKind::Frame, tree.root(), "High");
    Anchor p; p.point = "CENTER"; p.relativePoint = "CENTER";
    tree.addPoint(low, p);
    tree.addPoint(high, p);
    tree.get(low)->strata  = FrameStrata::Dialog;
    tree.get(low)->strataExplicit  = true;
    tree.get(high)->strata = FrameStrata::Tooltip;
    tree.get(high)->strataExplicit = true;
    tree.layout(kScreenW, kScreenH);

    tree.raise(low);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(low)->effStrata == FrameStrata::Dialog);
    REQUIRE(tree.get(high)->effStrata == FrameStrata::Tooltip);
}

TEST_CASE("Lower never sends a frame below zero", "[widget][layout]") {
    // A negative level sorts under the root and the frame stops being drawn.
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Sunk");
    Anchor p; p.point = "CENTER"; p.relativePoint = "CENTER";
    tree.addPoint(f, p);
    tree.layout(kScreenW, kScreenH);
    for (int i = 0; i < 5; ++i) { tree.lower(f); tree.layout(kScreenW, kScreenH); }
    REQUIRE(tree.get(f)->effLevel >= 0);
}

TEST_CASE("Scale sizes a frame and its offsets, and compounds", "[widget][layout]") {
    WidgetTree tree;
    const uint32_t outer = tree.create(WidgetKind::Frame, tree.root(), "Outer");
    tree.setWidth(outer, 200.0f);
    tree.setHeight(outer, 100.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    a.x = 100.0f; a.y = 50.0f;
    tree.addPoint(outer, a);
    tree.get(outer)->scale = 0.5f;

    // A child at its own scale: the two multiply, as they do in WoW.
    const uint32_t inner = tree.create(WidgetKind::Frame, outer, "Inner");
    tree.setWidth(inner, 100.0f);
    tree.setHeight(inner, 100.0f);
    Anchor b; b.point = "BOTTOMLEFT"; b.relativePoint = "BOTTOMLEFT";
    tree.addPoint(inner, b);
    tree.get(inner)->scale = 0.5f;
    tree.layout(kScreenW, kScreenH);

    REQUIRE(tree.get(outer)->rectW == Catch::Approx(100.0f));   // 200 * 0.5
    REQUIRE(tree.get(outer)->left  == Catch::Approx(50.0f));    // offset scaled too
    REQUIRE(tree.get(inner)->effScale == Catch::Approx(0.25f));
    REQUIRE(tree.get(inner)->rectW == Catch::Approx(25.0f));    // 100 * 0.25
}

TEST_CASE("An unscaled tree lays out exactly as before", "[widget][layout]") {
    // The safety property the whole change rests on: with every scale at 1,
    // each multiplication is by 1 and nothing moves.
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Plain");
    tree.setWidth(f, 200.0f);
    tree.setHeight(f, 100.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    a.x = 37.0f; a.y = 11.0f;
    tree.addPoint(f, a);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(f)->effScale == Catch::Approx(1.0f));
    REQUIRE(tree.get(f)->left   == Catch::Approx(37.0f));
    REQUIRE(tree.get(f)->bottom == Catch::Approx(11.0f));
    REQUIRE(tree.get(f)->rectW  == Catch::Approx(200.0f));
    REQUIRE(tree.get(f)->rectH  == Catch::Approx(100.0f));
}

TEST_CASE("Hit rect insets bring the clickable area in", "[widget][input]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Panel");
    tree.setWidth(f, 200.0f);
    tree.setHeight(f, 100.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    tree.addPoint(f, a);
    tree.get(f)->mouseEnabled = true;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(190.0f, 90.0f) == f);   // inside, before insets

    // PaperDollFrame's shape: nothing off the left or top, a strip off the
    // right and a deeper one off the bottom.
    tree.get(f)->hitInsetRight  = 30.0f;
    tree.get(f)->hitInsetBottom = 45.0f;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(190.0f, 90.0f) != f);   // now in the inset strip
    REQUIRE(tree.hitTest(150.0f, 90.0f) == f);   // still inside
    REQUIRE(tree.hitTest(150.0f, 20.0f) != f);   // below the bottom inset
    REQUIRE(tree.hitTest(5.0f,  90.0f) == f);    // left edge untouched
}

TEST_CASE("A negative inset reaches outside the frame", "[widget][input]") {
    // WoW's sense: negative expands. Small buttons use it to be clickable
    // beyond their art.
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Tiny");
    tree.setWidth(f, 20.0f);
    tree.setHeight(f, 20.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    a.x = 100.0f; a.y = 100.0f;
    tree.addPoint(f, a);
    tree.get(f)->mouseEnabled = true;
    tree.get(f)->hitInsetLeft = -10.0f;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(95.0f, 110.0f) == f);   // outside the rect, inside the hit area
}

TEST_CASE("A frame inset to nothing takes no clicks", "[widget][input]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, tree.root(), "Closed");
    tree.setWidth(f, 40.0f);
    tree.setHeight(f, 40.0f);
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    tree.addPoint(f, a);
    tree.get(f)->mouseEnabled = true;
    tree.get(f)->hitInsetLeft = 25.0f;
    tree.get(f)->hitInsetRight = 25.0f;   // overlapping insets
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.hitTest(20.0f, 20.0f) != f);
}

TEST_CASE("Raising a window carries its regions with it", "[widget][layout]") {
    // The shape the character sheet has: a window, a frame inside it, and a
    // label inside that. Raising the window must lift all three, or the label
    // sorts under the window's own art and vanishes.
    WidgetTree tree;
    const uint32_t win = tree.create(WidgetKind::Frame, tree.root(), "Window");
    const uint32_t inner = tree.create(WidgetKind::Frame, win, "Inner");
    const uint32_t label = tree.create(WidgetKind::FontString, inner, "Label");
    Anchor p; p.point = "CENTER"; p.relativePoint = "CENTER";
    tree.addPoint(win, p);
    tree.addPoint(inner, p);
    tree.addPoint(label, p);
    const uint32_t other = tree.create(WidgetKind::Frame, tree.root(), "Other");
    tree.addPoint(other, p);
    tree.layout(kScreenW, kScreenH);

    tree.raise(win);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(win)->effLevel > tree.get(other)->effLevel);
    REQUIRE(tree.get(inner)->effLevel > tree.get(win)->effLevel);
    REQUIRE(tree.get(label)->effLevel > tree.get(inner)->effLevel);
}

TEST_CASE("A child with its own level still follows when the parent is raised",
          "[widget][layout]") {
    // SetFrameLevel makes a child's level explicit, and FrameXML calls it
    // freely - RaiseFrameLevelByTwo alone is used all over. In WoW a child's
    // level is relative to its parent, so raising the parent must carry it.
    WidgetTree tree;
    const uint32_t win = tree.create(WidgetKind::Frame, tree.root(), "Window");
    const uint32_t inner = tree.create(WidgetKind::Frame, win, "Inner");
    Anchor p; p.point = "CENTER"; p.relativePoint = "CENTER";
    tree.addPoint(win, p);
    tree.addPoint(inner, p);
    tree.layout(kScreenW, kScreenH);
    // What RaiseFrameLevelByTwo does.
    tree.get(inner)->level = tree.get(inner)->effLevel + 2;
    tree.get(inner)->levelExplicit = true;
    tree.layout(kScreenW, kScreenH);
    const int before = tree.get(inner)->effLevel - tree.get(win)->effLevel;

    tree.raise(win);
    tree.layout(kScreenW, kScreenH);
    const int after = tree.get(inner)->effLevel - tree.get(win)->effLevel;
    REQUIRE(after == before);           // the gap is kept
    REQUIRE(tree.get(inner)->effLevel > tree.get(win)->effLevel);
}

TEST_CASE("An edge and a centre do not resize a frame", "[widget][layout]") {
    // The reputation rows are the case this is about, and they use both rules
    // at once. The XML hangs each row's TOPRIGHT under the row above; then
    // ReputationFrame_SetRowType adds a LEFT anchor to indent it.
    //
    // On x that is a 0 and a 1 - opposite edges - so the row is meant to
    // stretch from its indent to the frame's right edge. On y it is a top edge
    // and a *centre*, which is not a pair WoW sizes from: the row keeps the
    // height its template gave it.
    //
    // Sizing from that pair instead gave twice the gap between the top and the
    // frame's middle, so every row was stretched to most of the frame's height
    // and drawn over the one before it.
    WidgetTree tree;
    const uint32_t frame = tree.create(WidgetKind::Frame, tree.root(), "RepFrame");
    tree.setWidth(frame, 300.0f);
    tree.setHeight(frame, 340.0f);
    Anchor fa; fa.point = "BOTTOMLEFT"; fa.relativePoint = "BOTTOMLEFT";
    tree.addPoint(frame, fa);

    const uint32_t row = tree.create(WidgetKind::Frame, frame, "Row1");
    tree.setWidth(row, 240.0f);
    tree.setHeight(row, 21.0f);
    Anchor top; top.point = "TOPRIGHT"; top.relativeTo = frame;
    top.relativePoint = "TOPRIGHT"; top.x = -8.0f; top.y = -83.0f;
    tree.addPoint(row, top);
    Anchor left; left.point = "LEFT"; left.relativeTo = frame;
    left.relativePoint = "LEFT"; left.x = 44.0f;
    tree.addPoint(row, left);

    tree.layout(kScreenW, kScreenH);
    const Widget* w = tree.get(row);

    // Height untouched by the centre anchor.
    REQUIRE(w->rectH == Catch::Approx(21.0f));
    // Top edge still where the XML anchor put it.
    REQUIRE(w->bottom + w->rectH ==
            Catch::Approx(tree.get(frame)->bottom + 340.0f - 83.0f));
    // Width spans indent to right edge, which is what the two x anchors mean.
    REQUIRE(w->rectW == Catch::Approx(300.0f - 44.0f - 8.0f));
}

TEST_CASE("SetParent moves the frame, not just the answer to GetParent",
          "[widget_tree]") {
    // SetParent used to write a field on the Lua table and nothing else, so a
    // reparented frame kept being laid out, clipped and shown or hidden by the
    // parent it had left. QuestInfo reparents every element of a quest into
    // either the quest giver's panel or the quest log's scroll child each time
    // it displays one, so the quest text was being anchored into a window it
    // did not belong to.
    WidgetTree tree;
    const uint32_t giver = tree.create(WidgetKind::Frame, tree.root(), "Giver");
    const uint32_t log   = tree.create(WidgetKind::Frame, tree.root(), "Log");
    const uint32_t text  = tree.create(WidgetKind::Frame, giver, "QuestText");

    REQUIRE(tree.get(text)->parent == giver);
    REQUIRE(tree.get(giver)->children.size() == 1u);

    tree.setParent(text, log);
    CHECK(tree.get(text)->parent == log);
    // Exactly one parent holds it, or layout draws it twice or not at all.
    CHECK(tree.get(giver)->children.empty());
    CHECK(tree.get(log)->children.size() == 1u);
}

TEST_CASE("a reparented frame inherits the new parent's visibility",
          "[widget_tree]") {
    // The half that made this worth more than tidiness. Visibility is resolved
    // down the parent chain at layout, so a frame moved into a hidden window
    // must go with it - and one moved out of a hidden window must come back.
    WidgetTree tree;
    const uint32_t shown  = tree.create(WidgetKind::Frame, tree.root(), "Shown");
    const uint32_t hidden = tree.create(WidgetKind::Frame, tree.root(), "Hidden");
    for (uint32_t f : {shown, hidden}) {
        Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
        tree.addPoint(f, a);
        tree.setWidth(f, 100.0f);
        tree.setHeight(f, 100.0f);
    }
    tree.get(hidden)->shown = false;

    const uint32_t child = tree.create(WidgetKind::Frame, shown, "Child");
    Anchor ca; ca.point = "BOTTOMLEFT"; ca.relativeTo = shown;
    ca.relativePoint = "BOTTOMLEFT";
    tree.addPoint(child, ca);
    tree.setWidth(child, 10.0f);
    tree.setHeight(child, 10.0f);

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(child)->visible);

    tree.setParent(child, hidden);
    tree.layout(kScreenW, kScreenH);
    CHECK_FALSE(tree.get(child)->visible);

    tree.setParent(child, shown);
    tree.layout(kScreenW, kScreenH);
    CHECK(tree.get(child)->visible);
}

TEST_CASE("a frame cannot be put inside itself", "[widget_tree]") {
    // Layout walks children, so a cycle never returns. Refused rather than
    // clamped: there is no sensible parent to fall back to.
    WidgetTree tree;
    const uint32_t outer = tree.create(WidgetKind::Frame, tree.root(), "Outer");
    const uint32_t inner = tree.create(WidgetKind::Frame, outer, "Inner");

    tree.setParent(outer, inner);
    CHECK(tree.get(outer)->parent == tree.root());
    tree.setParent(outer, outer);
    CHECK(tree.get(outer)->parent == tree.root());
}

TEST_CASE("a frame that asks for a size and resolves to none is distinguishable",
          "[widget_tree]") {
    // The signature the takeover check looks for, pinned here because the
    // check itself only runs in a live client. A zero anywhere up the scale
    // chain leaves a frame laid out, shown, and occupying nothing - which is
    // what a CVar read answering "0" for uiScale did to every dropdown in the
    // interface: built, drawn, invisible.
    //
    // The distinction that keeps the check quiet is between a frame that never
    // asked for a size and one whose size did not survive. Only the second is
    // a fault.
    WidgetTree tree;
    const uint32_t sized = tree.create(WidgetKind::Frame, tree.root(), "Sized");
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    tree.addPoint(sized, a);
    tree.setWidth(sized, 200.0f);
    tree.setHeight(sized, 100.0f);

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(sized)->rectW > 0.0f);
    REQUIRE(tree.get(sized)->rectH > 0.0f);

    // Scaled to nothing: still shown, still asking for 200x100, resolving to
    // nothing at all.
    tree.get(sized)->scale = 0.0f;
    tree.layout(kScreenW, kScreenH);
    const Widget* w = tree.get(sized);
    CHECK(w->visible);
    CHECK(w->width == Catch::Approx(200.0f));
    CHECK(w->rectW == Catch::Approx(0.0f));

    // A frame that never asked for a size must not look the same, or the
    // check reports a page of them every run.
    const uint32_t bare = tree.create(WidgetKind::Frame, tree.root(), "Bare");
    Anchor b; b.point = "BOTTOMLEFT"; b.relativePoint = "BOTTOMLEFT";
    tree.addPoint(bare, b);
    tree.layout(kScreenW, kScreenH);
    CHECK(tree.get(bare)->width == Catch::Approx(0.0f));
    CHECK(tree.get(bare)->height == Catch::Approx(0.0f));
}

TEST_CASE("an unanchored frame is not drawn and is still running",
          "[widget_tree]") {
    // Two questions that look like one. WoW does not draw a frame with no
    // anchors - that is why a stray panel does not land in the middle of the
    // screen - but such a frame is still shown, and its OnUpdate still runs.
    // Eight of FrameXML's drivers are exactly that shape: a CreateFrame that
    // is never positioned and carries nothing but an OnUpdate.
    // frameFadeManager drives every fade in the interface, frameFlashManager
    // every flash, AnimUpdateFrame the animation system.
    //
    // Asking `visible` - which means "would be drawn" - stops all of them.
    WidgetTree tree;
    const uint32_t driver = tree.create(WidgetKind::Frame, tree.root(), "Driver");
    tree.layout(kScreenW, kScreenH);

    const Widget* w = tree.get(driver);
    CHECK(w->visibleChain);        // shown, and the root is shown
    CHECK_FALSE(w->visible);       // but nowhere to be drawn
}

TEST_CASE("a hidden ancestor stops the chain as well as the drawing",
          "[widget_tree]") {
    // The half that must keep working: hiding a window has to stop what is
    // inside it, driver frame or not.
    WidgetTree tree;
    const uint32_t window = tree.create(WidgetKind::Frame, tree.root(), "Window");
    Anchor a; a.point = "BOTTOMLEFT"; a.relativePoint = "BOTTOMLEFT";
    tree.addPoint(window, a);
    tree.setWidth(window, 100.0f);
    tree.setHeight(window, 100.0f);
    const uint32_t driver = tree.create(WidgetKind::Frame, window, "InnerDriver");

    tree.layout(kScreenW, kScreenH);
    REQUIRE(tree.get(driver)->visibleChain);

    tree.get(window)->shown = false;
    tree.layout(kScreenW, kScreenH);
    CHECK_FALSE(tree.get(driver)->visibleChain);
    CHECK_FALSE(tree.get(driver)->visible);
}

// ── Colour picker arithmetic ────────────────────────────────────────────
//
// Pure conversion, and the one part of the picker that can be checked without
// a device: the wheel, the bar and the drag all read hue-saturation-value and
// every one of them has to agree on what that means.

TEST_CASE("hsv and rgb round-trip through each other", "[widget_tree]") {
    // The six corners of the cube plus a middling colour, because the sector
    // arithmetic branches six ways and each boundary is where an off-by-one
    // hides. Blizzard's own defaults live at these corners: chat channel
    // colours are largely saturated primaries.
    const float cases[][3] = {
        {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f},
        {0.25f, 0.5f, 0.75f}, {0.6f, 0.6f, 0.6f},
    };
    for (const auto& rgb : cases) {
        float hsv[3], back[3];
        rgbToHsv(rgb, hsv);
        hsvToRgb(hsv, back);
        CHECK(back[0] == Catch::Approx(rgb[0]).margin(0.002));
        CHECK(back[1] == Catch::Approx(rgb[1]).margin(0.002));
        CHECK(back[2] == Catch::Approx(rgb[2]).margin(0.002));
    }
}

TEST_CASE("grey reports no saturation and black no value", "[widget_tree]") {
    // Why the picker keeps its own hue rather than reading it back: these two
    // have no hue to report, and taking the zero would swing the wheel thumb
    // to red every time the value bar reached the bottom.
    float hsv[3];
    const float grey[3] = {0.5f, 0.5f, 0.5f};
    rgbToHsv(grey, hsv);
    CHECK(hsv[1] == Catch::Approx(0.0f));
    CHECK(hsv[2] == Catch::Approx(0.5f));

    const float black[3] = {0.0f, 0.0f, 0.0f};
    rgbToHsv(black, hsv);
    CHECK(hsv[2] == Catch::Approx(0.0f));
}

TEST_CASE("hue wraps rather than running off either end", "[widget_tree]") {
    // The drag computes hue from atan2, which answers in (-pi, pi] - half of
    // the wheel is a negative turn. A hue outside [0,1) has to mean the same
    // colour as the one inside it, or dragging through the six-o'clock
    // position would jump the colour.
    float below[3], above[3];
    const float hsvBelow[3] = {-0.25f, 1.0f, 1.0f};
    const float hsvAbove[3] = {0.75f, 1.0f, 1.0f};
    hsvToRgb(hsvBelow, below);
    hsvToRgb(hsvAbove, above);
    CHECK(below[0] == Catch::Approx(above[0]).margin(0.002));
    CHECK(below[1] == Catch::Approx(above[1]).margin(0.002));
    CHECK(below[2] == Catch::Approx(above[2]).margin(0.002));
}

TEST_CASE("a texture the client renders into is drawn without a file",
          "[widget_tree]") {
    // The player's portrait is a Texture declared with no file, because the
    // picture is a character rendered offscreen and handed over as a live
    // handle. Both halves of the draw path have to agree that "no file" is not
    // "nothing to draw" - the renderer's own copy of this test was the missing
    // half, and the portrait was discarded every frame for four days.
    WidgetTree tree;
    const uint32_t live = tree.create(WidgetKind::Texture, tree.root(), "PlayerPortrait");
    Anchor a; a.point = "CENTER"; a.relativePoint = "CENTER";
    tree.addPoint(live, a);
    tree.setWidth(live, 60.0f);
    tree.setHeight(live, 60.0f);

    tree.layout(kScreenW, kScreenH);
    bool drawn = false;
    for (const Widget* w : tree.drawOrder()) drawn |= (w->id == live);
    CHECK_FALSE(drawn);            // no file and no handle: nothing to draw

    tree.get(live)->externalTexture = 0xABCD;
    tree.layout(kScreenW, kScreenH);
    drawn = false;
    for (const Widget* w : tree.drawOrder()) drawn |= (w->id == live);
    CHECK(drawn);
}

TEST_CASE("a portrait belongs to one unit at a time", "[widget_tree]") {
    // TargetFrame's portrait is asked for "target" and for "player" in turn,
    // as the player targets themselves and then something else. A texture left
    // claimed by both is handed two faces a frame and keeps whichever was
    // written last, which is how a portrait shows the wrong unit.
    WidgetTree tree;
    const uint32_t tex = tree.create(WidgetKind::Texture, tree.root(), "TargetFramePortrait");

    tree.setPortraitUnit(tex, "target");
    CHECK(tree.portraitsFor("target").size() == 1);
    CHECK(tree.portraitsFor("player").empty());

    // Claiming it for someone else releases it from the first, and clears the
    // handle with it: dropping the claim alone would leave the last face on
    // screen until something overwrote it.
    tree.get(tex)->externalTexture = 0x1234;
    tree.setPortraitUnit(tex, "player");
    CHECK(tree.portraitsFor("target").empty());
    CHECK(tree.portraitsFor("player").size() == 1);
    CHECK(tree.get(tex)->externalTexture == 0);

    // An empty unit releases it outright.
    tree.get(tex)->externalTexture = 0x5678;
    tree.setPortraitUnit(tex, "");
    CHECK(tree.portraitsFor("player").empty());
    CHECK(tree.get(tex)->externalTexture == 0);
}

TEST_CASE("claiming the same portrait twice claims it once", "[widget_tree]") {
    // And does not clear the handle on the way, which a release-then-claim
    // would: the draw loop writes the handle once a frame and a claim that
    // zeroed it in between would flicker.
    WidgetTree tree;
    const uint32_t tex = tree.create(WidgetKind::Texture, tree.root(), "P");
    tree.setPortraitUnit(tex, "pet");
    tree.get(tex)->externalTexture = 0x99;
    tree.setPortraitUnit(tex, "pet");
    CHECK(tree.portraitsFor("pet").size() == 1);
    CHECK(tree.get(tex)->externalTexture == 0x99);
}

TEST_CASE("a model frame carries what it has been asked to try on",
          "[widget_tree]") {
    // The dressing room's contents belong to the frame, not to the
    // application: a second one would have its own, and closing this one is
    // what empties it.
    WidgetTree tree;
    const uint32_t model = tree.create(WidgetKind::Frame, tree.root(), "DressUpModel");
    CHECK(tree.get(model)->tryOnItems.empty());

    tree.get(model)->tryOnItems.push_back({1234u, 5u});
    tree.get(model)->tryOnItems.push_back({5678u, 1u});
    CHECK(tree.get(model)->tryOnItems.size() == 2);

    tree.get(model)->tryOnItems.clear();
    CHECK(tree.get(model)->tryOnItems.empty());
}

TEST_CASE("The screen sits above UIParent so a frame can leave it",
          "[widget][layout]") {
    // Hiding UIParent is how FrameXML clears the interface out of the way of a
    // fullscreen panel: uiparent.lua's fullscreen path is UIParent:Hide()
    // followed by frame:Show(). A frame that cannot get out from under
    // UIParent goes down with everything the call was meant to make room for,
    // which is what left the world map hiding the interface and then itself.
    WidgetTree tree;
    REQUIRE(tree.root() != tree.uiParentId());
    REQUIRE(tree.get(tree.uiParentId())->parent == tree.root());

    // A frame with no parent named hangs off UIParent, as almost everything
    // does...
    const uint32_t child = tree.create(WidgetKind::Frame, tree.uiParentId(), "Child");
    tree.setAllPoints(child, tree.uiParentId());
    // ...and SetParent(nil) takes it out to the screen, not back to UIParent.
    // Anchored like the world map is, which is what makes it drawable at all:
    // a frame with no anchor points is not displayed, whoever its parent is.
    const uint32_t loose = tree.create(WidgetKind::Frame, tree.uiParentId(), "Loose");
    tree.setParent(loose, 0);
    tree.setAllPoints(loose, tree.root());
    REQUIRE(tree.get(loose)->parent == tree.root());
    REQUIRE(tree.get(child)->parent == tree.uiParentId());

    // So hiding UIParent takes one down and leaves the other standing.
    tree.get(tree.uiParentId())->shown = false;
    tree.layout(kScreenW, kScreenH);
    CHECK(tree.get(child)->visible == false);
    CHECK(tree.get(loose)->visible == true);
}

TEST_CASE("A rect that is not a number is never hit", "[widget][hittest]") {
    // Every comparison against a nan is false, so a naive range test lets one
    // through at any coordinate: `x < left` and `x > right` are both false and
    // the frame reads as hit wherever the cursor is. One mouse-enabled frame
    // answering every hit test tells the rest of the client the interface owns
    // the mouse, and the camera stops turning anywhere on screen - which is
    // what a chat window whose saved position had gone to nan did.
    WidgetTree tree;
    const uint32_t good = tree.create(WidgetKind::Frame, tree.uiParentId(), "Good");
    Widget* g = tree.get(good);
    g->mouseEnabled = true;
    g->visible = true;
    g->left = 10.0f; g->bottom = 10.0f; g->rectW = 20.0f; g->rectH = 20.0f;
    REQUIRE(tree.hitTest(15.0f, 15.0f) == good);
    REQUIRE(tree.hitTest(500.0f, 500.0f) == 0);

    const uint32_t bad = tree.create(WidgetKind::Frame, tree.uiParentId(), "Bad");
    Widget* b = tree.get(bad);
    b->mouseEnabled = true;
    b->visible = true;
    b->left = std::numeric_limits<float>::quiet_NaN();
    b->bottom = 0.0f; b->rectW = 100.0f; b->rectH = 100.0f;

    // Nowhere near it, and it must still lose.
    CHECK(tree.hitTest(500.0f, 500.0f) == 0);
    // And it must not take a hit away from a frame that really is there.
    CHECK(tree.hitTest(15.0f, 15.0f) == good);
}

TEST_CASE("An edge anchor positions the frame, not the centre beside it",
          "[widget][layout]") {
    // The talent frame's points bar, which is a common shape: LEFT, RIGHT and
    // BOTTOM together.
    //
    // An anchor constrains both axes whether it meant to or not. LEFT is the
    // left edge and the vertical centre; RIGHT is the right edge and the same
    // centre. So on y this frame has three constraints - two centres that came
    // along with the horizontal pair, and the one that is actually about y.
    //
    // Positioning from the first anchor put the bar at its parent's vertical
    // centre and ignored the bottom edge it was given. The scroll frame above
    // it is anchored to its top, so the talent tree got half the height it
    // should have, was cut off, and would not scroll - it had been made
    // shorter than the content it was showing rather than taller.
    WidgetTree tree;
    const uint32_t frame = tree.create(WidgetKind::Frame, tree.root(), "Panel");
    tree.setWidth(frame, 384.0f);
    tree.setHeight(frame, 512.0f);
    tree.addPoint(frame, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});

    const uint32_t bar = tree.create(WidgetKind::Frame, frame, "PointsBar");
    tree.setWidth(bar, 331.0f);
    tree.setHeight(bar, 26.0f);
    tree.addPoint(bar, Anchor{"LEFT", frame, "LEFT", 16.0f, 0.0f});
    tree.addPoint(bar, Anchor{"RIGHT", frame, "RIGHT", -36.0f, 0.0f});
    tree.addPoint(bar, Anchor{"BOTTOM", frame, "BOTTOM", 0.0f, 81.0f});

    tree.layout(kScreenW, kScreenH);
    const Widget* w = tree.get(bar);
    const Widget* p = tree.get(frame);

    // The bottom edge it was given, not the centre it was not.
    REQUIRE(w->bottom == Catch::Approx(p->bottom + 81.0f));
    REQUIRE(w->bottom != Catch::Approx(p->bottom + (512.0f - 26.0f) * 0.5f));
    // Height untouched: a centre and an edge still do not resize.
    REQUIRE(w->rectH == Catch::Approx(26.0f));
    // Width from the opposite pair, which is what LEFT and RIGHT do mean.
    REQUIRE(w->rectW == Catch::Approx(384.0f - 16.0f - 36.0f));
}

TEST_CASE("The wheel finds a frame that took the wheel and not the mouse",
          "[widget][scroll][hittest]") {
    // EnableMouseWheel and EnableMouse are separate in WoW, and
    // UIPanelScrollFrameTemplate asks for only the first: it declares
    // OnMouseWheel and never enables the mouse. Hit-testing the wheel with the
    // test written for the mouse therefore found no scroll frame in the whole
    // interface, and the wheel fell through to the camera - nothing with a
    // scroll bar scrolled, the talent tree included.
    WidgetTree tree;
    const uint32_t scroll = tree.create(WidgetKind::Frame, tree.root(), "S");
    tree.get(scroll)->isScrollFrame = true;
    tree.get(scroll)->wheelEnabled = true;
    tree.get(scroll)->mouseEnabled = false;   // as the template leaves it
    tree.get(scroll)->width = 100.0f;
    tree.get(scroll)->height = 100.0f;
    tree.addPoint(scroll, Anchor{"BOTTOMLEFT", 0, "BOTTOMLEFT", 0.0f, 0.0f});

    tree.layout(kScreenW, kScreenH);

    // Invisible to the mouse, which is correct and is why it must not be the
    // test the wheel uses.
    REQUIRE(tree.hitTest(50.0f, 50.0f) != scroll);
    REQUIRE(tree.hitTestWheel(50.0f, 50.0f) == scroll);
}

// A measurement taken between two moves.
//
// Rects used to be answered only from the once-a-frame pass, so a frame
// anchored inside a handler measured as though it had never been placed - its
// own size sitting at the origin - until the next frame came round. The
// interface is written against a client that answers whenever it is asked: the
// quest tracker anchors each objective line and immediately reads the edge it
// landed on to know how tall the block grew. Every read came back zero, the
// tracker concluded it had nothing to show, and collapsed itself.
//
// Nothing about this is visible in a single pass, which is why it wants
// pinning: lay out, and both the old behaviour and the new one agree.
TEST_CASE("A rect is resolved when it is asked for, not when the frame ends",
          "[widget][anchor][layout]") {
    WidgetTree tree;
    tree.layout(kScreenW, kScreenH);

    const uint32_t a = tree.create(WidgetKind::Frame, tree.uiParentId(), "A");
    tree.setWidth(a, 100.0f);
    tree.setHeight(a, 40.0f);
    Anchor top;
    top.point = "TOPLEFT";
    top.relativeTo = tree.uiParentId();
    top.relativePoint = "TOPLEFT";
    top.x = 10.0f;
    top.y = -20.0f;
    tree.addPoint(a, top);

    // Asked for without a pass in between, exactly as a handler would.
    tree.resolveWidget(a);
    const Widget* wa = tree.get(a);
    REQUIRE(wa != nullptr);
    CHECK(wa->bottom + wa->rectH == Catch::Approx(kScreenH - 20.0f));
    CHECK(wa->left == Catch::Approx(10.0f));

    // And a second frame hung off the first, which is the shape the tracker
    // builds: each line is anchored to the one above it and then measured.
    const uint32_t b = tree.create(WidgetKind::Frame, tree.uiParentId(), "B");
    tree.setWidth(b, 100.0f);
    tree.setHeight(b, 40.0f);
    Anchor under;
    under.point = "TOP";
    under.relativeTo = a;
    under.relativePoint = "BOTTOM";
    under.y = -5.0f;
    tree.addPoint(b, under);

    tree.resolveWidget(b);
    const Widget* wb = tree.get(b);
    REQUIRE(wb != nullptr);
    // A's bottom is 748 - 40 = 708, and B hangs 5 below that.
    CHECK(wb->bottom + wb->rectH == Catch::Approx(kScreenH - 20.0f - 40.0f - 5.0f));

    // The screen and UIParent are placed by the full pass and have no anchors
    // of their own, so a chain that walks onto them must stop rather than run
    // the anchor solver over them - that gave the screen a rect derived from
    // nothing and put everything measured against it in the wrong place.
    const Widget* ui = tree.get(tree.uiParentId());
    REQUIRE(ui != nullptr);
    CHECK(ui->rectH == Catch::Approx(kScreenH));
    CHECK(ui->bottom == Catch::Approx(0.0f));
}

TEST_CASE("Two frames anchored to each other do not resolve for ever",
          "[widget][anchor][layout]") {
    WidgetTree tree;
    tree.layout(kScreenW, kScreenH);
    const uint32_t a = tree.create(WidgetKind::Frame, tree.uiParentId(), "CycleA");
    const uint32_t b = tree.create(WidgetKind::Frame, tree.uiParentId(), "CycleB");
    tree.setWidth(a, 10.0f); tree.setHeight(a, 10.0f);
    tree.setWidth(b, 10.0f); tree.setHeight(b, 10.0f);
    Anchor toB; toB.point = "TOP"; toB.relativeTo = b; toB.relativePoint = "BOTTOM";
    Anchor toA; toA.point = "TOP"; toA.relativeTo = a; toA.relativePoint = "BOTTOM";
    tree.addPoint(a, toB);
    tree.addPoint(b, toA);
    // The rects are whatever they are; what is pinned is that asking returns.
    tree.resolveWidget(a);
    SUCCEED("resolving a cycle terminated");
}

// The other kind of size, measured the moment it is created.
//
// Width can be stated or it can fall out of two opposing anchors, and only the
// second goes through the solver. The achievement rows use it - each gates its
// own layout on `objectives:GetHeight() > 0` and then anchors LEFT and RIGHT to
// its neighbours - so a size that resolves for stated widths and not for solved
// ones would leave that whole panel collapsed while the simpler cases worked.
TEST_CASE("A size that comes out of the solver resolves when asked for too",
          "[widget][anchor][layout]") {
    WidgetTree tree;
    tree.layout(kScreenW, kScreenH);

    const uint32_t parent = tree.create(WidgetKind::Frame, tree.uiParentId(), "P");
    tree.setWidth(parent, 300.0f);
    tree.setHeight(parent, 200.0f);
    Anchor centre;
    centre.point = "CENTER";
    centre.relativeTo = tree.uiParentId();
    centre.relativePoint = "CENTER";
    tree.addPoint(parent, centre);

    const uint32_t child = tree.create(WidgetKind::Frame, parent, "C");
    tree.setHeight(child, 50.0f);
    Anchor l; l.point = "LEFT";  l.relativeTo = parent; l.relativePoint = "LEFT";  l.x =  20.0f;
    Anchor r; r.point = "RIGHT"; r.relativeTo = parent; r.relativePoint = "RIGHT"; r.x = -30.0f;
    tree.addPoint(child, l);
    tree.addPoint(child, r);

    tree.resolveWidget(child);
    const Widget* wc = tree.get(child);
    REQUIRE(wc != nullptr);
    CHECK(wc->rectW == Catch::Approx(300.0f - 20.0f - 30.0f));
    CHECK(wc->rectH == Catch::Approx(50.0f));
}

// Where a status bar's fill draws, relative to the bar's own art.
//
// In the real client the fill is a region of the bar like any other, and
// <StatusBar drawLayer="..."> is that region's layer. Here the bar draws its
// own fill, so the bar has to sort where the fill belongs rather than where a
// frame would - otherwise the fill goes under everything the bar owns whatever
// it asked for, and a bar with a dark backing of its own wears it over the
// fill.
//
// CastingBarFrameTemplate is exactly that shape and is what this reproduces: a
// BACKGROUND backing, a BORDER fill, and ARTWORK border art. Twenty bars in
// the interface declare a layer this way and every one of them was ignored,
// because the declaration is emitted as a call on the bar and the bar is a
// frame.
TEST_CASE("A bar's fill draws in the layer it asked for", "[widget][statusbar][layer]") {
    WidgetTree tree;

    const uint32_t bar = tree.create(WidgetKind::Frame, tree.uiParentId(), "Bar");
    {
        Widget* w = tree.get(bar);
        REQUIRE(w != nullptr);
        w->isStatusBar = true;
        w->barTexture = "Interface\\TargetingFrame\\UI-StatusBar";
        w->barLayer = DrawLayer::Border;
        w->width = 100.0f;
        w->height = 20.0f;
    }
    Anchor at;
    at.point = "TOPLEFT";
    at.relativeTo = tree.uiParentId();
    at.relativePoint = "TOPLEFT";
    tree.addPoint(bar, at);

    // The bar's own regions, in the two layers that bracket the fill.
    auto region = [&](const char* name, DrawLayer layer) {
        const uint32_t id = tree.create(WidgetKind::Texture, bar, name);
        Widget* w = tree.get(id);
        w->layer = layer;
        w->texturePath = "Interface\\Test";
        Anchor a;
        a.point = "TOPLEFT";
        a.relativeTo = bar;
        a.relativePoint = "TOPLEFT";
        tree.addPoint(id, a);
        w->width = 100.0f;
        w->height = 20.0f;
        return id;
    };
    const uint32_t backing = region("Backing", DrawLayer::Background);
    const uint32_t border  = region("BorderArt", DrawLayer::Artwork);

    tree.layout(kScreenW, kScreenH);

    auto indexOf = [&](uint32_t id) {
        const auto& order = tree.drawOrder();
        for (size_t i = 0; i < order.size(); ++i) {
            if (order[i]->id == id) return static_cast<int>(i);
        }
        return -1;
    };
    const int iBacking = indexOf(backing);
    const int iFill    = indexOf(bar);
    const int iBorder  = indexOf(border);

    INFO("backing " << iBacking << ", fill " << iFill << ", border art " << iBorder);
    REQUIRE(iBacking >= 0);
    REQUIRE(iFill >= 0);
    REQUIRE(iBorder >= 0);
    CHECK(iBacking < iFill);    // the backing stays behind the fill
    CHECK(iFill < iBorder);     // and the frame's art stays in front of it
}

TEST_CASE("A bar with no fill still sorts as the frame it is",
          "[widget][statusbar][layer]") {
    // The bump above is for the fill. A bar that has no fill texture has
    // nothing to place, and moving it would shuffle a frame for no reason.
    WidgetTree tree;
    const uint32_t bar = tree.create(WidgetKind::Frame, tree.uiParentId(), "EmptyBar");
    {
        Widget* w = tree.get(bar);
        w->isStatusBar = true;
        w->hasBackdrop = true;          // so it is drawn at all
        w->barLayer = DrawLayer::Overlay;
        w->width = 100.0f;
        w->height = 20.0f;
    }
    Anchor at;
    at.point = "TOPLEFT";
    at.relativeTo = tree.uiParentId();
    at.relativePoint = "TOPLEFT";
    tree.addPoint(bar, at);

    const uint32_t sibling = tree.create(WidgetKind::Texture, bar, "Above");
    {
        Widget* w = tree.get(sibling);
        w->layer = DrawLayer::Background;
        w->texturePath = "Interface\\Test";
        w->width = 10.0f;
        w->height = 10.0f;
    }
    Anchor a;
    a.point = "TOPLEFT";
    a.relativeTo = bar;
    a.relativePoint = "TOPLEFT";
    tree.addPoint(sibling, a);

    tree.layout(kScreenW, kScreenH);
    auto indexOf = [&](uint32_t id) {
        const auto& order = tree.drawOrder();
        for (size_t i = 0; i < order.size(); ++i) {
            if (order[i]->id == id) return static_cast<int>(i);
        }
        return -1;
    };
    // No fill, so the bar keeps its own level and its child draws over it,
    // however the bar's unused fill layer is set.
    CHECK(indexOf(bar) < indexOf(sibling));
}

TEST_CASE("Raise and Lower are idempotent", "[widget][level]") {
    // ShowUIPanel raises on every panel open, so a Raise that adds one whether
    // or not anything is above it is a leak measured in panel opens. A quest
    // frame that starts at level 3 was found at 344 after a play session.
    WidgetTree tree;
    const uint32_t root = tree.create(WidgetKind::Frame, 0, "Root");
    const uint32_t a = tree.create(WidgetKind::Frame, root, "A");
    const uint32_t b = tree.create(WidgetKind::Frame, root, "B");
    tree.layout(1920.0f, 1080.0f);

    tree.raise(a);
    tree.layout(1920.0f, 1080.0f);
    const int afterFirst = tree.get(a)->effLevel;

    // Nine more, with nothing else moving. A frame already on top is already
    // where Raise is asking it to be.
    for (int i = 0; i < 9; ++i) {
        tree.raise(a);
        tree.layout(1920.0f, 1080.0f);
    }
    CHECK(tree.get(a)->effLevel == afterFirst);

    // And it still does its job: B raised above A goes higher, once.
    tree.raise(b);
    tree.layout(1920.0f, 1080.0f);
    CHECK(tree.get(b)->effLevel > tree.get(a)->effLevel);
    const int bLevel = tree.get(b)->effLevel;
    tree.raise(b);
    tree.layout(1920.0f, 1080.0f);
    CHECK(tree.get(b)->effLevel == bLevel);

    // Lower is the same shape in the other direction, and never goes below
    // zero - a negative level sorts under the root and stops being drawn.
    for (int i = 0; i < 12; ++i) {
        tree.lower(a);
        tree.layout(1920.0f, 1080.0f);
    }
    CHECK(tree.get(a)->effLevel >= 0);
}

TEST_CASE("A scroll frame clips its scroll child, not its scroll bar",
          "[widget][layout][scroll]") {
    // A scroll frame shows a window onto a taller child, and everything under
    // that child is bounded by the window. Its *own* children are not all that
    // child: the scroll bar is one too, and it sits alongside the window
    // rather than inside it.
    //
    // Clipping every child put each bar entirely outside its own clip rect,
    // which does not trim it - it deletes it. Bar, track, thumb and both
    // buttons were laid out, drawn and cut away to nothing, and the same rect
    // is what the hit test consults, so they could not be clicked either.
    WidgetTree tree;
    const uint32_t panel = tree.create(WidgetKind::Frame, 0, "Panel");
    tree.get(panel)->width = 400.0f;
    tree.get(panel)->height = 400.0f;
    tree.addPoint(panel, Anchor{});

    const uint32_t scroll = tree.create(WidgetKind::Frame, panel, "Scroll");
    tree.markScrollFrame(scroll);
    tree.get(scroll)->width = 300.0f;
    tree.get(scroll)->height = 300.0f;
    tree.addPoint(scroll, Anchor{});

    const uint32_t child = tree.create(WidgetKind::Frame, scroll, "ScrollChild");
    tree.get(scroll)->scrollChild = child;
    tree.get(child)->width = 300.0f;
    tree.get(child)->height = 900.0f;   // taller than the window, as it should be
    tree.addPoint(child, Anchor{});

    const uint32_t bar = tree.create(WidgetKind::Frame, scroll, "ScrollBar");
    tree.get(bar)->width = 16.0f;
    tree.get(bar)->height = 300.0f;
    tree.addPoint(bar, Anchor{});

    // Something inside the scroll child, however deep, is still bounded by the
    // window - that is the whole point of a scroll frame.
    const uint32_t deep = tree.create(WidgetKind::Frame, child, "Deep");
    tree.get(deep)->width = 50.0f;
    tree.get(deep)->height = 50.0f;
    tree.addPoint(deep, Anchor{});

    tree.layout(kScreenW, kScreenH);

    CHECK(tree.get(child)->clipTo == scroll);
    CHECK(tree.get(deep)->clipTo == scroll);
    // The bar inherits whatever clips the scroll frame itself - here, nothing.
    CHECK(tree.get(bar)->clipTo != scroll);
    CHECK(tree.get(bar)->clipTo == tree.get(scroll)->clipTo);
}

TEST_CASE("Button art fills the button on an axis its anchors leave open",
          "[widget][layout][buttonart]") {
    // CharacterFrameTabButtonTemplate's highlight carries a LEFT and a RIGHT
    // anchor and no <Size>. That pair is opposite edges on x, so the width
    // comes out of it; on y both anchors are the same centre, which resizes
    // nothing, and the height came out zero. A region with no height is not
    // drawn, so the highlight behind every tab on the character sheet, the
    // merchant, the mail, the friends list and the auction house was built,
    // positioned and never seen.
    WidgetTree tree;
    const uint32_t button = tree.create(WidgetKind::Frame, 0, "Tab");
    tree.get(button)->width = 120.0f;
    tree.get(button)->height = 32.0f;
    tree.addPoint(button, Anchor{});

    auto edge = [&](uint32_t id, const char* point, float x) {
        Anchor a;
        a.point = point;
        a.relativePoint = point;
        a.relativeTo = button;
        a.x = x;
        tree.addPoint(id, a);
    };

    const uint32_t art = tree.create(WidgetKind::Texture, button, "Highlight");
    tree.get(art)->texturePath = "Interface\\Buttons\\Highlight.blp";
    tree.get(art)->buttonArt = ButtonArt::Highlight;
    edge(art, "LEFT", 10.0f);
    edge(art, "RIGHT", -10.0f);

    // And one that states its own size, which must be left exactly as it is -
    // TabButtonTemplate's highlight says 5 by 32 and means it.
    const uint32_t sized = tree.create(WidgetKind::Texture, button, "SizedArt");
    tree.get(sized)->texturePath = "Interface\\Buttons\\Highlight.blp";
    tree.get(sized)->buttonArt = ButtonArt::Highlight;
    tree.get(sized)->width = 5.0f;
    tree.get(sized)->height = 12.0f;
    tree.addPoint(sized, Anchor{});

    tree.layout(kScreenW, kScreenH);

    // The pair still decides the width; only the empty axis is filled in.
    CHECK(tree.get(art)->rectW == Catch::Approx(100.0f));
    CHECK(tree.get(art)->rectH == Catch::Approx(32.0f));
    CHECK(tree.get(art)->bottom == Catch::Approx(tree.get(button)->bottom));

    CHECK(tree.get(sized)->rectW == Catch::Approx(5.0f));
    CHECK(tree.get(sized)->rectH == Catch::Approx(12.0f));
}

TEST_CASE("Edges that cross give no size rather than a negative one",
          "[widget][layout]") {
    // A scroll bar's middle piece stretches from the top piece's bottom edge to
    // the bottom piece's top edge. On a bar shorter than the two end pieces
    // together those are the wrong way round: the macro frame's bar is 146 tall
    // and its ends are 102 and 106, so the middle solved to minus 55. Retail's
    // art overlaps the same way on a short bar - the overlap is not the fault,
    // carrying the negative onward is.
    //
    // A rect with a negative extent is inverted rather than empty, so every
    // containment test it takes part in answers about a region that is not
    // there: what clips it, what it clips, whether a click landed inside it.
    WidgetTree tree;
    const uint32_t bar = tree.create(WidgetKind::Frame, 0, "Bar");
    tree.get(bar)->width = 31.0f;
    tree.get(bar)->height = 146.0f;
    tree.addPoint(bar, Anchor{});

    auto piece = [&](const char* name, const char* point, float h) {
        const uint32_t id = tree.create(WidgetKind::Texture, bar, name);
        tree.get(id)->texturePath = "Interface\\ScrollBar.blp";
        tree.get(id)->width = 31.0f;
        tree.get(id)->height = h;
        Anchor a;
        a.point = point;
        a.relativePoint = point;
        a.relativeTo = bar;
        tree.addPoint(id, a);
        return id;
    };
    const uint32_t top = piece("Top", "TOP", 102.0f);
    const uint32_t bottom = piece("Bottom", "BOTTOM", 106.0f);

    const uint32_t middle = tree.create(WidgetKind::Texture, bar, "Middle");
    tree.get(middle)->texturePath = "Interface\\ScrollBar.blp";
    tree.get(middle)->width = 31.0f;
    tree.get(middle)->height = 1.0f;
    {
        Anchor a;
        a.point = "TOP"; a.relativePoint = "BOTTOM"; a.relativeTo = top;
        tree.addPoint(middle, a);
        Anchor b;
        b.point = "BOTTOM"; b.relativePoint = "TOP"; b.relativeTo = bottom;
        tree.addPoint(middle, b);
    }

    tree.layout(kScreenW, kScreenH);

    // The two ends really do overlap here - that is the situation, not a bug.
    REQUIRE(tree.get(top)->bottom < tree.get(bottom)->bottom + tree.get(bottom)->rectH);
    CHECK(tree.get(middle)->rectH == Catch::Approx(0.0f));
    CHECK(tree.get(middle)->rectH >= 0.0f);
}

TEST_CASE("Anchors decide a size only when they pin opposite edges",
          "[widget][anchor]") {
    // The rule the layout solves by, asked from the anchors alone so the passes
    // that must know a size *before* the solve get the same answer. A texture
    // sized from its own image must not overrule a pair of anchors, and must
    // still fill in an axis those anchors say nothing about.
    auto at = [](const char* point) {
        Anchor a;
        a.point = point;
        a.relativePoint = point;
        return a;
    };

    CHECK(anchorsSpanAxis({at("LEFT"), at("RIGHT")}, true));
    CHECK_FALSE(anchorsSpanAxis({at("LEFT"), at("RIGHT")}, false));

    CHECK(anchorsSpanAxis({at("TOP"), at("BOTTOM")}, false));
    CHECK_FALSE(anchorsSpanAxis({at("TOP"), at("BOTTOM")}, true));

    // Corners pin both.
    CHECK(anchorsSpanAxis({at("TOPLEFT"), at("BOTTOMRIGHT")}, true));
    CHECK(anchorsSpanAxis({at("TOPLEFT"), at("BOTTOMRIGHT")}, false));

    // An edge and a centre are two different fractions and size nothing -
    // the distinction that kept every reputation row from being stretched.
    CHECK_FALSE(anchorsSpanAxis({at("TOP"), at("CENTER")}, false));
    CHECK_FALSE(anchorsSpanAxis({at("LEFT"), at("CENTER")}, true));

    // One anchor is a position, never a size.
    CHECK_FALSE(anchorsSpanAxis({at("LEFT")}, true));
    CHECK_FALSE(anchorsSpanAxis({}, true));
}

TEST_CASE("A scroll frame's child is visible without anchors of its own",
          "[widget][layout][scroll]") {
    // A scroll child carries no anchors: SetScrollChild positions it, the
    // anchor solver does not. The unanchored-frame rule that hides a frame
    // with nothing to anchor to must not catch it, or it takes every element
    // inside it down with it - which blanked the whole quest dialog while
    // every frame in it measured correct.
    WidgetTree tree;
    const uint32_t panel = tree.create(WidgetKind::Frame, 0, "Panel");
    tree.get(panel)->width = 400.0f;
    tree.get(panel)->height = 400.0f;
    tree.addPoint(panel, Anchor{});

    const uint32_t scroll = tree.create(WidgetKind::Frame, panel, "Scroll");
    tree.markScrollFrame(scroll);
    tree.get(scroll)->width = 300.0f;
    tree.get(scroll)->height = 300.0f;
    tree.addPoint(scroll, Anchor{});

    // No anchors on the child - exactly as SetScrollChild leaves it.
    const uint32_t child = tree.create(WidgetKind::Frame, scroll, "Child");
    tree.get(scroll)->scrollChild = child;
    tree.get(child)->width = 300.0f;
    tree.get(child)->height = 900.0f;

    const uint32_t text = tree.create(WidgetKind::FontString, child, "Text");
    tree.get(text)->text = "Quest description";
    tree.addPoint(text, Anchor{});

    tree.layout(kScreenW, kScreenH);

    CHECK(tree.get(child)->visible);
    CHECK(tree.get(text)->visible);

    // And an ordinary frame with no anchors and no scroll parent is still
    // hidden - the rule this exempts one case from must otherwise hold.
    const uint32_t stray = tree.create(WidgetKind::Frame, panel, "Stray");
    tree.get(stray)->width = 50.0f;
    tree.get(stray)->height = 50.0f;
    tree.layout(kScreenW, kScreenH);
    CHECK_FALSE(tree.get(stray)->visible);
}

// A font string is only re-measured when the text it holds differs from the
// text it was last measured with. So zeroing its height has to clear that mark,
// exactly as zeroing its width already does - otherwise setting the SAME words
// back skips the measure as a no-op and the zero stands, and a region with no
// height is dropped from the draw order entirely.
//
// That is the quest tracker going blank. WatchFrameLineTemplate_Reset ends with
// `self.text:SetHeight(0)`, WatchFrame_ClearDisplay calls it on every line, and
// collapsing the tracker runs ClearDisplay through OnSizeChanged. The rebuild
// wrote each objective back unchanged, so every line measured zero and the
// tracker drew its POI badges over empty rows.
TEST_CASE("Zeroing a font string's height lets it be measured again",
          "[widget][fontstring]") {
    WidgetTree tree;
    const uint32_t fs = tree.create(WidgetKind::FontString, 0, "Label");
    Widget* w = tree.get(fs);

    w->text = "Kobold Vermin slain: 3/8";
    w->measuredText = w->text;   // as though it had already been sized
    w->width = 160.0f;
    w->height = 14.4f;

    tree.setHeight(fs, 0.0f);

    // Cleared, so the next sizing pass measures it even though the text it is
    // about to be given is the text it already had.
    REQUIRE(w->measuredText.empty());
    REQUIRE(w->height == Catch::Approx(0.0f));

    SECTION("a real height leaves the mark alone") {
        Widget* w2 = tree.get(fs);
        w2->measuredText = w2->text;
        tree.setHeight(fs, 20.0f);
        REQUIRE(w2->measuredText == w2->text);
        REQUIRE(w2->rectH == Catch::Approx(20.0f));
    }

    SECTION("a frame is not a font string and keeps its mark") {
        const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
        Widget* fw = tree.get(f);
        fw->measuredText = "unchanged";
        tree.setHeight(f, 0.0f);
        REQUIRE(fw->measuredText == "unchanged");
    }
}

// A <ThumbTexture> declares a size and no anchors at all - in WoW the slider is
// what places it, at the point along the track its value names. The ordinary
// solve has no anchors to work from and falls through to "centre it on the
// parent", which left every scroll bar in the interface drawing its grip at the
// middle of the track whatever the bar was worth: a scroll bar you cannot drag,
// because the one part that should answer never moves.
//
// Value 0 belongs at the TOP of a vertical bar - it is the top of the content -
// and that has to agree with how a drag reads the cursor back into a value, or
// the grip walks the opposite way from the hand holding it.
TEST_CASE("A slider's thumb sits where its value says", "[widget][slider]") {
    WidgetTree tree;
    const uint32_t bar = tree.create(WidgetKind::Frame, 0, "Bar");
    Widget* b = tree.get(bar);
    b->isSlider = true;
    b->barVertical = true;
    b->width = 16.0f;
    b->height = 277.0f;
    Anchor a;
    a.point = "BOTTOMLEFT";
    a.relativePoint = "BOTTOMLEFT";
    a.x = 0.0f;
    a.y = 272.0f;
    tree.addPoint(bar, a);

    const uint32_t thumb = tree.create(WidgetKind::Texture, bar, "BarThumbTexture");
    Widget* t = tree.get(thumb);
    t->width = 18.0f;
    t->height = 24.0f;
    b->thumbRegion = thumb;

    // Track runs 272..549; the grip travels 277 - 24 = 253 of it.
    b->barMin = 0.0f; b->barMax = 200.0f;

    b->barValue = 0.0f;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(t->bottom == Catch::Approx(525.0f));           // grip top == track top
    REQUIRE(t->bottom + t->rectH == Catch::Approx(549.0f));

    b->barValue = 100.0f;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(t->bottom == Catch::Approx(398.5f));           // dead centre

    b->barValue = 200.0f;
    tree.layout(kScreenW, kScreenH);
    REQUIRE(t->bottom == Catch::Approx(272.0f));           // grip bottom == track bottom

    SECTION("it is centred across the bar, which is narrower than the grip") {
        REQUIRE(t->left == Catch::Approx(-1.0f));          // (16 - 18) / 2
    }

    SECTION("a horizontal slider runs the other axis, minimum at the left") {
        b->barVertical = false;
        b->width = 277.0f;
        b->height = 16.0f;
        t->width = 24.0f;
        t->height = 18.0f;
        b->barValue = 0.0f;
        tree.layout(kScreenW, kScreenH);
        REQUIRE(t->left == Catch::Approx(0.0f));
        b->barValue = 200.0f;
        tree.layout(kScreenW, kScreenH);
        REQUIRE(t->left == Catch::Approx(253.0f));
    }
}

// A scroll child is very often smaller than what is inside it, because nothing
// in the interface resizes it - the client does. The talent tree is the plainest
// case: PlayerTalentFrameScrollChildFrame declares 320x50 and holds eleven rows
// of talents 63 apart, and no line of FrameXML ever gives it a height. Taking
// the declared 50 meant a scroll range of zero and a tree that would not scroll.
TEST_CASE("A scroll child's reach is its contents, not its declared size",
          "[widget][scroll]") {
    WidgetTree tree;
    const uint32_t child = tree.create(WidgetKind::Frame, 0, "Child");
    Widget* c = tree.get(child);
    c->width = 320.0f;
    c->height = 50.0f;
    Anchor ca;
    ca.point = "TOPLEFT";
    ca.relativePoint = "TOPLEFT";
    tree.addPoint(child, ca);

    const uint32_t tall = tree.create(WidgetKind::Frame, child, "Tall");
    Widget* t = tree.get(tall);
    t->width = 300.0f;
    t->height = 600.0f;
    Anchor ta;
    ta.point = "TOPLEFT";
    ta.relativePoint = "TOPLEFT";
    ta.relativeTo = child;
    tree.addPoint(tall, ta);
    tree.layout(kScreenW, kScreenH);

    float w = 0.0f, h = 0.0f;
    tree.scrollContentExtent(child, w, h);
    REQUIRE(h == Catch::Approx(600.0f));

    SECTION("hidden content is not content") {
        // A pool of buttons parked out of the way would otherwise scroll the
        // view into empty space.
        tree.get(tall)->shown = false;
        tree.layout(kScreenW, kScreenH);
        float w2 = 0.0f, h2 = 0.0f;
        tree.scrollContentExtent(child, w2, h2);
        REQUIRE(h2 == Catch::Approx(50.0f));
    }

    SECTION("a child that does size itself keeps its own height") {
        // A HybridScrollFrame sets the child's height from its rows; this may
        // only ever report more room than declared, never less.
        tree.get(tall)->height = 20.0f;
        tree.layout(kScreenW, kScreenH);
        float w3 = 0.0f, h3 = 0.0f;
        tree.scrollContentExtent(child, w3, h3);
        REQUIRE(h3 == Catch::Approx(50.0f));
    }
}

// An edit box paints its own text and caret, the way a status bar paints its
// fill and a message frame its lines. Dropped from the draw order as "a frame
// with nothing of its own to paint", the chat box still opened, took the focus
// and filled with what was typed - every character going into a widget that was
// never drawn. The bar looked empty rather than missing, because its art is
// child textures and those draw on their own.
TEST_CASE("An edit box is drawn for its own text, not skipped as a container",
          "[widget][draworder][editbox]") {
    WidgetTree tree;
    const uint32_t box = tree.create(WidgetKind::Frame, tree.root(), "Box");
    Widget* b = tree.get(box);
    b->isEditBox = true;
    b->width = 440.0f;
    b->height = 32.0f;
    Anchor a;
    a.point = "BOTTOMLEFT";
    a.relativePoint = "BOTTOMLEFT";
    tree.addPoint(box, a);
    tree.layout(kScreenW, kScreenH);

    auto drawn = [&](uint32_t id) {
        for (const Widget* w : tree.drawOrder()) if (w->id == id) return true;
        return false;
    };

    // No backdrop and no text: the caret still has to show which box is
    // listening, so it is drawn regardless.
    REQUIRE(drawn(box));

    SECTION("and with text in it") {
        tree.get(box)->editText = "hello world";
        tree.layout(kScreenW, kScreenH);
        REQUIRE(drawn(box));
    }

    SECTION("a plain container beside it is still skipped") {
        const uint32_t plain = tree.create(WidgetKind::Frame, tree.root(), "Plain");
        Widget* p = tree.get(plain);
        p->width = 100.0f;
        p->height = 20.0f;
        tree.addPoint(plain, a);
        tree.layout(kScreenW, kScreenH);
        REQUIRE_FALSE(drawn(plain));
    }
}

TEST_CASE("A child frame never sits below its parent", "[widget][level]") {
    // The level a child inherits is read off the parent, so a parent raised
    // after its children were resolved leaves them at a level computed from
    // where it used to be. A dropdown list is raised as it opens: the list came
    // out at 3, its item buttons at 2 and its own backdrop at 4, so the
    // backdrop painted over the items and the hit test - which takes the
    // highest level under the cursor - answered the list rather than a button.
    WidgetTree tree;
    const uint32_t list = tree.create(WidgetKind::Frame, tree.uiParentId(), "List");
    const uint32_t item = tree.create(WidgetKind::Frame, list, "ListItem");

    tree.get(list)->level = 3;
    tree.get(list)->levelExplicit = true;
    // As if resolved against the list's older, lower level.
    tree.get(item)->level = 2;
    tree.get(item)->levelExplicit = true;

    tree.layout(1920.0f, 1080.0f);

    CHECK(tree.get(item)->effLevel > tree.get(list)->effLevel);
}

TEST_CASE("Ordinary children still stack one above their parent", "[widget][level]") {
    WidgetTree tree;
    const uint32_t outer = tree.create(WidgetKind::Frame, tree.uiParentId(), "Outer");
    const uint32_t inner = tree.create(WidgetKind::Frame, outer, "Inner");
    tree.layout(1920.0f, 1080.0f);
    CHECK(tree.get(inner)->effLevel == tree.get(outer)->effLevel + 1);
}

// ── reset ───────────────────────────────────────────────────────────────────

// The interface belongs to the Lua state that built it. Loading a second one
// over the first is how a logout and another login without quitting produced a
// doubled interface: create() appends, so every frame existed twice and both
// copies drew.
TEST_CASE("Reset leaves nothing but the screen and UIParent", "[widget][reset]") {
    WidgetTree tree;
    const uint32_t before = tree.create(WidgetKind::Frame, tree.uiParentId(), "PlayerFrame");
    tree.get(before)->width = 100.0f;
    tree.get(before)->height = 40.0f;
    tree.layout(kScreenW, kScreenH);

    tree.reset();

    // The frame is gone by name as well as by id: findByName is what the client
    // and FrameXML both reach a frame through.
    CHECK(tree.findByName("PlayerFrame") == nullptr);
    CHECK(tree.get(before) == nullptr);

    // And the two the tree is built around are back, in that order, with
    // UIParent inside the screen.
    REQUIRE(tree.root() != 0);
    REQUIRE(tree.uiParentId() != 0);
    CHECK(tree.uiParentId() != tree.root());
    CHECK(tree.get(tree.uiParentId())->parent == tree.root());
    CHECK(tree.findByName("UIParent") == tree.get(tree.uiParentId()));
    // The screen has no name, so nothing in FrameXML can find it.
    CHECK(tree.get(tree.root())->name.empty());
    CHECK(tree.get(tree.root())->children.size() == 1);
}

TEST_CASE("A second load after a reset draws one interface, not two", "[widget][reset]") {
    WidgetTree tree;
    auto buildInterface = [&] {
        const uint32_t f = tree.create(WidgetKind::Frame, tree.uiParentId(), "PlayerFrame");
        Widget* w = tree.get(f);
        w->width = 100.0f;
        w->height = 40.0f;
        tree.addPoint(f, Anchor{});
        const uint32_t art = tree.create(WidgetKind::Texture, f, "PlayerFrameTexture");
        tree.get(art)->texturePath = "Interface\\TargetingFrame\\UI-PlayerFrame.blp";
        tree.setAllPoints(art, f);
        return f;
    };

    const uint32_t first = buildInterface();
    tree.reset();
    const uint32_t second = buildInterface();
    tree.layout(kScreenW, kScreenH);

    CHECK(tree.drawOrder().size() == 1);
    CHECK(tree.findByName("PlayerFrame")->id == second);
    // Reused, because the reset gave the ids back.
    CHECK(second == first);
}

// ── SimpleHTML ──────────────────────────────────────────────────────────────

// A SimpleHTML holds its text on the frame rather than in a font string child,
// which is how FrameXML fills ItemTextFrame's page. A plain frame with nothing
// to paint is culled from the draw order, so one holding a page of text has to
// be kept - otherwise the letter opens blank, which is what it did.
TEST_CASE("A SimpleHTML holding text is drawn", "[widget][simplehtml]") {
    WidgetTree tree;
    const uint32_t page = tree.create(WidgetKind::Frame, tree.uiParentId(), "PageText");
    Widget* w = tree.get(page);
    w->width = 270.0f;
    w->height = 304.0f;
    tree.addPoint(page, Anchor{});
    w->isSimpleHtml = true;

    SECTION("with a page of text it is in the draw order") {
        w->text = "Meet me at the tower when you are able.";
        tree.layout(kScreenW, kScreenH);
        bool found = false;
        for (const Widget* d : tree.drawOrder()) {
            if (d->id == page) { found = true; break; }
        }
        CHECK(found);
    }

    SECTION("empty it is culled like any other bare frame") {
        w->text.clear();
        tree.layout(kScreenW, kScreenH);
        bool found = false;
        for (const Widget* d : tree.drawOrder()) {
            if (d->id == page) { found = true; break; }
        }
        CHECK_FALSE(found);
    }
}

// A frame with a scale of its own lives in two coordinate spaces, and the two
// halves of moving it have to agree about which is which: a drawn rect is in
// screen units, while a declared size and an anchor offset are in the frame's
// own - layoutWidget multiplies the offset by the scale and nothing else.
//
// Dragging read the cursor delta straight into the offset, so a window at 1.25
// travelled a quarter again as far as the hand holding it; pinning wrote the
// drawn position into the offset, so it jumped by its own scale the moment it
// was picked up. Both are the bag window, which Interface > Bags scales.
TEST_CASE("A scaled frame moves with the cursor, not ahead of it",
          "[widget][layout][scale]") {
    WidgetTree tree;
    const uint32_t f = tree.create(WidgetKind::Frame, 0, "F");
    Widget* w = tree.get(f);
    w->width = 100.0f;
    w->height = 50.0f;
    w->scale = 1.25f;
    Anchor a;
    a.point = "BOTTOMLEFT";
    a.relativePoint = "BOTTOMLEFT";
    a.x = 100.0f;
    a.y = 100.0f;
    tree.addPoint(f, a);
    tree.layout(kScreenW, kScreenH);

    // The offset is in the frame's units, so the drawn corner is a scale out.
    REQUIRE(w->left == Catch::Approx(125.0f));
    REQUIRE(w->rectW == Catch::Approx(125.0f));

    // Forty pixels of cursor is forty pixels of frame.
    tree.nudge(f, 40.0f, 0.0f);
    tree.layout(kScreenW, kScreenH);
    REQUIRE(w->left == Catch::Approx(165.0f));

    SECTION("and pinning it where it is does not move it") {
        const float left = w->left, bottom = w->bottom;
        const float rectW = w->rectW, rectH = w->rectH;
        tree.pinToCurrentPosition(f);
        tree.layout(kScreenW, kScreenH);
        REQUIRE(w->left == Catch::Approx(left));
        REQUIRE(w->bottom == Catch::Approx(bottom));
        REQUIRE(w->rectW == Catch::Approx(rectW));
        REQUIRE(w->rectH == Catch::Approx(rectH));
    }

    SECTION("and a resize follows the cursor too") {
        tree.resizeBy(f, "BOTTOMRIGHT", 25.0f, 0.0f);
        tree.layout(kScreenW, kScreenH);
        // Twenty-five pixels wider on screen, whatever the frame calls it.
        REQUIRE(w->rectW == Catch::Approx(150.0f));
    }
}

// The colour picker's art is generated, not loaded, so the rule that drops a
// texture with nothing to show must not drop it.
//
// <ColorWheelTexture> and <ColorValueTexture> carry no file because there is no
// file: a wheel of every hue and a bar of every brightness are a function of the
// colour being picked, and the renderer paints them. Dropped from the draw order
// they never reach it, and the picker opens as a frame, two buttons and two
// thumbs over nothing - which is every colour in the interface, a chat window's
// background among them.
TEST_CASE("The colour picker's generated art stays in the draw order",
          "[widget][draworder][colorpicker]") {
    WidgetTree tree;
    const uint32_t picker = tree.create(WidgetKind::Frame, 0, "ColorPickerFrame");
    tree.get(picker)->width = 200.0f;
    tree.get(picker)->height = 200.0f;
    tree.get(picker)->hasBackdrop = true;
    tree.addPoint(picker, Anchor{});

    const uint32_t wheel = tree.create(WidgetKind::Texture, picker, "ColorPickerWheel");
    tree.get(wheel)->width = 128.0f;
    tree.get(wheel)->height = 128.0f;
    tree.get(wheel)->colorRole = Widget::ColorRole::Wheel;
    tree.addPoint(wheel, Anchor{});

    // The same texture with no role and no file is still nothing to draw.
    const uint32_t blank = tree.create(WidgetKind::Texture, picker, "Blank");
    tree.get(blank)->width = 128.0f;
    tree.get(blank)->height = 128.0f;
    tree.addPoint(blank, Anchor{});

    tree.layout(kScreenW, kScreenH);

    bool wheelDrawn = false, blankDrawn = false;
    for (const Widget* w : tree.drawOrder()) {
        if (w->id == wheel) wheelDrawn = true;
        if (w->id == blank) blankDrawn = true;
    }
    CHECK(wheelDrawn);
    CHECK_FALSE(blankDrawn);
}
