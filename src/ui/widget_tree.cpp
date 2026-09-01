#include "ui/widget_tree.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <ranges>

namespace wowee {
namespace ui {

namespace {

bool contains(const std::string& s, const char* sub) {
    return s.find(sub) != std::string::npos;
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(::toupper(static_cast<unsigned char>(c)));
    return s;
}

int strataRank(FrameStrata s) { return static_cast<int>(s); }
int layerRank(DrawLayer l) { return static_cast<int>(l); }

} // namespace

AnchorPoint resolveAnchorPoint(const std::string& rawName) {
    const std::string name = upper(rawName);
    AnchorPoint p;
    // Vertical: TOP is 1, BOTTOM is 0, neither is centred. Tested before the
    // horizontal half because the names combine (TOPLEFT is both).
    if (contains(name, "TOP"))         p.fy = 1.0f;
    else if (contains(name, "BOTTOM")) p.fy = 0.0f;
    else                               p.fy = 0.5f;

    if (contains(name, "LEFT"))        p.fx = 0.0f;
    else if (contains(name, "RIGHT"))  p.fx = 1.0f;
    else                               p.fx = 0.5f;
    return p;
}

bool anchorsSpanAxis(const std::vector<Anchor>& anchors, bool xAxis) {
    if (anchors.size() < 2) return false;
    float lo = 2.0f, hi = -1.0f;
    for (const Anchor& a : anchors) {
        const AnchorPoint p = resolveAnchorPoint(a.point);
        const float f = xAxis ? p.fx : p.fy;
        lo = std::min(lo, f);
        hi = std::max(hi, f);
    }
    // The same 0-and-1 test the solve uses. An edge and a centre are two
    // different fractions and size nothing.
    return lo < 0.01f && hi > 0.99f;
}

DrawLayer parseDrawLayer(const std::string& rawName) {
    const std::string n = upper(rawName);
    if (n == "BACKGROUND") return DrawLayer::Background;
    if (n == "BORDER")     return DrawLayer::Border;
    if (n == "OVERLAY")    return DrawLayer::Overlay;
    if (n == "HIGHLIGHT")  return DrawLayer::Highlight;
    return DrawLayer::Artwork;   // Blizzard's default
}

FrameStrata parseStrata(const std::string& rawName) {
    const std::string n = upper(rawName);
    if (n == "WORLD")             return FrameStrata::World;
    if (n == "BACKGROUND")        return FrameStrata::Background;
    if (n == "LOW")               return FrameStrata::Low;
    if (n == "HIGH")              return FrameStrata::High;
    if (n == "DIALOG")            return FrameStrata::Dialog;
    if (n == "FULLSCREEN")        return FrameStrata::Fullscreen;
    if (n == "FULLSCREEN_DIALOG") return FrameStrata::FullscreenDialog;
    if (n == "TOOLTIP")           return FrameStrata::Tooltip;
    return FrameStrata::Medium;
}

const char* strataName(FrameStrata strata) {
    switch (strata) {
        case FrameStrata::World:            return "WORLD";
        case FrameStrata::Background:       return "BACKGROUND";
        case FrameStrata::Low:              return "LOW";
        case FrameStrata::Medium:           return "MEDIUM";
        case FrameStrata::High:             return "HIGH";
        case FrameStrata::Dialog:           return "DIALOG";
        case FrameStrata::Fullscreen:       return "FULLSCREEN";
        case FrameStrata::FullscreenDialog: return "FULLSCREEN_DIALOG";
        case FrameStrata::Tooltip:          return "TOOLTIP";
    }
    return "MEDIUM";
}

WidgetTree::WidgetTree() {
    reset();
}

void WidgetTree::reset() {
    widgets_.clear();
    drawOrder_.clear();
    linkRects_.clear();
    scrollFrames_.clear();
    ownedTooltips_.clear();
    portraitsByUnit_.clear();
    portraitUnitOf_.clear();
    // Every one of these names a widget by id, and the ids are about to mean
    // something else. A drag or a focus held across the reset would be held on
    // whatever frame the new load happens to give that number to.
    hoveredId_ = 0;
    pressedId_ = 0;
    movingWid_ = 0;
    sizingWid_ = 0;
    sizingPoint_.clear();
    rootId_ = 0;
    uiParentId_ = 0;
    nextOrder_ = 1;
    lastPixelW_ = 0.0f;
    lastPixelH_ = 0.0f;
    layingOut_ = false;
    markLayoutDirty();

    widgets_.emplace_back();          // id 0 is "none"
    // The screen, then UIParent inside it. The screen carries no name: nothing
    // in FrameXML may find it, and GetParent on a detached frame answers nil
    // rather than naming something WoW has no word for.
    rootId_ = create(WidgetKind::Frame, 0, "");
    uiParentId_ = create(WidgetKind::Frame, rootId_, "UIParent");
}

uint32_t WidgetTree::create(WidgetKind kind, uint32_t parent, const std::string& name) {
    markLayoutDirty();
    const uint32_t id = static_cast<uint32_t>(widgets_.size());
    widgets_.emplace_back();
    Widget& w = widgets_.back();
    w.id = id;
    w.kind = kind;
    w.name = name;
    w.creationOrder = nextOrder_++;
    // Regions belong to the frame that made them; a widget with no parent
    // hangs off the screen, which is the root and sits above UIParent.
    if (parent == 0 && id != rootId_ && rootId_ != 0) parent = rootId_;
    w.parent = parent;
    if (parent != 0 && parent < widgets_.size()) {
        widgets_[parent].children.push_back(id);
        // Its place in the stack, known now rather than at the first layout.
        //
        // GetFrameLevel answers with this, and FrameXML asks during OnLoad -
        // RaiseFrameLevel is frame:SetFrameLevel(frame:GetFrameLevel() + 1),
        // and a frame that has never been laid out answered zero. So the
        // adjustment was computed against nothing: MainMenuBarArtFrame set
        // itself to 1 rather than to one above its parent, its buttons
        // followed, and the bar they sit on stayed above all of them and took
        // every click. Elsewhere the same sum went negative.
        w.effLevel = widgets_[parent].effLevel + 1;
        w.effStrata = widgets_[parent].effStrata;
    }
    return id;
}

void WidgetTree::setParent(uint32_t id, uint32_t newParent) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w || id == rootId_) return;
    // SetParent(nil) detaches to the screen - above UIParent, not under it.
    // This is exactly what WorldMap_ToggleSizeUp does before the map is shown
    // full screen: with the screen and UIParent being one node, nil landed
    // back on UIParent and the map was hidden by the same call that was
    // supposed to clear the way for it.
    if (newParent == 0) newParent = rootId_;
    if (newParent == w->parent) return;
    if (!get(newParent)) return;

    // A frame cannot be put inside itself or inside anything it contains -
    // layout walks children and would never come back.
    for (uint32_t up = newParent; up != 0;) {
        if (up == id) return;
        const Widget* p = get(up);
        if (!p) break;
        up = p->parent;
    }

    if (Widget* old = get(w->parent)) {
        auto& kids = old->children;
        kids.erase(std::remove(kids.begin(), kids.end(), id), kids.end());
    }
    w->parent = newParent;
    get(newParent)->children.push_back(id);
}

void WidgetTree::markScrollFrame(uint32_t id) {
    Widget* w = get(id);
    if (!w || w->isScrollFrame) return;
    w->isScrollFrame = true;
    scrollFrames_.push_back(id);
}

void WidgetTree::setTooltipOwner(uint32_t tooltipId, uint32_t ownerId) {
    Widget* w = get(tooltipId);
    if (!w) return;
    w->tooltipOwnerId = ownerId;
    if (ownerId == 0) return;
    if (std::find(ownedTooltips_.begin(), ownedTooltips_.end(), tooltipId) ==
        ownedTooltips_.end()) {
        ownedTooltips_.push_back(tooltipId);
    }
}

void WidgetTree::hideOrphanedTooltips() {
    for (uint32_t id : ownedTooltips_) {
        Widget* tip = get(id);
        if (!tip || !tip->shown) continue;
        const Widget* owner = get(tip->tooltipOwnerId);
        // Gone outright, or hidden with the panel it sat in. visibleChain is
        // the one that answers the second: a loot button is still shown in its
        // own right after LootFrame hides above it.
        if (owner && owner->visibleChain) continue;
        tip->shown = false;
        tip->visible = false;
        tip->visibleChain = false;
    }
}

void WidgetTree::setPortraitUnit(uint32_t id, const std::string& unit) {
    if (id == 0) return;

    auto had = portraitUnitOf_.find(id);
    if (had != portraitUnitOf_.end()) {
        if (had->second == unit) return;          // already this unit's
        auto& list = portraitsByUnit_[had->second];
        for (size_t i = 0; i < list.size(); ++i) {
            if (list[i] != id) continue;
            list[i] = list.back();
            list.pop_back();
            break;
        }
        portraitUnitOf_.erase(had);
        // The handle as well as the claim. Dropping off the list only stops
        // the updates, so the face it was last given would stay on screen -
        // which is how the next target wore the player's face, and a game
        // object made it obvious by having no portrait of its own to overwrite
        // it with.
        if (auto* w = get(id)) w->externalTexture = 0;
    }

    if (unit.empty()) return;
    portraitsByUnit_[unit].push_back(id);
    portraitUnitOf_[id] = unit;
}

const std::vector<uint32_t>& WidgetTree::portraitsFor(const std::string& unit) const {
    static const std::vector<uint32_t> kNone;
    auto it = portraitsByUnit_.find(unit);
    return (it == portraitsByUnit_.end()) ? kNone : it->second;
}

// Kept as a scan on purpose.
//
// A perf profile put this at 6.51% of the headless interface load, the largest
// first-party function in it, and indexing name -> ids dropped it to 0.06%.
// Against the clock that was a loss: ten interleaved runs per build measured
// the FrameXML load at 514ms scanning and 528ms indexed, stdev 4.1 and 2.4.
//
// The lookups got faster and the index cost more to build than they saved: a
// hash insert with a string key for every widget created, which the profile
// never showed as a hotspot because it spread across the creation path.
//
// Worth revisiting only with a measurement of lookups at runtime rather than
// during load, where the tree is full and each scan is longest.
Widget* WidgetTree::findByName(std::string_view name) {
    if (name.empty()) return nullptr;
    // Backwards, so the last frame to take the name is the one found - the
    // same rule as the global it was published under.
    for (auto& widget : std::views::reverse(widgets_)) {
        if (widget.id != 0 && widget.name == name) return &widget;
    }
    return nullptr;
}

const Widget* WidgetTree::findByName(std::string_view name) const {
    return const_cast<WidgetTree*>(this)->findByName(name);
}

Widget* WidgetTree::get(uint32_t id) {
    if (id == 0 || id >= widgets_.size()) return nullptr;
    return &widgets_[id];
}

const Widget* WidgetTree::get(uint32_t id) const {
    if (id == 0 || id >= widgets_.size()) return nullptr;
    return &widgets_[id];
}

void WidgetTree::clearPoints(uint32_t id) {
    markLayoutDirty();
    if (Widget* w = get(id)) w->anchors.clear();
}

void WidgetTree::setWidth(uint32_t id, float width) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w || !std::isfinite(width)) return;
    // Zero on a font string means "as wide as your text", not "no width".
    // That is WoW's convention and the interface leans on it:
    // PanelTemplates_TabResize ends with tabText:SetWidth(0) for a tab that
    // is not being capped, meaning let the label size itself.
    //
    // Taken literally it left the label zero wide, and a region with no width
    // is not drawn at all - which is why every tab on the character sheet had
    // its text set correctly and showed nothing. Clearing the measured mark
    // is what lets it be measured again; without that the label keeps the
    // zero, because it has already been measured once and its text has not
    // changed since.
    if (width <= 0.0f && w->kind == WidgetKind::FontString) {
        w->autoSized = false;
        w->measuredText.clear();
        // And it is no longer a paragraph: there is no width left to wrap
        // inside, which is the whole of what that meant.
        w->wrapsToWidth = false;
    }
    w->width = width;
    // Provisional, so a read before the next layout sees what was just set.
    // The layout overwrites it from the anchors, which is the final answer
    // where anchors decide the size.
    w->rectW = width;
}

void WidgetTree::setHeight(uint32_t id, float height) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w || !std::isfinite(height)) return;
    // Zero height on a font string is "be as tall as your text needs", the
    // same shape SetWidth(0) handles above - and it needs the same clearing of
    // the measured mark, for a reason that is easy to miss: the string is only
    // re-measured when the text it holds differs from the text it was last
    // measured with. Zero the height, then set the SAME words back, and the
    // measure is skipped as a no-op and the zero stands. A region with no
    // height is dropped from the draw order outright, so the label goes
    // silently blank while its frame keeps its size.
    //
    // That is the quest tracker. WatchFrameLineTemplate_Reset ends with
    // `self.text:SetHeight(0)`, WatchFrame_ClearDisplay calls it on every line,
    // and collapsing the tracker runs ClearDisplay through OnSizeChanged. The
    // rebuild then writes each objective back unchanged, so every line measured
    // zero and the tracker showed its POI badges over empty rows. Objectives
    // whose text had changed in the meantime - a kill count ticking over - were
    // re-measured and did appear, which is what made it look intermittent.
    if (height <= 0.0f && w->kind == WidgetKind::FontString) {
        w->measuredText.clear();
    }
    w->height = height;
    w->rectH = height;
}

void WidgetTree::pinToCurrentPosition(uint32_t id) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w) return;
    const Widget* parent = get(w->parent);
    const float px = parent ? parent->left : 0.0f;
    const float py = parent ? parent->bottom : 0.0f;

    // One anchor leaves the size to be stated rather than solved, so a frame
    // that was sized by two opposing corners keeps the size it had rather than
    // collapsing the moment it is picked up.
    if (w->width <= 0.0f)  w->width  = w->rectW;
    if (w->height <= 0.0f) w->height = w->rectH;

    Anchor a;
    a.point = "BOTTOMLEFT";
    a.relativePoint = "BOTTOMLEFT";
    a.relativeTo = 0;   // the parent
    a.x = w->left - px;
    a.y = w->bottom - py;
    w->anchors.clear();
    w->anchors.push_back(a);
    w->userMoved = true;
}

namespace {
/// Pulls a frame inside the screen it must stay within.
///
/// An axis where the frame is larger than the screen is left alone: there is
/// no position that satisfies both edges, and snapping to one of them moves
/// the frame for no benefit.
/// Keep a frame on screen, allowing for the insets it declared.
///
/// The insets move the edges of the rectangle that has to stay on screen,
/// which is not the same as the frame's own rectangle. Positive is inward, as
/// everywhere else in WoW: a positive right inset lets that much of the frame
/// hang past the right edge, and a negative one holds it that much clear of it.
///
/// The world map names the case exactly - SetClampRectInsets(0, 0, 0, -60)
/// with "don't overlap the xp/rep bars" beside it, so a negative bottom keeps
/// the frame sixty above the bottom edge rather than letting it reach.
void clampInside(const Widget& screen, float rectW, float rectH,
                 float& left, float& bottom,
                 float insetL = 0.0f, float insetR = 0.0f,
                 float insetT = 0.0f, float insetB = 0.0f) {
    const float loX = screen.left - insetL;
    const float hiX = screen.left + screen.rectW - rectW + insetR;
    const float loY = screen.bottom - insetB;
    const float hiY = screen.bottom + screen.rectH - rectH + insetT;
    if (hiX >= loX) left   = std::clamp(left,   loX, hiX);
    if (hiY >= loY) bottom = std::clamp(bottom, loY, hiY);
}
}  // namespace

// Resize from whichever corner the grabber took hold of.
//
// The point names the corner that MOVES. Dragging BOTTOMRIGHT grows the frame
// right and down, so its top-left stays put and only the size changes; dragging
// TOPLEFT has to move the frame as well, because the corner the player is not
// touching must not travel. Getting that wrong makes a frame walk across the
// screen as it is resized, which is the usual way this is done wrongly.
void WidgetTree::resizeBy(uint32_t id, const std::string& point,
                          float dx, float dy) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w) return;

    // A frame sized by two opposing anchors has no width of its own to change,
    // so pin it to what it is currently drawn at first - the same reason
    // pinToCurrentPosition does this before a move.
    if (w->width <= 0.0f)  w->width  = w->rectW;
    if (w->height <= 0.0f) w->height = w->rectH;

    const bool movesLeft   = point.find("LEFT")   != std::string::npos;
    const bool movesBottom = point.find("BOTTOM") != std::string::npos;
    // A corner with neither LEFT nor RIGHT in it does not change the width, and
    // the same for TOP/BOTTOM and the height - "BOTTOM" alone is a bottom edge.
    const bool changesW = movesLeft || point.find("RIGHT") != std::string::npos;
    const bool changesH = movesBottom || point.find("TOP") != std::string::npos;

    const float oldW = w->width, oldH = w->height;
    if (changesW) w->width  += movesLeft   ? -dx : dx;
    if (changesH) w->height += movesBottom ? -dy : dy;

    // Bounds. A zero maximum means unbounded, which is how a frame that never
    // called SetMaxResize reads.
    if (w->minResizeW > 0.0f) w->width  = std::max(w->width,  w->minResizeW);
    if (w->minResizeH > 0.0f) w->height = std::max(w->height, w->minResizeH);
    if (w->maxResizeW > 0.0f) w->width  = std::min(w->width,  w->maxResizeW);
    if (w->maxResizeH > 0.0f) w->height = std::min(w->height, w->maxResizeH);
    // Never inside out, whatever the bounds say.
    w->width  = std::max(w->width, 1.0f);
    w->height = std::max(w->height, 1.0f);

    // Move by however much the size actually changed, not by the cursor delta:
    // once a bound is reached the frame must stop rather than keep sliding.
    if (movesLeft)   { const float d = w->width  - oldW; for (Anchor& a : w->anchors) a.x -= d; }
    if (movesBottom) { const float d = w->height - oldH; for (Anchor& a : w->anchors) a.y -= d; }
}

void WidgetTree::nudge(uint32_t id, float dx, float dy) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w) return;
    // A clamped frame stops at the screen edge. The rect used is the one the
    // last layout produced, which is a frame behind the cursor and close
    // enough - the alternative is re-solving the whole tree per mouse move.
    //
    // A frame already outside is pulled back rather than pinned where it is:
    // that is what lets one recover, and it is what WoW does when a clamped
    // frame is restored from saved variables at a smaller resolution.
    if (w->clampedToScreen && w->rectW > 0.0f && w->rectH > 0.0f) {
        if (const Widget* screen = get(rootId_)) {
            // Back to a delta, because a drag moves the anchors rather than
            // the rect: the clamped position is what the anchors have to add
            // up to, not something that can be written to left/bottom here.
            float left = w->left + dx, bottom = w->bottom + dy;
            clampInside(*screen, w->rectW, w->rectH, left, bottom,
                        w->clampInsetL, w->clampInsetR, w->clampInsetT, w->clampInsetB);
            dx = left - w->left;
            dy = bottom - w->bottom;
        }
    }
    for (Anchor& a : w->anchors) { a.x += dx; a.y += dy; }
}

/// Move every descendant that carries its own level by the same amount.
///
/// A child's level is relative to its parent in WoW, and FrameXML sets levels
/// freely - RaiseFrameLevelByTwo alone is used throughout. Without this, a
/// raised window keeps its own art in front but leaves anything that set its
/// own level behind: the character sheet's name label sat at 6 while the panel
/// it belongs to went to 174, and sorting by level drew the name underneath
/// the panel, where it cannot be seen.
void WidgetTree::shiftExplicitLevels(uint32_t id, int delta) {
    if (delta == 0) return;
    const Widget* w = get(id);
    if (!w) return;
    // A copy, because get() invalidates nothing but the recursion below may.
    const std::vector<uint32_t> kids = w->children;
    for (uint32_t child : kids) {
        if (Widget* c = get(child)) {
            if (c->levelExplicit) c->level += delta;
        }
        shiftExplicitLevels(child, delta);
    }
}

void WidgetTree::raise(uint32_t id) {
    Widget* w = get(id);
    if (!w) return;
    // The highest of the *others*, not of everything including this frame.
    //
    // Seeded with w->effLevel, a frame already on top still came out one
    // higher, so every call added one whether or not anything was above it -
    // and ShowUIPanel raises on every panel open. Levels ratcheted all session:
    // a quest frame that starts at 3 was found at 344 in a play session, and
    // two frames raising alternately climb without bound.
    int highest = 0;
    for (const Widget& other : widgets_) {
        if (other.id == 0 || other.id == id) continue;
        if (other.effStrata != w->effStrata) continue;
        if (other.effLevel > highest) highest = other.effLevel;
    }
    // Already above everything: raising is what was asked for and it is
    // already true, so leave the level alone. Raise has to be idempotent or
    // repeating it is a slow leak.
    if (w->effLevel > highest) return;
    // Explicit from here on, or the next layout would recompute it from the
    // parent and undo the raise immediately.
    const int newLevel = highest + 1;
    shiftExplicitLevels(id, newLevel - w->effLevel);
    w->level = newLevel;
    w->levelExplicit = true;
}

void WidgetTree::lower(uint32_t id) {
    Widget* w = get(id);
    if (!w) return;
    // The lowest of the others, for the same reason raise takes the highest of
    // the others: seeded with this frame's own level, a frame already at the
    // bottom went one lower on every call, and level zero is the floor - so
    // this leaked in the other direction until it hit it.
    int lowest = INT_MAX;
    for (const Widget& other : widgets_) {
        if (other.id == 0 || other.id == id) continue;
        if (other.effStrata != w->effStrata) continue;
        if (other.effLevel < lowest) lowest = other.effLevel;
    }
    if (lowest == INT_MAX || w->effLevel < lowest) return;
    // Never below zero: a negative level sorts under the root and the frame
    // stops being drawn at all.
    const int newLevel = (lowest > 0) ? lowest - 1 : 0;
    shiftExplicitLevels(id, newLevel - w->effLevel);
    w->level = newLevel;
    w->levelExplicit = true;
}

void WidgetTree::addPoint(uint32_t id, const Anchor& anchor) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w) return;
    // Geometry that is not a number never enters the tree. Once one does it
    // spreads: the frame's rect goes to nan, everything anchored to it
    // follows, and a nan rect is hit by every mouse position because every
    // comparison against a nan is false - which stops the camera turning
    // anywhere on screen. FrameXML computes offsets as fractions of the
    // screen, so an arithmetic slip upstream arrives here rather than being
    // caught where it was made.
    if (!std::isfinite(anchor.x) || !std::isfinite(anchor.y)) return;
    // One anchor per point: setting a point that is already set replaces it
    // rather than adding a second. FrameXML depends on this, because it
    // repositions frames with a bare SetPoint and no ClearAllPoints -
    // UIParentManageFramePositions moves the durability frame with
    // SetPoint("TOPRIGHT", ...), expecting it to displace the TOPRIGHT the XML
    // declared. Keeping both left two constraints on the same edge, which is
    // not a solvable system; the first won, and every frame Blizzard
    // repositions this way stayed where its XML put it. The durability frame
    // sat forty units past the right edge of the screen.
    // The interface positioning a frame that a drag had moved starts from
    // scratch, because the anchor the move left is on whichever point it was
    // picked up by and would otherwise fight the one being set.
    if (w->userMoved) {
        w->userMoved = false;
        w->anchors.clear();
    }
    for (Anchor& existing : w->anchors) {
        if (existing.point == anchor.point) {
            existing = anchor;
            return;
        }
    }
    w->anchors.push_back(anchor);
}

void WidgetTree::setAllPoints(uint32_t id, uint32_t relativeTo) {
    markLayoutDirty();
    Widget* w = get(id);
    if (!w) return;
    w->anchors.clear();
    // Two opposing corners, which is exactly what makes the size fall out of
    // the solver below rather than needing an explicit one.
    Anchor tl; tl.point = "TOPLEFT";     tl.relativePoint = "TOPLEFT";     tl.relativeTo = relativeTo;
    Anchor br; br.point = "BOTTOMRIGHT"; br.relativePoint = "BOTTOMRIGHT"; br.relativeTo = relativeTo;
    w->anchors.push_back(tl);
    w->anchors.push_back(br);
}

void WidgetTree::scrollContentExtent(uint32_t childId, float& outW, float& outH) const {
    outW = 0.0f;
    outH = 0.0f;
    const Widget* child = get(childId);
    if (!child) return;

    // Seeded with the child's own rect, so this can only ever report more room
    // than it declares, never less.
    float minX = child->left,   maxX = child->left + child->rectW;
    float minY = child->bottom, maxY = child->bottom + child->rectH;

    // Measured against the child's own frame rather than the screen: everything
    // under a scroll child moves with it, so the span between the outermost
    // edges is the same whatever the frame is scrolled to. Reading screen
    // positions instead would grow the range as the view moved and never settle.
    std::vector<uint32_t> stack(child->children.begin(), child->children.end());
    while (!stack.empty()) {
        const uint32_t id = stack.back();
        stack.pop_back();
        const Widget* w = get(id);
        if (!w || !w->shown) continue;
        if (w->rectW > 0.0f || w->rectH > 0.0f) {
            minX = std::min(minX, w->left);
            maxX = std::max(maxX, w->left + w->rectW);
            minY = std::min(minY, w->bottom);
            maxY = std::max(maxY, w->bottom + w->rectH);
        }
        stack.insert(stack.end(), w->children.begin(), w->children.end());
    }

    outW = maxX - minX;
    outH = maxY - minY;
}

void WidgetTree::resolveWidget(uint32_t id) {
    if (layingOut_) return;
    // Nothing has run a full pass yet, so there is no screen size to resolve
    // against. Better a stale zero than a rect measured against a guess.
    if (lastPixelW_ <= 0.0f || lastPixelH_ <= 0.0f) return;
    const Widget* w = get(id);
    if (!w || w->resolvedGen == layoutGeneration_) return;
    const float screenW = (uiScale_ > 0.0f) ? (lastPixelW_ / uiScale_) : lastPixelW_;
    const float screenH = (uiScale_ > 0.0f) ? (lastPixelH_ / uiScale_) : lastPixelH_;
    int depth = 0;
    resolveChain(id, screenW, screenH, depth);
}

void WidgetTree::resolveChain(uint32_t id, float screenW, float screenH, int& depth) {
    if (id == 0) return;
    Widget* w = get(id);
    if (!w || w->resolvedGen == layoutGeneration_) return;
    // The screen and UIParent are placed by the full pass and have no anchors
    // of their own - running the anchor solver over them would give the screen
    // a rect derived from nothing, and everything measured against it after
    // that. They are already correct; the walk stops on them.
    if (id == rootId_ || id == uiParentId_) {
        w->resolvedGen = layoutGeneration_;
        return;
    }
    // A frame anchored to something anchored back to it would otherwise walk
    // for ever. WoW rejects such a pair outright; here the chain simply stops
    // and the rect stays where the last full pass left it, which is what the
    // frame before this one already answered.
    if (++depth > 64) { --depth; return; }
    // Claimed before the walk rather than after, so a cycle that slips past
    // the depth guard still terminates: whichever widget is reached twice
    // stops the second visit itself.
    w->resolvedGen = layoutGeneration_;
    const uint32_t parent = w->parent;
    // Copied: resolving a dependency can create widgets and move the container
    // this widget's anchors live in.
    std::vector<uint32_t> deps;
    deps.reserve(w->anchors.size());
    for (const Anchor& a : w->anchors) {
        if (a.relativeTo != 0 && a.relativeTo != id) deps.push_back(a.relativeTo);
    }
    // The parent first: a widget's rect is measured from its parent's, and its
    // scale is the parent's times its own.
    if (parent != 0 && parent != id) resolveChain(parent, screenW, screenH, depth);
    for (uint32_t d : deps) resolveChain(d, screenW, screenH, depth);
    --depth;
    layoutWidgetSelf(id, screenW, screenH);
}

void WidgetTree::layout(float pixelW, float pixelH) {
    // Reentry would be the layout of a layout: this is called from the rect
    // getters now, and it moves widgets, and moving a widget is what raises
    // the flag those getters watch.
    if (layingOut_) return;
    layingOut_ = true;
    struct Done { bool& f; ~Done() { f = false; } } done{layingOut_};
    lastPixelW_ = pixelW;
    lastPixelH_ = pixelH;
    // How many pixels one interface unit is worth. Everything below works in
    // units; only the renderer and hit testing convert.
    // The screen's height decides the base, and the player's UI Scale
    // multiplies it. A smaller scale means a smaller interface and more units
    // of room, which is what the slider is understood to do.
    uiScale_ = ((pixelH > 0.0f) ? (pixelH / kInterfaceHeight) : 1.0f) * userScale_;
    const float screenW = (uiScale_ > 0.0f) ? (pixelW / uiScale_) : pixelW;
    // The same division as the width, and it used to be the constant instead.
    // The two agree at a user scale of 1 and only there: the screen shows
    // pixelH / uiScale_ units, so at any other scale the root was laid out at
    // a height the screen does not have. Above 1 that put everything anchored
    // to the top off the top of the screen - and it is why raising the scale
    // ceiling made the options frame unreachable rather than merely large.
    const float screenH = (uiScale_ > 0.0f) ? (pixelH / uiScale_) : pixelH;

    Widget& rootW = widgets_[rootId_];
    rootW.left = 0.0f;
    rootW.bottom = 0.0f;
    rootW.rectW = screenW;
    rootW.rectH = screenH;
    rootW.visibleChain = rootW.shown;
    rootW.visible = rootW.shown;
    rootW.effStrata = rootW.strata;
    rootW.effLevel = 0;
    rootW.effScale = 1.0f;

    // UIParent fills the screen and is laid out here rather than by anchors:
    // it is created before any XML is read, so it has none, and a frame with
    // no anchors is not displayed. Its own shown flag still decides whether
    // anything under it is, which is the whole point of it being a frame.
    if (Widget* ui = get(uiParentId_); ui && ui != &rootW) {
        ui->left = 0.0f;
        ui->bottom = 0.0f;
        ui->rectW = screenW;
        ui->rectH = screenH;
        ui->visibleChain = rootW.visible && ui->shown;
        ui->visible = ui->visibleChain;
        ui->effStrata = ui->strata;
        ui->effLevel = 0;
        ui->effScale = 1.0f;
        for (uint32_t child : ui->children) layoutWidget(child, screenW, screenH);
    }

    for (uint32_t child : rootW.children) {
        if (child == uiParentId_) continue;
        layoutWidget(child, screenW, screenH);
    }
    collectDrawOrder();
}

void WidgetTree::layoutWidget(uint32_t id, float screenW, float screenH) {
    layoutWidgetSelf(id, screenW, screenH);
    if (const Widget* w = get(id)) {
        // Copied, because resolving a child can create widgets and reallocate
        // the container this vector lives in.
        const std::vector<uint32_t> kids = w->children;
        for (uint32_t child : kids) layoutWidget(child, screenW, screenH);
    }
}

void WidgetTree::layoutWidgetSelf(uint32_t id, float screenW, float screenH) {
    Widget* w = get(id);
    if (!w) return;
    w->resolvedGen = layoutGeneration_;
    const Widget* parent = get(w->parent);

    // A frame with no anchor points is not displayed. That is WoW's rule, and
    // without it every frame FrameXML declares without anchors - a money
    // frame, a dropdown, a quest reward panel - falls to the centre-on-parent
    // default and sits in the middle of the screen looking like a bug in
    // something else. Regions differ: an unanchored one fills its parent, and
    // that is handled below.
    //
    // The root is the exception: it is the screen, and has nothing to anchor
    // to.
    // A scroll frame's scroll child is the other exception: it carries no
    // anchors because SetScrollChild positions it, not the anchor solver - the
    // scroll frame places it at its own top-left and slides it by the scroll
    // offset. Treating it as unanchored marked it, and therefore every element
    // inside it, invisible: the quest dialog's title, description and
    // objectives are all children of QuestDetailScrollChildFrame, so the whole
    // dialog read as blank while every frame in it measured correct. It draws
    // where it is placed like any other frame, so it must not be hidden for
    // lacking anchors it was never meant to have.
    const bool isScrollChild = parent && parent->isScrollFrame &&
                               parent->scrollChild == id;
    const bool unanchoredFrame = (w->kind == WidgetKind::Frame) &&
                                 w->anchors.empty() && id != rootId_ &&
                                 !isScrollChild;
    // Two questions, and they are not the same one. Running is shown with
    // every ancestor shown; drawing additionally needs somewhere to be drawn.
    // Inherited from the parent's chain rather than its `visible`, or a child
    // of an unanchored driver frame would stop running too.
    w->visibleChain = w->shown && (!parent || parent->visibleChain);
    // Drawing inherits from the parent's *drawing*, not from the chain: an
    // anchored child of an unanchored frame has nowhere to be either, because
    // the thing it is anchored to has no position. Deriving this from the
    // chain instead put those children back on screen.
    w->visible = w->shown && (!parent || parent->visible) && !unanchoredFrame;
    // Clipping is inherited: anything under a scroll frame is bounded by it,
    // however deep, because a scroll child holds frames of its own.
    //
    // But it starts at the scroll *child*, not at every child of the scroll
    // frame. A scroll frame's own children include its scroll bar, and the bar
    // sits alongside the window rather than inside it - 329 to 345 on a frame
    // spanning 23 to 323 for the quest dialog. Clipping it to the window put
    // every scroll bar in the interface entirely outside its own clip rect,
    // which does not trim them, it deletes them: the bar, its track, its
    // thumb and both buttons were laid out, drawn, and cut away to nothing.
    // Found by sweeping for content clipped away sideways - vertical is
    // ordinary, since a scroll child is meant to be taller than its window,
    // but nothing scrolls back into view from beside it.
    const bool clippedByParent = parent && parent->isScrollFrame &&
                                 w->id == parent->scrollChild;
    w->clipTo = parent ? (clippedByParent ? parent->id : parent->clipTo) : 0;
    // Strata and level are inherited unless the widget set its own. A child
    // frame sits one level above its parent so it draws over it, which is what
    // makes a button's own regions land on top of the frame holding it.
    w->effStrata = w->strataExplicit ? w->strata : (parent ? parent->effStrata : FrameStrata::Medium);
    w->effLevel  = w->levelExplicit  ? w->level  : (parent ? parent->effLevel + 1 : 0);
    // ...and never below the parent, whatever the two lines above worked out.
    //
    // The level a child inherits is read off the parent, so a parent raised
    // after its children were resolved leaves them at a level computed from
    // where it used to be. A dropdown list is raised as it opens, and its item
    // buttons kept the old answer: the list came out at level 3 with its
    // buttons at 2 and its own backdrop at 4. The backdrop then painted over
    // the items, which is why they looked greyed, and the hit test - which
    // takes the highest level under the cursor - answered the list rather than
    // the button, which is why clicking one did nothing.
    //
    // Ties are settled by creation order, so a backdrop declared before the
    // buttons still sits behind them once all three are on the same level.
    if (parent && w->effLevel < parent->effLevel) {
        w->effLevel = parent->effLevel + 1;
    }
    // Multiplied down the chain, so scaling a window scales everything in it.
    w->effScale  = (parent ? parent->effScale : 1.0f) * w->scale;
    const float es = w->effScale;

    // Solve each axis from the anchors. An anchor says "this fraction of my rect
    // sits at that point", which is one linear constraint; two constraints with
    // different fractions give the size as well as the position, and that is how
    // a frame pinned at two opposing corners gets sized without anyone calling
    // SetSize.
    struct Constraint { float f; float target; };
    std::vector<Constraint> cx, cy;
    cx.reserve(w->anchors.size());
    cy.reserve(w->anchors.size());

    for (const Anchor& a : w->anchors) {
        const Widget* rel = (a.relativeTo != 0) ? get(a.relativeTo) : parent;
        float relLeft, relBottom, relW, relH;
        if (rel) {
            relLeft = rel->left; relBottom = rel->bottom; relW = rel->rectW; relH = rel->rectH;
        } else {
            relLeft = 0.0f; relBottom = 0.0f; relW = screenW; relH = screenH;
        }
        const AnchorPoint rp = resolveAnchorPoint(a.relativePoint);
        const AnchorPoint mp = resolveAnchorPoint(a.point);
        // The offset is in this frame's units; the anchor it hangs from is
        // already resolved, so only the offset is scaled.
        cx.push_back({.f = mp.fx, .target = relLeft   + rp.fx * relW + a.x * es});
        cy.push_back({.f = mp.fy, .target = relBottom + rp.fy * relH + a.y * es});
    }

    auto solveAxis = [](const std::vector<Constraint>& cs, float explicitSize,
                        float parentOrigin, float parentSize,
                        float& outOrigin, float& outSize) {
        if (cs.empty()) {
            // Unanchored: WoW leaves this undefined, and centring on the parent
            // is the least surprising thing to draw.
            outSize = explicitSize;
            outOrigin = parentOrigin + (parentSize - explicitSize) * 0.5f;
            return;
        }
        // A frame is resized on an axis only when two anchors pin *opposite
        // edges* of it - a 0 and a 1. That is the rule WoW follows, and the
        // difference from "any two fractions that differ" is not academic.
        //
        // The reputation rows are built from both: the XML anchors each row's
        // TOPRIGHT under the previous row, and ReputationFrame_SetRowType then
        // adds a LEFT anchor to set the indent. On x those are 0 and 1, so the
        // row correctly spans from its indent to the frame's right edge. On y
        // they are 1 (a top edge) and 0.5 (a centre) - not opposite edges, and
        // nothing WoW would resize from.
        //
        // Deriving a height from an edge and a centre gave twice the distance
        // between them, which for a row anchored near the top of a frame whose
        // centre is halfway down is most of the frame. Every faction row was
        // stretched to that height and drawn on top of the last, which is why
        // the reputation tab showed a stack of overlapping names behind two
        // enormous yellow bars.
        size_t lo = 0, hi = 0;
        for (size_t i = 1; i < cs.size(); ++i) {
            if (cs[i].f < cs[lo].f) lo = i;
            if (cs[i].f > cs[hi].f) hi = i;
        }
        if (cs[lo].f < 0.01f && cs[hi].f > 0.99f) {
            outSize = cs[hi].target - cs[lo].target;
            outOrigin = cs[lo].target;
            // Two edges that have crossed describe nothing, not a negative
            // region. It happens with art that assumes more room than it got:
            // a scroll bar's middle stretches from the top piece's bottom to
            // the bottom piece's top, and on a bar shorter than the two pieces
            // together those are the wrong way round - the macro frame's bar
            // is 146 tall and its two ends are 102 and 106, so the middle
            // solved to minus 55. Retail's art overlaps the same way there;
            // the overlap is not the fault, carrying the negative onward is.
            //
            // Nothing downstream means anything by it. A rect with a negative
            // extent is inverted rather than empty, so every containment test
            // it takes part in - what clips it, what it clips, whether a click
            // is inside it - answers about a region that is not there.
            if (outSize < 0.0f) outSize = 0.0f;
        } else {
            // Positioned by the first anchor that names an *edge* on this
            // axis, and only by the first anchor of any kind when none does.
            //
            // An anchor constrains both axes whether or not it meant to. LEFT
            // is the left edge and the vertical centre; BOTTOM is the bottom
            // edge and the horizontal centre. So a frame given LEFT, RIGHT and
            // BOTTOM - which is how the talent frame's points bar is written,
            // and it is a common shape - has three constraints on y: two
            // centres that came along with the horizontal pair, and the one
            // that was actually about y. Taking the first put the bar at its
            // parent's vertical centre and ignored the bottom edge it was
            // given, which left it floating in the middle of the window with
            // half the frame empty beneath it. The scroll frame above it is
            // anchored to its top, so the talent tree was squeezed into the
            // half above that, cut off, and unscrollable because the tree it
            // was showing had been made shorter than its own content.
            //
            // An edge is the specific statement and a centre is the incidental
            // one, so the edge wins. This does not change the case above,
            // where two opposite edges size the axis, nor the one the
            // reputation rows depend on - their y constraints are a top edge
            // and a centre, and the top edge is both the first anchor and the
            // edge, so it is chosen either way.
            size_t pick = 0;
            for (size_t i = 0; i < cs.size(); ++i) {
                if (cs[i].f < 0.01f || cs[i].f > 0.99f) { pick = i; break; }
            }
            outSize = explicitSize;
            outOrigin = cs[pick].target - cs[pick].f * outSize;
        }
    };

    const float pLeft   = parent ? parent->left   : 0.0f;
    const float pBottom = parent ? parent->bottom : 0.0f;
    const float pW      = parent ? parent->rectW  : screenW;
    const float pH      = parent ? parent->rectH  : screenH;

    // A region that says nothing about where it is or how big fills its
    // parent. That is WoW's default for a Texture or FontString declared in a
    // Layer with neither <Size> nor <Anchors>, and it is not a rare shorthand:
    // PlayerFrameTexture is the entire player frame's art and MinimapBorder is
    // the ring around the minimap, and both are written this way. Centring
    // them at no size instead meant they were laid out to nothing, never
    // reached the draw order, and so were never even uploaded.
    if (w->kind != WidgetKind::Frame && w->anchors.empty() &&
        w->width <= 0.0f && w->height <= 0.0f && parent && !w->isTooltip) {
        w->left   = parent->left;
        w->bottom = parent->bottom;
        w->rectW  = parent->rectW;
        w->rectH  = parent->rectH;
    } else {
        solveAxis(cx, w->width * es,  pLeft,   pW, w->left,   w->rectW);
        solveAxis(cy, w->height * es, pBottom, pH, w->bottom, w->rectH);
        // A button's art fills the button on any axis its anchors left open.
        //
        // The rule above covers art that says nothing at all about where it
        // goes. This is the same thing said half way: CharacterFrameTabButton's
        // highlight carries a LEFT and a RIGHT anchor and no <Size>, so its
        // width comes out of the pair and its height out of nothing. A region
        // with no height is not drawn, so the highlight behind every tab on the
        // character sheet, the merchant, the mail, the friends list and the
        // auction house - sixteen of them - was built, positioned, and never
        // seen. Nothing about it reads wrong from Lua: it is shown, it has its
        // texture, and only the one number that decides whether any of it
        // reaches the screen is missing.
        //
        // Only for button art, and only for an axis that came out empty, so a
        // highlight that does declare its own size - TabButtonTemplate's says
        // 5 by 32 - keeps it.
        if (w->buttonArt != ButtonArt::None && parent) {
            if (w->rectW <= 0.0f) { w->left   = pLeft;   w->rectW = pW; }
            if (w->rectH <= 0.0f) { w->bottom = pBottom; w->rectH = pH; }
        }
    }
    // After the solve, so it displaces the result rather than becoming another
    // constraint on it.
    w->left   += w->animOffsetX;
    w->bottom += w->animOffsetY;

    // The scroll offset, applied to the child a scroll frame holds. Scrolling
    // down means seeing content further down a taller child, which is the
    // child moving up - and up is a larger bottom in these coordinates.
    if (parent && parent->isScrollFrame && parent->scrollChild == id) {
        w->left   -= parent->scrollX;
        w->bottom += parent->scrollY;
    }

    // A slider's grip goes where its value says. <ThumbTexture> declares a size
    // and no anchors - in WoW the slider is what places it - so the ordinary
    // solve fell through to "unanchored, centre it on the parent" and every
    // scroll bar in the interface drew its knob at the middle of the track and
    // left it there, whatever the bar was worth. It reads as a scroll bar you
    // cannot drag, because the one part that should answer never moves.
    //
    // The far edge is the minimum on a vertical bar: value 0 is the top of the
    // content and so the top of the track. That matches how a drag reads the
    // cursor back into a value, and the two must agree or the grip walks the
    // opposite way from the hand holding it.
    if (parent && parent->isSlider && parent->thumbRegion == id &&
        parent->rectW > 0.0f && parent->rectH > 0.0f) {
        const float f = parent->barFraction();
        if (parent->barVertical) {
            const float span = parent->rectH - w->rectH;
            w->bottom = parent->bottom + (span > 0.0f ? span * (1.0f - f) : 0.0f);
            w->left   = parent->left + (parent->rectW - w->rectW) * 0.5f;
        } else {
            const float span = parent->rectW - w->rectW;
            w->left   = parent->left + (span > 0.0f ? span * f : 0.0f);
            w->bottom = parent->bottom + (parent->rectH - w->rectH) * 0.5f;
        }
    }

    // A clamped frame stays on screen however it was placed, not only when it
    // was dragged there.
    //
    // GameTooltipTemplate declares clampedToScreen="true" and every tooltip in
    // the interface inherits it, but the clamp lived only in the drag path -
    // and a tooltip is never dragged. It is anchored beside whatever it
    // describes, so one owned by a frame near an edge simply ran off it: the
    // minimap's calendar button put its tooltip past the right of the screen,
    // where it was laid out, drawn, and invisible.
    //
    // Before the children, so they follow the clamped position rather than the
    // one it was moved out of.
    if (w->clampedToScreen && id != rootId_ &&
        w->rectW > 0.0f && w->rectH > 0.0f) {
        if (const Widget* screen = get(rootId_)) {
            clampInside(*screen, w->rectW, w->rectH, w->left, w->bottom,
                        w->clampInsetL, w->clampInsetR, w->clampInsetT, w->clampInsetB);
        }
    }

}

uint32_t WidgetTree::hitTest(float x, float y) const {
    return hitTestFor(x, y, false);
}

uint32_t WidgetTree::hitTestWheel(float x, float y) const {
    return hitTestFor(x, y, true);
}

uint32_t WidgetTree::hitTestFor(float x, float y, bool forWheel) const {
    const Widget* best = nullptr;
    for (const Widget& w : widgets_) {
        if (w.id == 0 || w.kind != WidgetKind::Frame) continue;
        if (!w.visible) continue;
        // The wheel is enabled separately from the mouse and a scroll frame
        // asks for only the wheel, so requiring mouseEnabled for both hid
        // every one of them from the cursor.
        if (!w.mouseEnabled && !(forWheel && w.wheelEnabled)) continue;
        if (w.rectW <= 0.0f || w.rectH <= 0.0f) continue;
        // A rect with a NaN in it matches *everything*. Every comparison
        // against a NaN is false, so both `x < left` and `x > right` are false
        // and the frame below is treated as hit wherever the cursor is - and
        // one mouse-enabled frame answering every hit test tells the rest of
        // the client the interface owns the mouse, so the camera stops turning
        // and never starts again. A chat window whose saved position had gone
        // to nan did exactly that. Checked here as well as where positions are
        // written, because this is the one place the damage is total.
        if (!std::isfinite(w.left) || !std::isfinite(w.bottom) ||
            !std::isfinite(w.rectW) || !std::isfinite(w.rectH)) continue;
        // The hit rect, which is the frame's rect brought in by its insets.
        // Top and bottom are named the way WoW names them: top is the upper
        // edge, and y grows upward here, so it comes off bottom + height.
        const float hx0 = w.left + w.hitInsetLeft;
        const float hx1 = w.left + w.rectW - w.hitInsetRight;
        const float hy0 = w.bottom + w.hitInsetBottom;
        const float hy1 = w.bottom + w.rectH - w.hitInsetTop;
        if (hx1 <= hx0 || hy1 <= hy0) continue;  // inset to nothing: unclickable
        if (x < hx0 || x > hx1) continue;
        if (y < hy0 || y > hy1) continue;
        // Scrolled out of sight is out of reach. A scroll frame shows a window
        // onto a taller child, and the part of that child above or below the
        // window is not drawn - so it must not be clickable either, or a quest
        // log answers clicks on entries nobody can see.
        if (w.clipTo != 0) {
            const Widget* clip = get(w.clipTo);
            if (clip && (x < clip->left || x > clip->left + clip->rectW ||
                         y < clip->bottom || y > clip->bottom + clip->rectH)) {
                continue;
            }
        }
        if (!best) { best = &w; continue; }
        // Same comparison the draw order uses, read the other way round: the
        // last thing painted is the first thing clicked.
        const int sa = strataRank(w.effStrata), sb = strataRank(best->effStrata);
        if (sa != sb) { if (sa > sb) best = &w; continue; }
        if (w.effLevel != best->effLevel) { if (w.effLevel > best->effLevel) best = &w; continue; }
        if (w.creationOrder > best->creationOrder) best = &w;
    }
    return best ? best->id : 0;
}

bool WidgetTree::buttonArtVisible(const Widget& w) const {
    if (w.buttonArt == ButtonArt::None) return true;

    // The art belongs to the frame holding it, not to itself: it is the button
    // that is hovered, pressed or disabled.
    const Widget* owner = get(w.parent);
    if (!owner) return true;

    // Hovered counts for anything under the button too, since its own regions
    // sit on top of it and are what the cursor actually lands on.
    bool hovered = false;
    for (uint32_t at = hoveredId_; at != 0; ) {
        if (at == owner->id) { hovered = true; break; }
        const Widget* a = get(at);
        if (!a) break;
        at = a->parent;
    }
    bool pressed = false;
    for (uint32_t at = pressedId_; at != 0; ) {
        if (at == owner->id) { pressed = true; break; }
        const Widget* a = get(at);
        if (!a) break;
        at = a->parent;
    }

    // A state asked for outright wins over what the cursor is doing.
    switch (owner->forcedState) {
        case Widget::Forced::Pushed:   pressed = true;  break;
        case Widget::Forced::Normal:   pressed = false; break;
        case Widget::Forced::Disabled: break;
        case Widget::Forced::None:     break;
    }
    const bool usable = owner->enabled &&
                        owner->forcedState != Widget::Forced::Disabled;

    switch (w.buttonArt) {
        case ButtonArt::Highlight:       return (hovered || owner->highlightLocked) && usable;
        case ButtonArt::Disabled:        return !usable;
        case ButtonArt::Pushed:          return usable && pressed;
        case ButtonArt::Normal:          return usable && !pressed;
        case ButtonArt::Checked:         return owner->checked && usable;
        case ButtonArt::DisabledChecked: return owner->checked && !usable;
        default:                         return true;
    }
}

void WidgetTree::collectDrawOrder() {
    drawOrder_.clear();
    for (const Widget& w : widgets_) {
        if (w.id == 0) continue;
        if (!w.visible) continue;
        if (w.alpha <= 0.001f) continue;
        // Frames are containers, except when they carry a backdrop or are a
        // status bar - then the frame itself has something to paint, and it
        // paints underneath its own regions because they sit a level above it.
        // A frame the client renders into paints itself, the same as one with
        // a backdrop. The paperdoll's model frame is a frame, not a texture,
        // so without this the character would be rendered and never drawn.
        // An edit box paints its own text and its caret, the way a status bar
        // paints its fill and a message frame its lines - so it belongs with
        // them here and not with the containers.
        //
        // Without it the chat box was dropped as "a frame with nothing of its
        // own to paint": the say bar opened, took the focus and filled with
        // what was typed, and every character went into a widget that was never
        // drawn. The bar itself still appeared, because its art is child
        // textures and those draw on their own - so it looked like an empty box
        // rather than a missing one.
        //
        // Not gated on holding text. The caret is what says which box is
        // listening, and an empty box still has to show it.
        if (w.kind == WidgetKind::Frame && !w.hasBackdrop && !w.isStatusBar &&
            !w.isEditBox && w.externalTexture == 0 &&
            !(w.isSimpleHtml && !w.text.empty()) &&
            !(w.isMessageFrame && !w.messages.empty()) &&
            !(w.isTooltip && !w.tooltipLines.empty())) continue;
        if (w.rectW <= 0.0f || w.rectH <= 0.0f) continue;
        if (w.kind == WidgetKind::Texture && w.texturePath.empty() &&
            !w.solidColor && w.externalTexture == 0) continue;
        if (w.kind == WidgetKind::Frame && w.isStatusBar && w.barTexture.empty() &&
            !w.hasBackdrop) continue;
        if (w.kind == WidgetKind::FontString && w.text.empty()) continue;
        // A button shows one of its state textures, not all of them.
        if (!buttonArtVisible(w)) continue;
        drawOrder_.push_back(&w);
    }

    // A status bar draws its own fill, where the real client makes the fill a
    // region of the bar like any other. So the bar sorts where its fill
    // belongs rather than where the frame does: one level in, among the bar's
    // own regions, ranked by the layer the fill asked for.
    //
    // Without this the fill went under everything the bar owns, whatever it
    // declared - and a bar with a dark backing of its own wore it over the
    // fill. The cast bar is exactly that: a BACKGROUND backing, a BORDER fill
    // and ARTWORK border art, which has to come out in that order.
    // An edit box draws its own text, the way a status bar draws its own fill,
    // and it sorts the same way and for the same reason: one level in, among
    // its own regions.
    //
    // Left at the frame's own level the text goes under everything the box
    // owns - and a chat input box owns UI-ChatInputBorder-Mid2, a dark strip
    // across its whole width in the BACKGROUND layer. So what you typed was
    // painted at full white and then shaded to a third of it, while the "Say:"
    // header beside it stayed bright: that one is a font string in ARTWORK and
    // is drawn after the strip rather than under it. Measured at the pixel:
    // (255,255,255) for the header, (81,82,81) for the text.
    //
    // ARTWORK, so it lands above the box's background and border art and
    // alongside the header rather than over it - the two never overlap, the
    // insets see to that.
    auto sortLevel = [](const Widget* w) {
        return ((w->isStatusBar && !w->barTexture.empty()) || w->isEditBox)
                   ? w->effLevel + 1
                   : w->effLevel;
    };
    auto sortLayer = [](const Widget* w) {
        if (w->isEditBox) return layerRank(DrawLayer::Artwork);
        return layerRank((w->isStatusBar && !w->barTexture.empty()) ? w->barLayer : w->layer);
    };
    std::sort(drawOrder_.begin(), drawOrder_.end(),
              [&](const Widget* a, const Widget* b) {
                  const int sa = strataRank(a->effStrata), sb = strataRank(b->effStrata);
                  if (sa != sb) return sa < sb;
                  const int va = sortLevel(a), vb = sortLevel(b);
                  if (va != vb) return va < vb;
                  const int la = sortLayer(a), lb = sortLayer(b);
                  if (la != lb) return la < lb;
                  if (a->subLevel != b->subLevel) return a->subLevel < b->subLevel;
                  // Ties resolve by creation order, so a region added later sits
                  // on top of one added earlier - the same rule the real client
                  // uses within a layer.
                  return a->creationOrder < b->creationOrder;
              });
}

void hsvToRgb(const float hsv[3], float rgb[3]) {
    const float h = hsv[0] - std::floor(hsv[0]);   // one turn, wrapped
    const float s = std::clamp(hsv[1], 0.0f, 1.0f);
    const float v = std::clamp(hsv[2], 0.0f, 1.0f);
    // The wheel in six segments: within each, one channel is full, one is
    // rising or falling across the segment, and one is at the saturation floor.
    const float sector = h * 6.0f;
    const int i = static_cast<int>(sector) % 6;
    const float f = sector - std::floor(sector);
    const float p = v * (1.0f - s);
    const float q = v * (1.0f - s * f);
    const float t = v * (1.0f - s * (1.0f - f));
    switch (i) {
        case 0: rgb[0] = v; rgb[1] = t; rgb[2] = p; break;
        case 1: rgb[0] = q; rgb[1] = v; rgb[2] = p; break;
        case 2: rgb[0] = p; rgb[1] = v; rgb[2] = t; break;
        case 3: rgb[0] = p; rgb[1] = q; rgb[2] = v; break;
        case 4: rgb[0] = t; rgb[1] = p; rgb[2] = v; break;
        default: rgb[0] = v; rgb[1] = p; rgb[2] = q; break;
    }
}

void rgbToHsv(const float rgb[3], float hsv[3]) {
    const float r = rgb[0], g = rgb[1], b = rgb[2];
    const float hi = std::max(r, std::max(g, b));
    const float lo = std::min(r, std::min(g, b));
    hsv[2] = hi;
    const float span = hi - lo;
    hsv[1] = (hi > 0.0f) ? span / hi : 0.0f;
    if (span <= 0.0f) {
        // Grey has no hue. Zero rather than anything cleverer, and the caller
        // that cares keeps the hue it already had instead of asking.
        hsv[0] = 0.0f;
        return;
    }
    float h;
    if (hi == r)      h = (g - b) / span;
    else if (hi == g) h = 2.0f + (b - r) / span;
    else              h = 4.0f + (r - g) / span;
    h /= 6.0f;
    hsv[0] = h - std::floor(h);
}

} // namespace ui
} // namespace wowee
