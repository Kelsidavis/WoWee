#include "ui/paper_ui.hpp"

#include "ui/interface_fonts.hpp"
#include "ui/text_edit.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace wowee::ui {

namespace {

constexpr float kPi = 3.14159265358979f;

/// A number in [-1, 1] that depends only on `seed`.
///
/// The wobble in every hand-drawn line comes from here rather than from a
/// random number generator, because a line whose waver is redrawn each frame
/// does not look hand-drawn, it looks like it is boiling.
float jitterAt(uint32_t seed) {
    uint32_t x = seed * 747796405u + 2891336453u;
    uint32_t w = ((x >> ((x >> 28) + 4)) ^ x) * 277803737u;
    w = (w >> 22) ^ w;
    return static_cast<float>(w & 0xFFFFu) / 32767.5f - 1.0f;
}

/// A seed from where something is, so two controls of the same size waver
/// differently and neither of them changes shape while it sits still.
uint32_t seedFor(ImVec2 a, ImVec2 b) {
    const auto q = [](float v) { return static_cast<uint32_t>(static_cast<int>(v * 4.0f)); };
    return q(a.x) * 73856093u ^ q(a.y) * 19349663u ^ q(b.x) * 83492791u ^ q(b.y) * 2654435761u;
}

bool isContinuationByte(char c) { return (static_cast<unsigned char>(c) & 0xC0) == 0x80; }

/// How many characters precede `byte`. What a masked field counts, since it
/// draws one dot per character rather than one per byte.
size_t codepointsBefore(const std::string& s, size_t byte) {
    size_t n = 0;
    for (size_t i = 0; i < byte && i < s.size(); ++i)
        if (!isContinuationByte(s[i])) ++n;
    return n;
}

size_t byteForCodepoint(const std::string& s, size_t index) {
    size_t n = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        if (isContinuationByte(s[i])) continue;
        if (n == index) return i;
        ++n;
    }
    return s.size();
}

} // namespace

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

ImU32 PaperUI::hashId(const char* s) {
    // FNV-1a. The pointer would be cheaper and is not safe: identical string
    // literals in one translation unit are commonly folded to one address,
    // which would give two different controls the same identity.
    uint32_t h = 2166136261u;
    for (const char* p = s; *p; ++p) {
        h ^= static_cast<unsigned char>(*p);
        h *= 16777619u;
    }
    return h ? h : 1u;  // zero means "nothing"
}

void PaperUI::begin(float deltaSeconds, float scale) {
    scale_ = scale > 0.0f ? scale : 1.0f;
    page_ = ImGui::GetBackgroundDrawList();
    overlay_ = ImGui::GetForegroundDrawList();
    dl_ = page_;

    bodyFace_ = interfaceFace("frizqt__.ttf");
    if (!bodyFace_) bodyFace_ = ImGui::GetFont();
    titleFace_ = interfaceFace("morpheus.ttf");
    if (!titleFace_) titleFace_ = bodyFace_;

    blink_ += deltaSeconds;
    inert_ = 0;
    textInputWanted_ = false;
    mouseWanted_ = false;
    popups_.clear();
    firstField_ = lastField_ = 0;
    takeFocusNext_ = false;
    focusLastAtEnd_ = false;
}

void PaperUI::end() {
    // Tab off the last field wraps to the first, and Shift+Tab off the first
    // wraps to the last. Neither is known until every field has been drawn.
    if (takeFocusNext_) {
        focus_ = firstField_;
        justFocused_ = firstField_;
        takeFocusNext_ = false;
    }
    if (focusLastAtEnd_) {
        focus_ = lastField_;
        justFocused_ = lastField_;
        focusLastAtEnd_ = false;
    }

    for (const PopupDraw& p : popups_) {
        ImDrawList* dl = overlay_;
        dl->AddRectFilled(ImVec2(p.a.x + px(3), p.a.y + px(4)),
                          ImVec2(p.b.x + px(3), p.b.y + px(4)), theme_.shadow, px(3));
        dl->AddRectFilled(p.a, p.b, theme_.paperTop, px(3));
        dl->AddRect(p.a, p.b, theme_.paperEdge, px(3), 0, px(1));

        for (int i = 0; i < static_cast<int>(p.items.size()); ++i) {
            const float y0 = p.a.y + px(4) + static_cast<float>(i) * p.rowHeight;
            const float y1 = y0 + p.rowHeight;
            if (i == p.hovered) {
                // A highlighter stroke rather than a selection bar: it stops
                // short of the edges and does not quite line up with them.
                dl->AddRectFilled(ImVec2(p.a.x + px(3), y0 + px(1.5f)),
                                  ImVec2(p.b.x - px(4), y1 - px(1.5f)),
                                  theme_.highlighter, px(2));
            }
            const float size = p.rowHeight * 0.62f;
            const ImU32 col = (i == p.selected) ? theme_.ink : theme_.inkSoft;
            dl->AddText(bodyFace_, size, ImVec2(p.a.x + px(20), y0 + (p.rowHeight - size) * 0.5f),
                        col, p.items[i].c_str());
            if (i == p.selected) {
                // A tick in the margin, the way a chosen line gets marked.
                const float cy = y0 + p.rowHeight * 0.5f;
                const float cx = p.a.x + px(10);
                dl->AddLine(ImVec2(cx - px(3.5f), cy), ImVec2(cx - px(1.0f), cy + px(3.0f)),
                            theme_.crayonRed, px(1.6f));
                dl->AddLine(ImVec2(cx - px(1.0f), cy + px(3.0f)), ImVec2(cx + px(4.0f), cy - px(3.5f)),
                            theme_.crayonRed, px(1.6f));
            }
        }
        wobblyRect(dl, ImVec2(p.a.x + px(2), p.a.y + px(2)), ImVec2(p.b.x - px(2), p.b.y - px(2)),
                   theme_.ink, px(1.3f), px(0.7f), seedFor(p.a, p.b));
    }
    popups_.clear();

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        active_ = 0;
        dragging_ = 0;
        pressSwallowed_ = false;
    }

    if (textInputWanted_) {
        // What raises the on-screen keyboard on Android: UIManager reads this
        // after the frame is built. Nothing else sets it now that the login
        // screen has no ImGui text box in it.
        ImGui::GetIO().WantTextInput = true;
        ImGui::SetNextFrameWantCaptureKeyboard(true);
    }
    if (mouseWanted_) ImGui::SetNextFrameWantCaptureMouse(true);
}

void PaperUI::setLayer(PaperLayer layer) {
    dl_ = (layer == PaperLayer::Overlay) ? overlay_ : page_;
}

void PaperUI::pushInert() { ++inert_; }
void PaperUI::popInert() { if (inert_ > 0) --inert_; }

void PaperUI::claimMouse(ImVec2 a, ImVec2 b) {
    if (inert_ == 0 && hovered(a, b)) mouseWanted_ = true;
}

ImFont* PaperUI::face(bool titleFace) const { return titleFace ? titleFace_ : bodyFace_; }

bool PaperUI::pressed() const {
    return !pressSwallowed_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
}

bool PaperUI::hovered(ImVec2 a, ImVec2 b) {
    const ImVec2 m = ImGui::GetIO().MousePos;
    return m.x >= a.x && m.x < b.x && m.y >= a.y && m.y < b.y;
}

PaperUI::FieldState& PaperUI::fieldState(ImU32 id) {
    for (auto& entry : fieldStates_)
        if (entry.first == id) return entry.second;
    fieldStates_.emplace_back(id, FieldState{});
    return fieldStates_.back().second;
}

void PaperUI::focus(const char* id) {
    focus_ = hashId(id);
    justFocused_ = focus_;
    resetBlink();
}

void PaperUI::clearFocus() {
    focus_ = 0;
    justFocused_ = 0;
}

// ---------------------------------------------------------------------------
// Ornament
// ---------------------------------------------------------------------------

void PaperUI::rule(ImVec2 a, ImVec2 b, ImU32 col, float thickness, uint32_t seed,
                   float amplitude) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.5f) return;
    const float nx = -dy / len;
    const float ny = dx / len;
    if (amplitude <= 0.0f) amplitude = std::max(px(0.5f), thickness * 0.55f);

    const int segments = std::clamp(static_cast<int>(len / px(14.0f)), 2, 28);
    ImVec2 points[30];
    for (int i = 0; i <= segments; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(segments);
        // Tapered to nothing at both ends, so a line still starts and stops
        // where it was asked to even though the middle of it wanders.
        const float taper = std::sin(t * kPi);
        const float off = amplitude * taper * jitterAt(seed * 131u + static_cast<uint32_t>(i));
        points[i] = ImVec2(a.x + dx * t + nx * off, a.y + dy * t + ny * off);
    }
    dl_->AddPolyline(points, segments + 1, col, 0, thickness);
}

void PaperUI::squiggle(ImVec2 a, ImVec2 b, ImU32 col, float thickness, uint32_t seed) {
    // Two passes, the second shorter and offset - a pen gone back over a line
    // it had already drawn.
    rule(a, b, col, thickness, seed);
    const float inset = (b.x - a.x) * 0.12f;
    rule(ImVec2(a.x + inset, a.y + thickness * 1.6f),
         ImVec2(b.x - inset * 0.4f, b.y + thickness * 1.9f), paperFade(col, 0.45f),
         thickness * 0.8f, seed + 7919u);
}

void PaperUI::wobblyRect(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float thickness,
                         float amplitude, uint32_t seed) {
    ImDrawList* keep = dl_;
    dl_ = dl;
    // Each side drawn past the corner, the way a hand overshoots one.
    const float over = amplitude * 2.0f + px(1.0f);
    rule(ImVec2(a.x - over, a.y), ImVec2(b.x + over, a.y), col, thickness, seed + 1u, amplitude);
    rule(ImVec2(b.x, a.y - over), ImVec2(b.x, b.y + over), col, thickness, seed + 2u, amplitude);
    rule(ImVec2(b.x + over, b.y), ImVec2(a.x - over, b.y), col, thickness, seed + 3u, amplitude);
    rule(ImVec2(a.x, b.y + over), ImVec2(a.x, a.y - over), col, thickness, seed + 4u, amplitude);
    dl_ = keep;
}

void PaperUI::sheet(ImVec2 a, ImVec2 b, bool taped) { drawSheetBody(dl_, a, b, taped); }

void PaperUI::drawSheetBody(ImDrawList* dl, ImVec2 a, ImVec2 b, bool taped) {
    // The shadow, as a handful of expanding rectangles rather than a blur.
    for (int i = 5; i >= 1; --i) {
        const float spread = px(static_cast<float>(i) * 1.8f);
        const float drop = px(static_cast<float>(i) * 0.9f);
        dl->AddRectFilled(ImVec2(a.x - spread, a.y - spread * 0.4f + drop),
                          ImVec2(b.x + spread, b.y + spread * 0.7f + drop),
                          paperFade(theme_.shadow, 0.045f), px(6));
    }

    dl->AddRectFilled(a, b, theme_.paperTop, px(2));
    // Paper is never one flat colour; it goes warmer towards the bottom.
    dl->AddRectFilledMultiColor(ImVec2(a.x, a.y + (b.y - a.y) * 0.30f), b,
                                paperFade(theme_.paperBottom, 0.0f),
                                paperFade(theme_.paperBottom, 0.0f),
                                theme_.paperBottom, theme_.paperBottom);
    dl->AddRect(a, b, theme_.paperEdge, px(2), 0, px(1));

    // The drawn border: one firm line and one lighter one just inside it.
    const uint32_t seed = seedFor(a, b);
    wobblyRect(dl, ImVec2(a.x + px(8), a.y + px(8)), ImVec2(b.x - px(8), b.y - px(8)),
               theme_.ink, px(1.7f), px(0.9f), seed);
    wobblyRect(dl, ImVec2(a.x + px(11.5f), a.y + px(11.5f)), ImVec2(b.x - px(11.5f), b.y - px(11.5f)),
               paperFade(theme_.ink, 0.35f), px(0.9f), px(0.7f), seed + 4441u);

    if (!taped) return;
    const float length = px(56);
    const float width = px(17);
    const auto strip = [&](ImVec2 center, float angle) {
        const float ca = std::cos(angle);
        const float sa = std::sin(angle);
        const auto pt = [&](float x, float y) {
            return ImVec2(center.x + x * ca - y * sa, center.y + x * sa + y * ca);
        };
        const ImVec2 p0 = pt(-length * 0.5f, -width * 0.5f);
        const ImVec2 p1 = pt(length * 0.5f, -width * 0.5f);
        const ImVec2 p2 = pt(length * 0.5f, width * 0.5f);
        const ImVec2 p3 = pt(-length * 0.5f, width * 0.5f);
        dl->AddQuadFilled(p0, p1, p2, p3, IM_COL32(0xFF, 0xFC, 0xF0, 0x62));
        dl->AddLine(p0, p1, IM_COL32(0xFF, 0xFF, 0xFF, 0x55), px(1));
        dl->AddLine(p3, p2, IM_COL32(0x6B, 0x58, 0x3C, 0x33), px(1));
    };
    strip(ImVec2(a.x, a.y), -kPi * 0.25f);
    strip(ImVec2(b.x, a.y), kPi * 0.25f);
}

void PaperUI::scrim(float alpha) {
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    dl_->AddRectFilled(ImVec2(0, 0), screen, IM_COL32(0x20, 0x16, 0x0C,
                       static_cast<int>(std::clamp(alpha, 0.0f, 1.0f) * 255.0f)));
}

// ---------------------------------------------------------------------------
// Text
// ---------------------------------------------------------------------------

float PaperUI::textWidth(const char* s, float size, bool titleFace) const {
    if (!s || !*s) return 0.0f;
    ImFont* f = face(titleFace);
    if (!f) return 0.0f;
    return f->CalcTextSizeA(size, FLT_MAX, 0.0f, s).x;
}

float PaperUI::lineHeight(float size, bool) const { return size * 1.28f; }

/// How far the ink reaches below the top of a line drawn at `size`.
///
/// A size is an em, which is not what the glyphs occupy: FRIZQT spans 1215
/// units of a 1000-unit em and Morpheus more still, and the atlas is rasterized
/// with that correction so the face draws at its true proportions. So a rule
/// placed one size below the top of a title sits inside the title's own
/// descenders rather than under them - which is where the red line under every
/// heading on the login screens went when the correction arrived.
///
/// Ascent to descent off the baked face, which carries the correction, with the
/// em as the answer for a face that has not baked.
float PaperUI::inkHeight(float size, bool titleFace) const {
    ImFont* f = face(titleFace);
    if (!f) return size;
    ImFontBaked* baked = f->GetFontBaked(size);
    if (!baked || baked->Size <= 0.0f) return size;
    return (baked->Ascent - baked->Descent) * (size / baked->Size);
}

void PaperUI::text(ImVec2 at, const char* s, float size, ImU32 col, bool titleFace) {
    if (!s || !*s) return;
    dl_->AddText(face(titleFace), size, at, col, s);
}

void PaperUI::textCentered(float cx, float y, const char* s, float size, ImU32 col,
                           bool titleFace) {
    text(ImVec2(cx - textWidth(s, size, titleFace) * 0.5f, y), s, size, col, titleFace);
}

void PaperUI::textRight(float rx, float y, const char* s, float size, ImU32 col,
                        bool titleFace) {
    text(ImVec2(rx - textWidth(s, size, titleFace), y), s, size, col, titleFace);
}

namespace {

/// Breaks `s` into lines no wider than `width`, calling `emit` for each.
template <typename Emit, typename Measure>
float layOutWrapped(const char* s, float width, float lineStep, const Measure& measure,
                    const Emit& emit) {
    if (!s || !*s) return 0.0f;
    std::string line;
    std::string word;
    float y = 0.0f;
    const auto flush = [&]() {
        if (line.empty()) return;
        emit(line, y);
        y += lineStep;
        line.clear();
    };
    const char* p = s;
    while (true) {
        const bool atEnd = (*p == '\0');
        if (atEnd || *p == ' ' || *p == '\n') {
            if (!word.empty()) {
                const std::string candidate = line.empty() ? word : line + " " + word;
                if (!line.empty() && measure(candidate.c_str()) > width) {
                    flush();
                    line = word;
                } else {
                    line = candidate;
                }
                word.clear();
            }
            if (!atEnd && *p == '\n') flush();
            if (atEnd) break;
        } else {
            word.push_back(*p);
        }
        ++p;
    }
    flush();
    return y;
}

} // namespace

float PaperUI::wrapped(ImVec2 at, float width, const char* s, float size, ImU32 col) {
    const float step = lineHeight(size);
    ImFont* f = bodyFace_;
    return layOutWrapped(
        s, width, step,
        [&](const char* t) { return f ? f->CalcTextSizeA(size, FLT_MAX, 0.0f, t).x : 0.0f; },
        [&](const std::string& line, float y) {
            dl_->AddText(f, size, ImVec2(at.x, at.y + y), col, line.c_str());
        });
}

float PaperUI::wrappedHeight(float width, const char* s, float size) const {
    const float step = lineHeight(size);
    ImFont* f = bodyFace_;
    return layOutWrapped(
        s, width, step,
        [&](const char* t) { return f ? f->CalcTextSizeA(size, FLT_MAX, 0.0f, t).x : 0.0f; },
        [](const std::string&, float) {});
}

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------

bool PaperUI::button(const char* idStr, ImVec2 a, ImVec2 b, const char* label,
                     ButtonKind kind, bool enabled) {
    const ImU32 id = hashId(idStr);
    const bool live = listening() && enabled;
    const bool over = live && hovered(a, b);
    if (over) mouseWanted_ = true;

    bool clicked = false;
    if (over && pressed()) active_ = id;
    const bool held = (active_ == id && ImGui::IsMouseDown(ImGuiMouseButton_Left));
    if (active_ == id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        clicked = over;
        active_ = 0;
    }

    // Pressing pushes the button into the paper rather than moving it.
    const float sink = held ? px(1.5f) : 0.0f;
    const ImVec2 pa(a.x, a.y + sink);
    const ImVec2 pb(b.x, b.y + sink);
    const float rounding = px(5);
    const uint32_t seed = seedFor(a, b);
    const float size = (b.y - a.y) * 0.44f;
    const float textY = pa.y + ((pb.y - pa.y) - size) * 0.5f - px(1);
    const float cx = (pa.x + pb.x) * 0.5f;

    if (kind == ButtonKind::Primary) {
        if (!enabled) {
            dl_->AddRectFilled(pa, pb, paperFade(theme_.pencil, 0.35f), rounding);
            wobblyRect(dl_, pa, pb, paperFade(theme_.pencil, 0.7f), px(1.4f), px(0.8f), seed);
            textCentered(cx, textY, label, size, paperFade(theme_.pencil, 0.9f));
            return false;
        }
        if (!held) {
            dl_->AddRectFilled(ImVec2(pa.x + px(1.5f), pa.y + px(3)),
                               ImVec2(pb.x + px(1.5f), pb.y + px(3)),
                               paperFade(theme_.shadow, 0.22f), rounding);
        }
        const ImU32 fill = over ? theme_.crayonRedLit : theme_.crayonRed;
        dl_->AddRectFilled(pa, pb, fill, rounding);
        // Wax builds up at the edges of a crayon stroke and thins in the
        // middle; two translucent bands are enough to suggest it.
        dl_->AddRectFilledMultiColor(ImVec2(pa.x, pa.y + (pb.y - pa.y) * 0.45f), pb,
                                     paperFade(theme_.crayonRedDim, 0.0f),
                                     paperFade(theme_.crayonRedDim, 0.0f),
                                     paperFade(theme_.crayonRedDim, 0.55f),
                                     paperFade(theme_.crayonRedDim, 0.55f));
        wobblyRect(dl_, pa, pb, theme_.crayonRedDim, px(2.0f), px(1.0f), seed);
        textCentered(cx, textY, label, size, theme_.onCrayon);
        return clicked;
    }

    const ImU32 line = enabled ? theme_.ink : theme_.pencil;
    if (over) dl_->AddRectFilled(pa, pb, paperFade(theme_.ink, 0.07f), rounding);
    wobblyRect(dl_, pa, pb, paperFade(line, over ? 1.0f : 0.75f), px(1.4f), px(0.9f), seed);
    textCentered(cx, textY, label, size, enabled ? theme_.ink : theme_.pencil);
    return clicked;
}

float PaperUI::chipWidth(const char* label, float height) const {
    return textWidth(label, height * 0.46f) + height * 0.86f;
}

bool PaperUI::chip(const char* idStr, ImVec2 a, ImVec2 b, const char* label, bool selected,
                   ImU32 accent) {
    const ImU32 id = hashId(idStr);
    const bool live = listening();
    const bool over = live && hovered(a, b);
    if (over) mouseWanted_ = true;

    bool clicked = false;
    if (over && pressed()) active_ = id;
    const bool held = (active_ == id && ImGui::IsMouseDown(ImGuiMouseButton_Left));
    if (active_ == id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        clicked = over;
        active_ = 0;
    }

    const float h = b.y - a.y;
    const float sink = held ? px(1.0f) : 0.0f;
    const ImVec2 pa(a.x, a.y + sink);
    const ImVec2 pb(b.x, b.y + sink);
    const float rounding = h * 0.5f;
    const float size = h * 0.46f;
    const float textY = pa.y + (h - size) * 0.5f - px(1);
    const float cx = (pa.x + pb.x) * 0.5f;
    const uint32_t seed = seedFor(a, b);

    if (selected) {
        dl_->AddRectFilled(pa, pb, over ? paperFade(accent, 0.92f) : accent, rounding);
        wobblyRect(dl_, pa, pb, paperFade(theme_.ink, 0.6f), px(1.4f), px(0.8f), seed);
        textCentered(cx, textY, label, size, theme_.onCrayon);
        return clicked;
    }

    if (over) dl_->AddRectFilled(pa, pb, paperFade(accent, 0.16f), rounding);
    wobblyRect(dl_, pa, pb, over ? theme_.ink : paperFade(theme_.pencil, 0.95f), px(1.2f),
               px(0.8f), seed);
    textCentered(cx, textY, label, size, over ? theme_.ink : theme_.inkSoft);
    return clicked;
}

void PaperUI::drawGlyph(ImDrawList* dl, ImVec2 c, float r, Glyph glyph, ImU32 col) {
    const float t = std::max(px(1.4f), r * 0.16f);
    switch (glyph) {
        case Glyph::Gear: {
            dl->AddCircle(c, r * 0.52f, col, 0, t);
            for (int i = 0; i < 8; ++i) {
                const float ang = static_cast<float>(i) * (kPi * 0.25f);
                const ImVec2 in(c.x + std::cos(ang) * r * 0.58f, c.y + std::sin(ang) * r * 0.58f);
                const ImVec2 out(c.x + std::cos(ang) * r * 0.98f, c.y + std::sin(ang) * r * 0.98f);
                dl->AddLine(in, out, col, t);
            }
            dl->AddCircleFilled(c, r * 0.16f, col);
            break;
        }
        case Glyph::Cross: {
            const float k = r * 0.62f;
            dl->AddLine(ImVec2(c.x - k, c.y - k), ImVec2(c.x + k, c.y + k), col, t);
            dl->AddLine(ImVec2(c.x + k, c.y - k), ImVec2(c.x - k, c.y + k), col, t);
            break;
        }
        case Glyph::Eye:
        case Glyph::EyeClosed: {
            const float w = r * 0.95f;
            const float h = r * 0.62f;
            dl->PathClear();
            dl->PathLineTo(ImVec2(c.x - w, c.y));
            dl->PathBezierQuadraticCurveTo(ImVec2(c.x, c.y - h * 1.7f), ImVec2(c.x + w, c.y), 12);
            dl->PathBezierQuadraticCurveTo(ImVec2(c.x, c.y + h * 1.7f), ImVec2(c.x - w, c.y), 12);
            dl->PathStroke(col, 0, t);
            dl->AddCircleFilled(c, r * 0.24f, col);
            if (glyph == Glyph::EyeClosed) {
                dl->AddLine(ImVec2(c.x - w * 0.95f, c.y + h * 0.95f),
                            ImVec2(c.x + w * 0.95f, c.y - h * 0.95f), col, t * 1.15f);
            }
            break;
        }
    }
}

bool PaperUI::glyphButton(const char* idStr, ImVec2 center, float radius, Glyph glyph) {
    const ImU32 id = hashId(idStr);
    const ImVec2 a(center.x - radius, center.y - radius);
    const ImVec2 b(center.x + radius, center.y + radius);
    const bool live = listening();
    const bool over = live && hovered(a, b);
    if (over) mouseWanted_ = true;

    bool clicked = false;
    if (over && pressed()) active_ = id;
    if (active_ == id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        clicked = over;
        active_ = 0;
    }
    const bool held = (active_ == id && ImGui::IsMouseDown(ImGuiMouseButton_Left));

    if (over) dl_->AddCircleFilled(center, radius, paperFade(theme_.ink, held ? 0.16f : 0.09f));
    drawGlyph(dl_, center, radius * 0.72f, glyph,
              over ? theme_.ink : paperFade(theme_.inkSoft, 0.85f));
    return clicked;
}

bool PaperUI::link(const char* idStr, ImVec2 at, const char* label, float size, ImU32 col) {
    const ImU32 id = hashId(idStr);
    const float w = textWidth(label, size);
    const ImVec2 a(at.x, at.y);
    const ImVec2 b(at.x + w, at.y + lineHeight(size));
    const bool live = listening();
    const bool over = live && hovered(a, b);
    if (over) mouseWanted_ = true;

    bool clicked = false;
    if (over && pressed()) active_ = id;
    if (active_ == id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        clicked = over;
        active_ = 0;
    }

    const ImU32 base = col ? col : theme_.inkSoft;
    const ImU32 drawn = over ? theme_.crayonRed : base;
    text(at, label, size, drawn);
    // Underlined by hand, and only while it is being pointed at.
    if (over) {
        rule(ImVec2(a.x, b.y - px(2)), ImVec2(b.x, b.y - px(2)), paperFade(drawn, 0.8f),
             px(1.1f), hashId(label));
    }
    return clicked;
}

// ---------------------------------------------------------------------------
// Fields
// ---------------------------------------------------------------------------

float PaperUI::advanceTo(const TextEdit& edit, size_t byte, bool password, float size) const {
    if (password) {
        return static_cast<float>(codepointsBefore(edit.text(), byte)) * (size * 0.60f);
    }
    if (byte == 0 || !bodyFace_) return 0.0f;
    const char* begin = edit.text().c_str();
    return bodyFace_->CalcTextSizeA(size, FLT_MAX, 0.0f, begin, begin + byte).x;
}

size_t PaperUI::byteAtOffset(const TextEdit& edit, float dx, bool password, float size) const {
    const std::string& s = edit.text();
    if (s.empty()) return 0;
    if (password) {
        const float step = size * 0.60f;
        const size_t count = edit.codepointCount();
        const long idx = std::lround(dx / std::max(step, 0.001f));
        const size_t clamped = static_cast<size_t>(std::clamp<long>(idx, 0, static_cast<long>(count)));
        return byteForCodepoint(s, clamped);
    }
    // Walk the boundaries and take the nearest. The string is a login field,
    // so there are at most a few hundred of them.
    size_t best = 0;
    float bestDelta = std::abs(dx);
    for (size_t at = edit.nextBoundary(0); ; at = edit.nextBoundary(at)) {
        const float delta = std::abs(advanceTo(edit, at, false, size) - dx);
        if (delta < bestDelta) {
            bestDelta = delta;
            best = at;
        }
        if (at >= s.size()) break;
    }
    return best;
}

void PaperUI::drawFieldText(ImDrawList* dl, ImVec2 in0, ImVec2 in1, const TextEdit& edit,
                            const FieldOpts& opts, FieldState& state, bool focused,
                            float size) {
    const float baseY = in0.y + ((in1.y - in0.y) - size) * 0.5f - px(1);
    const float originX = in0.x - state.scroll;

    dl->PushClipRect(in0, in1, true);

    if (focused && edit.hasSelection()) {
        const float x0 = originX + advanceTo(edit, edit.selectionBegin(), opts.password, size);
        const float x1 = originX + advanceTo(edit, edit.selectionEnd(), opts.password, size);
        dl->AddRectFilled(ImVec2(x0 - px(1), baseY - px(2)), ImVec2(x1 + px(1), baseY + size + px(2)),
                          theme_.highlighter, px(2));
    }

    if (edit.empty() && opts.placeholder && !focused) {
        dl->AddText(bodyFace_, size, ImVec2(in0.x, baseY), paperFade(theme_.pencil, 0.85f),
                    opts.placeholder);
    } else if (opts.password) {
        // Drawn rather than typeset: the bullet is not in every face the
        // client might have loaded, and a row of missing glyphs is worse than
        // a row of circles.
        const float step = size * 0.60f;
        const float radius = size * 0.155f;
        const size_t count = edit.codepointCount();
        for (size_t i = 0; i < count; ++i) {
            const float cx = originX + (static_cast<float>(i) + 0.5f) * step;
            if (cx < in0.x - step || cx > in1.x + step) continue;
            dl->AddCircleFilled(ImVec2(cx, baseY + size * 0.55f), radius, theme_.ink, 12);
        }
    } else {
        dl->AddText(bodyFace_, size, ImVec2(originX, baseY), theme_.ink, edit.c_str());
    }

    if (focused) {
        // Roughly the rhythm every other caret blinks at.
        const float phase = std::fmod(blink_, 1.06f);
        if (phase < 0.62f) {
            const float cx = originX + advanceTo(edit, edit.caret(), opts.password, size);
            dl->AddRectFilled(ImVec2(cx, baseY - px(2)),
                              ImVec2(cx + px(1.6f), baseY + size + px(2)), theme_.ink);
        }
    }

    dl->PopClipRect();
}

PaperUI::FieldResult PaperUI::field(const char* idStr, ImVec2 a, ImVec2 b, TextEdit& edit,
                                    const FieldOpts& opts) {
    const ImU32 id = hashId(idStr);
    FieldResult out;
    FieldState& state = fieldState(id);

    const ImU32 drawnBefore = lastField_;
    if (firstField_ == 0) firstField_ = id;
    lastField_ = id;
    if (takeFocusNext_) {
        focus_ = id;
        justFocused_ = id;
        takeFocusNext_ = false;
        resetBlink();
    }

    // Focus arriving from Tab or from the screen itself, rather than from a
    // click. Only then does a field offer its contents up for replacement -
    // a click is a request to put the caret somewhere particular.
    if (focus_ == id && justFocused_ == id) {
        justFocused_ = 0;
        if (opts.selectAllOnFocus) edit.selectAll();
    }

    const bool live = listening();
    const float pad = px(9);
    const float size = (b.y - a.y) * 0.46f;
    const ImVec2 in0(a.x + pad, a.y);
    const ImVec2 in1(b.x - pad, b.y);
    const float innerW = std::max(in1.x - in0.x, px(8));

    ImGuiIO& io = ImGui::GetIO();
    const bool over = live && hovered(a, b);
    if (over) mouseWanted_ = true;

    if (over && pressed()) {
        focus_ = id;
        justFocused_ = 0;
        active_ = id;
        dragging_ = id;
        const float dx = io.MousePos.x - in0.x + state.scroll;
        const int clicks = ImGui::GetMouseClickedCount(ImGuiMouseButton_Left);
        if (clicks >= 3) {
            edit.selectAll();
        } else if (clicks == 2) {
            edit.selectWordAt(byteAtOffset(edit, dx, opts.password, size));
        } else {
            edit.setCaret(byteAtOffset(edit, dx, opts.password, size), io.KeyShift);
        }
        resetBlink();
    } else if (live && pressed() && focus_ == id) {
        // A click anywhere else puts the keyboard down.
        focus_ = 0;
    }

    if (dragging_ == id && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        // Sweeping past either edge scrolls the text under the pointer.
        if (io.MousePos.x < in0.x) state.scroll -= px(320) * io.DeltaTime;
        else if (io.MousePos.x > in1.x) state.scroll += px(320) * io.DeltaTime;
        const float dx = std::clamp(io.MousePos.x, in0.x, in1.x) - in0.x + state.scroll;
        edit.setCaret(byteAtOffset(edit, dx, opts.password, size), true);
        resetBlink();
    }

    const bool focused = (focus_ == id);
    out.focused = focused;

    if (focused && live) {
        textInputWanted_ = true;

        const bool shift = io.KeyShift;
        const bool shortcut = io.KeyCtrl || io.KeySuper;
        // Word motion is Alt+Arrow on macOS and Ctrl+Arrow elsewhere; taking
        // both costs nothing and neither platform has another meaning for the
        // one it does not use.
        const bool word = io.KeyAlt || (io.KeyCtrl && !io.KeySuper);
        const bool lineJump = io.KeySuper;  // Cmd+Arrow, the macOS Home/End
        const auto pressed = [](ImGuiKey k) { return ImGui::IsKeyPressed(k, true); };

        if (ImGui::IsKeyPressed(ImGuiKey_Tab, false)) {
            if (shift) {
                if (drawnBefore != 0) {
                    focus_ = drawnBefore;
                    justFocused_ = drawnBefore;
                } else {
                    focusLastAtEnd_ = true;
                }
            } else {
                focus_ = 0;
                takeFocusNext_ = true;
            }
            resetBlink();
        }

        if (pressed(ImGuiKey_LeftArrow)) {
            if (lineJump) edit.moveHome(shift); else edit.moveLeft(shift, word);
            resetBlink();
        }
        if (pressed(ImGuiKey_RightArrow)) {
            if (lineJump) edit.moveEnd(shift); else edit.moveRight(shift, word);
            resetBlink();
        }
        if (pressed(ImGuiKey_Home)) { edit.moveHome(shift); resetBlink(); }
        if (pressed(ImGuiKey_End)) { edit.moveEnd(shift); resetBlink(); }
        if (pressed(ImGuiKey_Backspace)) {
            out.changed |= edit.backspace(word);
            resetBlink();
        }
        if (pressed(ImGuiKey_Delete)) {
            out.changed |= edit.deleteForward(word);
            resetBlink();
        }
        if (pressed(ImGuiKey_Enter) || pressed(ImGuiKey_KeypadEnter)) out.submitted = true;

        if (shortcut && ImGui::IsKeyPressed(ImGuiKey_A, false)) {
            edit.selectAll();
            resetBlink();
        }
        if (shortcut && (ImGui::IsKeyPressed(ImGuiKey_C, false) ||
                         ImGui::IsKeyPressed(ImGuiKey_X, false))) {
            // A masked field never puts its contents on the clipboard. Cut
            // still removes what was selected - refusing that as well would
            // read as the key being broken.
            if (!opts.password && edit.hasSelection())
                ImGui::SetClipboardText(edit.selectedText().c_str());
            if (ImGui::IsKeyPressed(ImGuiKey_X, false)) out.changed |= edit.deleteSelection();
            resetBlink();
        }
        if (shortcut && ImGui::IsKeyPressed(ImGuiKey_V, false)) {
            if (const char* clip = ImGui::GetClipboardText()) {
                if (opts.digitsOnly) {
                    std::string digits;
                    for (const char* p = clip; *p; ++p)
                        if (*p >= '0' && *p <= '9') digits.push_back(*p);
                    out.changed |= edit.insert(digits);
                } else {
                    out.changed |= edit.insert(clip);
                }
            }
            resetBlink();
        }

        // AltGr is Ctrl+Alt on Windows and does produce characters, so only a
        // shortcut without Alt in it suppresses typing.
        const bool suppressTyping = (io.KeyCtrl && !io.KeyAlt) || io.KeySuper;
        if (!suppressTyping) {
            for (ImWchar c : io.InputQueueCharacters) {
                if (opts.digitsOnly && !(c >= '0' && c <= '9')) continue;
                out.changed |= edit.insertCodepoint(static_cast<unsigned int>(c));
            }
            if (!io.InputQueueCharacters.empty()) resetBlink();
        }
    }

    // Keep the caret in view, and show the start of the text when the field
    // is not the one being typed into.
    const float caretX = advanceTo(edit, edit.caret(), opts.password, size);
    const float fullW = advanceTo(edit, edit.size(), opts.password, size);
    if (focused) {
        if (caretX - state.scroll < 0.0f) state.scroll = caretX;
        if (caretX - state.scroll > innerW) state.scroll = caretX - innerW;
        state.scroll = std::clamp(state.scroll, 0.0f, std::max(0.0f, fullW - innerW + px(2)));
    } else {
        state.scroll = 0.0f;
    }

    // The box: a form field ruled on paper, firmer when it is the live one.
    const uint32_t seed = seedFor(a, b);
    dl_->AddRectFilled(a, b, theme_.fieldFill, px(3));
    if (focused) dl_->AddRectFilled(a, b, paperFade(theme_.highlighter, 0.14f), px(3));
    drawFieldText(dl_, in0, in1, edit, opts, state, focused, size);
    // The ruled line the writing sits on.
    rule(ImVec2(in0.x, b.y - px(6)), ImVec2(in1.x, b.y - px(6)),
         paperFade(theme_.crayonBlue, focused ? 0.55f : 0.3f), px(1.0f), seed + 991u);
    wobblyRect(dl_, a, b, focused ? theme_.ink : paperFade(theme_.pencil, 0.9f),
               focused ? px(1.9f) : px(1.2f), px(0.9f), seed);
    return out;
}

// ---------------------------------------------------------------------------
// Lists and pictures
// ---------------------------------------------------------------------------

ImU32 PaperUI::onPaper(ImVec4 colour) const {
    const auto channel = [](ImU32 c, int shift) {
        return static_cast<float>((c >> shift) & 0xFFu);
    };
    const auto blend = [&](float value, int shift) {
        const float raw = std::clamp(value, 0.0f, 1.0f) * 255.0f;
        const float mixed = raw * 0.62f + channel(theme_.ink, shift) * 0.38f;
        return static_cast<ImU32>(std::clamp(mixed, 0.0f, 255.0f)) << shift;
    };
    return blend(colour.x, IM_COL32_R_SHIFT) | blend(colour.y, IM_COL32_G_SHIFT) |
           blend(colour.z, IM_COL32_B_SHIFT) | (0xFFu << IM_COL32_A_SHIFT);
}

bool PaperUI::hover(ImVec2 a, ImVec2 b) {
    if (inert_ != 0) return false;
    if (!hovered(a, b)) return false;
    mouseWanted_ = true;
    return true;
}

void PaperUI::image(ImTextureID texture, ImVec2 a, ImVec2 b) {
    const float mat = px(9);
    const ImVec2 ma(a.x - mat, a.y - mat);
    const ImVec2 mb(b.x + mat, b.y + mat);

    // A soft shadow rather than a hard one: the point of the mount is to ease
    // the step down to the picture, and a heavy edge under it puts one back.
    dl_->AddRectFilled(ImVec2(ma.x + px(2), ma.y + px(3)), ImVec2(mb.x + px(2), mb.y + px(3)),
                       paperFade(theme_.shadow, 0.22f), px(2));
    dl_->AddRectFilled(ma, mb, theme_.photoMat, px(2));

    dl_->AddImage(texture, a, b, ImVec2(0, 0), ImVec2(1, 1), theme_.photoTint);
    if (theme_.photoFade > 0.0f) {
        dl_->AddRectFilled(a, b, paperFade(theme_.paperTop, theme_.photoFade));
    }

    // Feather the four edges into the mount. Four gradient strips; the corners
    // take two of them and fade a little further, which is what a corner
    // should do anyway.
    const float feather = std::min(px(12), std::min(b.x - a.x, b.y - a.y) * 0.25f);
    if (feather > 0.5f) {
        const ImU32 edge = paperFade(theme_.photoMat, 0.85f);
        const ImU32 gone = paperFade(theme_.photoMat, 0.0f);
        dl_->AddRectFilledMultiColor(a, ImVec2(b.x, a.y + feather), edge, edge, gone, gone);
        dl_->AddRectFilledMultiColor(ImVec2(a.x, b.y - feather), b, gone, gone, edge, edge);
        dl_->AddRectFilledMultiColor(a, ImVec2(a.x + feather, b.y), edge, gone, gone, edge);
        dl_->AddRectFilledMultiColor(ImVec2(b.x - feather, a.y), b, gone, edge, edge, gone);
    }

    dl_->AddRect(ma, mb, paperFade(theme_.paperEdge, 0.7f), px(2), 0, px(1));
}

PaperUI::ListResult PaperUI::list(const char* idStr, ImVec2 a, ImVec2 b, int rowCount,
                                  float rowHeight, int selected, const RowDrawer& drawRow) {
    const ImU32 id = hashId(idStr);
    ListResult out;
    FieldState& state = fieldState(id);

    const bool live = listening();
    const float viewH = std::max(b.y - a.y, 1.0f);
    const float contentH = static_cast<float>(rowCount) * rowHeight;
    const float maxScroll = std::max(0.0f, contentH - viewH);
    const bool over = live && hovered(a, b);
    if (over) {
        mouseWanted_ = true;
        const float wheel = ImGui::GetIO().MouseWheel;
        if (wheel != 0.0f) state.scrollY -= wheel * rowHeight * 2.0f;
    }

    // A selection that moved by some other route - the arrow keys, or a
    // character that has just been created - drags the view to it.
    if (selected != state.lastSelected) {
        state.lastSelected = selected;
        if (selected >= 0) {
            const float top = static_cast<float>(selected) * rowHeight;
            if (top < state.scrollY) state.scrollY = top;
            if (top + rowHeight > state.scrollY + viewH) state.scrollY = top + rowHeight - viewH;
        }
    }
    state.scrollY = std::clamp(state.scrollY, 0.0f, maxScroll);

    dl_->PushClipRect(a, b, true);
    const int first = std::max(0, static_cast<int>(std::floor(state.scrollY / rowHeight)));
    const int last = std::min(rowCount - 1,
                              static_cast<int>(std::floor((state.scrollY + viewH) / rowHeight)));
    for (int i = first; i <= last; ++i) {
        const float y0 = a.y - state.scrollY + static_cast<float>(i) * rowHeight;
        const ImVec2 rowA(a.x, y0);
        const ImVec2 rowB(b.x, y0 + rowHeight);
        const bool rowHovered = over && hovered(rowA, rowB);
        if (rowHovered) out.hovered = i;

        const bool isSelected = (i == selected);
        if (isSelected) {
            dl_->AddRectFilled(ImVec2(rowA.x + px(2), rowA.y + px(1)),
                               ImVec2(rowB.x - px(2), rowB.y - px(1)), theme_.highlighter,
                               px(2));
        } else if (rowHovered) {
            dl_->AddRectFilled(ImVec2(rowA.x + px(2), rowA.y + px(1)),
                               ImVec2(rowB.x - px(2), rowB.y - px(1)),
                               paperFade(theme_.ink, 0.05f), px(2));
        }
        if (i > 0) {
            rule(ImVec2(rowA.x + px(6), rowA.y), ImVec2(rowB.x - px(6), rowA.y),
                 paperFade(theme_.pencil, 0.35f), px(0.9f), id + static_cast<uint32_t>(i));
        }

        drawRow(i, rowA, rowB, isSelected, rowHovered);

        if (rowHovered && pressed()) {
            out.clicked = i;
            if (ImGui::GetMouseClickedCount(ImGuiMouseButton_Left) >= 2) out.activated = i;
        }
    }
    dl_->PopClipRect();

    if (maxScroll <= 0.0f) return out;

    // The mark in the margin, and the ability to drag it - a list too long to
    // fit is otherwise reachable only by a wheel not every pointer has.
    const float trackX = b.x - px(6);
    const float trackTop = a.y + px(4);
    const float trackBottom = b.y - px(4);
    const float trackH = std::max(trackBottom - trackTop, 1.0f);
    const float thumbH = std::max(px(26), trackH * (viewH / contentH));
    const ImVec2 barA(trackX - px(5), a.y);
    const ImVec2 barB(b.x, b.y);
    if (live && hovered(barA, barB) && pressed())
        active_ = id;
    if (active_ == id && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        const float t = std::clamp((ImGui::GetIO().MousePos.y - trackTop - thumbH * 0.5f) /
                                       std::max(trackH - thumbH, 1.0f),
                                   0.0f, 1.0f);
        state.scrollY = maxScroll * t;
        mouseWanted_ = true;
    }
    rule(ImVec2(trackX, trackTop), ImVec2(trackX, trackBottom), paperFade(theme_.pencil, 0.5f),
         px(1.2f), id + 77u);
    const float thumbY = trackTop + (trackH - thumbH) * (state.scrollY / maxScroll);
    dl_->AddRectFilled(ImVec2(trackX - px(2.5f), thumbY),
                       ImVec2(trackX + px(2.5f), thumbY + thumbH),
                       paperFade(theme_.crayonRed, 0.8f), px(2.5f));
    return out;
}

// ---------------------------------------------------------------------------
// Dropdown
// ---------------------------------------------------------------------------

bool PaperUI::dropdown(const char* idStr, ImVec2 a, ImVec2 b, const std::string& preview,
                       const std::vector<std::string>& items, int* index) {
    const ImU32 id = hashId(idStr);
    const bool open = (openPopup_ == id);
    const bool live = listeningAs(id);
    const bool over = (inert_ == 0) && hovered(a, b);
    if (over && live) mouseWanted_ = true;

    bool changed = false;
    const float size = (b.y - a.y) * 0.44f;
    const float rowHeight = (b.y - a.y) * 0.86f;

    // The list, sized and placed before anything is hit-tested, because both
    // the click that chooses a row and the click that lands outside need to
    // know where it is.
    const float listHeight = px(8) + rowHeight * static_cast<float>(items.size());
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    float listTop = b.y + px(4);
    if (listTop + listHeight > screen.y - px(8)) listTop = std::max(px(8), a.y - px(4) - listHeight);
    const ImVec2 la(a.x, listTop);
    const ImVec2 lb(b.x, listTop + listHeight);

    int hoveredRow = -1;
    if (open && inert_ == 0) {
        for (int i = 0; i < static_cast<int>(items.size()); ++i) {
            const float y0 = la.y + px(4) + static_cast<float>(i) * rowHeight;
            if (hovered(ImVec2(la.x, y0), ImVec2(lb.x, y0 + rowHeight))) hoveredRow = i;
        }
        if (hovered(la, lb)) mouseWanted_ = true;

        if (pressed()) {
            if (hoveredRow >= 0) {
                // Picking the row that was already picked is not a change.
                // Callers act on this - switching expansion reloads every
                // asset the client holds - and must not be made to do it
                // again for a click that chose nothing new.
                if (index && *index != hoveredRow) {
                    *index = hoveredRow;
                    changed = true;
                }
                openPopup_ = 0;
            } else if (!hovered(la, lb) && !over) {
                openPopup_ = 0;
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) openPopup_ = 0;
    }

    if (over && live && pressed()) {
        openPopup_ = open ? 0u : id;
    }

    // The closed control looks like a field that cannot be typed into.
    const uint32_t seed = seedFor(a, b);
    dl_->AddRectFilled(a, b, paperFade(theme_.fieldFill, 0.75f), px(3));
    if (over && live) dl_->AddRectFilled(a, b, paperFade(theme_.ink, 0.05f), px(3));
    const float pad = px(9);
    dl_->PushClipRect(ImVec2(a.x + pad, a.y), ImVec2(b.x - px(26), b.y), true);
    text(ImVec2(a.x + pad, a.y + ((b.y - a.y) - size) * 0.5f - px(1)), preview.c_str(), size,
         theme_.ink);
    dl_->PopClipRect();

    // A scratched-in arrow, pointing the other way while the list is down.
    const float cx = b.x - px(15);
    const float cy = (a.y + b.y) * 0.5f;
    const float w = px(5);
    const float h = px(3.2f) * (openPopup_ == id ? -1.0f : 1.0f);
    dl_->AddLine(ImVec2(cx - w, cy - h * 0.6f), ImVec2(cx, cy + h), theme_.inkSoft, px(1.6f));
    dl_->AddLine(ImVec2(cx, cy + h), ImVec2(cx + w, cy - h * 0.6f), theme_.inkSoft, px(1.6f));

    wobblyRect(dl_, a, b, paperFade(theme_.pencil, (over && live) ? 1.0f : 0.85f), px(1.2f),
               px(0.9f), seed);

    if (openPopup_ == id && !items.empty()) {
        PopupDraw draw;
        draw.a = la;
        draw.b = lb;
        draw.items = items;
        draw.selected = index ? *index : -1;
        draw.hovered = hoveredRow;
        draw.rowHeight = rowHeight;
        popups_.push_back(std::move(draw));
    }
    return changed;
}

// ---------------------------------------------------------------------------
// Checkbox and sliders
// ---------------------------------------------------------------------------

bool PaperUI::checkbox(const char* idStr, ImVec2 at, float boxSize, const char* label,
                       bool* value) {
    const ImU32 id = hashId(idStr);
    const float size = boxSize * 0.78f;
    const float labelW = textWidth(label, size);
    const ImVec2 a = at;
    const ImVec2 b(at.x + boxSize + (labelW > 0.0f ? px(9) + labelW : 0.0f), at.y + boxSize);

    const bool live = listening();
    const bool over = live && hovered(a, b);
    if (over) mouseWanted_ = true;

    bool changed = false;
    if (over && pressed()) active_ = id;
    if (active_ == id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (over && value) {
            *value = !*value;
            changed = true;
        }
        active_ = 0;
    }

    const ImVec2 boxA = a;
    const ImVec2 boxB(a.x + boxSize, a.y + boxSize);
    dl_->AddRectFilled(boxA, boxB, paperFade(theme_.fieldFill, over ? 1.0f : 0.7f), px(2));
    wobblyRect(dl_, boxA, boxB, over ? theme_.ink : paperFade(theme_.pencil, 0.95f), px(1.3f),
               px(0.8f), seedFor(boxA, boxB));
    if (value && *value) {
        // A tick that runs slightly outside its box, as a pen does.
        const float x0 = boxA.x + boxSize * 0.20f;
        const float y0 = boxA.y + boxSize * 0.52f;
        const float x1 = boxA.x + boxSize * 0.42f;
        const float y1 = boxA.y + boxSize * 0.76f;
        const float x2 = boxA.x + boxSize * 0.92f;
        const float y2 = boxA.y + boxSize * 0.14f;
        dl_->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), theme_.crayonRed, px(2.1f));
        dl_->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), theme_.crayonRed, px(2.1f));
    }
    if (label && *label) {
        text(ImVec2(boxB.x + px(9), a.y + (boxSize - size) * 0.5f - px(1)), label, size,
             over ? theme_.ink : theme_.inkSoft);
    }
    return changed;
}

bool PaperUI::sliderFloat(const char* idStr, ImVec2 a, ImVec2 b, float* value, float lo,
                          float hi, const char* fmt) {
    if (!value || hi <= lo) return false;
    const ImU32 id = hashId(idStr);
    const float size = (b.y - a.y) * 0.52f;

    char shown[32];
    std::snprintf(shown, sizeof(shown), fmt ? fmt : "%.0f", *value);
    const float valueW = textWidth(shown, size) + px(10);

    const ImVec2 ta(a.x, a.y);
    const ImVec2 tb(std::max(a.x + px(30), b.x - valueW), b.y);
    const float cy = (ta.y + tb.y) * 0.5f;
    const float knobR = (b.y - a.y) * 0.28f;
    const float x0 = ta.x + knobR;
    const float x1 = tb.x - knobR;

    const bool live = listening();
    const bool over = live && hovered(ImVec2(ta.x, ta.y - px(2)), ImVec2(tb.x, tb.y + px(2)));
    if (over) mouseWanted_ = true;

    bool changed = false;
    if (over && pressed()) active_ = id;
    if (active_ == id && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        const float t = std::clamp((ImGui::GetIO().MousePos.x - x0) / std::max(x1 - x0, 1.0f),
                                   0.0f, 1.0f);
        const float next = lo + (hi - lo) * t;
        if (next != *value) {
            *value = next;
            changed = true;
        }
        mouseWanted_ = true;
    }

    const float t = std::clamp((*value - lo) / (hi - lo), 0.0f, 1.0f);
    const float kx = x0 + (x1 - x0) * t;
    const uint32_t seed = seedFor(a, b);
    rule(ImVec2(x0, cy), ImVec2(x1, cy), paperFade(theme_.pencil, 0.9f), px(1.6f), seed);
    rule(ImVec2(x0, cy), ImVec2(kx, cy), paperFade(theme_.crayonBlue, 0.9f), px(2.4f), seed + 3u);
    dl_->AddCircleFilled(ImVec2(kx, cy), knobR, over || active_ == id ? theme_.crayonRedLit
                                                                     : theme_.crayonRed);
    dl_->AddCircle(ImVec2(kx, cy), knobR, theme_.crayonRedDim, 0, px(1.4f));
    textRight(b.x, cy - size * 0.62f, shown, size, theme_.inkSoft);
    return changed;
}

bool PaperUI::sliderInt(const char* idStr, ImVec2 a, ImVec2 b, int* value, int lo, int hi) {
    if (!value) return false;
    float v = static_cast<float>(*value);
    if (!sliderFloat(idStr, a, b, &v, static_cast<float>(lo), static_cast<float>(hi), "%.0f"))
        return false;
    const int rounded = static_cast<int>(std::lround(v));
    if (rounded == *value) return false;
    *value = rounded;
    return true;
}

} // namespace wowee::ui
