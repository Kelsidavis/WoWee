#include <cstring>
#include "ui/widget_renderer.hpp"
#include "ui/text_markup.hpp"
#include "ui/link_hit.hpp"
#include "ui/text_wrap.hpp"
#include <set>

#include "ui/widget_tree.hpp"
#include "ui/framexml_takeover.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "rendering/vk_context.hpp"
#include "core/app_clock.hpp"
#include "ui/interface_fonts.hpp"
#include "core/logger.hpp"

#include "imgui.h"

#include <algorithm>
#include <cfloat>
#include <cstdlib>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace wowee {
namespace ui {

namespace {

/// A path that resolved to nothing. Stored rather than retried, so one bad
/// SetTexture in an addon does not read a missing file every frame forever.
constexpr VkDescriptorSet kMissing = VK_NULL_HANDLE;

uint32_t packColor(const float rgba[4], float alpha) {
    auto ch = [](float v) {
        return static_cast<uint32_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return IM_COL32(ch(rgba[0]), ch(rgba[1]), ch(rgba[2]), ch(rgba[3] * alpha));
}


/// WoW's inline markup, split into runs of text that share a colour.
///
using TextRun = wowee::ui::WrapRun;
using wowee::ui::parseMarkup;

/// The same string with every escape taken out, for measuring.
std::string strippedText(const std::string& in) {
    std::string out;
    for (const TextRun& r : parseMarkup(in)) out += r.text;
    return out;
}

} // namespace

void WidgetRenderer::initialize(pipeline::AssetManager* assets,
                                rendering::VkContext* vkCtx) {
    assets_ = assets;
    vkCtx_ = vkCtx;
}

// Additive art is uploaded as its own image, because the same file can be
// asked for both ways and the two differ in their alpha channel.
static std::string cacheKey(const std::string& path, bool add) {
    return add ? path + "|add" : path;
}

VkDescriptorSet WidgetRenderer::resident(const std::string& path, bool add) const {
    if (path.empty()) return kMissing;
    auto it = textures_.find(cacheKey(path, add));
    return (it == textures_.end()) ? kMissing : it->second;
}

std::vector<uint8_t> WidgetRenderer::readTextureFile(const std::string& path,
                                                     std::string& resolvedOut) {
    if (!assets_ || path.empty()) return {};

    // Addons write "Interface\\Foo\\Bar" without the extension as often as with
    // it, and the real client accepts both.
    std::string resolved = path;
    auto endsWith = [&resolved](const char* ext) {
        if (resolved.size() <= 4) return false;
        const size_t at = resolved.size() - 4;
        for (size_t i = 0; i < 4; ++i) {
            if (std::tolower(static_cast<unsigned char>(resolved[at + i])) != ext[i]) {
                return false;
            }
        }
        return true;
    };
    // .tga means .blp. Blizzard's own markup still names 33 files that way -
    // the world map's quest icons and small frame edges, the login screen's
    // rating art - and no .tga has shipped in the archives since long before
    // 3.3.5. The real client accepts the name and loads the BLP beside it;
    // taking the extension at its word left every one of those blank, which
    // looks exactly like art nobody meant to draw.
    if (endsWith(".tga")) resolved.replace(resolved.size() - 4, 4, ".blp");
    else if (!endsWith(".blp")) resolved += ".blp";

    auto data = assets_->readFile(resolved);
    if (data.empty()) {
        // The interface is WotLK's; the assets are whichever expansion this
        // install carries, and the two do not always agree on a folder. The
        // quest icon is the one that differs here - FrameXML asks for
        // Interface\GossipFrame\AvailableQuestIcon, which is where 3.3.5 keeps
        // it, and this install has Interface\Gossip\. Retried rather than
        // aliased at extraction time, because the extractor preserves the
        // paths its MPQs use and is right to.
        static constexpr struct { const char* from; const char* to; } kFolders[] = {
            {.from = "gossipframe\\", .to = "gossip\\"},
        };
        std::string lower = resolved;
        for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        for (const auto& swap : kFolders) {
            const size_t at = lower.find(swap.from);
            if (at == std::string::npos) continue;
            std::string alt = resolved;
            alt.replace(at, std::strlen(swap.from), swap.to);
            data = assets_->readFile(alt);
            if (!data.empty()) { resolved = alt; break; }
        }
    }
    resolvedOut = resolved;
    return data;
}

VkDescriptorSet WidgetRenderer::texture(const std::string& path, bool add) {
    const std::string key = cacheKey(path, add);
    auto it = textures_.find(key);
    if (it != textures_.end()) return it->second;
    if (!assets_ || !vkCtx_ || path.empty()) return kMissing;

    std::string resolved;
    auto data = readTextureFile(path, resolved);
    if (data.empty()) {
        LOG_WARNING("Widget texture not found: ", path);
        textures_[key] = kMissing;
        return kMissing;
    }
    auto image = pipeline::BLPLoader::load(data);
    if (!image.isValid()) {
        LOG_WARNING("Widget texture unreadable: ", resolved);
        textures_[key] = kMissing;
        return kMissing;
    }
    if (add) {
        // Additive blending is not something a single ImGui draw list can be
        // asked for - it has one pipeline and one blend state. But the art it
        // is used for is a glow on black, with no alpha channel of its own, and
        // over a dark scene "add" and "blend with alpha taken from brightness"
        // put nearly the same pixels on the screen. Black stays invisible,
        // which is the whole difference between a glow and a slab.
        for (size_t i = 0; i + 3 < image.data.size(); i += 4) {
            const uint8_t lum = std::max({image.data[i], image.data[i + 1],
                                          image.data[i + 2]});
            image.data[i + 3] = static_cast<uint8_t>((image.data[i + 3] * lum) / 255);
        }
    }
    VkDescriptorSet set = vkCtx_->uploadImGuiTexture(image.data.data(),
                                                     image.width, image.height);
    textures_[key] = set;
    return set;
}


bool WidgetRenderer::textureSize(const std::string& path, float& w, float& h) {
    if (path.empty()) return false;
    auto known = textureSizes_.find(path);
    if (known != textureSizes_.end()) {
        if (known->second.first <= 0.0f) return false;   // looked up, no good
        w = known->second.first;
        h = known->second.second;
        return true;
    }
    // Deliberately no vkCtx_ here. Asking how big a picture is does not need a
    // GPU, and tying it to one would mean the headless harness could never
    // check any of this - which is what kept this fix unwritten.
    std::string resolved;
    auto data = readTextureFile(path, resolved);
    if (data.empty()) { textureSizes_[path] = {0.0f, 0.0f}; return false; }
    const auto image = pipeline::BLPLoader::load(data);
    if (!image.isValid() || image.width == 0 || image.height == 0) {
        textureSizes_[path] = {0.0f, 0.0f};
        return false;
    }
    w = static_cast<float>(image.width);
    h = static_cast<float>(image.height);
    textureSizes_[path] = {w, h};
    return true;
}

/// A texture with nothing else to go on is as big as its own picture.
///
/// WoW sizes a region from its image when neither a <Size> nor a pair of
/// opposing anchors says otherwise, and FrameXML leans on it: the status icon
/// beside the friends list's dropdown carries one LEFT anchor and no size at
/// all, so without this it is a region zero by zero and the player's online
/// status never appears.
///
/// Only the axis that has nothing. A texture given a width by two anchors and
/// left open vertically keeps the width and takes only the height, which is how
/// a stretched border piece is meant to work.
void WidgetRenderer::sizeTextures(WidgetTree& tree) {
    if (!assets_) return;
    for (size_t id = 1; id < tree.size(); ++id) {
        Widget* w = tree.get(static_cast<uint32_t>(id));
        if (!w || w->kind != WidgetKind::Texture) continue;
        if (w->texturePath.empty()) continue;
        // A texture with no anchors at all fills its parent, and that rule is
        // older and stronger than this one. Sizing it from its image instead
        // takes every portrait ring, action button icon and bag slot off the
        // frame it is meant to cover and leaves it at whatever the artist
        // happened to save the file at, centred - which warps the entire
        // interface. Reported from a screenshot within minutes of this pass
        // existing.
        //
        // SetAllPoints needs no such guard: it lays down two opposing corners,
        // so the span test below already sees a size and stands aside.
        if (w->anchors.empty()) continue;
        // Button art has its own rule and it is the stronger one: it fills the
        // button on any axis its anchors leave open. Sizing it from its image
        // instead made InterfaceOptionsFrameTab's highlight 32 tall on a 24
        // tall tab, overhanging it by eight. The two rules answer the same
        // question, so only one of them may run.
        if (w->buttonArt != ButtonArt::None) continue;
        // Anything it already knows about itself wins. Two opposing anchors
        // are a statement about size just as much as <Size> is, so a piece
        // stretched between two others must not be pulled back to its file's
        // dimensions - that is exactly the scroll bar middle, which would stop
        // stretching and become 31 by 256.
        const bool spanX = anchorsSpanAxis(w->anchors, true);
        const bool spanY = anchorsSpanAxis(w->anchors, false);
        const bool needsW = w->width <= 0.0f && !spanX;
        const bool needsH = w->height <= 0.0f && !spanY;
        if (!needsW && !needsH) continue;
        float iw = 0.0f, ih = 0.0f;
        if (!textureSize(w->texturePath, iw, ih)) continue;
        if (needsW) w->width = iw;
        if (needsH) w->height = ih;
    }
}

void WidgetRenderer::sizeFontStrings(WidgetTree& tree) {
    ImFont* font = interfaceFace("frizqt__");
    if (!font) font = ImGui::GetFont();
    if (!font) return;

    for (size_t id = 1; id < tree.size(); ++id) {
        Widget* w = tree.get(static_cast<uint32_t>(id));
        if (!w || w->kind != WidgetKind::FontString) continue;
        if (w->text.empty()) continue;
        // Anchors that span an axis give the size on that axis, and the string
        // does not get a say about it. Asked per axis, the way the texture
        // sizing above asks it - anchorsSpanAxis exists for exactly this and
        // this loop was counting instead.
        //
        // Counting is wrong whenever two anchors pin the same axis, or pin one
        // axis twice and the other not at all. Every options category button
        // is that: OptionsList_DisplayButton does
        // `button.text:SetPoint("LEFT", 8, 2)` on a ButtonText that already
        // carries one, so the label had two anchors, neither of which says how
        // tall it is - and the measure that would have said was skipped. The
        // label kept height 0 and drew nothing.
        //
        // That emptied the category list of every options frame at once:
        // Video, Interface and Audio all list their categories with this
        // button. The entries were there and the buttons were there; the names
        // were invisible, so nothing could be read or clicked, and no setting
        // added to the schema could be reached however correctly it registered.
        const bool spansX = anchorsSpanAxis(w->anchors, true);
        const bool spansY = anchorsSpanAxis(w->anchors, false);
        if (spansX && spansY) continue;

        // A width with no height is a paragraph, not a request to be measured.
        //
        // `<AbsDimension x="285" y="0"/>` is WoW's way of saying "wrap inside
        // 285 and be as tall as that takes", and it is what every block of
        // prose in the interface declares. Reading the zero height as "size
        // yourself" and measuring the text on one unbounded line replaced the
        // 285 with however wide the sentence happened to be - 424 for a short
        // quest description, far more for a real one - and marked the string
        // auto-sized, which is also what tells the draw below not to wrap it.
        // So it drew one line out through the side of the scroll frame that
        // clips it, and everything anchored beneath it sat on top of the lines
        // that should have pushed it down.
        const bool paragraph =
            w->wrapsToWidth ||
            (!w->autoSized && w->width > 0.0f && w->height <= 0.0f);

        // The size the draw will use, not a flat twelve.
        //
        // A label with no fontHeight of its own is drawn at the current font
        // size and was measured at twelve, so wherever those differ the rect
        // came out narrower than the glyphs that go in it and the text ran out
        // of its right edge. The money frame is where it shows: the number is
        // anchored to end exactly where the coin icon begins, so the overspill
        // lands on top of the coin.
        //
        // Same face too - the widget's own, then the interface default -
        // rather than whichever face this measure happened to be handed.
        ImFont* runFont = interfaceFaceOrDefault(w->fontFace);
        if (!runFont) runFont = font;
        const float size = interfaceFontSize(w->fontHeight);
        const auto measureRun = [&](const std::string& piece) {
            return runFont->CalcTextSizeA(size, FLT_MAX, 0.0f, piece.c_str()).x;
        };

        // Already the right size for this text, measured the way it would be
        // measured now. A label that has changed what it says is measured
        // again; one sized by its XML is left alone.
        //
        // The size and the face are part of that question. The interface's own
        // typeface is registered after the first frames have been laid out, and
        // a label measured before that kept a rect built from the fallback font
        // for the rest of the session - its text had not changed, so nothing
        // ever looked again. Every one of them then drew glyphs wider than the
        // box it was given, the backpack's coin amounts among them: that rect
        // is anchored to end exactly where the coin picture begins, so the
        // overspill lands on the coin.
        const bool sameMeasurement = w->measuredText == w->text &&
                                     w->measuredSize == size &&
                                     w->measuredFace == w->fontFace;
        if (paragraph) {
            if (sameMeasurement) continue;
        } else {
            if (w->autoSized && sameMeasurement) continue;
            if (!w->autoSized && w->width > 0.0f && w->height > 0.0f) continue;
        }

        if (paragraph) {
            // The width stays as declared and the height follows the wrap.
            // autoSized stays false on purpose: it is what the draw reads to
            // decide whether there is a box to wrap inside, and this string is
            // exactly the kind that has one.
            w->wrapsToWidth = true;
            const int rows = static_cast<int>(
                wrapText(parseMarkup(w->text), w->width, w->nonSpaceWrap,
                         measureRun).size());
            w->wrappedLines = rows > 0 ? rows : 1;
            w->height = size * 1.2f * static_cast<float>(w->wrappedLines);
            w->measuredText = w->text;
            w->measuredSize = size;
            w->measuredFace = w->fontFace;
            continue;
        }

        const ImVec2 measured =
            runFont->CalcTextSizeA(size, FLT_MAX, 0.0f, strippedText(w->text).c_str());
        w->width = measured.x;
        // As tall as the lines it holds. A label sized by its own text can
        // still be several lines: |n breaks one at any width, and giving it a
        // single line's height put whatever anchors below it over the top of
        // the rest.
        const int rows = static_cast<int>(
            wrapText(parseMarkup(w->text), 0.0f, false, measureRun).size());
        w->wrappedLines = rows > 0 ? rows : 1;
        if (w->height <= 0.0f || w->autoSized) {
            w->height = size * 1.2f * static_cast<float>(w->wrappedLines);
        }
        w->autoSized = true;
        w->measuredText = w->text;
        w->measuredSize = size;
        w->measuredFace = w->fontFace;
    }
}

void WidgetRenderer::sizeTooltips(WidgetTree& tree) {
    ImFont* font = interfaceFace("frizqt__");
    if (!font) font = ImGui::GetFont();
    if (!font) return;

    for (size_t id = 1; id < tree.size(); ++id) {
        Widget* w = tree.get(static_cast<uint32_t>(id));
        if (!w || !w->isTooltip) continue;
        if (w->tooltipLines.empty()) continue;
        // Only something that really is a tooltip. The flag is set by whichever
        // widget a tooltip setter was called on, and a stray call sets it on a
        // frame that is not one - which then gets resized to fit lines it never
        // meant to show. The character sheet's model frame was 85x34 for that
        // reason: one line of text plus a tooltip's padding, in place of the
        // 233x215 its XML asks for.
        if (w->objectType != "GameTooltip") continue;

        const float size = interfaceFontSize(w->fontHeight);
        const float lineH = size * 1.2f;
        // Ten units of padding a side, which is what the tooltip backdrop's
        // own insets come to.
        constexpr float kPad = 10.0f;
        // A wrapped line breaks to fit rather than setting the width, so the
        // width comes from the lines that cannot break. Bounded either way: a
        // tooltip of one long sentence used to stretch across the screen, and
        // one of nothing but short lines should not force prose into a column.
        constexpr float kMinWrap = 180.0f, kMaxWrap = 320.0f;
        float widest = 0.0f;
        for (const auto& line : w->tooltipLines) {
            if (line.wrap) continue;
            float wide = font->CalcTextSizeA(size, FLT_MAX, 0.0f,
                                             strippedText(line.left).c_str()).x;
            if (!line.right.empty()) {
                // Left and right text share a line with a gap between them.
                wide += 20.0f +
                        font->CalcTextSizeA(size, FLT_MAX, 0.0f,
                                            strippedText(line.right).c_str()).x;
            }
            if (wide > widest) widest = wide;
        }
        const float wrapW = std::clamp(widest > 0.0f ? widest : kMinWrap,
                                       kMinWrap, kMaxWrap);

        int rows = 0;
        for (const auto& line : w->tooltipLines) {
            // Every line, wrapping or not: one carrying |n is two rows tall
            // whether or not it is also being broken to fit.
            line.lines = static_cast<int>(
                wrapText(parseMarkup(line.left), line.wrap ? wrapW : 0.0f, false,
                         [&](const std::string& piece) {
                             return font->CalcTextSizeA(size, FLT_MAX, 0.0f,
                                                        piece.c_str()).x;
                         }).size());
            rows += line.lines;
        }

        // The floor SetMinimumWidth asked for, applied to the finished width
        // rather than to the text box, because that is what FrameXML measures
        // against: it compares the money frame's width to GetMinimumWidth and
        // widens the tooltip to hold it.
        w->width  = std::max(std::max(widest, wrapW) + kPad * 2.0f,
                             w->tooltipMinWidth);
        w->height = lineH * static_cast<float>(rows) + kPad * 2.0f;
    }
}


void WidgetRenderer::drawMarkupText(ImDrawList* dl, ImFont* font, float size,
                                    ImVec2 at, uint32_t fallback, float alpha,
                                    const std::string& text, float wrapWidth,
                                    bool nonSpaceWrap, const char* justifyH,
                                    bool forceColor, WidgetTree* linkSink,
                                    uint32_t linkOwner) {
    const auto lines = wrapText(parseMarkup(text), wrapWidth, nonSpaceWrap,
                                [&](const std::string& piece) {
                                    return font->CalcTextSizeA(size, FLT_MAX, 0.0f,
                                                               piece.c_str()).x;
                                });
    const float lineH = size * 1.2f;
    float y = at.y;
    for (const auto& line : lines) {
        // A picture contributes its own width, not its (empty) text's, or a
        // centred line of coins would be justified as if it were blank.
        const auto runWidth = [&](const TextRun& run) {
            if (!run.texture.empty()) {
                return run.texWidth > 0.0f    ? run.texWidth
                     : run.texHeight > 0.0f   ? run.texHeight
                                              : size;
            }
            return font->CalcTextSizeA(size, FLT_MAX, 0.0f, run.text.c_str()).x;
        };
        float lineW = 0.0f;
        for (const TextRun& run : line) lineW += runWidth(run);
        // Each line justifies inside the box on its own, which is what makes a
        // centred paragraph look centred rather than ragged from one offset.
        float x = at.x;
        if (wrapWidth > 0.0f && justifyH) {
            if (std::string(justifyH) == "CENTER") x = at.x + (wrapWidth - lineW) * 0.5f;
            else if (std::string(justifyH) == "RIGHT") x = at.x + wrapWidth - lineW;
        }
        for (const TextRun& run : line) {
            if (!run.texture.empty()) {
                const float w = runWidth(run);
                const float h = run.texHeight > 0.0f ? run.texHeight : size;
                // Only on the pass that draws the text. The shadow and outline
                // passes repeat the same runs offset by a pixel, and an icon
                // drawn three times reads as a smear rather than as depth.
                if (!forceColor) {
                    VkDescriptorSet tex = resident(run.texture);
                    if (tex != kMissing) {
                        // Sat on the baseline like a capital, which is where
                        // the interface's own money frames put a coin.
                        const float top = y + (lineH - h) * 0.5f;
                        dl->AddImage(reinterpret_cast<ImTextureID>(tex),
                                     ImVec2(x, top), ImVec2(x + w, top + h),
                                     ImVec2(0, 0), ImVec2(1, 1),
                                     IM_COL32(255, 255, 255,
                                              static_cast<int>(alpha * 255.0f)));
                    }
                }
                x += w;
                continue;
            }
            uint32_t col = fallback;
            // The shadow and outline passes draw the same glyphs in one colour;
            // a run's own colour would light them up.
            if (run.hasColor && !forceColor) {
                float rgba[4] = {run.rgba[0], run.rgba[1], run.rgba[2], run.rgba[3]};
                col = packColor(rgba, alpha);
            }
            dl->AddText(font, size, ImVec2(x, y), col, run.text.c_str());
            const float runW = font->CalcTextSizeA(size, FLT_MAX, 0.0f,
                                                   run.text.c_str()).x;
            // Where the link landed, for the click that arrives in another
            // pass entirely. Only from the pass that draws the text itself -
            // the outline and shadow draw the same glyphs and would file the
            // same link two or three times over.
            if (linkSink && !forceColor && !run.link.empty()) {
                // Into the space the click arrives in, which is not the space
                // this draws in. dispatchMouse takes DisplaySize.y - MousePos.y
                // and divides by the interface scale, so the tree is in
                // interface units with y growing upward, while every
                // coordinate here is a screen pixel with y growing down.
                // Filing the pixels would have made the hit test miss by the
                // scale factor and by the whole height of the screen.
                linkSink->addLinkRect(linkRectFromDraw(
                    linkOwner, run.link, run.text, x, y, runW, lineH,
                    linkScreenH_, linkScale_));
            }
            x += runW;
        }
        y += lineH;
    }
}

void WidgetRenderer::drawBackdrop(ImDrawList* dl, const Widget& w, float scale,
                                  float x0, float y0, float x1, float y1) {
    // Background sits inside the insets, which is what keeps it from showing
    // through the border drawn over it. The insets are interface units and the
    // rect is pixels, so they are scaled here - without it a tooltip's border
    // was half its proper thickness on a 1528-tall display and would be twice
    // it on a short one.
    const float bx0 = x0 + w.insetLeft * scale;
    const float by0 = y0 + w.insetTop * scale;
    const float bx1 = x1 - w.insetRight * scale;
    const float by1 = y1 - w.insetBottom * scale;
    if (bx1 > bx0 && by1 > by0) {
        VkDescriptorSet bg = resident(w.bgFile);
        const uint32_t col = packColor(w.backdropColor, w.alpha);
        if (bg != kMissing) {
            // Tiling repeats the art at its own size instead of stretching it,
            // which is the difference between a stone wall and a smear.
            float u1 = 1.0f, v1 = 1.0f;
            if (w.tileBackground && w.edgeSize > 0.0f) {
                // In units on both sides of the division, so the art repeats
                // at its authored size rather than at a rate that changes with
                // the window.
                u1 = (bx1 - bx0) / (w.edgeSize * scale);
                v1 = (by1 - by0) / (w.edgeSize * scale);
            }
            dl->AddImage(reinterpret_cast<ImTextureID>(bg), ImVec2(bx0, by0), ImVec2(bx1, by1),
                         ImVec2(0.0f, 0.0f), ImVec2(u1, v1), col);
        }
        // A backdrop with no background file has no background - only its edge.
        // Filling the rect instead painted every such frame in the backdrop
        // colour, which defaults to opaque white, and a wide one is a white
        // slab across the screen. Nothing is drawn here either while the art is
        // still being read.
    }

    VkDescriptorSet edge = resident(w.edgeFile);
    if (edge == kMissing || w.edgeSize <= 0.0f) return;

    // The edge file is eight square tiles in a row. Measured against the art
    // rather than assumed: UI-Tooltip-Border is 128x16 and UI-DialogBox-Border
    // 256x32, both exactly eight tiles wide.
    const float e = w.edgeSize * scale;
    const uint32_t col = packColor(w.borderColor, w.alpha);
    auto piece = [&](int index, float px0, float py0, float px1, float py1) {
        const float u0 = index / 8.0f, u1 = (index + 1) / 8.0f;
        dl->AddImage(reinterpret_cast<ImTextureID>(edge), ImVec2(px0, py0), ImVec2(px1, py1),
                     ImVec2(u0, 0.0f), ImVec2(u1, 1.0f), col);
    };
    // An edge is a run of square tiles, and the top and bottom ones are stored
    // on their side.
    //
    // Decoding UI-Tooltip-Border settles both points. It is 128x16 - eight
    // 16x16 tiles - and measuring each one's opacity by row and by column
    // gives: tiles 0 to 3 are all *vertical* lines, and 4 to 7 are the four
    // corners. There is no horizontal line anywhere in the file. Tiles 2 and 3
    // are the top and bottom edges kept rotated, which is why their lines sit
    // at opposite sides of the tile: rotated into place, one lands along the
    // top of its strip and the other along the bottom.
    //
    // Drawing them unrotated repeats a vertical line along a horizontal edge,
    // which is a row of tick marks across the top and bottom of every tooltip.
    // Stretching one instead - which is what this did before - smears that
    // line into a gradient, which is quieter but no more correct.
    //
    // Tiled by drawing repeated quads rather than by letting the UVs run past
    // 1: the eight tiles share one texture, so a u outside its own eighth
    // samples the neighbouring corner rather than repeating.
    auto run = [&](int index, float px0, float py0, float px1, float py1,
                   bool vertical) {
        const float span = vertical ? (py1 - py0) : (px1 - px0);
        if (span <= 0.0f) return;
        // Below a couple of pixels a tile carries no visible detail, and a
        // full-width frame would ask for a thousand quads to say nothing. One
        // tile stretched over the whole span is indistinguishable there and
        // bounded - and it goes through the same loop rather than a separate
        // path, so it keeps the rotation the horizontal edges need.
        // Bounded, because this is the one thing here whose cost grows with
        // the frame it is drawing. A border tiles at its authored size, so a
        // very wide frame would ask for a quad every sixteen pixels across -
        // fine for a tooltip, and hundreds for anything full-width. Past the
        // cap the remainder is stretched, which is what this did everywhere
        // before tiling and is indistinguishable at that length.
        constexpr float kMaxTiles = 64.0f;
        float step = (e < 2.0f) ? span : e;
        if (step > 0.0f && span / step > kMaxTiles) step = span / kMaxTiles;
        const float tu0 = index / 8.0f, tu1 = (index + 1) / 8.0f;
        // Counted rather than accumulated: adding step to a float each pass
        // drifts, and the last tile's length is derived from the position.
        // step is span when the edge inset is degenerate, so it can be zero
        // and the count has to tolerate that.
        const int tileCount = (step > 0.0f)
            ? static_cast<int>(std::ceil(span / step))
            : 0;
        for (int tile = 0; tile < tileCount; ++tile) {
            const float at = static_cast<float>(tile) * step;
            // The last tile is cut short rather than overhanging, and its UVs
            // are cut with it so the art is cropped rather than squeezed into
            // the remainder.
            const float len = std::min(step, span - at);
            const float frac = len / step;
            if (vertical) {
                dl->AddImage(reinterpret_cast<ImTextureID>(edge),
                             ImVec2(px0, py0 + at), ImVec2(px1, py0 + at + len),
                             ImVec2(tu0, 0.0f), ImVec2(tu1, frac), col);
            } else {
                // Quarter-turned: the strip runs along x, so screen x walks
                // the tile's v and screen y walks its u. That puts the tile's
                // left column along the top of the strip, which is where the
                // top edge's line lives - and the bottom edge's line, sitting
                // at the opposite side of its own tile, lands along the bottom
                // by the same mapping. One rotation serves both.
                const float ax0 = px0 + at, ax1 = px0 + at + len;
                dl->AddImageQuad(reinterpret_cast<ImTextureID>(edge),
                                 ImVec2(ax0, py0), ImVec2(ax1, py0),
                                 ImVec2(ax1, py1), ImVec2(ax0, py1),
                                 ImVec2(tu0, 0.0f),  ImVec2(tu0, frac),
                                 ImVec2(tu1, frac),  ImVec2(tu1, 0.0f), col);
            }
        }
    };
    // Edges first, then corners over them, so a corner is never clipped by the
    // run it meets.
    run(0, x0,     y0 + e, x0 + e, y1 - e, true);    // left
    run(1, x1 - e, y0 + e, x1,     y1 - e, true);    // right
    run(2, x0 + e, y0,     x1 - e, y0 + e, false);   // top
    run(3, x0 + e, y1 - e, x1 - e, y1,     false);   // bottom
    piece(4, x0,     y0,     x0 + e, y0 + e);   // top-left
    piece(5, x1 - e, y0,     x1,     y0 + e);   // top-right
    piece(6, x0,     y1 - e, x0 + e, y1);       // bottom-left
    piece(7, x1 - e, y1 - e, x1,     y1);       // bottom-right
}

void WidgetRenderer::drawStatusBar(ImDrawList* dl, const Widget& w,
                                   float x0, float y0, float x1, float y1) {
    const float f = w.barFraction();
    if (f <= 0.0f) return;
    // Horizontal bars fill from the left; vertical ones from the bottom, which
    // in screen terms means growing upward from y1.
    const float fx1 = w.barVertical ? x1 : x0 + (x1 - x0) * f;
    const float fy0 = w.barVertical ? y1 - (y1 - y0) * f : y0;
    const uint32_t col = packColor(w.barColor, w.alpha);

    VkDescriptorSet tex = resident(w.barTexture);
    if (tex != kMissing) {
        // The texture is cropped to the filled part rather than squashed into
        // it, so a bar at half value shows half its art at its own scale.
        const float u1 = w.barVertical ? 1.0f : f;
        const float v0 = w.barVertical ? (1.0f - f) : 0.0f;
        dl->AddImage(reinterpret_cast<ImTextureID>(tex), ImVec2(x0, fy0), ImVec2(fx1, y1),
                     ImVec2(0.0f, v0), ImVec2(u1, 1.0f), col);
    } else if (w.barTexture.empty()) {
        dl->AddRectFilled(ImVec2(x0, fy0), ImVec2(fx1, y1), col);
    }
}

void WidgetRenderer::drawColorPicker(ImDrawList* dl, const WidgetTree& tree,
                                     const Widget& w, const Widget& picker,
                                     float screenH,
                                     float x0, float y0, float x1, float y1) {
    const float* hsv = picker.pickerHSV;

    // Finds the wheel or the bar among the picker's children, which is how a
    // thumb learns the rect it has to sit on. The thumbs carry no anchors in
    // the XML at all - Blizzard's client moves them, and so does this.
    auto sibling = [&](Widget::ColorRole role) -> const Widget* {
        for (uint32_t id : picker.children) {
            const Widget* c = tree.get(id);
            if (c && c->colorRole == role) return c;
        }
        return nullptr;
    };
    // A widget's rect in screen pixels, flipped the same way the draw loop
    // flips its own.
    const float s = tree.uiScale();
    auto rectOf = [&](const Widget& r, float& rx0, float& ry0,
                      float& rx1, float& ry1) {
        rx0 = r.left * s;
        rx1 = (r.left + r.rectW) * s;
        ry1 = screenH - r.bottom * s;
        ry0 = ry1 - r.rectH * s;
    };

    switch (w.colorRole) {
        case Widget::ColorRole::Wheel: {
            // Hue around, saturation outward, at the brightness the bar is set
            // to - so the wheel dims with the colour rather than showing a
            // brightness that is not being chosen.
            //
            // Drawn as flat cells rather than a gradient mesh: a hundred and
            // ninety-two of them across a 128-pixel disc is finer than the eye
            // resolves, and it costs no vertex writing into a draw list whose
            // texture binding belongs to someone else.
            constexpr int kSectors = 48, kRings = 4;
            const float cx = (x0 + x1) * 0.5f, cy = (y0 + y1) * 0.5f;
            const float radius = std::min(x1 - x0, y1 - y0) * 0.5f;
            for (int ring = 0; ring < kRings; ++ring) {
                const float r0 = radius * static_cast<float>(ring) / kRings;
                const float r1 = radius * static_cast<float>(ring + 1) / kRings;
                const float sat = (static_cast<float>(ring) + 0.5f) / kRings;
                for (int seg = 0; seg < kSectors; ++seg) {
                    const float a0 = 6.2831853f * seg / kSectors;
                    const float a1 = 6.2831853f * (seg + 1) / kSectors;
                    const float cell[3] = {(seg + 0.5f) / kSectors, sat, hsv[2]};
                    float rgb[3];
                    hsvToRgb(cell, rgb);
                    const ImU32 col = IM_COL32(int(rgb[0] * 255), int(rgb[1] * 255),
                                               int(rgb[2] * 255), 255);
                    // Screen y grows downward and the wheel's hue runs
                    // anticlockwise, so the sine is negated to keep red at the
                    // right and the order the same as the client's.
                    dl->AddQuadFilled(
                        ImVec2(cx + std::cos(a0) * r0, cy - std::sin(a0) * r0),
                        ImVec2(cx + std::cos(a1) * r0, cy - std::sin(a1) * r0),
                        ImVec2(cx + std::cos(a1) * r1, cy - std::sin(a1) * r1),
                        ImVec2(cx + std::cos(a0) * r1, cy - std::sin(a0) * r1), col);
                }
            }
            break;
        }
        case Widget::ColorRole::Value: {
            // Full brightness at the top down to black, in the hue and
            // saturation the wheel is showing.
            const float top[3] = {hsv[0], hsv[1], 1.0f};
            float rgb[3];
            hsvToRgb(top, rgb);
            const ImU32 hi = IM_COL32(int(rgb[0] * 255), int(rgb[1] * 255),
                                      int(rgb[2] * 255), 255);
            const ImU32 lo = IM_COL32(0, 0, 0, 255);
            dl->AddRectFilledMultiColor(ImVec2(x0, y0), ImVec2(x1, y1),
                                        hi, hi, lo, lo);
            break;
        }
        case Widget::ColorRole::WheelThumb: {
            const Widget* wheel = sibling(Widget::ColorRole::Wheel);
            if (!wheel) break;
            float wx0, wy0, wx1, wy1;
            rectOf(*wheel, wx0, wy0, wx1, wy1);
            const float cx = (wx0 + wx1) * 0.5f, cy = (wy0 + wy1) * 0.5f;
            const float radius = std::min(wx1 - wx0, wy1 - wy0) * 0.5f;
            const float angle = hsv[0] * 6.2831853f;
            const float px = cx + std::cos(angle) * hsv[1] * radius;
            const float py = cy - std::sin(angle) * hsv[1] * radius;
            const float hw = (x1 - x0) * 0.5f, hh = (y1 - y0) * 0.5f;
            drawThumb(dl, w, px - hw, py - hh, px + hw, py + hh);
            break;
        }
        case Widget::ColorRole::ValueThumb: {
            const Widget* bar = sibling(Widget::ColorRole::Value);
            if (!bar) break;
            float bx0, by0, bx1, by1;
            rectOf(*bar, bx0, by0, bx1, by1);
            // Value one at the top of the bar, zero at the bottom.
            const float py = by0 + (1.0f - hsv[2]) * (by1 - by0);
            const float hw = (x1 - x0) * 0.5f, hh = (y1 - y0) * 0.5f;
            const float cx = (bx0 + bx1) * 0.5f;
            drawThumb(dl, w, cx - hw, py - hh, cx + hw, py + hh);
            break;
        }
        case Widget::ColorRole::None:
            break;
    }
}

/// A picker thumb: its own art if the file is resident, and a plain outline
/// while it is not. The outline matters - the thumb is the only thing on the
/// wheel that says where the current colour is, and a picker that will not say
/// cannot be used at all.
void WidgetRenderer::drawThumb(ImDrawList* dl, const Widget& w,
                               float x0, float y0, float x1, float y1) {
    VkDescriptorSet tex = resident(w.texturePath);
    if (tex != VK_NULL_HANDLE && tex != kMissing) {
        dl->AddImage(reinterpret_cast<ImTextureID>(tex), ImVec2(x0, y0), ImVec2(x1, y1),
                     ImVec2(w.texCoord[0], w.texCoord[2]),
                     ImVec2(w.texCoord[1], w.texCoord[3]));
        return;
    }
    dl->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(255, 255, 255, 230), 0.0f, 0, 2.0f);
    dl->AddRect(ImVec2(x0 - 1, y0 - 1), ImVec2(x1 + 1, y1 + 1), IM_COL32(0, 0, 0, 200));
}

void WidgetRenderer::drawSlider(ImDrawList* dl, const Widget& w,
                                float x0, float y0, float x1, float y1) {
    // A slider that declared a <ThumbTexture> has a real region for its grip,
    // and the layout now moves that region to the value. Painting a second one
    // here would double it - and worse, would ignore the region's own shown
    // state: ScrollFrame_OnScrollRangeChanged hides `<bar>ThumbTexture` when the
    // list fits, and a knob painted from the file path alone stayed on a bar
    // with nothing to scroll.
    if (w.thumbRegion != 0) return;

    VkDescriptorSet thumb = resident(w.thumbTexture);
    if (thumb == kMissing) return;

    // The thumb sits at the value along the track, and is as wide as the track
    // is narrow - a scroll bar's grip is square to its channel.
    const float f = w.barFraction();
    const uint32_t col = packColor(w.barColor, w.alpha);
    if (w.barVertical) {
        const float size = x1 - x0;
        // Screen y grows downward while a slider's value grows upward, so the
        // full value belongs at the top of the track.
        const float span = (y1 - y0) - size;
        const float top = y1 - size - f * span;
        dl->AddImage(reinterpret_cast<ImTextureID>(thumb), ImVec2(x0, top),
                     ImVec2(x1, top + size), ImVec2(0, 0), ImVec2(1, 1), col);
    } else {
        const float size = y1 - y0;
        const float span = (x1 - x0) - size;
        const float left = x0 + f * span;
        dl->AddImage(reinterpret_cast<ImTextureID>(thumb), ImVec2(left, y0),
                     ImVec2(left + size, y1), ImVec2(0, 0), ImVec2(1, 1), col);
    }
}

void WidgetRenderer::drawCooldown(ImDrawList* dl, const Widget& w,
                                  float x0, float y0, float x1, float y1) {
    if (w.cooldownDuration <= 0.0) return;
    const double elapsed = core::appTimeSeconds() - w.cooldownStart;
    if (elapsed < 0.0 || elapsed >= w.cooldownDuration) return;
    const float done = static_cast<float>(elapsed / w.cooldownDuration);
    // Reversed, the wedge covers what has passed rather than what is left.
    const float remaining = w.cooldownReverse ? done : (1.0f - done);

    // A wedge from twelve o'clock, shrinking clockwise as the time runs out.
    // Drawn to the corners rather than to an inscribed circle - the thing being
    // covered is a square icon, and a circle would leave its corners lit - and
    // clipped to the frame so the overrun does not spill onto its neighbours.
    const ImVec2 centre((x0 + x1) * 0.5f, (y0 + y1) * 0.5f);
    const float radius = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    constexpr float kTwoPi = 6.2831853f;
    constexpr int kSegments = 48;
    const int used = std::max(1, static_cast<int>(kSegments * remaining));

    dl->PushClipRect(ImVec2(x0, y0), ImVec2(x1, y1), true);
    dl->PathClear();
    dl->PathLineTo(centre);
    for (int i = 0; i <= used; ++i) {
        // -pi/2 starts at the top; positive sweep runs clockwise on a screen
        // whose y grows downward.
        const float a = -kTwoPi * 0.25f +
                        kTwoPi * remaining * (static_cast<float>(i) / used);
        dl->PathLineTo(ImVec2(centre.x + std::cos(a) * radius,
                              centre.y + std::sin(a) * radius));
    }
    dl->PathFillConvex(IM_COL32(0, 0, 0, 160));
    if (w.cooldownDrawEdge) {
        // One bright spoke along the leading edge, which is what makes a slow
        // sweep readable at a glance.
        const float edge = -kTwoPi * 0.25f + kTwoPi * remaining;
        dl->AddLine(centre,
                    ImVec2(centre.x + std::cos(edge) * radius,
                           centre.y + std::sin(edge) * radius),
                    IM_COL32(255, 255, 255, 180), 1.5f);
    }
    dl->PopClipRect();
}

void WidgetRenderer::render(WidgetTree& tree, float screenW, float screenH) {
    layout(tree, screenW, screenH);
    draw(tree, screenW, screenH);
}

void WidgetRenderer::layout(WidgetTree& tree, float screenW, float screenH) {
    linkScreenH_ = screenH;
    linkScale_ = tree.uiScale();
    // Every descriptor set in this cache belongs to the context, which frees
    // them all together. Drawing with one afterwards is a fault the GPU answers
    // by resetting, so the cache goes when they do - at the cost of re-uploading
    // whatever is on screen.
    if (vkCtx_) {
        const uint32_t generation = vkCtx_->uiTextureGeneration();
        if (generation != uiTextureGenerationSeen_) {
            uiTextureGenerationSeen_ = generation;
            if (!textures_.empty()) {
                LOG_WARNING("The UI textures were destroyed; dropping ",
                            textures_.size(),
                            " cached textures rather than drawing with the "
                            "descriptor sets that went with them");
                textures_.clear();
            }
        }
    }

    static const std::vector<std::string> kSuppressed = frameXmlSuppressedFrames();
    // Which of these never resolved, said once a few seconds in.
    //
    // A name that matches no frame suppresses nothing, silently - the window
    // it was meant to hide keeps opening beside this client's own, and the
    // list looks complete. Reported late rather than at load because several
    // of these frames belong to addons that are not loaded until something
    // asks for them, and a name is only wrong if it is still unresolved after
    // the interface has settled.
    static bool reportedUnresolved = false;
    static double firstSeen = 0.0;
    const double now = core::appTimeSeconds();
    if (firstSeen == 0.0) firstSeen = now;

    // Resolved once and kept, because findByName is a linear scan over every
    // widget with a string compare on each. Sixty-two names against several
    // thousand FrameXML widgets is a few hundred thousand comparisons a frame,
    // every frame, for a list that barely changes - the cost arrived with the
    // list, which grew from fifteen names to sixty-two in one sitting.
    //
    // The id is verified against the name before use, so a slot reused by a
    // different frame re-resolves rather than hiding the wrong one. Names that
    // do not resolve are retried on a timer, not per frame: several belong to
    // load-on-demand addons and only appear once something asks for them.
    static std::vector<uint32_t> resolved(kSuppressed.size(), 0);
    static double lastRetry = 0.0;
    const bool retryMisses = (now - lastRetry) > 1.0;
    if (retryMisses) lastRetry = now;

    for (size_t i = 0; i < kSuppressed.size(); ++i) {
        const std::string& name = kSuppressed[i];
        Widget* w = (resolved[i] != 0) ? tree.get(resolved[i]) : nullptr;
        if (w && w->name != name) w = nullptr;   // that slot is someone else now
        if (!w && (resolved[i] != 0 || retryMisses)) {
            w = tree.findByName(name);
            resolved[i] = w ? w->id : 0;
        }
        if (w) w->shown = false;
    }
    // Suppression is a drawing decision and nothing more. A frame hidden here
    // keeps every event it registered, and the frame dispatch does not filter
    // on visibility - rightly, because that is how the real client behaves and
    // FrameXML relies on it, registering and unregistering in OnShow/OnHide
    // where it wants otherwise.
    //
    // So a suppressed window is still running its handlers on every event this
    // client fires, and can still raise from one. Eighteen of them are live
    // this way. When judging whether a fault in some window can be reached, the
    // question is whether its events are fired, never whether it is on screen.
    //
    // What they do *not* run is OnShow and OnHide, and that is worth keeping
    // that way. LuaEngine::updateVisibility reports a change by comparing
    // `visible` against what it last reported, and the application calls it
    // after this render - so a frame that a handler showed earlier in the same
    // iteration is already false again by the time it looks, and never counts
    // as having appeared.
    //
    // That ordering is load-bearing rather than incidental. LootFrame_OnHide
    // calls CloseLoot(), which releases the loot on the server. Report
    // visibility before this runs and every suppressed loot window would show
    // for an instant, hide, and take the player's loot with it - through the
    // client's own loot window, which is the one actually on screen.

    if (!reportedUnresolved && (now - firstSeen) > 20.0) {
        reportedUnresolved = true;
        static const std::vector<std::string> kLazy = frameXmlLazySuppressedFrames();
        for (const std::string& name : kSuppressed) {
            // A frame from a load-on-demand addon is absent until that addon is
            // asked for, which is not a fault and must not read as one.
            if (std::find(kLazy.begin(), kLazy.end(), name) != kLazy.end()) continue;
            if (!tree.findByName(name)) {
                LOG_WARNING("FrameXML: nothing is named '", name,
                            "' - that suppression is doing nothing, and "
                            "whatever it was meant to hide is still shown");
            }
        }
    }

    sizeTooltips(tree);
    // Same reason, for every label that never stated a size: it takes the size
    // of its own text, and anything anchored to it is placed from that.
    sizeFontStrings(tree);
    // Before the solve, like the two above: this decides a size the solve
    // then places.
    sizeTextures(tree);

    tree.layout(screenW, screenH);

    reportOverflowingText(tree);
    reportLetteredAmounts(tree);
}

/// Labels whose glyphs are wider than the rect they were given.
///
/// A font string with no width of its own is measured and the rect it gets is
/// that measurement, so the two agree by construction - unless the draw and the
/// measure disagree about the face or the size, and then the text runs out of
/// its own right edge. Where the label is anchored so that its right edge is
/// something else's left edge, that overspill lands on top of whatever is
/// there: the backpack's coin amounts are drawn over the coins that way.
///
/// Named nothing in particular so it catches whichever label it happens to be,
/// and each name is said once.
/// Labels holding a coin amount with a letter on the end of it.
///
/// "19g", "81s", "56c" - WoW writes the amount and the coin's picture, and the
/// only thing in the interface that writes the letter is the colourblind branch
/// of MoneyFrame_Update. Reported over three passes as letters beside the coins
/// in the backpack, and turning that branch off did not stop it, so whatever
/// writes them is somewhere else. This says which label holds one, by name.
void WidgetRenderer::reportLetteredAmounts(WidgetTree& tree) {
    for (size_t id = 1; id < tree.size(); ++id) {
        const Widget* w = tree.get(static_cast<uint32_t>(id));
        if (!w || w->kind != WidgetKind::FontString || w->text.empty()) continue;
        // A run of digits with a single g, s or c after it and nothing else.
        const std::string& s = w->text;
        const char last = s.back();
        if (last != 'g' && last != 's' && last != 'c') continue;
        if (s.size() < 2) continue;
        bool digits = true;
        for (size_t i = 0; i + 1 < s.size(); ++i) {
            if (s[i] < '0' || s[i] > '9') { digits = false; break; }
        }
        if (!digits) continue;

        static std::set<std::string> said;
        const std::string key = w->name.empty() ? std::string("(unnamed)") : w->name;
        if (!said.insert(key).second) continue;
        LOG_WARNING("Coin amount written with a letter: ", key, " holds \"", s,
                    "\" - WoW writes the amount and the coin's picture, and only "
                    "MoneyFrame_Update's colourblind branch writes the letter. "
                    "ENABLE_COLORBLIND_MODE is \"", "0", "\" here, so this came "
                    "from somewhere else");
    }
}

void WidgetRenderer::reportOverflowingText(WidgetTree& tree) {
    for (size_t id = 1; id < tree.size(); ++id) {
        const Widget* w = tree.get(static_cast<uint32_t>(id));
        if (!w || w->kind != WidgetKind::FontString) continue;
        if (!w->autoSized || w->text.empty() || w->rectW <= 0.0f) continue;

        // Against the width this label was measured to need, not against a
        // fresh measurement: a label stretched between two anchors is given
        // their span and is legitimately narrower than its text, which is
        // ordinary clipping and not what this is for. What is worth saying is
        // a label that asked for a width, was measured, and then got a rect
        // narrower than the answer.
        //
        // A pixel of slack, because rounding between the two is not a fault.
        // Through the scale its chain is drawn at: the rect is in scaled units
        // and the measurement is not, so a frame that has been scaled down -
        // every boss frame is - would otherwise read as an overflow when it is
        // simply smaller.
        float chainScale = 1.0f;
        for (uint32_t up = static_cast<uint32_t>(id), guard = 0;
             up != 0 && guard <= tree.size(); ++guard) {
            const Widget* node = tree.get(up);
            if (!node) break;
            chainScale *= node->scale;
            up = node->parent;
        }
        const float drawn = w->width * chainScale;
        if (w->width <= 0.0f || w->rectW >= drawn - 1.0f) continue;

        static std::set<std::string> said;
        const std::string key = w->name.empty() ? std::string("(unnamed)") : w->name;
        if (!said.insert(key).second) continue;
        LOG_WARNING("Text wider than its rect: ", key, " \"", w->text,
                    "\" draws ", drawn, " into ", w->rectW,
                    " - fontHeight=", w->fontHeight,
                    " size=", interfaceFontSize(w->fontHeight),
                    " face=", w->fontFace.empty() ? "(default)" : w->fontFace.c_str(),
                    " currentFontSize=", ImGui::GetFontSize(),
                    " scale=", tree.uiScale(),
                    " - whatever its right edge is anchored to is being drawn over");
    }
}

// What is on screen, and what should be but is not.
//
// None of this draws anything. It is the instrumentation the FrameXML
// transition is being carried out with: a list of every drawn widget with its
// rect and its text, so a stray label can be traced back to the frame that put
// it there; a check that the elements handed over actually arrived; and a
// report of frames FrameXML is drawing that no element accounts for.
//
// Split out of draw() because it was 536 of its 1193 lines - a function named
// draw that spent nearly half its length not drawing.
namespace {

/// WOWEE_WIDGET_DUMP - how much this renderer says about what it drew.
///
/// 1 lists what is drawn; 2 lists every named widget whether drawn or not,
/// which is what shows a container's own rect - a frame paints nothing itself,
/// so the thing that mispositioned everything under it never appears in a list
/// of what was painted. 3 and 4 outline widgets in place, and 5 draws ImGui's
/// own font atlas as a control.
///
/// Read once. It was a static local of draw(), which is where the reporting
/// used to live too; the reporting moved out and the drawing still needs it.
int widgetDumpLevel() {
    static const int level = [] {
        const char* v = std::getenv("WOWEE_WIDGET_DUMP");
        if (!v || !*v) return 0;
        return std::atoi(v);
    }();
    return level;
}

}  // namespace

void WidgetRenderer::reportWidgetDiagnostics(WidgetTree& tree,
                                             const std::vector<const Widget*>& order,
                                             float s, float screenW, float screenH) {
    // What is actually on screen, named, once, when asked for.
    //
    // A stray label is very hard to identify from a screenshot: the text says
    // what it says, and nothing says which frame put it there. This lists every
    // drawn widget with its name, its rect and its text, which turns "what is
    // that in the middle of the screen" into one line of log.
    // 1 lists what is drawn; 2 lists every named widget whether drawn or not,
    // which is what shows a container's own rect - a frame paints nothing
    // itself, so the thing that mispositioned everything under it never
    // appears in a list of what was painted.
    // Not on the first frame. Textures upload a few per frame, so a dump taken
    // immediately reports nothing resident and says only that the load had not
    // finished - which is true and useless. A couple of seconds in, what is
    // missing is missing for a reason.
    static int framesSeen = 0;
    static bool dumped = false;
    ++framesSeen;

    // Did the elements handed over actually arrive?
    //
    // Reported without being asked for, because it only happens when someone
    // has already said WOWEE_FRAMEXML_UI, and because the alternative is
    // reading a screenshot for whether a frame is present, hidden, or laid out
    // to nothing - three failures that look identical from outside and quite
    // different here. Late enough that textures have had time to upload.
    //
    // Twice: once when the interface has finished loading, and again once the
    // player is in the world. Those are two different pictures, because
    // FrameXML repositions and hides frames from PLAYER_ENTERING_WORLD - at
    // load every frame is still where its XML put it. Reading a load-time
    // report as though it described the running game cost several rounds of
    // chasing a durability frame that had already been moved by the time
    // anyone could see it.
    static int passesDone = 0;
    static int worldFrame = 0;
    if (worldFrame == 0 && frameXmlWorldEntered()) worldFrame = framesSeen;
    const bool loadPass  = (passesDone == 0 && framesSeen > 120);
    // Well after world entry, not just after it: FrameXML is loaded at world
    // entry rather than at startup, so a second pass a hundred frames later
    // lands in the same moment as the first and reports the same picture.
    const bool worldPass = (passesDone == 1 && worldFrame != 0 &&
                            framesSeen > worldFrame + 600);
    // The first time the character sheet is open, ask for a report of its own
    // accord. Its labels only exist to be looked at while it is up, and no
    // automatic checkpoint ever coincides with that.
    static bool sawCharacterSheet = false;
    if (!sawCharacterSheet) {
        if (const Widget* sheet = tree.findByName("CharacterFrame");
            sheet && sheet->visible) {
            sawCharacterSheet = true;
            frameXmlRequestCheck();
        }
    }

    // Frames FrameXML puts on screen that no element accounts for.
    //
    // Every frame around it, and this runs every frame rather than on the
    // checkpoints below, because the ones worth catching are the ones that are
    // only up for a moment: the zone banner fades in on a crossing and was
    // drawn beside this client's own for months without any check mentioning
    // it. A checkpoint pass would have to land inside the fade to see it.
    //
    // frameXmlReportUnaccountedElements cannot find these. It iterates the
    // element list, so it reports gaps among names somebody already thought of,
    // and zone text was not an element at all. This asks the question from the
    // other end - what is on screen - which is the end that needs no foresight.
    //
    // Said once per name, ever. A frame that is legitimately unlisted costs one
    // line for the life of the process.
    {
        static const std::set<std::string> accounted = [] {
            const auto all = frameXmlAccountedFrames();
            return std::set<std::string>(all.begin(), all.end());
        }();
        static std::set<std::string> said;
        // UIParent's own children only: every panel hangs off it, and the parts
        // inside a panel are that panel's business rather than the takeover's.
        //
        // The id is cached because findByName scans the tree backwards and
        // UIParent is built early, so looking it up by name every frame walks
        // almost the whole tree - a cost worth paying nowhere, least of all for
        // a diagnostic. Re-resolved if it ever stops naming UIParent, which is
        // what a rebuilt tree would look like from here.
        static uint32_t rootId = 0;
        const Widget* root = rootId ? tree.get(rootId) : nullptr;
        if (!root || root->name != "UIParent") {
            root = tree.findByName("UIParent");
            rootId = root ? root->id : 0;
        }
        if (root) {
            for (const uint32_t childId : root->children) {
                const Widget* w = tree.get(childId);
                if (!w || !w->visible || w->name.empty()) continue;
                if (w->rectW <= 0.0f || w->rectH <= 0.0f) continue;
                if (accounted.count(w->name) || said.count(w->name)) continue;
                said.insert(w->name);
                LOG_WARNING("FrameXML: '", w->name, "' is on screen and no "
                            "element accounts for it - if this client draws the "
                            "same thing, both are up");
            }
        }
    }

    const bool askedFor = frameXmlTakeCheckRequest();
    if (loadPass || worldPass || askedFor) {
        if (!askedFor) ++passesDone;
        const char* when = askedFor ? "on request" : (loadPass ? "at load" : "in world");

        // Before reporting anything, give back any element whose top-level
        // frame does not exist. Handing one over hides this client's own, so a
        // panel that did not build is not a worse panel - it is no panel, with
        // no way back to the one that worked. Only in world: at load a panel
        // that builds on demand has not been asked for yet.
        if (worldPass || askedFor) {
            const int given = frameXmlReleaseUnbuiltElements(
                [&tree](const std::string& name) { return tree.findByName(name) != nullptr; });
            if (given > 0) {
                LOG_WARNING("FrameXML: ", given, " element(s) handed back; this "
                            "client draws them for the rest of the session");
            }
        }

        const std::vector<std::string> wanted = frameXmlCheckFrames();
        if (!wanted.empty()) {
            LOG_WARNING("FrameXML takeover check ", when, ", on ", screenW, "x", screenH,
                        " px (scale ", s, "):");
            // Anything that landed off the screen, whoever it belongs to.
            //
            // A frame in the wrong place is only findable by name if you can
            // guess the name, and the thing that looks wrong on screen is
            // rarely the thing you would have thought to check. Position is
            // the question actually being asked, so ask it of everything.
            int offscreen = 0;
            for (size_t id = 1; id < tree.size(); ++id) {
                const Widget* w = tree.get(static_cast<uint32_t>(id));
                if (!w || !w->visible || w->name.empty()) continue;
                if (w->rectW <= 0.0f || w->rectH <= 0.0f) continue;
                const float l = w->left * s, b = w->bottom * s;
                const float r = (w->left + w->rectW) * s;
                const float t = (w->bottom + w->rectH) * s;
                if (l < screenW && r > 0.0f && b < screenH && t > 0.0f) continue;
                if (++offscreen > 12) break;
                LOG_WARNING("  OFF SCREEN ", w->name, " rect=(", w->left, ",",
                            w->bottom, " ", w->rectW, "x", w->rectH, ")");
            }
            if (offscreen > 12) LOG_WARNING("  ... and more");

            // Shown, and no size to be shown at.
            //
            // The off-screen scan above skips these deliberately - a rect of
            // zero is not off screen - so a frame that is laid out, told to
            // draw, and occupies nothing has been the one shape this check
            // could not see. It is a real signature rather than a curiosity:
            // uidropdownmenu asks for "uiscale", an exact-match CVar read
            // answered "0" for it, and every dropdown in the interface opened
            // at SetScale(0) - built, shown, and drawing nothing. Anything
            // that multiplies into a size can do that.
            //
            // Only where a size was ASKED FOR and did not survive. A frame
            // declared with no size of its own is sized by its anchors or not
            // at all, and plenty are deliberately sizeless - reporting those
            // would print a page every run, which trains the reader to skip
            // the whole check, the way the three shared UIParents would have.
            //
            // Declared non-zero and resolved to zero is unambiguous. The rect
            // carries the effective scale, so a zero anywhere up the scale
            // chain lands here and nowhere else.
            {
                int flat = 0;
                for (size_t id = 1; id < tree.size(); ++id) {
                    const Widget* w = tree.get(static_cast<uint32_t>(id));
                    if (!w || !w->visible || w->name.empty()) continue;
                    if (w->width <= 0.0f && w->height <= 0.0f) continue;
                    if (w->rectW > 0.0f && w->rectH > 0.0f) continue;
                    if (++flat > 10) break;
                    LOG_WARNING("  NO SIZE ", w->name, " asks for ", w->width,
                                "x", w->height, " and resolves to ", w->rectW,
                                "x", w->rectH, " (scale ", w->effScale, ")");
                }
                if (flat > 10) LOG_WARNING("  ... and more");
            }

            // Visible, named, and anchored to nothing.
            //
            // An unanchored frame falls to the centre of its parent, so this
            // is what a stray panel in the middle of the screen actually is -
            // and the shape is unmistakable once looked for, where hunting it
            // by name means guessing what it is from a screenshot.
            // Names carried by more than one visible widget.
            //
            // Only the last one to take a name can be found by it, so a
            // duplicate is invisible to every lookup while both are still
            // drawn - which reads as one label rendered twice in two places.
            {
                std::map<std::string, int> seen;
                for (size_t id = 1; id < tree.size(); ++id) {
                    const Widget* w = tree.get(static_cast<uint32_t>(id));
                    if (!w || !w->visible || w->name.empty()) continue;
                    ++seen[w->name];
                }
                int dupes = 0;
                for (const auto& [name, count] : seen) {
                    if (count < 2) continue;
                    // UIParent and WorldFrame are meant to be shared. The tree
                    // has a root of that name, the bootstrap builds a
                    // full-size one so FrameXML's own has something to fill,
                    // and FrameXML then declares it as well. Reporting the
                    // three every run trains the reader to skip the line, and
                    // the line's whole value is catching the duplicate nobody
                    // intended.
                    if (name == "UIParent" || name == "WorldFrame") continue;
                    if (++dupes > 10) break;
                    LOG_WARNING("  DUPLICATE ", name, " - ", count,
                                " visible widgets share this name");
                }
            }

            // Two labels showing the same words.
            //
            // A duplicate that shares no name is invisible to the scan above,
            // and the interface draws plenty of labels without one - so the
            // text itself is what identifies the pair.
            {
                std::map<std::string, int> texts;
                for (size_t id = 1; id < tree.size(); ++id) {
                    const Widget* w = tree.get(static_cast<uint32_t>(id));
                    if (!w || !w->visible || w->kind != WidgetKind::FontString) continue;
                    if (w->text.size() < 3) continue;   // too short to mean anything
                    ++texts[w->text];
                }
                int pairs = 0;
                for (size_t id = 1; id < tree.size() && pairs < 8; ++id) {
                    const Widget* w = tree.get(static_cast<uint32_t>(id));
                    if (!w || !w->visible || w->kind != WidgetKind::FontString) continue;
                    if (w->text.size() < 3 || texts[w->text] < 2) continue;
                    ++pairs;
                    LOG_WARNING("  SAME TEXT \"", w->text, "\" on ",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                " rect=(", w->left, ",", w->bottom, " ",
                                w->rectW, "x", w->rectH, ")");
                }
            }

            // Two visible labels sitting on top of each other.
            //
            // Catches a pair the text scan cannot: one still showing its XML
            // placeholder while the other has the real value reads as two
            // different strings, and by name they may share nothing at all.
            {
                std::vector<const Widget*> labels;
                for (size_t id = 1; id < tree.size(); ++id) {
                    const Widget* w = tree.get(static_cast<uint32_t>(id));
                    if (!w || !w->visible || w->kind != WidgetKind::FontString) continue;
                    if (w->text.empty() || w->rectW <= 0.0f || w->rectH <= 0.0f) continue;
                    labels.push_back(w);
                }
                int overlaps = 0;
                for (size_t i = 0; i < labels.size() && overlaps < 8; ++i) {
                    for (size_t j = i + 1; j < labels.size() && overlaps < 8; ++j) {
                        const Widget* a = labels[i];
                        const Widget* b = labels[j];
                        const float ox = std::min(a->left + a->rectW, b->left + b->rectW) -
                                         std::max(a->left, b->left);
                        const float oy = std::min(a->bottom + a->rectH, b->bottom + b->rectH) -
                                         std::max(a->bottom, b->bottom);
                        if (ox <= 0.0f || oy <= 0.0f) continue;
                        // Meaningfully on top of each other, not merely touching:
                        // stacked bar labels share an edge by a unit or two and
                        // are not what this is looking for.
                        const float smaller = std::min(a->rectW * a->rectH,
                                                       b->rectW * b->rectH);
                        if (smaller <= 0.0f || (ox * oy) < smaller * 0.5f) continue;
                        ++overlaps;
                        LOG_WARNING("  OVERLAPPING LABELS ",
                                    a->name.empty() ? "(unnamed)" : a->name.c_str(),
                                    " \"", a->text, "\" over ",
                                    b->name.empty() ? "(unnamed)" : b->name.c_str(),
                                    " \"", b->text, "\" at (", a->left, ",", a->bottom, ")");
                    }
                }
            }

            // Anything at all sitting over the paperdoll's rotate arrows.
            //
            // Text is drawn there and the label scan below does not see it, so
            // it is not a FontString - an edit box draws its own text and a
            // tooltip draws its lines, and neither is one.
            if (const Widget* arrow = tree.findByName("CharacterModelFrameRotateLeftButton");
                arrow && arrow->visible) {
                for (size_t id = 1; id < tree.size(); ++id) {
                    const Widget* w = tree.get(static_cast<uint32_t>(id));
                    if (!w || !w->visible || w->id == arrow->id) continue;
                    if (w->left > arrow->left + arrow->rectW + 60.0f) continue;
                    if (w->left + w->rectW < arrow->left - 20.0f) continue;
                    if (w->bottom > arrow->bottom + arrow->rectH) continue;
                    if (w->bottom + w->rectH < arrow->bottom) continue;
                    const bool hasWords = !w->text.empty() || !w->editText.empty() ||
                                          !w->tooltipLines.empty();
                    if (!hasWords) continue;
                    LOG_WARNING("  OVER THE ARROWS ",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                " kind=", static_cast<int>(w->kind),
                                " text=\"", w->text, "\" edit=\"", w->editText,
                                "\" lines=", w->tooltipLines.size(),
                                " rect=(", w->left, ",", w->bottom, " ",
                                w->rectW, "x", w->rectH, ")");
                }
            }

            // Every label inside the character sheet while it is open.
            //
            // Something is drawn over its rotate arrows and it is not a second
            // copy of the name - the same-text and overlap scans both rule that
            // out - so the way to find it is to list what is in there.
            if (const Widget* sheet = tree.findByName("CharacterFrame");
                sheet && sheet->visible) {
                int listed = 0;
                for (size_t id = 1; id < tree.size() && listed < 24; ++id) {
                    const Widget* w = tree.get(static_cast<uint32_t>(id));
                    if (!w || !w->visible || w->kind != WidgetKind::FontString) continue;
                    if (w->text.empty()) continue;
                    if (w->left < sheet->left || w->bottom < sheet->bottom) continue;
                    if (w->left > sheet->left + sheet->rectW) continue;
                    if (w->bottom > sheet->bottom + sheet->rectH) continue;
                    ++listed;
                    LOG_WARNING("  SHEET LABEL ",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                " \"", w->text, "\" rect=(", w->left, ",", w->bottom,
                                " ", w->rectW, "x", w->rectH, ")");
                }
                // The tabs themselves, with the state that decides whether a
                // click reaches them. WoW disables the tab you are already on,
                // so exactly one being disabled is right and all of them being
                // disabled is the fault - and the label dump above cannot tell
                // those two apart.
                for (int i = 1; i <= 5; ++i) {
                    const std::string tabName = "CharacterFrameTab" + std::to_string(i);
                    const Widget* tab = tree.findByName(tabName);
                    if (!tab) {
                        LOG_WARNING("  SHEET TAB ", tabName, " NOT BUILT");
                        continue;
                    }
                    LOG_WARNING("  SHEET TAB ", tabName,
                                tab->shown ? " shown" : " HIDDEN",
                                tab->enabled ? " enabled" : " DISABLED",
                                " rect=(", tab->left, ",", tab->bottom,
                                " ", tab->rectW, "x", tab->rectH, ")");
                }
            }

            int orphans = 0;
            for (size_t id = 1; id < tree.size(); ++id) {
                const Widget* w = tree.get(static_cast<uint32_t>(id));
                if (!w || !w->visible || w->name.empty()) continue;
                if (w->kind != WidgetKind::Frame) continue;
                if (!w->anchors.empty()) continue;
                if (w->rectW <= 0.0f || w->rectH <= 0.0f) continue;
                if (w->parent == tree.rootId()) {
                    // The root's own children legitimately fill it.
                    if (w->rectW >= screenW / s - 1.0f) continue;
                }
                if (++orphans > 10) break;
                LOG_WARNING("  UNANCHORED ", w->name, " rect=(", w->left, ",",
                            w->bottom, " ", w->rectW, "x", w->rectH, ")");
            }
            if (orphans > 10) LOG_WARNING("  ... and more");

            // The next elements, so readiness can be judged before the
            // client's own version is hidden and there is no way back within
            // the run.
            std::vector<std::string> all = wanted;
            const std::vector<std::string> candidates = frameXmlCandidateFrames();
            const size_t firstCandidate = all.size();
            all.insert(all.end(), candidates.begin(), candidates.end());

            size_t index = 0;
            for (const std::string& name : all) {
                const bool candidate = (index++ >= firstCandidate);
                if (candidate && index == firstCandidate + 1) {
                    LOG_WARNING("  -- not handed over yet, for readiness --");
                }
                const Widget* w = tree.findByName(name);
                if (!w) {
                    // Distinguished so the line does not read as a failure: an
                    // aura button that does not exist yet is the normal state
                    // for a character carrying no auras.
                    LOG_WARNING("  ", name,
                                frameXmlBuiltOnDemand(name)
                                    ? " - not built yet (created when needed)"
                                    : " - NOT BUILT");
                    continue;
                }
                const bool offscreen = (w->left * s > screenW) ||
                                       (w->bottom * s > screenH) ||
                                       ((w->left + w->rectW) * s < 0.0f) ||
                                       ((w->bottom + w->rectH) * s < 0.0f);
                // A status bar's numbers, because an empty bar and a bar
                // that was never given a value look identical, and the second
                // is the one that has been happening.
                std::string bar;
                if (w->isStatusBar) {
                    bar = " value=" + std::to_string(w->barValue) +
                          " of [" + std::to_string(w->barMin) + "," +
                          std::to_string(w->barMax) + "]" +
                          (w->barTexture.empty()
                               ? std::string(" NOBARTEXTURE")
                               : (w->visible && resident(w->barTexture) == kMissing
                                      ? " BARTEXNOTRESIDENT" : ""));
                }
                // Whether it can be clicked at all, which is a different
                // question from whether it is in the right place - a button
                // that takes no mouse looks identical to one whose handler
                // does nothing.
                const char* mouse = w->mouseEnabled ? " takesMouse" : "";
                // What a label actually says. A font string that is built,
                // shown and empty looks identical from every other field here,
                // and empty is the interesting case - a level with no number
                // in it is a frame that was never told the number.
                // Which slice of its atlas a texture is showing. A stack of
                // pieces cut from one file is only debuggable with these: the
                // rects can all look plausible while the slices are wrong.
                std::string slice;
                if (w->kind == WidgetKind::Texture &&
                    (w->texCoord[0] != 0.0f || w->texCoord[1] != 1.0f ||
                     w->texCoord[2] != 0.0f || w->texCoord[3] != 1.0f)) {
                    slice = " texcoord=[" + std::to_string(w->texCoord[0]) + "," +
                            std::to_string(w->texCoord[1]) + "," +
                            std::to_string(w->texCoord[2]) + "," +
                            std::to_string(w->texCoord[3]) + "]";
                }
                const std::string label =
                    (w->kind == WidgetKind::FontString)
                        ? (w->text.empty() ? std::string(" text=(empty)")
                                           : " text=\"" + w->text + "\"")
                        : std::string();
                // Where it sits in the stack, which is what decides a click
                // between a frame and the frame on top of it.
                // What kind of thing answered to the name, and how many anchors
                // it has. A frame reported at the size of a piece of text is
                // either a mislabelled region or a frame something resized, and
                // those need opposite fixes.
                const char* kindName =
                    (w->kind == WidgetKind::Texture)    ? " kind=texture"
                  : (w->kind == WidgetKind::FontString) ? " kind=label"
                                                        : "";
                const std::string anchors =
                    " anchors=" + std::to_string(w->anchors.size());
                const std::string stack =
                    " strata=" + std::to_string(static_cast<int>(w->effStrata)) +
                    " level=" + std::to_string(w->effLevel);
                LOG_WARNING("  ", name,
                            (w->visible ? " shown" : " HIDDEN"), mouse, kindName, anchors,
                            stack, bar, label, slice,
                            (w->rectW <= 0.0f || w->rectH <= 0.0f ? " NOSIZE" : ""),
                            (offscreen ? " OFFSCREEN" : ""),
                            " rect=(", w->left, ",", w->bottom, " ",
                            w->rectW, "x", w->rectH, ")",
                            // An external texture is what is actually drawn,
                            // and the path beside it is only the fallback the
                            // interface set - so the report has to say which
                            // of the two is on screen.
                            (w->externalTexture != 0 ? " LIVE" : ""),
                            // Only worth saying of something being drawn.
                            // Textures upload when first drawn, so anything
                            // hidden reports missing art whether or not the
                            // file exists - which reads as a fault and is not
                            // one. Twice now that has sent someone looking for
                            // a .blp that was on disk all along.
                            (w->visible && w->kind == WidgetKind::Texture &&
                             w->externalTexture == 0 && !w->texturePath.empty() &&
                             resident(w->texturePath, w->blendAdd) == kMissing
                                 ? " NOTRESIDENT" : ""),
                            (w->texturePath.empty() ? "" : " tex="), w->texturePath);
            }
        }
    }

    if (widgetDumpLevel() && !dumped && framesSeen > 180) {
        dumped = true;
        // The screen it was laid out against, because a coordinate means
        // nothing without it: 1920 is the middle of one display and off
        // the edge of another.
        LOG_WARNING("WidgetDump: ", order.size(), " widgets drawn on ",
                    screenW, "x", screenH, " px, ", screenW / s, "x",
                    screenH / s, " units (scale ", s, "), ",
                    textures_.size(), " textures resident");
        for (const Widget* w : order) {
            LOG_WARNING("  ", (w->name.empty() ? "(unnamed)" : w->name),
                        " kind=", static_cast<int>(w->kind),
                        " rect=(", w->left, ",", w->bottom, " ", w->rectW, "x", w->rectH, ")",
                        " alpha=", w->alpha,
                        // Whether its art has actually reached the GPU. A
                        // texture with a correct rect and nothing uploaded
                        // draws nothing at all, and looks identical in a list
                        // of what was "drawn" to one that worked.
                        (w->kind == WidgetKind::Texture && !w->solidColor
                             ? (resident(w->texturePath, w->blendAdd) == kMissing
                                    ? " NOTRESIDENT" : "")
                             : ""),
                        // The vertex colour multiplies the image, so a zero
                        // alpha here draws nothing while the widget's own alpha
                        // still reads one. And the UVs, because a collapsed
                        // pair samples a single pixel.
                        (w->kind == WidgetKind::Texture ? " rgba=" : ""),
                        (w->kind == WidgetKind::Texture ? w->color[0] : 0.0f), ",",
                        (w->kind == WidgetKind::Texture ? w->color[1] : 0.0f), ",",
                        (w->kind == WidgetKind::Texture ? w->color[2] : 0.0f), ",",
                        (w->kind == WidgetKind::Texture ? w->color[3] : 0.0f),
                        (w->kind == WidgetKind::Texture ? " uv=" : ""),
                        (w->kind == WidgetKind::Texture ? w->texCoord[0] : 0.0f), ",",
                        (w->kind == WidgetKind::Texture ? w->texCoord[1] : 0.0f), ",",
                        (w->kind == WidgetKind::Texture ? w->texCoord[2] : 0.0f), ",",
                        (w->kind == WidgetKind::Texture ? w->texCoord[3] : 0.0f),
                        (w->texturePath.empty() ? "" : " tex="), w->texturePath,
                        (w->text.empty() ? "" : " text='"), w->text,
                        (w->text.empty() ? "" : "'"));
        }
        if (widgetDumpLevel() >= 2) {
            LOG_WARNING("WidgetDump: every named widget, drawn or not");
            for (size_t id = 1; id < tree.size(); ++id) {
                const Widget* w = tree.get(static_cast<uint32_t>(id));
                if (!w || w->name.empty()) continue;
                LOG_WARNING("  ", w->name, " kind=", static_cast<int>(w->kind),
                            " rect=(", w->left, ",", w->bottom, " ",
                            w->rectW, "x", w->rectH, ")",
                            " anchors=", w->anchors.size(),
                            " shown=", w->shown ? 1 : 0,
                            " visible=", w->visible ? 1 : 0);
            }
        }
    }
}

void WidgetRenderer::draw(WidgetTree& tree, float screenW, float screenH) {
    // Cleared here rather than in layout(), which is the half that runs before
    // the frame's clicks are resolved. Emptying the list there and refilling it
    // here would leave nothing to click against in between, and a chat link
    // would never be hit at all. Clicks now test the rects this pass laid down
    // last frame - a frame behind, where a link that stopped being drawn stays
    // clickable for one more, which is the smaller of the two faults.
    tree.clearLinkRects();

    // The item on the cursor, drawn over everything. FrameXML never draws this
    // - in WoW the client does - so without it picking something up looked
    // exactly like nothing happening.
    if (const std::string& carried = frameXmlCursorItem(); !carried.empty()) {
        if (VkDescriptorSet icon = texture(carried); icon != kMissing) {
            const ImVec2 at = ImGui::GetIO().MousePos;
            const float side = 32.0f * tree.uiScale();
            ImDrawList* fg = ImGui::GetForegroundDrawList();
            fg->AddImage(reinterpret_cast<ImTextureID>(icon),
                         ImVec2(at.x, at.y),
                         ImVec2(at.x + side, at.y + side));
        }
    }

    const auto& order = tree.drawOrder();
    if (order.empty()) return;

    // Resolve textures before recording anything, and only a few per frame.
    //
    // Uploading one ends in vkDeviceWaitIdle. Doing that from inside the draw
    // loop stalls the whole device in the middle of building a frame, which is
    // the shape of problem this renderer has already been bitten by once -
    // enough synchronous submits in a row and the main loop stalls, a fence wait
    // fails, and the device is lost. Hoisting them out means the wait happens
    // between frames instead of during one, and the budget means a screen full
    // of new art costs several quiet frames rather than one very long one.
    // Three, not eight.
    //
    // Each one is a BLP decode, a staging buffer, a copy and a wait - and the
    // batch around them is synchronous, so the whole cost lands inside
    // uiManager->render. A live log shows that stage reaching 186ms while the
    // terrain was uploading M2 instances on the same queue, and the device was
    // lost shortly after with images reported as never having left
    // VK_IMAGE_LAYOUT_UNDEFINED.
    //
    // FrameXML wants far more distinct art than this client's own interface
    // did - icons per spellbook tab, per tracking type, per dropdown entry -
    // so the budget that was comfortable before is now reached every frame
    // during a load. Spreading the same work over more frames costs a texture
    // appearing a frame or two later, which is invisible, and shortens each
    // stall to something that cannot sit across a driver's patience.
    constexpr int kUploadsPerFrame = 3;
    std::vector<std::pair<const std::string*, bool>> wanted;
    wanted.reserve(kUploadsPerFrame);
    auto want = [&](const std::string& path, bool add = false) {
        if (static_cast<int>(wanted.size()) >= kUploadsPerFrame || path.empty()) return;
        if (textures_.find(cacheKey(path, add)) != textures_.end()) return;
        for (const auto& p : wanted) if (*p.first == path && p.second == add) return;
        wanted.emplace_back(&path, add);
    };
    for (const Widget* w : order) {
        if (static_cast<int>(wanted.size()) >= kUploadsPerFrame) break;
        if (w->kind == WidgetKind::Texture && !w->solidColor)
            want(w->texturePath, w->blendAdd);
        if (w->kind == WidgetKind::Frame) {
            if (w->hasBackdrop) { want(w->bgFile); want(w->edgeFile); }
            if (w->isStatusBar) want(w->barTexture);
            if (w->isSlider) want(w->thumbTexture);
        }
    }

    // One submit and one wait for the whole batch rather than one of each per
    // texture. Every upload used to be its own immediate submit, and with
    // FrameXML asking for hundreds of distinct files the seconds after a load
    // cost 70-140ms a frame. Batched, how many go in a frame stops mattering
    // much, which is why the budget can be larger and the burst shorter.
    //
    // Synchronous, because the draw below uses whatever was just uploaded; the
    // asynchronous form would let this frame sample an image whose copy has not
    // landed. Nothing to upload means no batch at all, so an idle frame does
    // not allocate a command buffer to record nothing into.
    if (!wanted.empty() && vkCtx_) {
        vkCtx_->beginUploadBatch();
        for (const auto& p : wanted) texture(*p.first, p.second);
        vkCtx_->endUploadBatchSync();
    }

    // Interface units to pixels. The tree is laid out against a virtual screen
    // 768 units tall so a frame is the same apparent size on every display;
    // this is the one place that becomes pixels.
    const float s = tree.uiScale();

    // Instrumentation, not drawing.
    reportWidgetDiagnostics(tree, order, s, screenW, screenH);

    // Behind ImGui's own windows, so the existing interface stays on top while
    // the two coexist, but still over the 3D scene.
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    if (!dl) return;

    // Level 5 draws ImGui's own font atlas through this same call, at a fixed
    // place. It is the one texture ImGui is certain to have uploaded, so it
    // separates two things that look identical from outside: whether AddImage
    // works on this draw list at all, and whether the descriptor sets this
    // renderer uploads are good. If the glyph sheet appears, the call is fine
    // and the textures are not.
    if (widgetDumpLevel() >= 5) {
        dl->AddImage(ImGui::GetIO().Fonts->TexRef, ImVec2(40.0f, 40.0f),
                     ImVec2(440.0f, 440.0f));
    }

    for (const Widget* w : order) {
        // Anything inside a scroll frame is bounded by it. Without this a
        // scroll child taller than its window draws over everything above and
        // below, which is not a window onto it at all.
        bool clipped = false;
        if (w->clipTo != 0) {
            if (const Widget* clip = tree.get(w->clipTo)) {
                // A clip that does not overlap what it is clipping removes it
                // from the screen entirely, and leaves no other trace: the
                // frame is shown, sized, positioned and simply not there.
                //
                // Every NPC dialog puts its text inside a scroll frame and the
                // vendor does not, which is the one thing the blank ones share.
                // Said once per clipping frame so a real mismatch names itself
                // instead of being inferred from what is missing.
                // Sideways only, or on both axes at once.
                //
                // A miss on the vertical alone is ordinary and is the whole
                // point of a scroll frame: the child is taller than its window
                // and the part above or below is meant to be out of sight until
                // it is scrolled to. Reporting that called every scrolled list
                // in the interface a fault - a macro button eighty pixels below
                // its own window, sitting exactly where it belongs. A
                // diagnostic that fires on correct behaviour is worse than none,
                // because the next real one is read as more of the same.
                //
                // Nothing scrolls back into view from beside its window, so a
                // horizontal miss is always wrong.
                const bool overlapsX =
                    w->left < clip->left + clip->rectW &&
                    w->left + w->rectW > clip->left;
                const bool overlapsY =
                    w->bottom < clip->bottom + clip->rectH &&
                    w->bottom + w->rectH > clip->bottom;
                if (!overlapsX && w->visible && w->rectW > 0.0f && w->rectH > 0.0f) {
                    static std::set<uint32_t> saidClipped;
                    if (saidClipped.insert(w->clipTo).second) {
                        LOG_WARNING(
                            "Clip: '", w->name.empty() ? "(unnamed)" : w->name,
                            "' at (", w->left, ",", w->bottom, " ", w->rectW,
                            "x", w->rectH, ") is entirely outside '",
                            clip->name.empty() ? "(unnamed)" : clip->name,
                            "' at (", clip->left, ",", clip->bottom, " ",
                            clip->rectW, "x", clip->rectH,
                            ")", overlapsY ? " - off to the side of its own "
                                             "window, which nothing scrolls back"
                                           : " - outside it on both axes",
                            "; it is shown and clipped away");
                    }
                }
                dl->PushClipRect(ImVec2(clip->left * s,
                                        screenH - (clip->bottom + clip->rectH) * s),
                                 ImVec2((clip->left + clip->rectW) * s,
                                        screenH - clip->bottom * s), true);
                clipped = true;
            }
        }
        struct ClipGuard {
            ImDrawList* dl; bool on;
            ~ClipGuard() { if (on) dl->PopClipRect(); }
        } clipGuard{dl, clipped};

        // WoW measures from the bottom-left and upward; the screen measures from
        // the top-left and downward. Flip here, at the one place it matters, so
        // every anchor rule upstream reads the way Blizzard documents it.
        // A rect that is not finite, or larger than any screen, is not drawn.
        //
        // ImGui takes these straight into vertex positions, and a triangle
        // with an infinity or a NaN in it is one the rasteriser can chew on
        // until the driver's watchdog resets the device - which surfaces as
        // several seconds inside endFrame and then VK_ERROR_DEVICE_LOST, with
        // nothing to say which frame did it. Named once so there is.
        {
            const float vals[4] = {w->left, w->bottom, w->rectW, w->rectH};
            bool sane = true;
            for (float v : vals) {
                if (!std::isfinite(v) || std::fabs(v) > 1.0e6f) { sane = false; break; }
            }
            if (!sane) {
                static std::set<uint32_t> reported;
                if (reported.insert(w->id).second) {
                    LOG_WARNING("Not drawing '",
                                w->name.empty() ? "(unnamed)" : w->name.c_str(),
                                "': its rect is (", w->left, ", ", w->bottom, " ",
                                w->rectW, "x", w->rectH,
                                ") - a value like that reaches the rasteriser as "
                                "geometry it may never finish");
                }
                continue;
            }
        }

        const float x0 = w->left * s;
        const float y0 = screenH - (w->bottom + w->rectH) * s;
        const float x1 = (w->left + w->rectW) * s;
        const float y1 = screenH - w->bottom * s;

        if (w->kind == WidgetKind::Frame) {
            // Whatever the client rendered for it, under its own regions -
            // a model frame is a window onto a scene and the art around it
            // belongs on top.
            if (w->externalTexture != 0) {
                // NOLINTNEXTLINE(performance-no-int-to-ptr) - the widget holds
                // the handle as an integer and ImTextureID is a pointer, so
                // the cast is the API boundary rather than a choice.
                dl->AddImage(reinterpret_cast<ImTextureID>(
                                 reinterpret_cast<VkDescriptorSet>(w->externalTexture)),
                             ImVec2(x0, y0), ImVec2(x1, y1),
                             ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
                             packColor(w->color, w->alpha));
            }
            if (w->hasBackdrop) drawBackdrop(dl, *w, s, x0, y0, x1, y1);
            if (w->isStatusBar) drawStatusBar(dl, *w, x0, y0, x1, y1);
            if (w->isSlider) drawSlider(dl, *w, x0, y0, x1, y1);
            if (w->isCooldown) drawCooldown(dl, *w, x0, y0, x1, y1);
            // A tooltip reads downward from the top, which is the other way
            // round from chat.
            if (w->isTooltip && w->objectType == "GameTooltip" &&
            !w->tooltipLines.empty()) {
                ImFont* font = interfaceFaceOrDefault(w->fontFace);
                const float size = interfaceFontSize(w->fontHeight) * s;
                const float lineH = size * 1.2f;
                const float pad = 10.0f * s;
                float y = y0 + pad;
                const float textW = (x1 - x0) - pad * 2.0f;
                for (const auto& line : w->tooltipLines) {
                    float lc[4] = {line.lc[0], line.lc[1], line.lc[2], line.lc[3]};
                    drawMarkupText(dl, font, size, ImVec2(x0 + pad, y),
                                   packColor(lc, w->alpha), w->alpha, line.left,
                                   line.wrap ? textW : 0.0f);
                    if (!line.right.empty()) {
                        float rc[4] = {line.rc[0], line.rc[1], line.rc[2], line.rc[3]};
                        const float rw = font->CalcTextSizeA(
                            size, FLT_MAX, 0.0f, strippedText(line.right).c_str()).x;
                        drawMarkupText(dl, font, size, ImVec2(x1 - pad - rw, y),
                                       packColor(rc, w->alpha), w->alpha, line.right);
                    }
                    // A wrapped line is as tall as the rows it produced, which
                    // the sizing pass counted - otherwise the next line draws
                    // over the middle of this one.
                    y += lineH * static_cast<float>(line.lines > 0 ? line.lines : 1);
                }
            }

            // Chat and its kind: the newest line sits at the bottom and the
            // older ones stack upward, as many as the frame is tall enough to
            // hold. Scrolling moves the window back through the history
            // rather than moving the lines.
            if (w->isMessageFrame && !w->messages.empty()) {
                ImFont* font = interfaceFaceOrDefault(w->fontFace);
                const float size = interfaceFontSize(w->fontHeight) * s;
                const float lineH = size * 1.15f + w->messagePadding * s;
                // A chat line is wrapped to the frame, not run off the end of
                // it. Every other label in the interface has wrapped for as
                // long as there has been a wrapper; this surface drew each
                // message as one unbroken line, so anything longer than the
                // window simply left it.
                const float wrapW = (x1 - x0) > size ? (x1 - x0) : 0.0f;
                auto lineCount = [&](const std::string& text) {
                    if (wrapW <= 0.0f) return size_t{1};
                    const auto lines = wrapText(
                        parseMarkup(text), wrapW, false,
                        [&](const std::string& piece) {
                            return font->CalcTextSizeA(size, FLT_MAX, 0.0f,
                                                       piece.c_str()).x;
                        });
                    return lines.empty() ? size_t{1} : lines.size();
                };
                float y = y1 - lineH;
                int painted = 0;
                const int scroll = w->messageScroll;
                for (int i = static_cast<int>(w->messages.size()) - 1 - scroll;
                     i >= 0 && y >= y0 - lineH; --i) {
                    const auto& m = w->messages[static_cast<size_t>(i)];
                    // A line whose time is up is still in the history - it
                    // comes back when the frame is scrolled - but it takes no
                    // room on screen, so the ones still lit pack together.
                    if (m.color[3] <= 0.0f) continue;
                    float rgba[4] = {m.color[0], m.color[1], m.color[2], m.color[3]};
                    // Through the markup parser, not straight to AddText.
                    //
                    // A chat line is the densest markup the interface produces:
                    // "|cff9d9d9d|Hitem:3299|h[Fractured Canine]|h|r" is one
                    // item link with a colour around it, and drawn raw that is
                    // what appeared on screen - every escape, every bar, in the
                    // middle of the sentence. It is also where the links are,
                    // so this is what files their rects and makes a click on
                    // one land somewhere.
                    // A wrapped message occupies several lines and grows
                    // upward, because the newest sits at the bottom: the block
                    // starts as many lines above the cursor as it needs, and
                    // the cursor then moves past all of them.
                    const size_t rows = lineCount(m.text);
                    const float top = y - static_cast<float>(rows - 1) * lineH;
                    drawMarkupText(dl, font, size, ImVec2(x0, top),
                                   packColor(rgba, w->alpha), w->alpha, m.text,
                                   wrapW, false, nullptr, false, &tree, w->id);
                    y -= lineH * static_cast<float>(rows);
                    ++painted;
                }
                // A window holding lines and painting none of them.
                //
                // Reported as no visible text at all in the chat window, and
                // every way of asking from outside says it is fine: the frame
                // is shown, sized, in the draw order, its lines are there and
                // lit. The harness agrees, because the harness never draws.
                // This is the one place that knows the difference, so it is the
                // one place that can say which of the reasons it was.
                //
                // Once per frame per session - it means nothing is on screen,
                // so it cannot be noisy for long.
                if (painted == 0) {
                    int lit = 0;
                    for (const auto& m : w->messages) {
                        if (m.color[3] > 0.0f) ++lit;
                    }
                    // A frame that fades its lines and has none left lit is
                    // doing its job, not failing at it - UIErrorsFrame holds
                    // errors for five seconds and is supposed to end up empty.
                    // The fault is lines that are lit and still not painted,
                    // or any line at all in a frame that never fades.
                    const bool fadesAndHasFaded = w->messageDuration > 0.0f && lit == 0;
                    static std::set<uint32_t> saidBlank;
                    if (!fadesAndHasFaded && saidBlank.insert(w->id).second) {
                        LOG_WARNING("'", w->name.empty() ? "(unnamed)" : w->name,
                                    "' holds ", w->messages.size(),
                                    " message(s), ", lit,
                                    " still lit, and painted none: box is (",
                                    x0, ",", y0, " to ", x1, ",", y1,
                                    "), line height ", lineH, ", scrolled back ",
                                    w->messageScroll,
                                    " - if the box has no height there is "
                                    "nowhere to draw, and if nothing is lit "
                                    "they have all faded");
                    }
                }
            }

            if (w->isEditBox) {
                // Its own text, drawn where a label would be, with a caret
                // while it has focus so it is clear which box is listening.
                ImFont* font = interfaceFaceOrDefault(w->fontFace);
                const float size = interfaceFontSize(w->fontHeight) * s;
                const uint32_t col = packColor(w->color, w->alpha);
                // Between the top and bottom insets rather than the whole
                // frame, so a box with art above its text does not sit high.
                const float boxTop = y0 + w->textInsetTop * s;
                const float boxBottom = y1 - w->textInsetBottom * s;
                const float ty = boxTop + ((boxBottom - boxTop) - size) * 0.5f;
                const float pad = w->textInsetLeft * s;
                if (!w->editText.empty()) {
                    // Through the markup parser like everything else. An edit
                    // box holds what the player typed, which used to mean plain
                    // words - but shift-clicking a link puts the whole
                    // "|Hitem:3299|h[Fractured Canine]|h" into it, and drawn
                    // raw that is what the player sees themselves typing.
                    // Links are only clickable at all as of this branch, so
                    // this became reachable at the same moment.
                    drawMarkupText(dl, font, size, ImVec2(x0 + pad, ty), col,
                                   w->alpha, w->editText);
                }
                if (w->editFocused) {
                    // Measured against what is drawn, not what is held. The
                    // caret sits after the text to its left, and to the left of
                    // a link is its display name - measuring the raw string
                    // would put the caret an escape sequence too far right.
                    const std::string upTo = strippedText(w->editText.substr(
                        0, std::min(w->cursorPos, w->editText.size())));
                    const float caret = font
                        ? font->CalcTextSizeA(size, FLT_MAX, 0.0f, upTo.c_str()).x
                        : 0.0f;
                    const float cx = x0 + pad + caret;
                    dl->AddLine(ImVec2(cx, ty), ImVec2(cx, ty + size), col);
                }
            }
            continue;
        }

        // Level 3 outlines every widget where it believes it is. If the
        // outlines appear and the art does not, the images are the problem; if
        // neither appears, the whole layer is being covered or discarded. That
        // is two possibilities told apart by looking, rather than inferred from
        // a screenshot.
        if (widgetDumpLevel() >= 3) {
            dl->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(255, 0, 255, 200));
        }
        // Level 4 paints every widget solid instead of drawing its art. An
        // outline can be missed against a busy scene and dark art can be
        // mistaken for nothing at all; a solid block either covers the bottom
        // of the screen or it does not, and that answers whether these pixels
        // are reached without anyone having to squint at a screenshot.
        if (widgetDumpLevel() >= 4) {
            dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1),
                              IM_COL32(255, 0, 255, 255));
            continue;
        }

        if (w->kind == WidgetKind::Texture &&
            w->colorRole != Widget::ColorRole::None) {
            // The colour picker's own art, which no file supplies: a wheel of
            // every hue and a bar of every brightness, both of them a function
            // of the colour the ColorSelect parent is holding rather than an
            // image of anything.
            const Widget* picker = tree.get(w->parent);
            if (!picker) continue;
            drawColorPicker(dl, tree, *w, *picker, screenH, x0, y0, x1, y1);
            continue;
        }

        if (w->kind == WidgetKind::Texture) {
            // A colour set with SetTexture(r,g,b) fills; a texture with no
            // file at all draws nothing. Treating the second as the first
            // painted every undecided region in the default white, which for a
            // full-width backdrop is a white slab across the screen.
            if (w->solidColor) {
                dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1),
                                  packColor(w->color, w->alpha));
                continue;
            }
            // A texture the client renders into has no file and does not need
            // one, so the "nothing to draw" test has to come after that and not
            // before it. Ahead of it, the player's portrait was discarded every
            // frame: PlayerPortrait is declared in playerframe.xml with no file
            // because the picture is a character rendered offscreen, which is
            // exactly what an undecided texture looks like from here. The
            // paperdoll survived only because CharacterModelFrame is a Frame
            // rather than a Texture and takes a different branch.
            if (w->texturePath.empty() && w->externalTexture == 0) continue;
            VkDescriptorSet tex = VK_NULL_HANDLE;
            if (w->externalTexture != 0) {
                // Supplied by the client, and only valid for as long as it says
                // so - a portrait's render target is recreated when the window
                // resizes, and the widget is told each frame rather than
                // holding a handle of its own.
                // NOLINTNEXTLINE(performance-no-int-to-ptr) - as above, the
                // handle is stored as an integer and VkDescriptorSet is a
                // pointer.
                tex = reinterpret_cast<VkDescriptorSet>(w->externalTexture);
            } else {
                // Only what is already resident. Anything still queued draws on
                // a later frame rather than forcing an upload here.
                auto it = textures_.find(cacheKey(w->texturePath, w->blendAdd));
                if (it == textures_.end() || it->second == kMissing) continue;
                tex = it->second;
            }
            if (tex == VK_NULL_HANDLE) continue;
            // SetTexCoord is left/right/top/bottom in WoW's own order, and its
            // vertical sense already matches the image, so it passes through.
            //
            // Except for a texture the client supplied: those coordinates were
            // chosen for the file the interface believes is there, and this is
            // a whole image rendered in its place. Applying them cropped the
            // player's portrait to whichever quarter of the class-circle atlas
            // SetPortraitTexture had picked out.
            //
            // What those coordinates were doing, though, still has to happen.
            // MicroButtonPortrait crops 0.2-0.8 across to fit a square face
            // into an 18x25 slot; ignoring that and drawing the whole image
            // into the slot squeezed the character narrow. So a live texture
            // is cropped to the shape of the frame instead of to numbers meant
            // for another file - the same trim, decided from what is actually
            // being drawn. A square frame takes the whole image, which is every
            // portrait frame in the interface but this one.
            const bool live = (w->externalTexture != 0);
            ImVec2 uv0 = live ? ImVec2(0.0f, 0.0f)
                              : ImVec2(w->texCoord[0], w->texCoord[2]);
            ImVec2 uv1 = live ? ImVec2(1.0f, 1.0f)
                              : ImVec2(w->texCoord[1], w->texCoord[3]);
            if (live) {
                const float rectW = x1 - x0;
                const float rectH = y1 - y0;
                if (rectW > 0.0f && rectH > 0.0f) {
                    const float aspect = rectW / rectH;
                    // Half-extents about the middle of a square source.
                    const float halfU = aspect > 1.0f ? 0.5f : 0.5f * aspect;
                    const float halfV = aspect > 1.0f ? 0.5f / aspect : 0.5f;
                    uv0 = ImVec2(0.5f - halfU, 0.5f - halfV);
                    uv1 = ImVec2(0.5f + halfU, 0.5f + halfV);
                }
            }
            if (!live && w->texCoordRotated) {
                // A UV per corner, so the art can sit in the frame at any
                // angle. WoW's order is upper-left, lower-left, upper-right,
                // lower-right; the quad wants them going round the rect.
                const float* q = w->texCoordQuad;
                dl->AddImageQuad(reinterpret_cast<ImTextureID>(tex),
                                 ImVec2(x0, y0), ImVec2(x1, y0),
                                 ImVec2(x1, y1), ImVec2(x0, y1),
                                 ImVec2(q[0], q[1]),   // upper-left
                                 ImVec2(q[4], q[5]),   // upper-right
                                 ImVec2(q[6], q[7]),   // lower-right
                                 ImVec2(q[2], q[3]),   // lower-left
                                 packColor(w->color, w->alpha));
            } else {
                dl->AddImage(reinterpret_cast<ImTextureID>(tex),
                             ImVec2(x0, y0), ImVec2(x1, y1), uv0, uv1,
                             packColor(w->color, w->alpha));
            }
        } else if (w->kind == WidgetKind::FontString) {
            // Font objects carry a height, and honouring it is most of what
            // makes a label look right - a heading and a footnote are the same
            // words at different sizes. The atlas holds one face, so this scales
            // it rather than swapping fonts; loading FRIZQT__ properly needs an
            // atlas rebuild, which cannot happen while a frame is being built.
            ImFont* font = interfaceFace(w->fontFace);
            // The interface's own default, not the client's: ImGui draws with
            // whatever was added first, and that is deliberately the built-in
            // face so this client's panels are left alone.
            if (!font) font = interfaceFace("frizqt__");
            if (!font) font = ImGui::GetFont();
            const float size = interfaceFontSize(w->fontHeight) * s;
            // A label whose width came from its own text has nothing to wrap
            // to; one given a width by its XML or by two anchors wraps inside
            // it. Nothing wrapped before, so every label of the second kind
            // drew one line straight out of its own frame.
            // Wider than one glyph, not merely positive. A label whose rect
            // has not been laid out yet reports a box a fraction of a pixel
            // wide, and wrapping to that puts one word on every line and makes
            // the label a hundred lines tall - worse than the overflow this
            // exists to stop.
            const float boxWidth = x1 - x0;
            // A region one line tall cannot show two. Wrapping there does not
            // fit the text, it spills it over whatever is drawn beneath - and
            // in a list of fixed-height rows that is the next row.
            //
            // The quest log is the case. QuestLogTitleButton_Resize measures the
            // title, works out how much of it would overrun the (Complete) tag,
            // and calls SetWidth with the room that is actually left. That is a
            // request to CUT the title there, not to reflow it: the row is 16
            // tall and the font string declares 10, one line. Reflowing put the
            // tail of every long quest name - and of any zone header wider than
            // its box - on top of the row below it.
            //
            // Measured in pixels on both sides. rectH is in the interface's own
            // units and `size` is already scaled, so comparing the two directly
            // would read as one line or many purely by the window's size.
            const float boxHeightPx = y1 - y0;
            const bool fitsOneLineOnly = (boxHeightPx > 0.0f && boxHeightPx < size * 1.8f);
            const float wrapW =
                (!w->autoSized && w->wordWrap && !fitsOneLineOnly && boxWidth > size)
                ? boxWidth : 0.0f;
            // Held to its box when the box is what decides its width. A string
            // sized from its own text has no box to be held to and is left
            // alone; one given a width has been told how much room it gets, so
            // the part that does not fit is cut rather than drawn over the tag
            // sitting beside it.
            //
            // Across only, never down: a font string's declared height is the
            // line it sits on rather than a box drawn around the glyphs - the
            // quest log's is 10 for a 12-point line - so clipping vertically to
            // it would shave the descenders off every label in the interface.
            const bool clipToBox = !w->autoSized && boxWidth > 0.0f;
            if (clipToBox) {
                dl->PushClipRect(ImVec2(x0, y0 - size), ImVec2(x1, y1 + size), true);
            }
            struct ClipPop {
                ImDrawList* dl; bool on;
                ~ClipPop() { if (on) dl->PopClipRect(); }
            } clipPop{dl, clipToBox};
            ImVec2 extent =
                font ? font->CalcTextSizeA(size, FLT_MAX, 0.0f, w->text.c_str())
                     : ImGui::CalcTextSize(w->text.c_str());
            // Counted even when nothing wraps: |n is a line break at any
            // width, so a label with one in it is two lines tall whether or
            // not it is also being broken to fit.
            if (font) {
                const auto lines = wrapText(
                    parseMarkup(w->text), wrapW, w->nonSpaceWrap,
                    [&](const std::string& piece) {
                        return font->CalcTextSizeA(size, FLT_MAX, 0.0f,
                                                   piece.c_str()).x;
                    });
                w->wrappedLines = static_cast<int>(lines.size());
                if (wrapW > 0.0f) {
                    extent.x = wrapW;
                    extent.y = size * 1.2f * static_cast<float>(lines.size());
                }
            } else {
                w->wrappedLines = 1;
            }
            // Against the box in pixels, not in interface units. rectW and
            // rectH are the widget's own units and extent comes back from a
            // font already scaled by s, so centring on the raw rect placed a
            // label off by half the difference between the two - around half
            // the box's width at this scale, which is most of the way out of it.
            // A held button moves its label. The offset is declared on the
            // button, and this is the label, so it comes from the parent - and
            // only while that parent is the frame being held.
            float pushX = 0.0f, pushY = 0.0f;
            if (const uint32_t held = tree.pressedWidget(); held != 0) {
                if (const Widget* owner = tree.get(w->parent)) {
                    if (owner->id == held &&
                        (owner->pushedTextOffsetX != 0.0f ||
                         owner->pushedTextOffsetY != 0.0f)) {
                        pushX = owner->pushedTextOffsetX * s;
                        // Down the screen for a negative y, as with the shadow:
                        // the interface counts y upward and this draws downward.
                        pushY = -owner->pushedTextOffsetY * s;
                    }
                }
            }
            const float boxW = x1 - x0, boxH = y1 - y0;
            float tx = x0;
            if (wrapW <= 0.0f) {
                if (w->justifyH == "CENTER")     tx = x0 + (boxW - extent.x) * 0.5f;
                else if (w->justifyH == "RIGHT") tx = x1 - extent.x;
            }
            float ty = y0 + (boxH - extent.y) * 0.5f;
            if (w->justifyV == "TOP")         ty = y0;
            else if (w->justifyV == "BOTTOM") ty = y1 - extent.y;
            tx += pushX;
            ty += pushY;
            // An outline is drawn as the same glyphs in black around the text.
            // ImGui has no outlined draw, and offsetting a few copies is what
            // the effect amounts to at these sizes - it is what keeps a
            // nameplate legible against whatever is behind it.
            if (!w->fontOutline.empty()) {
                const float d = (w->fontOutline == "THICK") ? 2.0f : 1.0f;
                const uint32_t shadow = IM_COL32(0, 0, 0,
                    static_cast<int>(std::clamp(w->alpha, 0.0f, 1.0f) * 255.0f));
                const ImVec2 around[8] = {
                    {-d, 0}, {d, 0}, {0, -d}, {0, d},
                    {-d, -d}, {d, -d}, {-d, d}, {d, d},
                };
                for (const ImVec2& o : around) {
                    // Through the same wrap as the text itself. Drawn straight
                    // it was one long line behind a wrapped label - a second
                    // copy of the words in a darker shade, out of line with
                    // the ones on top of it.
                    drawMarkupText(dl, font, size, ImVec2(tx + o.x, ty + o.y),
                                   shadow, w->alpha, w->text, wrapW,
                                   w->nonSpaceWrap, w->justifyH.c_str(), true);
                }
            }
            // The shadow sits under the text and over the outline: it is a
            // single offset copy rather than a ring, and the offset is in
            // interface units like everything the font object states, so it
            // scales with the rest.
            if (w->hasShadow) {
                float sc[4] = {w->shadowColor[0], w->shadowColor[1],
                               w->shadowColor[2], w->shadowColor[3]};
                drawMarkupText(dl, font, size,
                               ImVec2(tx + w->shadowX * s, ty - w->shadowY * s),
                               packColor(sc, w->alpha * w->shadowColor[3]),
                               w->alpha, w->text, wrapW, w->nonSpaceWrap,
                               w->justifyH.c_str(), true);
            }
            // A button's label takes its colour from the button's state. The
            // colour lives on the parent because that is where the template
            // declares it; only the colour changes, never the size, or a label
            // would jump about as the cursor crossed it.
            const float* textColor = w->color;
            if (const Widget* owner = tree.get(w->parent)) {
                if (!owner->enabled && owner->hasDisabledColor) {
                    textColor = owner->disabledColor;
                } else if (owner->enabled && owner->hasHighlightColor &&
                           owner->id == tree.hoveredWidget()) {
                    textColor = owner->highlightColor;
                }
            }
            drawMarkupText(dl, font, size, ImVec2(tx, ty),
                           packColor(textColor, w->alpha), w->alpha, w->text,
                           wrapW, w->nonSpaceWrap, w->justifyH.c_str(), false,
                           &tree, w->id);
        }
    }
}

} // namespace ui
} // namespace wowee
