#pragma once

// The interface's typefaces, by the name FrameXML calls them.
//
// A font object names its face as a path - "Fonts\MORPHEUS.ttf" - and the file
// on disk is lower case, so lookups go through a normalised key rather than the
// spelling either side happens to use. Faces are registered once, before the
// first frame, because the glyph atlas is built then and cannot take another
// afterwards without being torn down.

#include <cstddef>
#include <string>

struct ImFont;

namespace wowee::ui {

/// Records a face under a path or filename. Later registrations of the same
/// face replace the earlier one.
void registerInterfaceFace(const std::string& pathOrName, ImFont* font);

/// The face for a path a font object named, or null if it was never loaded -
/// in which case the caller should draw with whatever it was already using.
ImFont* interfaceFace(const std::string& pathOrName);

/// How wide a piece of text is, in interface units.
///
/// The one answer to that question. It was asked twice with two different
/// answers: the renderer drew a font string in the interface's own face at the
/// widget's height, and GetTextWidth measured it in whatever face this client
/// had current at a flat twelve points. Anything that sizes itself from its own
/// text - and MoneyFrame sizes all three coin buttons that way - came out
/// narrower than the digits drawn in it, so the numbers ran into each other.
///
/// `fontFace` is the widget's own, `fontHeight` its own point size; zero for
/// either means the default, which is what the renderer falls back to too.
float interfaceTextWidth(const std::string& text, const std::string& fontFace,
                         float fontHeight);

/// The face a widget draws in, resolved the way the renderer resolves it: its
/// own, then the interface default, then whatever is current.
ImFont* interfaceFaceOrDefault(const std::string& fontFace);

/// The point size a widget draws at, given the height it asked for.
float interfaceFontSize(float fontHeight);

/// What ImGui's rasterizer scale has to be multiplied by so a face asked for at
/// height N draws with its EM square N pixels tall.
///
/// FrameXML's <FontHeight> is an em size, the way every traditional font API
/// and CSS mean "font size": ten means the em square is ten pixels. ImGui asks
/// stb_truetype for stbtt_ScaleForPixelHeight instead, which fits the whole
/// ascender-to-descender span into those ten pixels - stb's own header calls
/// the other one "probably what traditional APIs compute". FRIZQT__ spans 1215
/// units of a 1000-unit em, so every label in the interface was drawn and
/// measured at 82.3% of the size WoW draws it.
///
/// Smaller text is the visible half. The other half is that anything positioned
/// against a caption's right edge lands short by 18% of that caption's width,
/// and the interface anchors a great deal that way: the auction house's rarity
/// dropdown hangs off "Level Range" and came to rest on top of the level boxes.
///
/// One (ascent - descent) / unitsPerEm, read off the file's own head and hhea
/// tables. Faces differ, so it is asked per face rather than written down.
/// Answers 1 for anything it cannot parse, which leaves ImGui's own scaling.
float fontEmSizeScale(const void* ttfData, size_t byteCount);

/// The same, for a face still on disk. Answers 1 if the file cannot be read.
float fontEmSizeScaleOfFile(const std::string& path);

} // namespace wowee::ui
