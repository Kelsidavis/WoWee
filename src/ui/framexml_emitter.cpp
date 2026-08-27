#include "ui/framexml_emitter.hpp"
#include <cctype>

#include "ui/xml_parser.hpp"

#include <algorithm>
#include <set>
#include <sstream>

namespace wowee {
namespace ui {

namespace {

/// Element names that produce a frame rather than a region. Everything here is
/// created through CreateFrame with its own type, so a Button gets a Button's
/// behaviour even where the widget system does not yet distinguish them.
bool isFrameElement(const std::string& n) {
    static const char* kFrames[] = {
        "Frame", "Button", "CheckButton", "StatusBar", "Slider", "EditBox",
        "ScrollFrame", "ScrollingMessageFrame", "MessageFrame", "SimpleHTML",
        "ColorSelect", "Model", "PlayerModel", "DressUpModel", "TabardModel",
        "Cooldown", "GameTooltip", "MovieFrame", "ArchaeologyDigSiteFrame",
        // Both are ordinary frames as far as this goes - the client draws what
        // is inside them. Leaving Minimap out skipped the whole minimap
        // subtree, which is why every MinimapNorthTag and MiniMapLFGFrame in
        // the interface read as a missing global.
        "Minimap", "WorldFrame"
    };
    for (const char* f : kFrames) if (n == f) return true;
    return false;
}

std::string quote(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:   out += c;
        }
    }
    out += "\"";
    return out;
}

/// A <Size> or <Offset> can be written as a child <AbsDimension x= y=> or, in
/// later files, as attributes directly on the element.
bool readDimension(const XmlNode& node, float& x, float& y) {
    if (const XmlNode* abs = node.child("AbsDimension")) {
        x = abs->attrFloat("x", 0.0f);
        y = abs->attrFloat("y", 0.0f);
        return true;
    }
    if (node.attr("x") || node.attr("y")) {
        x = node.attrFloat("x", 0.0f);
        y = node.attrFloat("y", 0.0f);
        return true;
    }
    return false;
}


/// The argument names a handler's body expects to find in scope. Blizzard's
/// inline scripts use them without declaring them, so they have to be the
/// function's parameters.
std::string scriptParameters(const std::string& script) {
    if (script == "OnUpdate")        return "self, elapsed";
    if (script == "OnEvent")         return "self, event, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9";
    if (script == "OnClick")         return "self, button, down";
    if (script == "OnDoubleClick")   return "self, button";
    if (script == "OnMouseDown" ||
        script == "OnMouseUp")       return "self, button";
    if (script == "OnDragStart" ||
        script == "OnDragStop" ||
        script == "OnReceiveDrag")   return "self, button";
    if (script == "OnEnter" ||
        script == "OnLeave")         return "self, motion";
    if (script == "OnChar")          return "self, text";
    if (script == "OnKeyDown" ||
        script == "OnKeyUp")         return "self, key";
    if (script == "OnValueChanged")  return "self, value";
    if (script == "OnTextChanged")   return "self, isUserInput";
    if (script == "OnMouseWheel")    return "self, delta";
    if (script == "OnSizeChanged")   return "self, width, height";
    if (script == "OnAttributeChanged") return "self, name, value";
    if (script == "OnHyperlinkClick" ||
        script == "OnHyperlinkEnter" ||
        script == "OnHyperlinkLeave") return "self, link, text, button";
    if (script == "OnTooltipSetItem" ||
        script == "OnTooltipSetUnit") return "self";
    // Scrolling. UIPanelScrollFrameTemplate's own OnVerticalScroll body opens
    // with scrollbar:SetValue(offset), and without the name that was nil on
    // every scroll frame in the interface.
    if (script == "OnVerticalScroll" ||
        script == "OnHorizontalScroll") return "self, offset";
    if (script == "OnScrollRangeChanged") return "self, xrange, yrange";
    if (script == "OnCursorChanged")   return "self, x, y, width, height";
    if (script == "OnColorSelect")     return "self, r, g, b";
    if (script == "OnMinMaxChanged")   return "self, min, max";
    if (script == "OnTooltipAddMoney") return "self, cost, maxcost";
    if (script == "OnTooltipSetDefaultAnchor") return "self, parent";
    if (script == "OnMovieShowSubtitle")       return "self, text";
    if (script == "OnInputLanguageChanged")    return "self, language";
    if (script == "OnFinished")        return "self, requested";
    // OnLoad, OnShow, OnHide and the rest take only self.
    return "self";
}

/// Named parameters, and then varargs regardless. A body is free to use `...`
/// whatever handler it belongs to, and a parameter list without it does not
/// merely leave the values behind - it fails to compile, taking the whole
/// template with it.
std::string scriptSignature(const std::string& script) {
    // OnEvent alone takes its arguments through the varargs rather than by
    // name. Both spellings are in the interface - plenty of bodies say `arg1`,
    // and fifty-three hand `...` straight on to a Lua function - and a named
    // parameter list serves only the first: `function(self, event, arg1, ...,
    // arg9, ...)` binds every value that arrives to a name, leaving `...`
    // empty for the body that forwards it.
    //
    // ContainerFrame is the one that shows: its whole OnEvent body is
    // ContainerFrame_OnEvent(self, event, ...), which opens with
    // `local arg1, arg2 = ...` and compares arg1 against the bag's own id. With
    // nothing in the varargs that comparison was false for every bag on every
    // BAG_UPDATE, so a bag never redrew while it was open - an item moved
    // within one, or out of it, stayed on screen where it had been.
    //
    // The names are bound inside instead, off the varargs, which serves both.
    if (script == "OnEvent") return "self, event, ...";
    return scriptParameters(script) + ", ...";
}

/// Statements to run before an inline body, or empty.
///
/// Only OnEvent has any: its argN names are locals taken off the varargs
/// rather than parameters, so that a body forwarding `...` still has something
/// to forward. Nine because that is how many the client's dispatcher will
/// send, and the same nine scriptParameters used to name.
std::string scriptPrelude(const std::string& script) {
    if (script != "OnEvent") return std::string();
    return "local arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9 = ...; ";
}

struct Emitter {
    EmitResult result;
    /// Virtual frames declared in this file, so an element written as one of
    /// their names is known to be a template rather than a typo. A template
    /// from another file is not in here and still reports itself - the frame is
    /// built either way, and the report is what says the name was a guess.
    std::set<std::string> virtualFrames;
    int temp = 0;
    /// True while emitting a template body. Inside one the owning frame is not
    /// known until the template is replayed, so $parent has to be resolved then
    /// rather than now.
    bool runtimeParentName = false;

    /// Temporaries live in a table rather than in locals. Lua allows 200 locals
    /// per function and a large file declares far more widgets than that -
    /// FriendsFrame and InterfaceOptionsPanels both went over, and the failure
    /// is the whole chunk refusing to compile rather than anything degrading.
    std::string nextVar() { return "__w[" + std::to_string(++temp) + "]"; }

    /// The Lua expression for a region or frame's name. A literal where the
    /// owning frame is known, and a concatenation against the real frame's name
    /// where it is not - which is what makes $parentBackdrop inside a template
    /// become FooFrameBackdrop on the frame that inherits it, rather than
    /// naming itself after the template.
    std::string nameArg(const std::string& rawName, const std::string& parentName,
                        const std::string& selfVar) {
        if (rawName.empty()) return "nil";
        const std::string token = "$parent";
        const bool isParented = rawName.compare(0, token.size(), token) == 0;
        if (!isParented) return quote(rawName);
        const std::string suffix = rawName.substr(token.size());
        if (runtimeParentName) {
            // No name on the owner means no name on the region, which is what
            // $parent means in WoW: a name is built from one, and there is
            // nothing to build from. Falling back to an empty string instead
            // published the bare suffix as a global - ContainerFrameTemplate
            // replayed onto an unnamed frame created a texture called
            // "Portrait", and every later frame doing the same overwrote it.
            return "(" + selfVar + ":GetName() and (" + selfVar +
                   ":GetName() .. " + quote(suffix) + ") or nil)";
        }
        return quote(parentName + suffix);
    }

    void line(const std::string& s) { result.lua += s; result.lua += "\n"; }

    void emitScripts(const XmlNode& scripts, const std::string& var) {
        for (const XmlNode& s : scripts.children) {
            // <OnClick function="Foo"/> names an existing global; an inline body
            // is a function literal. Both end up as the same SetScript call.
            // Present but empty is not a name. Emitted as one it produces
            // SetScript("X", ) - a syntax error that loses the whole file, not
            // just the handler.
            if (const std::string* fn = s.attr("function"); fn && !fn->empty()) {
                line(var + ":SetScript(" + quote(s.name) + ", " + *fn + ")");
                continue;
            }
            std::string body = s.text;
            if (body.find_first_not_of(" \t\r\n") == std::string::npos) continue;
            // Each handler's arguments have names, and the body uses them
            // directly without declaring them: an OnUpdate says `elapsed`, an
            // OnClick says `button`. Passing them positionally as arg1..argN
            // left those names nil, so every one of these bodies failed the
            // moment it touched its own argument - arithmetic on a nil elapsed
            // being the loudest of them.
            line(var + ":SetScript(" + quote(s.name) +
                 ", function(" + scriptSignature(s.name) + ") " +
                 scriptPrelude(s.name) + body + " end)");
        }
    }

    /// Returns the variable holding the region, so a caller that has to hand it
    /// to a setter afterwards - button art does - can name it. isTexture is
    /// explicit because button art does not carry it in the element name:
    /// <NormalTexture> is a texture and <ButtonText> a font string.
    std::string emitRegion(const XmlNode& node, const std::string& parentVar,
                           const std::string& parentName, const std::string& layerName,
                           bool isTexture, const std::string& nameVar = std::string()) {
        const std::string var = nextVar();
        const std::string rawName = node.attrOr("name", "");

        line(var + " = " + parentVar +
             (isTexture ? ":CreateTexture(" : ":CreateFontString(") +
             nameArg(rawName, parentName, nameVar.empty() ? parentVar : nameVar) +
             ", " + quote(layerName) + ")");

        emitParentKey(node, var, parentVar);
        emitRegionProperties(node, var, parentVar, parentName, isTexture);
        return var;
    }

    /// text="CHARACTER" names a global, not a caption.
    ///
    /// WoW looks the name up and falls back to the literal when there is no
    /// such global, which is how every tab and button in FrameXML gets a
    /// localised label out of one word of markup - 703 of them.
    ///
    /// It has to be applied to frames as well as regions. A <Button text="X">
    /// is a frame, and this only ran for textures and font strings, so no
    /// button in the interface was ever given its label. The character sheet's
    /// tabs show what that costs twice over: unlabelled, and - because a tab is
    /// sized from the width of its own text - collapsed to slivers as well.
    void emitTextAttr(const XmlNode& node, const std::string& var) {
        const std::string* text = node.attr("text");
        if (!text) return;
        bool looksLikeKey = !text->empty();
        for (char c : *text) {
            if (!(c == '_' || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
                looksLikeKey = false;
                break;
            }
        }
        if (looksLikeKey) {
            line(var + ":SetText(_G[" + quote(*text) + "] or " + quote(*text) + ")");
        } else {
            line(var + ":SetText(" + quote(*text) + ")");
        }
    }

    /// Everything a region declares about itself, applied to a region that
    /// already exists. Separate from creating one because a virtual Texture or
    /// FontString is exactly this and nothing else: a set of properties waiting
    /// for something to inherit them.
    void emitRegionProperties(const XmlNode& node, const std::string& var,
                              const std::string& parentVar,
                              const std::string& parentName, bool isTexture) {
        // The template first, so anything the region states itself wins over
        // what it inherited - the order a frame's template follows.
        //
        // A FontString's inherits is ambiguous by design: nearly always a font
        // object, which is where its size and colour come from, but FrameXML
        // also declares virtual FontStrings. Asking which it is at runtime is
        // the only way to tell, and cheaper than deciding here.
        // Declared opacity. SetAlpha is bound and the renderer multiplies every
        // colour by it; only the XML attribute went unread, so a texture meant
        // to sit at half strength drew solid.
        if (const std::string* a = node.attr("alpha")) {
            line(var + ":SetAlpha(" + *a + ")");
        }
        if (const std::string* inh = node.attr("inherits"); inh && !inh->empty()) {
            if (isTexture) {
                line("if __WoweeTemplates[" + quote(*inh) + "] then __WoweeTemplates[" +
                     quote(*inh) + "](" + var + ") end");
            } else {
                line("if __WoweeTemplates[" + quote(*inh) + "] then __WoweeTemplates[" +
                     quote(*inh) + "](" + var + ") else " + var +
                     ":SetFontObject(" + quote(*inh) + ") end");
            }
        }

        if (const std::string* file = node.attr("file")) {
            line(var + ":SetTexture(" + quote(*file) + ")");
        }
        if (const std::string* mode = node.attr("alphaMode")) {
            if (isTexture) line(var + ":SetBlendMode(" + quote(*mode) + ")");
        }
        emitTextAttr(node, var);
        if (const std::string* jv = node.attr("justifyV")) {
            line(var + ":SetJustifyV(" + quote(*jv) + ")");
        }
        if (const std::string* j = node.attr("justifyH")) {
            line(var + ":SetJustifyH(" + quote(*j) + ")");
        }
        if (node.attrBool("setAllPoints")) {
            line(var + ":SetAllPoints(" + parentVar + ")");
        }
        if (node.attrBool("hidden")) line(var + ":Hide()");

        if (const XmlNode* size = node.child("Size")) {
            float w = 0, h = 0;
            if (readDimension(*size, w, h))
                line(var + ":SetSize(" + std::to_string(w) + ", " + std::to_string(h) + ")");
        }
        if (const XmlNode* tc = node.child("TexCoords")) {
            line(var + ":SetTexCoord(" + std::to_string(tc->attrFloat("left", 0.0f)) + ", " +
                 std::to_string(tc->attrFloat("right", 1.0f)) + ", " +
                 std::to_string(tc->attrFloat("top", 0.0f)) + ", " +
                 std::to_string(tc->attrFloat("bottom", 1.0f)) + ")");
        }
        if (const XmlNode* col = node.child("Color")) {
            const std::string args =
                std::to_string(col->attrFloat("r", 1.0f)) + ", " +
                std::to_string(col->attrFloat("g", 1.0f)) + ", " +
                std::to_string(col->attrFloat("b", 1.0f)) + ", " +
                std::to_string(col->attrFloat("a", 1.0f));
            line(var + (isTexture ? ":SetVertexColor(" : ":SetTextColor(") + args + ")");
        }
        if (const XmlNode* anchors = node.child("Anchors"))
            emitAnchors(*anchors, var, parentVar, parentName);
        // A texture may animate itself. See emitAnimations: the achievement
        // banner's glow and shine are textures with an animIn of their own, and
        // losing them took the banner's whole fade-out with them.
        //
        // $parent inside those is the region, not the frame holding it - the
        // glow's group is AchievementAlertFrame1GlowAnimIn - so the name the
        // substitution is made against is the region's own.
        const std::string regionName =
            substituteParent(node.attrOr("name", ""), parentName);
        emitAnimations(node, var, regionName.empty() ? parentName : regionName);
    }

    /// A virtual Texture or FontString: recorded, not built, and replayed onto
    /// whatever inherits it. Twenty-two of these are declared at the top level
    /// of FrameXML - DialogButtonNormalTexture and its kind - and none of them
    /// was emitted at all, so every button that inherits its art had none.
    void emitRegionTemplate(const XmlNode& node, bool isTexture) {
        const std::string name = node.attrOr("name", "");
        if (name.empty()) {
            result.warnings.emplace_back("virtual region with no name was skipped");
            return;
        }
        Emitter inner;
        inner.runtimeParentName = true;
        inner.emitRegionProperties(node, "self", "self:GetParent()", name, isTexture);
        line("__WoweeTemplates[" + quote(name) + "] = function(self)");
        result.lua += inner.result.lua;
        for (auto& w : inner.result.warnings) result.warnings.push_back(w);
        line("end");
    }

    /// Button art declared as its own element rather than inside a Layer.
    ///
    /// These are ordinary regions with an implied draw layer and a setter to
    /// call afterwards, and the emitter used to ignore all of them. That lost
    /// the names they declare: a button whose <ButtonText name="$parentNormalText">
    /// was dropped leaves _G["DropDownList1Button1NormalText"] undefined, and
    /// with the API fallback on the lookup answers with a function instead of
    /// failing outright. HighlightTexture alone appears in 62 FrameXML files.
    void emitButtonRegions(const XmlNode& node, const std::string& var,
                           const std::string& name) {
        struct Slot { const char* element; const char* setter;
                      const char* layer; bool isTexture; };
        static constexpr Slot kSlots[] = {
            {.element = "NormalTexture",          .setter = "SetNormalTexture",          .layer = "ARTWORK",   .isTexture = true},
            {.element = "PushedTexture",          .setter = "SetPushedTexture",          .layer = "ARTWORK",   .isTexture = true},
            {.element = "DisabledTexture",        .setter = "SetDisabledTexture",        .layer = "ARTWORK",   .isTexture = true},
            {.element = "CheckedTexture",         .setter = "SetCheckedTexture",         .layer = "ARTWORK",   .isTexture = true},
            {.element = "DisabledCheckedTexture", .setter = "SetDisabledCheckedTexture", .layer = "ARTWORK",   .isTexture = true},
            // Drawn above the button's own art rather than beside it, which is
            // the whole point of the highlight layer.
            {.element = "HighlightTexture",       .setter = "SetHighlightTexture",       .layer = "HIGHLIGHT", .isTexture = true},
            // Over the art, so a label is never hidden by the face beneath it.
            {.element = "ButtonText",             .setter = "SetFontString",             .layer = "OVERLAY",   .isTexture = false},
            // A slider's grip, which draws over the channel it runs in.
            {.element = "ThumbTexture",           .setter = "SetThumbTexture",           .layer = "OVERLAY",   .isTexture = true},
            // The colour picker's four parts. Declared like button art rather
            // than inside a Layer, and dropped here for the same reason button
            // art used to be: the emitter only walked Layers. ColorPickerWheel
            // is a name FrameXML looks up, so losing the region lost the name
            // as well as the art.
            {.element = "ColorWheelTexture",      .setter = "SetColorWheelTexture",      .layer = "ARTWORK",   .isTexture = true},
            {.element = "ColorWheelThumbTexture", .setter = "SetColorWheelThumbTexture", .layer = "OVERLAY",   .isTexture = true},
            {.element = "ColorValueTexture",      .setter = "SetColorValueTexture",      .layer = "ARTWORK",   .isTexture = true},
            {.element = "ColorValueThumbTexture", .setter = "SetColorValueThumbTexture", .layer = "OVERLAY",   .isTexture = true},
        };
        for (const Slot& slot : kSlots) {
            const XmlNode* child = node.child(slot.element);
            if (!child) continue;
            const std::string regionVar =
                emitRegion(*child, var, name, slot.layer, slot.isTexture);
            line(var + ":" + slot.setter + "(" + regionVar + ")");
        }
    }

    /// <Animations> - one or more <AnimationGroup>, each holding <Alpha>,
    /// <Translation> and friends.
    ///
    /// Asked of regions as well as of frames, because a Texture may declare
    /// one. The achievement banner is what that costs: its glow and its shine
    /// are textures with an animIn of their own, and AlertFrame_AnimateIn plays
    /// all three in a row - so with the texture groups dropped, the second call
    /// raised on a nil and everything after it was lost, including the
    /// waitAndAnimOut that hides the banner. It appeared and stayed for the
    /// rest of the session.
    void emitAnimations(const XmlNode& node, const std::string& var,
                        const std::string& name) {
        // <Animations> - one or more <AnimationGroup>, each holding <Alpha>,
        // <Translation> and friends. The group is a Lua object rather than a
        // widget, so this emits the calls a script would make.
        for (const XmlNode& anims : node.children) {
            if (anims.name != "Animations") continue;
            for (const XmlNode& group : anims.children) {
                if (group.name != "AnimationGroup") continue;
                const std::string gvar = nextVar();
                const std::string gname = substituteParent(
                    group.attrOr("name", ""), name);
                line(gvar + " = " + var + ":CreateAnimationGroup(" +
                     (gname.empty() ? "nil" : quote(gname)) + ")");
                if (const std::string* loop = group.attr("looping")) {
                    line(gvar + ":SetLooping(" + quote(*loop) + ")");
                }
                if (const std::string* pk = group.attr("parentKey")) {
                    line(var + "." + *pk + " = " + gvar);
                }
                for (const XmlNode& gs : group.children) {
                    if (gs.name == "Scripts") emitScripts(gs, gvar);
                }
                for (const XmlNode& a : group.children) {
                    if (a.name != "Alpha" && a.name != "Translation" &&
                        a.name != "Scale" && a.name != "Rotation" &&
                        a.name != "Animation") {
                        continue;
                    }
                    const std::string avar = nextVar();
                    const std::string aname = substituteParent(
                        a.attrOr("name", ""), name);
                    line(avar + " = " + gvar + ":CreateAnimation(" +
                         quote(a.name == "Animation" ? "Alpha" : a.name) +
                         (aname.empty() ? "" : ", " + quote(aname)) + ")");
                    if (const std::string* d = a.attr("duration"))
                        line(avar + ":SetDuration(" + *d + ")");
                    if (const std::string* o = a.attr("order"))
                        line(avar + ":SetOrder(" + *o + ")");
                    if (const std::string* sd = a.attr("startDelay"))
                        line(avar + ":SetStartDelay(" + *sd + ")");
                    if (const std::string* c = a.attr("change"))
                        line(avar + ":SetChange(" + *c + ")");
                    if (a.attr("offsetX") || a.attr("offsetY")) {
                        line(avar + ":SetOffset(" + a.attrOr("offsetX", "0") +
                             ", " + a.attrOr("offsetY", "0") + ")");
                    }
                    // On the group, which is the animation's parent. This
                    // wrote it on the frame, so alertframes.xml's
                    // `frame.waitAndAnimOut.animOut:SetStartDelay(4.05)` found
                    // a nil and raised - taking the Play() on the line after it
                    // with it, which is the call that hides the banner.
                    if (const std::string* pk = a.attr("parentKey"))
                        line(gvar + "." + *pk + " = " + avar);
                    // An animation's own <Scripts>. Dropping these cost the
                    // alert banners: alertframes.xml hangs
                    // `self:GetRegionParent():Hide()` off the fade's
                    // OnFinished, so without it an achievement banner faded to
                    // nothing and stayed there.
                    for (const XmlNode& sc : a.children) {
                        if (sc.name == "Scripts") emitScripts(sc, avar);
                    }
                }
            }
        }
    }

    /// A <FontString> written straight inside a frame, rather than in a Layer.
    ///
    /// That is not a region the frame holds - it is the frame's own font: the
    /// face, size, colour and shadow its text is drawn in. Every EditBox in the
    /// interface says so this way, InputBoxTemplate included, and so do the
    /// chat window and UIErrorsFrame.
    ///
    /// Thirty-nine of them, and all thirty-nine were dropped: the walk above
    /// only looks inside <Layers>, and nothing else claimed the element. So the
    /// chat box drew what you typed in FRIZQT at twelve with no shadow, where
    /// ChatFontNormal is ARIALN at fourteen with a black one - thin, unshadowed
    /// pale text over the world, which is what "hard to read" is.
    ///
    /// The font instance only. A <Size> or <Anchors> here belongs to a region
    /// this frame does not have, and applying either would resize or move the
    /// frame itself.
    void emitFontInstance(const XmlNode& node, const std::string& var) {
        if (const std::string* inh = node.attr("inherits"); inh && !inh->empty()) {
            // Ambiguous the same way a region's is: nearly always a font
            // object, occasionally a virtual FontString. Asked at runtime.
            line("if __WoweeTemplates[" + quote(*inh) + "] then __WoweeTemplates[" +
                 quote(*inh) + "](" + var + ") else " + var +
                 ":SetFontObject(" + quote(*inh) + ") end");
        }
        float height = 0.0f;
        if (const XmlNode* fh = node.child("FontHeight")) {
            if (const XmlNode* abs = fh->child("AbsValue")) height = abs->attrFloat("val", 0.0f);
            else                                           height = fh->attrFloat("val", 0.0f);
        }
        const std::string* face = node.attr("font");
        const std::string* outline = node.attr("outline");
        if (face || outline) {
            // SetFont takes all three together, so what this node does not
            // state is left to what it inherited - nil rather than a default.
            line(var + ":SetFont(" + (face ? quote(*face) : std::string("nil")) + ", " +
                 (height > 0.0f ? std::to_string(height) : std::string("nil")) + ", " +
                 (outline ? quote(*outline) : std::string("nil")) + ")");
        } else if (height > 0.0f) {
            line(var + ":SetTextHeight(" + std::to_string(height) + ")");
        }
        if (const XmlNode* col = node.child("Color")) {
            line(var + ":SetTextColor(" +
                 std::to_string(col->attrFloat("r", 1.0f)) + ", " +
                 std::to_string(col->attrFloat("g", 1.0f)) + ", " +
                 std::to_string(col->attrFloat("b", 1.0f)) + ", " +
                 std::to_string(col->attrFloat("a", 1.0f)) + ")");
        }
        if (const XmlNode* sh = node.child("Shadow")) {
            float sx = 1.0f, sy = -1.0f;
            if (const XmlNode* off = sh->child("Offset")) {
                if (const XmlNode* abs = off->child("AbsDimension")) {
                    sx = abs->attrFloat("x", 1.0f);
                    sy = abs->attrFloat("y", -1.0f);
                }
            }
            line(var + ":SetShadowOffset(" + std::to_string(sx) + ", " +
                 std::to_string(sy) + ")");
            if (const XmlNode* sc = sh->child("Color")) {
                line(var + ":SetShadowColor(" +
                     std::to_string(sc->attrFloat("r", 0.0f)) + ", " +
                     std::to_string(sc->attrFloat("g", 0.0f)) + ", " +
                     std::to_string(sc->attrFloat("b", 0.0f)) + ", " +
                     std::to_string(sc->attrFloat("a", 1.0f)) + ")");
            }
        }
        if (const std::string* j = node.attr("justifyH"))
            line(var + ":SetJustifyH(" + quote(*j) + ")");
        if (const std::string* jv = node.attr("justifyV"))
            line(var + ":SetJustifyV(" + quote(*jv) + ")");
    }

    void emitAnchors(const XmlNode& anchors, const std::string& var,
                     const std::string& parentVar,
                     const std::string& parentNameForAnchors = std::string()) {
        for (const XmlNode& a : anchors.children) {
            if (a.name != "Anchor") continue;
            const std::string point = a.attrOr("point", "CENTER");
            // relativeTo names a frame. Emitted as a string rather than a bare
            // identifier, because SetPoint resolves a name for us and because
            // the name is often $parentSomething - which is not an identifier
            // at all, and pasting it into Lua is a syntax error that loses the
            // whole file. Without one, the anchor is to the parent, which is
            // what leaving it out means.
            std::string relative = parentVar;
            if (const std::string* rt = a.attr("relativeTo")) {
                // Resolved against whatever owns this anchor, which is parentVar
                // - the containing frame for a region, and the parent frame for
                // a frame's own anchors. It used to say "self" regardless, which
                // inside a template asked the wrong frame for its name.
                relative = nameArg(*rt, parentNameForAnchors, parentVar);
                if (relative == "nil") relative = parentVar;
            }
            const std::string relPoint = a.attrOr("relativePoint", point);

            float ox = 0, oy = 0;
            if (const XmlNode* off = a.child("Offset")) readDimension(*off, ox, oy);
            if (a.attr("x") || a.attr("y")) {
                ox = a.attrFloat("x", ox);
                oy = a.attrFloat("y", oy);
            }
            line(var + ":SetPoint(" + quote(point) + ", " + relative + ", " +
                 quote(relPoint) + ", " + std::to_string(ox) + ", " + std::to_string(oy) + ")");
        }
    }

    /// Binds a region or frame to a named field on the frame containing it.
    ///
    /// parentKey="icon" means the owner can say self.icon rather than looking
    /// the name up, and FrameXML's own handlers do exactly that:
    /// QuestHonorFrameTemplate's OnLoad opens with self.icon:SetTexture(...).
    /// Ignoring the attribute left every one of those fields nil - 242 of them
    /// across 31 files.
    ///
    /// Written in brackets because the key is arbitrary text, and a key that
    /// happens to be a Lua keyword would otherwise not parse.
    void emitParentKey(const XmlNode& node, const std::string& var,
                       const std::string& parentVar) {
        const std::string* key = node.attr("parentKey");
        if (!key || key->empty() || parentVar.empty()) return;
        line(parentVar + "[" + quote(*key) + "] = " + var);
    }

    /// A <Font> is not a widget - it is a named set of type settings that font
    /// strings inherit by name, and SetFontObject reads height and colour off
    /// it. Ignoring the element left all 42 of FrameXML's font objects
    /// undefined, so every label that inherits one fell back to a default size
    /// and colour it was never meant to have.
    void emitFont(const XmlNode& node) {
        const std::string name = node.attrOr("name", "");
        if (name.empty()) return;

        float height = 0.0f;
        if (const XmlNode* fh = node.child("FontHeight")) {
            if (const XmlNode* abs = fh->child("AbsValue"))
                height = abs->attrFloat("val", 0.0f);
            else
                height = fh->attrFloat("val", 0.0f);
        }

        // Inheriting copies the settings first, so anything stated here wins -
        // the same order a frame's template follows.
        if (const std::string* inh = node.attr("inherits"); inh && !inh->empty()) {
            line(name + " = {}");
            line("do local base = rawget(_G, " + quote(*inh) + ")");
            line("  if type(base) == 'table' then");
            line("    for k, v in pairs(base) do " + name + "[k] = v end");
            line("  end");
            line("end");
            // pairs() copies fields and not the metatable, and a font object's
            // methods live in one. Without this the copy answered nothing -
            // fontObject:GetTextColor(), which every options control calls to
            // put its label back when it is enabled, indexed a bare table.
            line("if __WoweeFontMT then setmetatable(" + name + ", __WoweeFontMT) end");
        } else {
            // rawget, because reading the name to see whether it is already
            // there is exactly what the missing-API fallback is watching for.
            // Through a plain read every font object in Fonts.xml reported
            // itself missing at the moment it was defined - thirty entries in
            // a list whose whole value is that everything in it is real.
            line(name + " = rawget(_G, " + quote(name) + ") or {}");
            // ...and one made fresh here has no metatable either.
            line("if __WoweeFontMT then setmetatable(" + name + ", __WoweeFontMT) end");
        }
        if (height > 0.0f) line(name + ".height = " + std::to_string(height));
        if (const std::string* f = node.attr("font"))
            line(name + ".font = " + quote(*f));
        if (const std::string* o = node.attr("outline"))
            line(name + ".outline = " + quote(*o));
        if (const XmlNode* col = node.child("Color")) {
            line(name + ".r = " + std::to_string(col->attrFloat("r", 1.0f)));
            line(name + ".g = " + std::to_string(col->attrFloat("g", 1.0f)));
            line(name + ".b = " + std::to_string(col->attrFloat("b", 1.0f)));
            line(name + ".a = " + std::to_string(col->attrFloat("a", 1.0f)));
        }
        // <Shadow> - a dark copy of the glyphs one pixel down and across,
        // which is what keeps a label readable over the world and over the
        // action bar art. Nineteen font objects declare one, and it is written
        // on the font rather than the label, so it arrives this way.
        if (const XmlNode* sh = node.child("Shadow")) {
            float sx = 1.0f, sy = -1.0f;
            if (const XmlNode* off = sh->child("Offset")) {
                if (const XmlNode* abs = off->child("AbsDimension")) {
                    sx = abs->attrFloat("x", 1.0f);
                    sy = abs->attrFloat("y", -1.0f);
                }
            }
            line(name + ".shadowX = " + std::to_string(sx));
            line(name + ".shadowY = " + std::to_string(sy));
            if (const XmlNode* sc = sh->child("Color")) {
                line(name + ".shadowR = " + std::to_string(sc->attrFloat("r", 0.0f)));
                line(name + ".shadowG = " + std::to_string(sc->attrFloat("g", 0.0f)));
                line(name + ".shadowB = " + std::to_string(sc->attrFloat("b", 0.0f)));
                line(name + ".shadowA = " + std::to_string(sc->attrFloat("a", 1.0f)));
            }
        }
    }

    /// Applies whatever this node inherits onto `var`. Templates apply before
    /// the frame's own settings, so anything stated on the frame overrides what
    /// it inherited - the order FrameXML relies on.
    void emitInherits(const XmlNode& node, const std::string& var) {
        const std::string* inherits = node.attr("inherits");
        if (!inherits) return;
        std::stringstream ss(*inherits);
        std::string one;
        while (std::getline(ss, one, ',')) {
            one.erase(0, one.find_first_not_of(" \t"));
            one.erase(one.find_last_not_of(" \t") + 1);
            if (one.empty()) continue;
            line("if __WoweeTemplates[" + quote(one) + "] then __WoweeTemplates[" +
                 quote(one) + "](" + var + ") else __WoweeMissingTemplate(" +
                 quote(one) + ") end");
        }
    }

    /// Returns the variable holding the new frame, or empty for a virtual
    /// one, so a caller that must hand it on - a scroll frame to its child -
    /// can name it.
    std::string emitFrame(const XmlNode& node, const std::string& parentVar,
                          const std::string& parentName,
                          const std::string& nameVar = std::string()) {
        const std::string rawName = node.attrOr("name", "");
        const std::string name = substituteParent(rawName, parentName);
        const bool isVirtual = node.attrBool("virtual");

        if (isVirtual) {
            // A template is not built now. It is recorded so a later inherits=
            // can replay it onto a real frame, which is the only thing "virtual"
            // means in FrameXML.
            if (name.empty()) {
                result.warnings.emplace_back("virtual frame with no name was skipped");
                return {};
            }
            Emitter inner;
            inner.temp = 0;
            inner.runtimeParentName = true;
            // Inside a template the containing frame is whatever inherits it,
            // so an unqualified anchor means "my parent" and has to be asked
            // for at replay time.
            inner.emitFrameBody(node, "self", name, "self:GetParent()",
                                std::string(), /*fireOnLoad=*/false);
            // What kind of frame the template makes, so an element written as
            // the template's own name can be built as one. Turtle's
            // LootFrame.xml declares <Button name="LootButton" virtual="true">
            // and then writes <LootButton name="LootButton1"/>, which is a
            // shape the real client accepts and this emitter had no answer for
            // - the element was unknown, nothing inside it was built, and the
            // loot window came up with no buttons in it.
            virtualFrames.insert(name);
            line("__WoweeTemplateTypes[" + quote(name) + "] = " + quote(node.name));
            line("__WoweeTemplates[" + quote(name) + "] = function(self)");
            line("local __w = {}");
            // Whether this frame arrived with a parent, read before any
            // inherited template runs - a base template setting one must not
            // make a derived template's own parent look redundant.
            line("local __noParent = not self:GetParent()");
            // A template can itself inherit one, and this branch used to return
            // before that was ever emitted - so InterfaceOptionsListButtonTemplate
            // silently dropped the OptionsListButtonTemplate it is built on,
            // arriving with no highlight texture and no size. First, so the
            // template's own body overrides what it inherited.
            emitInherits(node, "self");
            // A template's own parent, applied to whatever inherits it. This
            // was dropped: templates were replayed onto a frame that had
            // already been created, and only the creation call looked at
            // parent=. TargetFrameTemplate and MirrorTimerTemplate both say
            // parent="UIParent" and the frames built from them are declared
            // with no parent of their own, so this is the only place they can
            // get one. After the inherited templates so the most derived wins.
            //
            // Only onto a frame that has none. A template's parent= is read at
            // creation in WoW, so it settles a top-level frame declared without
            // one and says nothing about a nested frame - whose parent is the
            // element containing it. Applying it unconditionally tore children
            // out of their containers and re-hung them on UIParent, which moves
            // their strata and their level and puts whatever was full screen
            // above the world.
            if (const std::string* par = node.attr("parent"); par && !par->empty()) {
                line("if " + *par + " and __noParent then self:SetParent(" +
                     *par + ") end");
            }
            result.lua += inner.result.lua;
            for (auto& w : inner.result.warnings) result.warnings.push_back(w);
            line("end");
            return {};
        }

        const std::string var = nextVar();
        // The frames the panel system can put full screen, and only those.
        //
        // uiparent.lua's fullscreen path is UIParent:Hide() followed by
        // frame:Show(), so a frame it can do that to must not be a child of
        // UIParent - the map went down with everything the call was meant to
        // clear out of its way. In WoW these are parentless because they are
        // declared at XML top level with no parent of their own.
        //
        // Named rather than derived from that rule, which would take in a
        // hundred more: this list is UIPanelWindows' `area = "full"` entries,
        // and it is the whole set the mechanism applies to. Detaching the rest
        // changes what GetParent() answers for them - nil where it used to be
        // UIParent - and FrameXML asks that of frames it is about to position,
        // so a wider net blanks panels rather than freeing them.
        static const char* kFullscreenPanels[] = {
            "WorldMapFrame", "CinematicFrame",
        };
        bool detached = false;
        if (parentVar.empty() && !node.attr("parent")) {
            for (const char* n : kFullscreenPanels) {
                if (rawName == n) { detached = true; break; }
            }
        }
        const std::string parentArg = node.attr("parent")
            ? *node.attr("parent")
            : (parentVar.empty() ? (detached ? "nil" : "UIParent") : parentVar);
        // Through nameArg, the same as regions: inside a template a child named
        // $parentScrollBar has to work out its name when the template is
        // replayed, because the frame it belongs to is not known until then.
        // Baking the literal instead named every scroll bar after the template,
        // so the _G[self:GetName().."ScrollBar"] its own handlers look up never
        // existed - which is what took down most of FrameXML.
        // A known element names its own type; anything else is a template's
        // name, and the type is whatever that template was declared as. Read at
        // runtime because the template may live in a file loaded before this
        // one and the emitter sees one file at a time.
        const std::string typeArg =
            isFrameElement(node.name)
                ? quote(node.name)
                : "(__WoweeTemplateTypes[" + quote(node.name) + "] or \"Frame\")";
        line(var + " = CreateFrame(" + typeArg + ", " +
             nameArg(rawName, parentName, parentArg) + ", " + parentArg + ")");

        // Identity before anything is built on top of it. FrameXML makes names
        // out of the id - a party member's pet frame opens its OnLoad with
        // self:GetParent():GetID() - and a template's children load while the
        // template is being applied, which is before the frame's own body runs.
        // Set there, the parent was still answering zero.
        if (const std::string* id = node.attr("id"); id && !id->empty()) {
            line(var + ":SetID(" + *id + ")");
        }
        // Before the template applies, so a template body that reaches back
        // through its parent for a sibling finds it already bound.
        emitParentKey(node, var, parentArg);
        // The element's own name is the first template it inherits. Missing is
        // reported the same way a named inherits= is, so an element that is
        // genuinely nothing still says so.
        if (!isFrameElement(node.name)) {
            line("if __WoweeTemplates[" + quote(node.name) + "] then __WoweeTemplates[" +
                 quote(node.name) + "](" + var + ") else __WoweeMissingTemplate(" +
                 quote(node.name) + ") end");
        }
        emitInherits(node, var);
        emitFrameBody(node, var, name.empty() ? parentName : name, parentArg,
                      parentName, /*fireOnLoad=*/true,
                      rawName.empty() ? nameVar : std::string());
        return var;
    }

    /// ownerName is the name of the frame containing this one. A frame's own
    /// anchors say $parent meaning the frame they hang off, not themselves, so
    /// they need a different name from the one its regions use.
    void emitFrameBody(const XmlNode& node, const std::string& var,
                       const std::string& name, const std::string& parentVar,
                       const std::string& ownerName = std::string(),
                       bool fireOnLoad = true,
                       const std::string& nameVar = std::string()) {
        // What $parent resolves to for anything declared inside this frame. An
        // unnamed frame is not the answer - WoW walks up to the nearest named
        // ancestor, and PartyMemberPetFrameTemplate buries its $parentName two
        // unnamed frames deep, expecting PartyMemberFrame1PetFrameName. Asking
        // the unnamed frame for its name gave nil, so the region was called
        // "Name" and the lookup that wanted it found nothing.
        const std::string anchor = nameVar.empty() ? var : nameVar;

        // hidden="true" is the frame's state from the moment it exists, not
        // something done to it once it is built. This ran last - after the
        // children were created, after every OnLoad had fired - so a frame the
        // interface declares hidden was *visible* for the whole of its own
        // construction.
        //
        // FrameXML guards on exactly that. SpellButton_UpdateButton opens with
        // `if ( not this:IsVisible() ) then return end`, and the spell buttons
        // are built and fired before SpellBookFrame_OnLoad has run - so on the
        // real client the guard holds and the early call does nothing. Here it
        // did not hold: the button read as visible, ran on, and asked
        // SpellBook_GetSpellID for a page number out of a table that
        // SpellBookFrame_OnLoad fills a step later. That is a nil arithmetic
        // that takes spellbookframe.xml down whole on a 1.12 interface.
        //
        // Hiding first also lets an OnLoad that shows its own frame win, which
        // is what the real client does and what hiding afterwards silently
        // undid.
        if (node.attrBool("hidden")) line(var + ":Hide()");

        if (const XmlNode* size = node.child("Size")) {
            float w = 0, h = 0;
            if (readDimension(*size, w, h))
                line(var + ":SetSize(" + std::to_string(w) + ", " + std::to_string(h) + ")");
        }
        if (const std::string* strata = node.attr("frameStrata")) {
            line(var + ":SetFrameStrata(" + quote(*strata) + ")");
        }
        // A slider's range, step and orientation, declared as attributes. The
        // range has to be set before the value, or the value is clamped to a
        // default range it was never meant to sit in.
        if (node.attr("minValue") || node.attr("maxValue")) {
            line(var + ":SetMinMaxValues(" +
                 std::to_string(node.attrFloat("minValue", 0.0f)) + ", " +
                 std::to_string(node.attrFloat("maxValue", 1.0f)) + ")");
        }
        if (node.attr("valueStep")) {
            line(var + ":SetValueStep(" +
                 std::to_string(node.attrFloat("valueStep", 0.0f)) + ")");
        }
        if (const std::string* o = node.attr("orientation")) {
            line(var + ":SetOrientation(" + quote(*o) + ")");
        }
        if (node.attr("defaultValue")) {
            line(var + ":SetValue(" +
                 std::to_string(node.attrFloat("defaultValue", 0.0f)) + ")");
        }

        // A button only receives the clicks it asks for, and one file asks in
        // the XML rather than from a script.
        if (const std::string* clicks = node.attr("registerForClicks");
            clicks && !clicks->empty()) {
            std::stringstream ss(*clicks);
            std::string one, args;
            while (std::getline(ss, one, ',')) {
                one.erase(0, one.find_first_not_of(" \t"));
                one.erase(one.find_last_not_of(" \t") + 1);
                if (one.empty()) continue;
                if (!args.empty()) args += ", ";
                args += quote(one);
            }
            if (!args.empty()) line(var + ":RegisterForClicks(" + args + ")");
        }
        if (node.attr("enableMouse")) {
            line(var + ":EnableMouse(" + (node.attrBool("enableMouse") ? "true" : "false") + ")");
        } else {
            // A frame that declares OnEnter or OnLeave wants the mouse, whether
            // or not it says so. Blizzard's own StatFrameTemplate is the case
            // that proves it: the character sheet's stat rows carry both
            // handlers and no enableMouse, and hovering them in a real client
            // shows the breakdown tooltip. Without this the hit test skipped
            // them and the whole left column of the character sheet was inert.
            // Every script here is one the frame cannot receive without the
            // mouse. OnMouseWheel is deliberately absent: that needs
            // EnableMouseWheel, which is a separate switch.
            static const char* kMouseScripts[] = {
                "OnEnter", "OnLeave", "OnMouseDown", "OnMouseUp",
                "OnDragStart", "OnReceiveDrag",
                // OnClick most of all, and it was the one missing. A handler
                // that runs on a click is the plainest statement a frame can
                // make that it wants the mouse - plainer than OnEnter, which
                // was here from the start.
                //
                // OptionsListButtonTemplate is the case that showed it: it
                // declares OnClick and OnLoad and no enableMouse, so every
                // category button in the Video, Interface and Audio option
                // frames drew its name and refused the click. The list looked
                // finished and did nothing, which is a worse failure than an
                // empty one.
                "OnClick", "OnDoubleClick",
                // The hyperlink scripts are deliberately not here. A chat frame
                // declares OnHyperlinkClick and Blizzard's own
                // FloatingChatFrameTemplate then sets enableMouse="false" over
                // it, because a chat window is click-through by design - and
                // links in it are still clickable in a real client. So a link
                // is not the frame's click to take, and the hit test for one
                // runs whether or not the frame beneath it wants the mouse.
            };
            bool wantsMouse = false;
            for (const XmlNode& child : node.children) {
                if (child.name != "Scripts") continue;
                for (const XmlNode& script : child.children) {
                    for (const char* want : kMouseScripts) {
                        if (script.name == want) { wantsMouse = true; break; }
                    }
                    if (wantsMouse) break;
                }
                if (wantsMouse) break;
            }
            if (wantsMouse) line(var + ":EnableMouse(true)");
        }
        if (const std::string* a = node.attr("alpha")) {
            line(var + ":SetAlpha(" + *a + ")");
        }
        // A message frame holds as many lines as it says it does. Twenty-two
        // of them ask for three; without this each kept a hundred and
        // twenty-eight and drew far past the box drawn for it.
        if (const std::string* ml = node.attr("maxLines"); ml && !ml->empty()) {
            line(var + ":SetMaxLines(" + *ml + ")");
        }
        // How long a line stays, and which end a new one goes on. UIErrorsFrame
        // declares displayDuration="5" and insertMode="TOP"; both were dropped,
        // so every server refusal stayed on screen for good and they stacked
        // upward from the wrong end.
        if (const std::string* dd = node.attr("displayDuration"); dd && !dd->empty()) {
            line(var + ":SetTimeVisible(" + *dd + ")");
        }
        if (const std::string* fd = node.attr("fadeDuration"); fd && !fd->empty()) {
            line(var + ":SetFadeDuration(" + *fd + ")");
        }
        if (const std::string* im = node.attr("insertMode"); im && !im->empty()) {
            line(var + ":SetInsertMode(" + quote(*im) + ")");
        }
        // A cooldown that counts the other way, and the bright spoke along its
        // sweeping edge. The target frame's aura timers and the totem bar ask
        // for the first and were drawn backwards without it.
        if (node.attr("reverse")) {
            line(var + ":SetReverse(" +
                 (node.attrBool("reverse") ? "true" : "false") + ")");
        }
        if (node.attr("drawEdge")) {
            line(var + ":SetDrawEdge(" +
                 (node.attrBool("drawEdge") ? "true" : "false") + ")");
        }
        // Whether a label too long for its box breaks, and whether it may break
        // inside a word. Thirty-six ask for the second, all of them prose in a
        // narrow column, and both were dropped along with the wrapping itself.
        if (node.attr("wordwrap")) {
            line(var + ":SetWordWrap(" +
                 (node.attrBool("wordwrap") ? "true" : "false") + ")");
        }
        if (node.attr("nonspacewrap")) {
            line(var + ":SetNonSpaceWrap(" +
                 (node.attrBool("nonspacewrap") ? "true" : "false") + ")");
        }
        // The keyboard, on the same principle as the mouse and the wheel: a
        // frame that declares OnKeyDown or OnKeyUp wants keys. Every one that
        // does in FrameXML is a dialog hidden until it is wanted, so nothing
        // here listens during ordinary play.
        if (node.attr("enableKeyboard")) {
            line(var + ":EnableKeyboard(" +
                 (node.attrBool("enableKeyboard") ? "true" : "false") + ")");
        } else {
            bool wantsKeys = false;
            for (const XmlNode& child : node.children) {
                if (child.name != "Scripts") continue;
                for (const XmlNode& script : child.children) {
                    if (script.name == "OnKeyDown" || script.name == "OnKeyUp") {
                        wantsKeys = true;
                        break;
                    }
                }
                if (wantsKeys) break;
            }
            if (wantsKeys) line(var + ":EnableKeyboard(true)");
        }
        // <PushedTextOffset><AbsDimension x="1" y="-1"/></PushedTextOffset>
        for (const XmlNode& child : node.children) {
            if (child.name != "PushedTextOffset") continue;
            for (const XmlNode& d : child.children) {
                if (d.name != "AbsDimension") continue;
                line(var + ":SetPushedTextOffset(" + d.attrOr("x", "0") + ", " +
                     d.attrOr("y", "0") + ")");
            }
            break;
        }
        // <TitleRegion> - the part of a frame you can drag it by. All three in
        // FrameXML cover the whole frame, and the loot window is one of them:
        // without this it cannot be moved at all.
        //
        // Only wired when the frame declares no drag handling of its own. The
        // chat frame has a title region and five OnDragStart scripts, and
        // overwriting those with the generic pair would replace behaviour that
        // is deliberately more specific.
        // A window that says it is toplevel and movable counts too, title region
        // or not. Only three frames in FrameXML declare one, so keying on it
        // alone left the character sheet and the spellbook - both 384x512
        // panels declaring toplevel and movable, neither with a title region -
        // unable to be moved, along with thirty more windows.
        //
        // Both attributes, not just movable: forty-nine frames declare movable
        // and the set includes the unit frames, which declare it so that addons
        // can reposition them. Giving those a body drag would turn a click that
        // slipped four units from "target this party member" into "move their
        // frame". Pairing it with toplevel narrows it to the actual windows.
        {
            bool ownDrag = false;
            bool titleRegion = false;
            for (const XmlNode& child : node.children) {
                if (child.name == "TitleRegion") titleRegion = true;
                if (child.name != "Scripts") continue;
                for (const XmlNode& sc : child.children) {
                    if (sc.name == "OnDragStart" || sc.name == "OnDragStop") {
                        ownDrag = true;
                    }
                }
            }
            const bool window = node.attrBool("toplevel") && node.attrBool("movable");
            if (titleRegion || window) {
                line(var + ":SetMovable(true)");
                if (!ownDrag) {
                    line(var + ":RegisterForDrag(\"LeftButton\")");
                    line(var + ":SetScript(\"OnDragStart\", function(self) self:StartMoving() end)");
                    line(var + ":SetScript(\"OnDragStop\", function(self) self:StopMovingOrSizing() end)");
                }
            }
        }
        emitAnimations(node, var, name);

        // <NormalFont style="GameFontNormal"/> - the font a button's label is
        // drawn in. Only the normal one: this renderer does not draw a button's
        // text differently when it is highlighted or disabled, so emitting the
        // other two would be calls that read as support and change nothing.
        for (const XmlNode& child : node.children) {
            if (child.name != "NormalFont") continue;
            if (const std::string* style = child.attr("style")) {
                line(var + ":SetNormalFontObject(" + quote(*style) + ")");
            }
            break;
        }
        // The other two states, which this renderer distinguishes by colour.
        for (const XmlNode& child : node.children) {
            const char* setter = child.name == "HighlightFont"
                                     ? ":SetHighlightFontObject("
                                 : child.name == "DisabledFont"
                                     ? ":SetDisabledFontObject("
                                     : nullptr;
            if (!setter) continue;
            if (const std::string* style = child.attr("style")) {
                line(var + setter + quote(*style) + ")");
            }
        }
        // <HitRectInsets><AbsInset .../></HitRectInsets>
        for (const XmlNode& child : node.children) {
            if (child.name != "HitRectInsets") continue;
            for (const XmlNode& ins : child.children) {
                if (ins.name != "AbsInset") continue;
                line(var + ":SetHitRectInsets(" + ins.attrOr("left", "0") + ", " +
                     ins.attrOr("right", "0") + ", " + ins.attrOr("top", "0") + ", " +
                     ins.attrOr("bottom", "0") + ")");
            }
            break;
        }
        if (const std::string* sc = node.attr("scale")) {
            line(var + ":SetScale(" + *sc + ")");
        }
        if (node.attr("toplevel")) {
            line(var + ":SetToplevel(" +
                 (node.attrBool("toplevel") ? "true" : "false") + ")");
        }
        if (node.attr("clampedToScreen")) {
            line(var + ":SetClampedToScreen(" +
                 (node.attrBool("clampedToScreen") ? "true" : "false") + ")");
        }
        // <Backdrop> - the bordered panel look. The pieces are already there:
        // SetBackdrop parses the table, the widget carries the fields and the
        // renderer draws the nine slices. Only this step was missing, so every
        // backdrop declared in XML went undrawn - 77 of them, among which are
        // the tooltip background and the dialog panels, which is why tooltip
        // text sat straight on top of whatever was behind it.
        //
        // Addons reach the same code through SetBackdrop directly, and that
        // path was a no-op on the frame metatable until it was removed; a panel
        // an addon draws this way is the common case, not the rare one.
        for (const XmlNode& child : node.children) {
            if (child.name != "Backdrop") continue;
            std::string t = "{";
            if (const std::string* bg = child.attr("bgFile"))
                t += "bgFile=" + quote(*bg) + ", ";
            if (const std::string* edge = child.attr("edgeFile"))
                t += "edgeFile=" + quote(*edge) + ", ";
            t += std::string("tile=") + (child.attrBool("tile") ? "true" : "false");
            // TileSize and EdgeSize each wrap a single <AbsValue val="n"/>.
            for (const XmlNode& sub : child.children) {
                const char* key = sub.name == "TileSize" ? "tileSize"
                                : sub.name == "EdgeSize" ? "edgeSize"
                                                         : nullptr;
                if (!key) continue;
                for (const XmlNode& v : sub.children) {
                    if (const std::string* val = v.attr("val")) {
                        t += std::string(", ") + key + "=" + *val;
                    }
                }
            }
            for (const XmlNode& sub : child.children) {
                if (sub.name != "BackgroundInsets") continue;
                for (const XmlNode& ins : sub.children) {
                    if (ins.name != "AbsInset") continue;
                    t += ", insets={left=" + ins.attrOr("left", "0") +
                         ", right="  + ins.attrOr("right", "0") +
                         ", top="    + ins.attrOr("top", "0") +
                         ", bottom=" + ins.attrOr("bottom", "0") + "}";
                }
            }
            t += "}";
            line(var + ":SetBackdrop(" + t + ")");
            break;  // one backdrop to a frame
        }
        // The wheel, on the same principle and its own switch. No FrameXML file
        // sets the attribute - Blizzard leaves the handler to imply it - so the
        // quest log, the reputation list and the friends list all declared
        // OnMouseWheel and none of them could be scrolled with it. The
        // attribute is read too, for anything that does set it.
        if (node.attr("enableMouseWheel")) {
            line(var + ":EnableMouseWheel(" +
                 (node.attrBool("enableMouseWheel") ? "true" : "false") + ")");
        } else {
            bool wantsWheel = false;
            for (const XmlNode& child : node.children) {
                if (child.name != "Scripts") continue;
                for (const XmlNode& script : child.children) {
                    if (script.name == "OnMouseWheel") { wantsWheel = true; break; }
                }
                if (wantsWheel) break;
            }
            if (wantsWheel) line(var + ":EnableMouseWheel(true)");
        }
        // Whether the frame can be dragged around the screen. Declared in the
        // XML rather than set from Lua for most of what moves - the bag
        // windows, the character sheet - so leaving it unread meant StartMoving
        // was asked of a frame that had never been told it was movable, and
        // every one of those windows was nailed down.
        emitTextAttr(node, var);
        // A frame declares this the same way it declares movable, and it was
        // going nowhere: the chat window says resizable="true" and its size
        // grabber called StartSizing on a frame that had never been told it
        // could be sized, so the grabber lit up and nothing happened.
        if (node.attr("resizable")) {
            line(var + ":SetResizable(" +
                 (node.attrBool("resizable") ? "true" : "false") + ")");
        }
        if (node.attr("movable")) {
            line(var + ":SetMovable(" + (node.attrBool("movable") ? "true" : "false") + ")");
        }
        // What an edit box was declared to be. Every one of these has had a
        // method and a field behind it all along and no way to reach them from
        // the XML, so a box came out with whatever the defaults were.
        //
        // SendMailBodyEditBox says letters="500" multiLine="true", and without
        // these it was a single-line box with no limit - a letter nobody could
        // write a second line in.
        if (const std::string* letters = node.attr("letters"); letters && !letters->empty()) {
            line(var + ":SetMaxLetters(" + std::to_string(static_cast<int>(node.attrFloat("letters", 0.0f))) + ")");
        }
        // Whether the markup that draws nothing counts against that limit.
        // WoW's default is no and four boxes here ask for yes, the macro
        // editor among them - where the escapes are the point of the box.
        if (node.attrBool("countInvisibleLetters")) {
            line(var + ":SetCountInvisibleLetters(true)");
        }
        // Which layer a region draws in, which the tree sorts the draw order
        // by. StatusBar is the only element that declares it here, and
        // CastingBarFrameTemplate is one of them - so the cast bar's fill was
        // coming out in the default layer rather than the BORDER it asked for.
        // SetDrawLayer is a region method, so this is emitted for whatever
        // declares the attribute rather than for status bars alone.
        if (const std::string* layer = node.attr("drawLayer");
            layer && !layer->empty()) {
            line(var + ":SetDrawLayer(" + quote(*layer) + ")");
        }
        if (node.attr("multiLine")) {
            line(var + ":SetMultiLine(" + (node.attrBool("multiLine") ? "true" : "false") + ")");
        }
        if (node.attr("numeric")) {
            line(var + ":SetNumeric(" + (node.attrBool("numeric") ? "true" : "false") + ")");
        }
        // How many sent lines the box keeps, which the client walks with the
        // arrow keys. The chat box asks for thirty-two; the money fields, the
        // mail recipient and the knowledge base ask for none, and a box told to
        // keep none should not quietly keep twenty.
        if (const std::string* hist = node.attr("historyLines"); hist && !hist->empty()) {
            line(var + ":SetHistoryLines(" +
                 std::to_string(static_cast<int>(node.attrFloat("historyLines", 0.0f))) + ")");
        }
        // A box that declares this does not take the left and right arrows for
        // its cursor - they reach the game, which is how a player turns while
        // the chat box is open. Up and down still walk the history; that is
        // what the real client does with the same flag.
        if (node.attr("ignoreArrows")) {
            line(var + ":SetIgnoreArrows(" +
                 (node.attrBool("ignoreArrows") ? "true" : "false") + ")");
        }
        // Declared false on nearly every box in FrameXML, which is the whole
        // point of it: a box that takes focus when it appears swallows the
        // keyboard from whatever the player was doing.
        // A font string's line spacing. The only one of the remaining unread
        // attributes with a method behind it - nonspacewrap, horizTile,
        // vertTile and reverse have none, and emitting a call to a method that
        // does not exist is "attempt to call method", which is worse than
        // ignoring the attribute.
        if (node.attr("spacing")) {
            line(var + ":SetSpacing(" +
                 std::to_string(static_cast<int>(node.attrFloat("spacing", 0.0f))) + ")");
        }
        if (node.attr("autoFocus")) {
            line(var + ":SetAutoFocus(" + (node.attrBool("autoFocus") ? "true" : "false") + ")");
        }
        // Attributes declared in the XML, which is where FrameXML puts a
        // frame's initial state - UIParent's panel offsets among them. Set
        // before anything else runs, because SetAttribute fires
        // OnAttributeChanged and a handler reading a sibling attribute must
        // find it already there.
        if (const XmlNode* attrs = node.child("Attributes")) {
            for (const XmlNode& a : attrs->children) {
                if (a.name != "Attribute") continue;
                const std::string* an = a.attr("name");
                if (!an || an->empty()) continue;
                const std::string type = a.attrOr("type", "string");
                const std::string val = a.attrOr("value", "");
                std::string literal;
                if (type == "number")       literal = std::to_string(a.attrFloat("value", 0.0f));
                else if (type == "boolean") literal = a.attrBool("value") ? "true" : "false";
                else                        literal = quote(val);
                line(var + ":SetAttribute(" + quote(*an) + ", " + literal + ")");
            }
        }

        // A frame can fill its parent instead of stating anchors, and this was
        // honoured for regions and ignored for frames - all 139 of them across
        // 53 files. An unanchored frame falls to the centre-on-parent default
        // with no size, so its centre is the screen's: PlayerFrame's name sat
        // in the middle of the world because two frames above it said
        // setAllPoints and were heard by nobody.
        if (node.attrBool("setAllPoints")) {
            line(var + ":SetAllPoints(" + parentVar + ")");
        }
        if (const XmlNode* anchors = node.child("Anchors")) {
            // Anchored to the frame that contains it when no relativeTo is
            // given. This used to say UIParent for everything, so a nested
            // frame was positioned against the screen rather than its parent -
            // which for anything inside a panel puts it somewhere else
            // entirely, and FrameXML nests constantly.
            // ownerName, not name: a $parentApply here means the Apply button
            // beside this one on the frame that holds both, not a child of this
            // frame. Using its own name built VideoOptionsFrameCancelApply for
            // what should have been VideoOptionsFrameApply - a name nothing has,
            // so the anchor silently fell back to the parent.
            // A parent= attribute overrides where the frame sits in the file.
            // WorldMapTitleButton is written at the top level with
            // parent="WorldMapFrame", so its $parentMiniBorderLeft is the world
            // map's border and not a child of nothing - and with no containing
            // frame to take a name from, $parent collapsed to the bare suffix.
            // SetPoint then looked up a global called MiniBorderLeft, found
            // nothing, and put the title bar wherever the fallback landed.
            const std::string* declaredParent = node.attr("parent");
            const std::string anchorOwner =
                (declaredParent && !declaredParent->empty()) ? *declaredParent : ownerName;
            emitAnchors(*anchors, var, parentVar, anchorOwner);
        }
        // A status bar's own art and colour. Thirty-three bars across FrameXML
        // declare a BarTexture and none of it was emitted, so every one of them
        // fell back to a flat fill in the default white - which for a health
        // bar is not a bar with the wrong texture, it is a white block where
        // the health should be.
        if (const XmlNode* bar = node.child("BarTexture")) {
            if (const std::string* file = bar->attr("file")) {
                line(var + ":SetStatusBarTexture(" + quote(*file) + ")");
            }
        }
        if (const XmlNode* col = node.child("BarColor")) {
            line(var + ":SetStatusBarColor(" +
                 std::to_string(col->attrFloat("r", 1.0f)) + ", " +
                 std::to_string(col->attrFloat("g", 1.0f)) + ", " +
                 std::to_string(col->attrFloat("b", 1.0f)) + ", " +
                 std::to_string(col->attrFloat("a", 1.0f)) + ")");
        }

        if (const XmlNode* layers = node.child("Layers")) {
            for (const XmlNode& layer : layers->children) {
                if (layer.name != "Layer") continue;
                const std::string level = layer.attrOr("level", "ARTWORK");
                for (const XmlNode& region : layer.children) {
                    // Exactly, because canonicaliseNames has already rewritten
                    // every element name to the schema's spelling before any of
                    // these comparisons run - including FrameXML's own
                    // <Fontstring>, which is how FriendsMicroButtonCount gets
                    // built despite the typo.
                    if (region.name == "Texture" || region.name == "FontString")
                        emitRegion(region, var, name, level,
                                   region.name == "Texture", anchor);
                }
            }
        }
        // The frame's own font, which is a <FontString> child of the frame
        // itself rather than one inside a Layer. See emitFontInstance.
        for (const XmlNode& child : node.children) {
            if (child.name == "FontString") emitFontInstance(child, var);
        }
        // Before Frames and Scripts, so a child anchoring to $parentNormalTexture
        // and an OnLoad reading its own label both find something there.
        emitButtonRegions(node, var, name);
        // A scroll frame's content, which is a frame like any other but reached
        // through SetScrollChild rather than sitting in Frames.
        // HybridScrollFrameScrollChild_OnLoad does self:GetParent().scrollChild
        // = self, so it has to be built and its OnLoad run - ignoring the
        // element left self.scrollChild nil on every one of the 18 files that
        // declare one.
        if (const XmlNode* scrollChild = node.child("ScrollChild")) {
            for (const XmlNode& child : scrollChild->children) {
                if (!isFrameElement(child.name)) continue;
                const std::string childVar = emitFrame(child, var, name, anchor);
                if (!childVar.empty())
                    line(var + ":SetScrollChild(" + childVar + ")");
            }
        }
        if (const XmlNode* frames = node.child("Frames")) {
            for (const XmlNode& child : frames->children) {
                // A named element that is not a known type is a template's own
                // name used as an element - see emitFrame. Unnamed and unknown
                // is nothing this can build, and is left to the warning below.
                if (isFrameElement(child.name) || child.attr("name")) {
                    emitFrame(child, var, name, anchor);
                }
            }
        }
        // Scripts last: OnLoad runs against a frame that is already built, which
        // is what every handler in FrameXML assumes.
        //
        // Not from inside a template body, though. A frame is loaded once, when
        // it is finished - not once per template it is built from. Firing at
        // each template ran ChatFrameEditBoxTemplate's OnLoad before the edit
        // box's own OnLoad had set self.chatFrame, which is the very thing that
        // handler opens by indexing. The template only installs the script; the
        // frame it is applied to runs it, after its own body has had its say.
        if (const XmlNode* scripts = node.child("Scripts")) {
            emitScripts(*scripts, var);
        }
        // Whether or not this frame declared one: a template it inherits may
        // have installed the handler, and that frame still loads. The runtime
        // check costs nothing when there is none.
        // Whether a disabled button still hears the mouse. Set before OnLoad
        // rather than after, because a handler may disable the button it is
        // running for and the answer has to be in place by then.
        if (node.attrBool("motionScriptsWhileDisabled")) {
            line(var + ":SetMotionScriptsWhileDisabled(true)");
        }
        if (fireOnLoad) {
            // Through the engine rather than called straight from here.
            // An interface written before 3.0 reads the frame off the global
            // `this` rather than off its parameter - `this:Hide()` is the first
            // line of most OnLoad bodies in 1.12 - and that name is published
            // by whatever dispatches a handler. Everything else is dispatched
            // from lua_engine.cpp, which does it; this call is the one that is
            // not, so it goes through a binding that does the same.
            line("__WoweeFireOnLoad(" + var + ")");
        }
    }
};

} // namespace

std::string substituteParent(const std::string& name, const std::string& parentName) {
    const std::string token = "$parent";
    if (name.compare(0, token.size(), token) != 0) return name;
    return parentName + name.substr(token.size());
}

namespace {
/// Element names as the schema spells them, for matching a document's spelling
/// against.
///
/// WoW's parser does not care about case and this one did, which is a
/// difference nobody notices until it costs them an element. FrameXML itself
/// contains one: floatingchatframe.xml declares <Fontstring>, and that region
/// was never built - no error, no warning that meant anything, just a font
/// string missing from a frame. Addons are written far less carefully than
/// Blizzard's own files, so this is the more useful half of the fix.
const char* const kElementNames[] = {
    "AbsDimension", "AbsInset", "AbsValue", "Alpha", "Anchor", "Anchors",
    "Animation", "AnimationGroup", "Animations", "Attribute", "Attributes",
    "Backdrop", "BackgroundInsets", "BarColor", "BarTexture", "Binding",
    "Bindings", "BorderColor", "Button", "ButtonText", "CheckButton",
    "CheckedTexture", "Color", "ColorSelect", "ColorValueTexture",
    "ColorValueThumbTexture", "ColorWheelTexture", "ColorWheelThumbTexture",
    "Cooldown", "DisabledCheckedTexture", "DisabledFont", "DisabledTexture",
    "DressUpModel", "EdgeSize", "EditBox", "Font", "FontHeight", "FontString",
    "Frame", "Frames", "GameTooltip", "HighlightFont", "HighlightTexture",
    "HitRectInsets", "Include", "Layer", "Layers", "MessageFrame", "Minimap",
    "Model", "ModifiedClick", "NormalFont", "NormalTexture", "Offset",
    "PlayerModel", "PushedTextOffset", "PushedTexture", "QuestPOIFrame",
    "ResizeBounds", "Script", "Scripts", "ScrollChild", "ScrollFrame",
    "ScrollingMessageFrame", "Shadow", "SimpleHTML", "Size", "Slider",
    "StatusBar", "TabardModel", "TexCoords", "Texture", "ThumbTexture",
    "TileSize", "TitleRegion", "Translation", "Ui", "WorldFrame",
    "maxResize", "minResize",
};

bool sameNameIgnoringCase(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

/// Rewrite every element name to the schema's spelling, in place, before any
/// of the emitter's comparisons run. Anything unrecognised is left exactly as
/// written so it still reports itself as an unknown element type.
void canonicaliseNames(XmlNode& node) {
    for (const char* known : kElementNames) {
        if (node.name != known && sameNameIgnoringCase(node.name, known)) {
            node.name = known;
            break;
        }
    }
    for (XmlNode& child : node.children) canonicaliseNames(child);
}
}  // namespace

/// <Bindings> declares the commands the key bindings list shows, in the order
/// it shows them, and the Lua each one runs. Nothing is drawn for it, so it
/// emits data rather than frames - plus a closure per command, because
/// RunBinding has to be able to call one.
void emitBindings(Emitter& e, const XmlNode& root) {
    e.line("__WoweeBindings = __WoweeBindings or {}");
    // <ModifiedClick action="CHATLINK" default="SHIFT-BUTTON1"/> and fifteen
    // more. Nothing is drawn for these either, and they were dropped - while
    // IsModifiedClick answered from a table written out by hand beside them.
    // Two readings of one list, and they disagreed: FOCUSCAST is declared NONE
    // here and answered ALT there, and the two flyout actions are declared ALT
    // and fell through to a shift default.
    e.line("__WoweeModifiedClick = __WoweeModifiedClick or {}");
    e.line("__WoweeBindingScripts = __WoweeBindingScripts or {}");
    e.line("__WoweeBindingRunOnUp = __WoweeBindingRunOnUp or {}");
    for (const XmlNode& m : root.children) {
        if (m.name != "ModifiedClick") continue;
        const std::string action = m.attrOr("action", "");
        if (action.empty()) continue;
        e.line("__WoweeModifiedClick[" + quote(action) + "] = " +
               quote(m.attrOr("default", "NONE")));
    }
    for (const XmlNode& b : root.children) {
        if (b.name != "Binding") continue;
        const std::string name = b.attrOr("name", "");
        if (name.empty()) {
            e.result.warnings.emplace_back("<Binding> without a name is not a "
                                        "command anything can refer to");
            continue;
        }
        // A header attribute does not name the binding's section - it opens
        // one, as a row of its own above the command that carries it. The list
        // shows it through BINDING_HEADER_*, and everything after it belongs to
        // it until the next.
        if (const std::string* h = b.attr("header")) {
            if (!h->empty()) {
                e.line("__WoweeBindings[#__WoweeBindings+1] = \"HEADER_" + *h + "\"");
            }
        }
        e.line("__WoweeBindings[#__WoweeBindings+1] = \"" + name + "\"");
        if (b.attrBool("runOnUp")) {
            e.line("__WoweeBindingRunOnUp[\"" + name + "\"] = true");
        }
        if (!b.text.empty()) {
            // keystate arrives as a parameter rather than a global. The bodies
            // read it to tell a press from a release, and a global would carry
            // one binding's state into the next.
            e.result.lua += "__WoweeBindingScripts[\"" + name +
                            "\"] = function(keystate)\n";
            e.result.lua += b.text;
            e.result.lua += "\nend\n";
        }
    }
}

EmitResult emitFrameXml(const XmlNode& rootIn) {
    XmlNode root = rootIn;
    canonicaliseNames(root);
    Emitter e;
    // Bindings carry no frames, so the local every frame file opens with would
    // be an unused declaration in a file that is otherwise all data.
    if (root.name == "Bindings") {
        emitBindings(e, root);
        return e.result;
    }
    e.line("local __w = {}");
    if (root.name != "Ui") {
        e.result.warnings.push_back("root element is <" + root.name + ">, expected <Ui>");
    }
    for (const XmlNode& node : root.children) {
        if (node.name == "Script") {
            if (const std::string* file = node.attr("file")) e.result.scriptFiles.push_back(*file);
            else if (!node.text.empty()) e.result.lua += node.text + "\n";
        } else if (node.name == "Include") {
            if (const std::string* file = node.attr("file")) e.result.includeFiles.push_back(*file);
        } else if (node.name == "Font") {
            e.emitFont(node);
        } else if (isFrameElement(node.name)) {
            e.emitFrame(node, "", "");
        } else if (node.name == "Texture" || node.name == "FontString") {
            if (node.attrBool("virtual")) {
                e.emitRegionTemplate(node, node.name == "Texture");
            } else {
                e.result.warnings.push_back("<" + node.name + "> outside a frame "
                                            "has nothing to belong to");
            }
        } else if (node.attr("name")) {
            // A template's own name used as an element. Turtle's LootFrame.xml
            // writes <LootButton name="LootButton1"/> against a virtual
            // <Button name="LootButton">, which the real client accepts and
            // this reported as an unknown type - so the loot window was built
            // with no buttons in it. emitFrame reads the type off the template
            // and applies it; a name that turns out to be no template at all
            // still reports itself, from __WoweeMissingTemplate.
            if (!e.virtualFrames.count(node.name)) {
                e.result.warnings.push_back(
                    "<" + node.name + "> is not a frame type and no template of "
                    "that name is declared here - it is built from one if some "
                    "other file declares it, and from nothing if none does");
            }
            e.emitFrame(node, "", "");
        } else if (node.name == "Bindings" || node.name == "Binding" ||
                   node.name == "ModifiedClick") {
            // Key bindings, not frames. Nothing is drawn for them and nothing
            // is meant to be, so they are not a gap to report.
        } else {
            // Said out loud, because the failure is silence. An element type
            // this does not know is not a frame that comes out wrong - it is a
            // frame, and everything inside it, that is never created at all,
            // and the file still loads and still compiles. Minimap.xml produced
            // nothing whatever for exactly this reason, and the only trace was
            // a handful of globals reading as missing somewhere else entirely.
            e.result.warnings.push_back("<" + node.name +
                                        "> is not a known frame type; "
                                        "nothing inside it was built");
        }
    }
    return std::move(e.result);
}

} // namespace ui
} // namespace wowee
