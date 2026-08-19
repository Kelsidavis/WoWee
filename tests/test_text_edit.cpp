// The editing behind the login screen's text boxes.
//
// The boxes are drawn by this client rather than by ImGui, which means the
// awkward half of a text field - word motion, selection replacement, a caret
// that must never land inside a character - is this client's to get right.
// None of it needs a font or a device, so all of it is tested here.

#include "catch_amalgamated.hpp"

#include "ui/text_edit.hpp"

using wowee::ui::TextEdit;

namespace {

/// A field holding `s` with the caret at `caret` and nothing selected.
TextEdit at(const char* s, size_t caret) {
    TextEdit e;
    e.setText(s);
    e.setCaret(caret);
    return e;
}

} // namespace

TEST_CASE("setText puts the caret at the end and selects nothing", "[text_edit]") {
    TextEdit e;
    e.setText("localhost");
    REQUIRE(e.text() == "localhost");
    REQUIRE(e.caret() == 9);
    REQUIRE_FALSE(e.hasSelection());
}

TEST_CASE("typing inserts at the caret", "[text_edit]") {
    TextEdit e = at("host", 2);
    REQUIRE(e.insertCodepoint('X'));
    REQUIRE(e.text() == "hoXst");
    REQUIRE(e.caret() == 3);
}

TEST_CASE("typing over a selection replaces it", "[text_edit]") {
    TextEdit e;
    e.setText("wow.example.com");
    e.setCaret(0);
    e.setCaret(3, /*extend=*/true);
    REQUIRE(e.selectedText() == "wow");
    REQUIRE(e.insert("logon"));
    REQUIRE(e.text() == "logon.example.com");
    REQUIRE(e.caret() == 5);
    REQUIRE_FALSE(e.hasSelection());
}

TEST_CASE("a paste is stripped of what a single line cannot hold", "[text_edit]") {
    TextEdit e;
    // A password copied out of a text file arrives with its newline attached.
    REQUIRE(e.insert("hunter2\n"));
    REQUIRE(e.text() == "hunter2");

    TextEdit multi;
    REQUIRE(multi.insert("one\ttwo\r\nthree"));
    REQUIRE(multi.text() == "onetwothree");
}

TEST_CASE("a field will not take more than it holds", "[text_edit]") {
    TextEdit e{4};
    REQUIRE(e.insert("37241"));
    REQUIRE(e.text() == "3724");
    REQUIRE_FALSE(e.insertCodepoint('9'));
    REQUIRE(e.text() == "3724");
}

TEST_CASE("the cap never splits a character in half", "[text_edit]") {
    // Five bytes of room and three two-byte characters offered: the third
    // does not fit, and half of it is not an answer.
    TextEdit e{5};
    REQUIRE(e.insert("\xc3\xa9\xc3\xa9\xc3\xa9"));
    REQUIRE(e.text() == "\xc3\xa9\xc3\xa9");
    REQUIRE(e.codepointCount() == 2);

    TextEdit assigned{5};
    assigned.setText("\xc3\xa9\xc3\xa9\xc3\xa9");
    REQUIRE(assigned.text() == "\xc3\xa9\xc3\xa9");
}

TEST_CASE("the caret steps over whole characters", "[text_edit]") {
    TextEdit e;
    e.setText("a\xc3\xa9z");  // a, e-acute, z: four bytes, three characters
    REQUIRE(e.caret() == 4);
    e.moveLeft(false, false);
    REQUIRE(e.caret() == 3);
    e.moveLeft(false, false);
    REQUIRE(e.caret() == 1);  // skips the continuation byte, not onto it
    e.moveLeft(false, false);
    REQUIRE(e.caret() == 0);
    e.moveLeft(false, false);
    REQUIRE(e.caret() == 0);  // and stops there
    e.moveRight(false, false);
    REQUIRE(e.caret() == 1);
    e.moveRight(false, false);
    REQUIRE(e.caret() == 3);
}

TEST_CASE("a caret asked for a byte inside a character lands before it", "[text_edit]") {
    TextEdit e;
    e.setText("a\xc3\xa9z");
    e.setCaret(2);  // the continuation byte of the e-acute
    REQUIRE(e.caret() == 1);
}

TEST_CASE("backspace removes one character, not one byte", "[text_edit]") {
    TextEdit e;
    e.setText("a\xc3\xa9");
    REQUIRE(e.backspace());
    REQUIRE(e.text() == "a");
}

TEST_CASE("backspace over a selection removes the selection", "[text_edit]") {
    TextEdit e;
    e.setText("localhost");
    e.selectAll();
    REQUIRE(e.backspace());
    REQUIRE(e.text().empty());
    REQUIRE(e.caret() == 0);
    REQUIRE_FALSE(e.backspace());  // nothing left to remove
}

TEST_CASE("delete removes forwards", "[text_edit]") {
    TextEdit e = at("port", 0);
    REQUIRE(e.deleteForward());
    REQUIRE(e.text() == "ort");
    REQUIRE(e.caret() == 0);

    TextEdit end = at("port", 4);
    REQUIRE_FALSE(end.deleteForward());
}

TEST_CASE("word motion walks a hostname a label at a time", "[text_edit]") {
    // The run of separators has to be skippable, or Ctrl+Left crawls through
    // "logon.example.com" one dot at a time.
    TextEdit e;
    e.setText("logon.example.com");
    REQUIRE(e.caret() == 17);
    e.moveLeft(false, true);
    REQUIRE(e.text().substr(e.caret()) == "com");
    e.moveLeft(false, true);
    REQUIRE(e.text().substr(e.caret()) == ".com");
    e.moveLeft(false, true);
    REQUIRE(e.text().substr(e.caret()) == "example.com");
    e.moveLeft(false, true);
    REQUIRE(e.text().substr(e.caret()) == ".example.com");
    e.moveLeft(false, true);
    REQUIRE(e.caret() == 0);
}

TEST_CASE("word motion skips the space between words", "[text_edit]") {
    TextEdit e = at("two  words", 0);
    e.moveRight(false, true);
    REQUIRE(e.caret() == 5);  // past "two" and past both spaces
    e.moveRight(false, true);
    REQUIRE(e.caret() == 10);

    e.moveLeft(false, true);
    REQUIRE(e.caret() == 5);
    e.moveLeft(false, true);
    REQUIRE(e.caret() == 0);  // over the spaces and over "two" in one step
}

TEST_CASE("a shifted motion extends the selection and a bare one collapses it",
          "[text_edit]") {
    TextEdit e = at("abcdef", 3);
    e.moveLeft(/*extend=*/true, false);
    e.moveLeft(true, false);
    REQUIRE(e.selectedText() == "bc");
    REQUIRE(e.anchor() == 3);

    // Left with a selection goes to its near edge rather than one further on.
    e.moveLeft(false, false);
    REQUIRE(e.caret() == 1);
    REQUIRE_FALSE(e.hasSelection());

    TextEdit f = at("abcdef", 1);
    f.setCaret(4, true);
    f.moveRight(false, false);
    REQUIRE(f.caret() == 4);
    REQUIRE_FALSE(f.hasSelection());
}

TEST_CASE("home and end go to the ends", "[text_edit]") {
    TextEdit e = at("abcdef", 3);
    e.moveHome(false);
    REQUIRE(e.caret() == 0);
    e.moveEnd(true);
    REQUIRE(e.selectedText() == "abcdef");
}

TEST_CASE("a double click picks out the run it landed in", "[text_edit]") {
    TextEdit e;
    e.setText("logon.example.com");

    e.selectWordAt(8);  // inside "example"
    REQUIRE(e.selectedText() == "example");

    e.selectWordAt(5);  // the dot between two labels
    REQUIRE(e.selectedText() == ".");

    // Past the end belongs to the last character rather than to nothing.
    e.selectWordAt(999);
    REQUIRE(e.selectedText() == "com");
}

TEST_CASE("a double click in an empty field selects nothing and does not fall over",
          "[text_edit]") {
    TextEdit e;
    e.selectWordAt(4);
    REQUIRE(e.caret() == 0);
    REQUIRE_FALSE(e.hasSelection());
}

TEST_CASE("select all covers the string", "[text_edit]") {
    TextEdit e;
    e.setText("hunter2");
    e.selectAll();
    REQUIRE(e.selectionBegin() == 0);
    REQUIRE(e.selectionEnd() == 7);
    REQUIRE(e.selectedText() == "hunter2");
    e.deselect();
    REQUIRE_FALSE(e.hasSelection());
}

TEST_CASE("codepoints are counted, which is what a masked field draws", "[text_edit]") {
    TextEdit e;
    e.setText("a\xc3\xa9\xe4\xb8\xad");  // one, two and three bytes
    REQUIRE(e.size() == 6);
    REQUIRE(e.codepointCount() == 3);
}

TEST_CASE("control characters are never typed in", "[text_edit]") {
    TextEdit e;
    REQUIRE_FALSE(e.insertCodepoint('\n'));
    REQUIRE_FALSE(e.insertCodepoint(0x7F));
    REQUIRE(e.text().empty());
}

TEST_CASE("a lone surrogate is not encoded", "[text_edit]") {
    TextEdit e;
    REQUIRE_FALSE(e.insertCodepoint(0xD800));
    REQUIRE(e.text().empty());
}

TEST_CASE("the placeholder a remembered password shows survives being stored",
          "[text_edit]") {
    // The login screen puts eight 0x01 bytes in the password box to stand for
    // a hash it already has. They are control characters, and assigning them
    // has to keep them even though typing them would not.
    TextEdit e;
    e.setText("\x01\x01\x01\x01\x01\x01\x01\x01");
    REQUIRE(e.size() == 8);
    REQUIRE(e.codepointCount() == 8);
    REQUIRE(e.text() == "\x01\x01\x01\x01\x01\x01\x01\x01");
}
