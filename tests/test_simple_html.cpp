// The HTML a SimpleHTML page is given, read into the blocks it draws.
//
// Gordawg's Imprint is the case that made this: its whole text is one IMG in
// an HTML wrapper, and the page drew the markup itself instead of the picture.
#include "catch_amalgamated.hpp"
#include "ui/simple_html.hpp"

#include <string>

using wowee::ui::HtmlBlock;
using wowee::ui::looksLikeSimpleHtml;
using wowee::ui::parseSimpleHtml;

TEST_CASE("A page is HTML only when it opens with the tag", "[simple_html]") {
    // ItemTextFrame puts a line break in front of every page.
    CHECK(looksLikeSimpleHtml("\n<HTML><BODY></BODY></HTML>"));
    CHECK(looksLikeSimpleHtml("<html><body>x</body></html>"));
    CHECK_FALSE(looksLikeSimpleHtml("Dear sir, <HTML> is not how this starts."));
    CHECK_FALSE(looksLikeSimpleHtml("\n"));
    CHECK_FALSE(looksLikeSimpleHtml(""));
}

TEST_CASE("A picture on its own is one image block", "[simple_html]") {
    const auto blocks = parseSimpleHtml(
        "\n<HTML>\n<BODY>\n<IMG src=\"Interface\\Pictures\\24475_gordawg_256\"/>\n"
        "</BODY>\n</HTML>\n\n");
    REQUIRE(blocks.size() == 1);
    CHECK(blocks[0].kind == HtmlBlock::Kind::Image);
    CHECK(blocks[0].src == "Interface\\Pictures\\24475_gordawg_256");
    CHECK(blocks[0].width == 0.0f);
    CHECK(blocks[0].height == 0.0f);
}

TEST_CASE("An image keeps the size and alignment it asked for", "[simple_html]") {
    const auto blocks = parseSimpleHtml(
        "<html><body><img src='a\\b' width=\"128\" height=64 align=\"center\"/>"
        "</body></html>");
    REQUIRE(blocks.size() == 1);
    CHECK(blocks[0].src == "a\\b");
    CHECK(blocks[0].width == 128.0f);
    CHECK(blocks[0].height == 64.0f);
    CHECK(blocks[0].align == "CENTER");
}

TEST_CASE("Paragraphs and headings are blocks with their own align", "[simple_html]") {
    const auto blocks = parseSimpleHtml(
        "<HTML><BODY><H1 align=\"center\">Title</H1>"
        "<P>First   line\nwraps here.<BR/>Second</P></BODY></HTML>");
    REQUIRE(blocks.size() == 2);
    CHECK(blocks[0].text == "Title");
    CHECK(blocks[0].align == "CENTER");
    // Source whitespace collapses; BR is the only line break.
    CHECK(blocks[1].text == "First line wraps here.\nSecond");
    CHECK(blocks[1].align.empty());
}

TEST_CASE("A BR between paragraphs is a block of its own", "[simple_html]") {
    const auto blocks = parseSimpleHtml(
        "<HTML><BODY><P>One</P><BR/><P>Two</P></BODY></HTML>");
    REQUIRE(blocks.size() == 3);
    CHECK(blocks[0].text == "One");
    CHECK(blocks[1].text == "\n");
    CHECK(blocks[2].text == "Two");
}

TEST_CASE("Text either side of a picture keeps its paragraph", "[simple_html]") {
    const auto blocks = parseSimpleHtml(
        "<HTML><BODY><P align=\"right\">Before<IMG src=\"x\"/>After</P></BODY></HTML>");
    REQUIRE(blocks.size() == 3);
    CHECK(blocks[0].text == "Before");
    CHECK(blocks[1].kind == HtmlBlock::Kind::Image);
    CHECK(blocks[2].text == "After");
    CHECK(blocks[2].align == "RIGHT");
}

TEST_CASE("Links become the escape a label already draws", "[simple_html]") {
    const auto blocks = parseSimpleHtml(
        "<HTML><BODY><P>See <A href=\"quest:123\">this</A>.</P></BODY></HTML>");
    REQUIRE(blocks.size() == 1);
    CHECK(blocks[0].text == "See |Hquest:123|hthis|h.");
}

TEST_CASE("Entities are decoded and unknown tags dropped", "[simple_html]") {
    const auto blocks = parseSimpleHtml(
        "<HTML><BODY><P>a &lt;b&gt; &amp; <FONT>c</FONT> & d</P></BODY></HTML>");
    REQUIRE(blocks.size() == 1);
    CHECK(blocks[0].text == "a <b> & c & d");
}
