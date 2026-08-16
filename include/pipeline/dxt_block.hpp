#pragma once

#include <cstdint>

namespace wowee {
namespace pipeline {

/// The colour half of a DXT block, which DXT1, DXT3 and DXT5 all share.
///
/// All three store colour the same way: two RGB565 endpoints followed by a
/// 32-bit word of two-bit indices, one per pixel of the 4x4 block. Only what
/// happens around it differs, so blp_loader.cpp decoded the same eight bytes
/// three times over.
///
/// The one real difference is the punch-through mode, and it is not cosmetic.
/// In DXT1 the ordering of the two endpoints is a flag: with c0 greater than
/// c1 the block has four opaque colours, and otherwise it has three plus a
/// fully transparent index 3. DXT3 and DXT5 carry alpha separately and always
/// use the four-colour form, so applying DXT1's rule to them would turn a
/// quarter of some blocks transparent, and applying the four-colour rule to
/// DXT1 would fill cut-out foliage and grates with a solid colour.
struct DxtColorBlock {
    /// The four candidate colours, RGB.
    uint8_t rgb[4][3] = {};
    /// True when index 3 means "transparent" rather than a colour: DXT1's
    /// three-colour mode.
    bool index3IsTransparent = false;
    /// Two bits per pixel, pixel (px, py) at bit (py * 4 + px) * 2.
    uint32_t indices = 0;

    /// Which of the four entries pixel (px, py) uses.
    [[nodiscard]] int indexAt(int px, int py) const {
        return static_cast<int>((indices >> ((py * 4 + px) * 2)) & 0x3u);
    }
};

/// Reads the eight colour bytes of a DXT block.
///
/// `allowPunchThrough` is what tells DXT1 from DXT3 and DXT5: only DXT1 reads
/// the endpoint ordering as a mode flag.
///
/// The interpolated entries truncate rather than round, which is what every
/// decoder these textures were authored against does. Rounding instead shifts
/// two thirds of the pixels of a gradient by one step.
inline DxtColorBlock decodeDxtColorBlock(const uint8_t* colorBlock,
                                         bool allowPunchThrough) {
    const uint16_t c0 = static_cast<uint16_t>(colorBlock[0] | (colorBlock[1] << 8));
    const uint16_t c1 = static_cast<uint16_t>(colorBlock[2] | (colorBlock[3] << 8));

    // RGB565 to RGB888: R is bits 15:11 over 31, G is bits 10:5 over 63, and
    // B is bits 4:0 over 31.
    const uint8_t r0 = static_cast<uint8_t>(((c0 >> 11) & 0x1F) * 255 / 31);
    const uint8_t g0 = static_cast<uint8_t>(((c0 >> 5) & 0x3F) * 255 / 63);
    const uint8_t b0 = static_cast<uint8_t>((c0 & 0x1F) * 255 / 31);
    const uint8_t r1 = static_cast<uint8_t>(((c1 >> 11) & 0x1F) * 255 / 31);
    const uint8_t g1 = static_cast<uint8_t>(((c1 >> 5) & 0x3F) * 255 / 63);
    const uint8_t b1 = static_cast<uint8_t>((c1 & 0x1F) * 255 / 31);

    DxtColorBlock out;
    out.rgb[0][0] = r0; out.rgb[0][1] = g0; out.rgb[0][2] = b0;
    out.rgb[1][0] = r1; out.rgb[1][1] = g1; out.rgb[1][2] = b1;

    if (allowPunchThrough && c0 <= c1) {
        // Three colours and a transparent index: the halfway point, then nothing.
        out.rgb[2][0] = static_cast<uint8_t>((r0 + r1) / 2);
        out.rgb[2][1] = static_cast<uint8_t>((g0 + g1) / 2);
        out.rgb[2][2] = static_cast<uint8_t>((b0 + b1) / 2);
        out.rgb[3][0] = 0; out.rgb[3][1] = 0; out.rgb[3][2] = 0;
        out.index3IsTransparent = true;
    } else {
        // Four colours: the two thirds points.
        out.rgb[2][0] = static_cast<uint8_t>((2 * r0 + r1) / 3);
        out.rgb[2][1] = static_cast<uint8_t>((2 * g0 + g1) / 3);
        out.rgb[2][2] = static_cast<uint8_t>((2 * b0 + b1) / 3);
        out.rgb[3][0] = static_cast<uint8_t>((r0 + 2 * r1) / 3);
        out.rgb[3][1] = static_cast<uint8_t>((g0 + 2 * g1) / 3);
        out.rgb[3][2] = static_cast<uint8_t>((b0 + 2 * b1) / 3);
    }

    out.indices = static_cast<uint32_t>(colorBlock[4]) |
                  (static_cast<uint32_t>(colorBlock[5]) << 8) |
                  (static_cast<uint32_t>(colorBlock[6]) << 16) |
                  (static_cast<uint32_t>(colorBlock[7]) << 24);
    return out;
}

}  // namespace pipeline
}  // namespace wowee
