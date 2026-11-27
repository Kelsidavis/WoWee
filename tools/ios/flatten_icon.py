#!/usr/bin/env python3
"""Flatten a PNG's alpha channel onto an opaque background.

Apple's app icons must be fully opaque: the system applies its own rounded
rectangle mask, and a source image with transparent corners either renders
with a black fringe or is rejected outright by actool.  The macOS app icon
(`assets/Wowee.png`) is a rounded square with transparent corners, which is
fine for an .icns but not for iOS.

This script composites the image onto an opaque background.  The background is
not a flat fill: the transparent corner pixels are filled by diffusing the
nearest opaque pixels outwards, so the rounded tile's own edge gradient is
continued into the corners instead of being cut off with a visible seam.

Only the standard library is used - the build machines have no Pillow.
"""

import struct
import sys
import zlib


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def read_png(path):
    """Decode an 8-bit non-interlaced PNG into (width, height, pixels)."""
    data = open(path, "rb").read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path}: not a PNG")
    pos = 8
    width = height = depth = color = interlace = None
    idat = bytearray()
    palette = b""
    trns = b""
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        kind = data[pos + 4 : pos + 8]
        chunk = data[pos + 8 : pos + 8 + length]
        if kind == b"IHDR":
            width, height, depth, color, _comp, _filt, interlace = struct.unpack(
                ">IIBBBBB", chunk
            )
        elif kind == b"PLTE":
            palette = chunk
        elif kind == b"tRNS":
            trns = chunk
        elif kind == b"IDAT":
            idat += chunk
        elif kind == b"IEND":
            break
        pos += 12 + length

    if depth != 8:
        raise ValueError(f"{path}: only 8-bit PNGs are supported (got {depth})")
    if interlace:
        raise ValueError(f"{path}: interlaced PNGs are not supported")

    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[color]
    raw = zlib.decompress(bytes(idat))
    stride = width * channels
    out = bytearray(width * height * 4)
    prev = bytearray(stride)
    pos = 0
    for y in range(height):
        filt = raw[pos]
        pos += 1
        line = bytearray(raw[pos : pos + stride])
        pos += stride
        if filt == 1:
            for i in range(channels, stride):
                line[i] = (line[i] + line[i - channels]) & 0xFF
        elif filt == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif filt == 3:
            for i in range(stride):
                left = line[i - channels] if i >= channels else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif filt == 4:
            for i in range(stride):
                left = line[i - channels] if i >= channels else 0
                upleft = prev[i - channels] if i >= channels else 0
                line[i] = (line[i] + _paeth(left, prev[i], upleft)) & 0xFF
        elif filt != 0:
            raise ValueError(f"{path}: unknown PNG filter {filt}")
        for x in range(width):
            s = x * channels
            d = (y * width + x) * 4
            if color == 6:
                out[d : d + 4] = line[s : s + 4]
            elif color == 2:
                out[d : d + 3] = line[s : s + 3]
                out[d + 3] = 255
            elif color == 0:
                v = line[s]
                out[d] = out[d + 1] = out[d + 2] = v
                out[d + 3] = 255
            elif color == 4:
                v, a = line[s], line[s + 1]
                out[d] = out[d + 1] = out[d + 2] = v
                out[d + 3] = a
            else:  # palette
                idx = line[s]
                out[d : d + 3] = palette[idx * 3 : idx * 3 + 3]
                out[d + 3] = trns[idx] if idx < len(trns) else 255
        prev = line
    return width, height, out


def write_png(path, width, height, pixels, with_alpha=True):
    """Write 8-bit pixels as a PNG (RGBA, or RGB when with_alpha is false)."""
    channels = 4 if with_alpha else 3
    stride = width * channels
    raw = bytearray()
    for y in range(height):
        raw.append(0)  # filter: none
        row = pixels[y * width * 4 : (y + 1) * width * 4]
        if not with_alpha:
            row = bytearray(
                b for i in range(width) for b in row[i * 4 : i * 4 + 3]
            )
        raw += row

    def chunk(kind, payload):
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    out = b"\x89PNG\r\n\x1a\n"
    color = 6 if with_alpha else 2
    out += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0))
    out += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    out += chunk(b"IEND", b"")
    open(path, "wb").write(out)


def flatten(width, height, pixels, passes=256):
    """Fill the transparent pixels by diffusing the opaque ones outwards."""
    total = width * height
    alpha = pixels[3::4]
    if all(a == 255 for a in alpha):
        return pixels, 0

    # Opaque pixels are the source; the rest are holes to be filled.
    known = bytearray(1 if a >= 250 else 0 for a in alpha)
    holes = sum(1 for k in known if not k)
    if holes == 0:
        return pixels, 0

    # Average colour of the opaque pixels, used for anything still unfilled.
    count = 0
    mean = [0, 0, 0]
    for i in range(total):
        if known[i]:
            count += 1
            mean[0] += pixels[i * 4]
            mean[1] += pixels[i * 4 + 1]
            mean[2] += pixels[i * 4 + 2]
    mean = [c // max(count, 1) for c in mean]

    # Diffuse: each pass fills the holes whose neighbours are known, taking
    # the average of the known neighbours' colours.
    for _ in range(passes):
        newly = []
        for y in range(height):
            row = y * width
            for x in range(width):
                i = row + x
                if known[i]:
                    continue
                rs = gs = bs = n = 0
                for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                    if 0 <= nx < width and 0 <= ny < height:
                        j = ny * width + nx
                        if known[j]:
                            rs += pixels[j * 4]
                            gs += pixels[j * 4 + 1]
                            bs += pixels[j * 4 + 2]
                            n += 1
                if n:
                    newly.append((i, rs // n, gs // n, bs // n))
        if not newly:
            break
        for i, r, g, b in newly:
            pixels[i * 4] = r
            pixels[i * 4 + 1] = g
            pixels[i * 4 + 2] = b
            pixels[i * 4 + 3] = 255
            known[i] = 1

    # Anything still open (fully enclosed holes) takes the mean colour.
    for i in range(total):
        if not known[i]:
            pixels[i * 4] = mean[0]
            pixels[i * 4 + 1] = mean[1]
            pixels[i * 4 + 2] = mean[2]
            pixels[i * 4 + 3] = 255

    return pixels, holes


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    strip = "--strip-alpha" in argv
    if len(args) != 2:
        print(f"usage: {argv[0]} <input.png> <output.png> [--strip-alpha]",
              file=sys.stderr)
        return 2
    width, height, pixels = read_png(args[0])
    if strip:
        # Drop the alpha channel outright.  iOS app icons must not carry one
        # at all, not even a fully opaque one.  sips' resampler rounds a few
        # pixels to 254 on the way, so only a genuinely transparent pixel is
        # an error here.
        alphas = pixels[3::4]
        if any(a < 250 for a in alphas):
            print(f"{args[0]}: still has transparent pixels", file=sys.stderr)
            return 1
        for i in range(3, len(pixels), 4):
            pixels[i] = 255
        write_png(args[1], width, height, pixels, with_alpha=False)
        print(f"{args[1]}: {width}x{height}, alpha channel removed")
        return 0
    pixels, holes = flatten(width, height, pixels)
    write_png(args[1], width, height, pixels)
    print(f"{args[1]}: {width}x{height}, filled {holes} transparent pixels")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
