#pragma once

#include <array>
#include <cstddef>
#include <vector>
#include <cstdint>
#include <string>

namespace wowee {
namespace network {

class Packet {
public:
    Packet() = default;
    explicit Packet(uint16_t opcode);
    Packet(uint16_t opcode, const std::vector<uint8_t>& data);
    Packet(uint16_t opcode, std::vector<uint8_t>&& data);

    void writeUInt8(uint8_t value);
    void writeUInt16(uint16_t value);
    void writeUInt32(uint32_t value);
    void writeUInt64(uint64_t value);
    void writeFloat(float value);
    void writeString(const std::string& value);
    void writeBytes(const uint8_t* data, size_t length);

    uint8_t readUInt8();
    uint16_t readUInt16();
    uint32_t readUInt32();
    uint64_t readUInt64();
    float readFloat();
    uint64_t readPackedGuid();
    void writePackedGuid(uint64_t guid);
    std::string readString();

    /// A name written as a 32-bit length followed by that many bytes, the
    /// length counting a trailing null the server includes.
    ///
    /// Three chat parsers read this by hand and only one of them checked the
    /// length against what the packet actually had left. Without that check a
    /// truncated packet yields a name padded out of thin air - readUInt8
    /// answers zero past the end rather than failing - and every field after
    /// it is read from beyond the data too. The name comes back plausible and
    /// the target guid comes back nothing.
    ///
    /// False means the length is not one this packet can honour, and the read
    /// position is left where it was.
    bool readSizedString(std::string& out, uint32_t maxLength = 256);

    // ------------------------------------------------------------------
    // Bit-level access.
    //
    // Cataclysm onward marshals a packet as a bit stream interleaved with the
    // byte stream, in one buffer: some bits, then some bytes, then more bits.
    // Bits go in most-significant first, and a partial byte is padded with
    // zeros when the stream goes back to bytes.
    //
    // That padding is why every byte write below flushes first and every byte
    // read resets the bit cursor. A caller who has to remember it is a caller
    // who will forget it in one of the hundred and eighty places this is
    // reached from, and the failure is a packet that reads plausibly and
    // wrongly rather than one that fails.
    //
    // Costs the pre-Cata paths one predictable compare per write, and nothing
    // else: with no bits pending, flushBits() returns immediately.
    // ------------------------------------------------------------------

    void writeBit(bool value);
    /// The low `count` bits of `value`, most significant first. count is 1..64.
    void writeBits(uint64_t value, int count);
    /// Pad the open bit byte out and emit it. No-op when none is open.
    void flushBits();

    bool readBit();
    /// `count` bits, most significant first, as the low bits of the result.
    [[nodiscard]] uint64_t readBits(int count);
    /// Discard the rest of the open bit byte, so the next readBit refills.
    void resetBitReader();

    /// The eight bytes of a GUID part-way through a bit-streamed decode.
    ///
    /// The mask pass sets an entry to 1 where the byte is present, the byte
    /// pass replaces it with the byte itself, and guidFromBytes assembles it.
    /// The two passes are separate because the wire separates them, often with
    /// other fields in between, and because their orders differ.
    using GuidBytes = std::array<uint8_t, 8>;

    /// One bit per byte index in `order`: set when that byte is non-zero.
    void writeGuidMask(uint64_t guid, const uint8_t* order, size_t count);
    /// Each non-zero byte named by `order`, XOR'd with 1, in that order.
    void writeGuidBytes(uint64_t guid, const uint8_t* order, size_t count);

    void readGuidMask(GuidBytes& guid, const uint8_t* order, size_t count);
    void readGuidBytes(GuidBytes& guid, const uint8_t* order, size_t count);

    /// The orders are per-opcode tables, so they arrive as arrays.
    template <size_t N>
    void writeGuidMask(uint64_t guid, const uint8_t (&order)[N]) { writeGuidMask(guid, order, N); }
    template <size_t N>
    void writeGuidBytes(uint64_t guid, const uint8_t (&order)[N]) { writeGuidBytes(guid, order, N); }
    template <size_t N>
    void readGuidMask(GuidBytes& guid, const uint8_t (&order)[N]) { readGuidMask(guid, order, N); }
    template <size_t N>
    void readGuidBytes(GuidBytes& guid, const uint8_t (&order)[N]) { readGuidBytes(guid, order, N); }

    [[nodiscard]] static uint64_t guidFromBytes(const GuidBytes& guid);

    [[nodiscard]] uint16_t getOpcode() const { return opcode; }
    [[nodiscard]] const std::vector<uint8_t>& getData() const { return data; }
    [[nodiscard]] size_t getReadPos() const { return readPos; }
    [[nodiscard]] size_t getSize() const { return data.size(); }
    // Clamp to 0 instead of wrapping to ~(size_t)0 when readPos overshoots
    // (can happen via setReadPos with an unchecked offset).
    [[nodiscard]] size_t getRemainingSize() const { return (readPos <= data.size()) ? (data.size() - readPos) : 0; }
    [[nodiscard]] bool hasRemaining(size_t need) const { return readPos <= data.size() && need <= (data.size() - readPos); }
    [[nodiscard]] bool hasFullPackedGuid() const {
        if (readPos >= data.size()) return false;
        uint8_t mask = data[readPos];
        size_t guidBytes = 1;
        for (int bit = 0; bit < 8; ++bit)
            if (mask & (1u << bit)) ++guidBytes;
        return getRemainingSize() >= guidBytes;
    }
    void setReadPos(size_t pos) { readPos = pos; }
    [[nodiscard]] bool hasData() const { return readPos < data.size(); }
    void skipAll() { readPos = data.size(); }

private:
    uint16_t opcode = 0;
    std::vector<uint8_t> data;
    size_t readPos = 0;

    // The open bit byte on each side. 8 means none is open, which is what
    // makes flushBits() and resetBitReader() free on the byte paths.
    uint8_t writeBitValue = 0;
    uint8_t writeBitPos = 8;
    uint8_t readBitValue = 0;
    uint8_t readBitPos = 8;
};

} // namespace network
} // namespace wowee
