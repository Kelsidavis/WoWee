#include "network/packet.hpp"

#include <bit>
#include <cstring>
#include <utility>

namespace wowee {
namespace network {

Packet::Packet(uint16_t opcode) : opcode(opcode) {}

Packet::Packet(uint16_t opcode, const std::vector<uint8_t>& data)
    : opcode(opcode), data(data) {}

Packet::Packet(uint16_t opcode, std::vector<uint8_t>&& data)
    : opcode(opcode), data(std::move(data)) {}

void Packet::writeUInt8(uint8_t value) {
    flushBits();
    data.push_back(value);
}

void Packet::writeUInt16(uint16_t value) {
    flushBits();
    data.push_back(value & 0xFF);
    data.push_back((value >> 8) & 0xFF);
}

void Packet::writeUInt32(uint32_t value) {
    flushBits();
    data.push_back(value & 0xFF);
    data.push_back((value >> 8) & 0xFF);
    data.push_back((value >> 16) & 0xFF);
    data.push_back((value >> 24) & 0xFF);
}

void Packet::writeUInt64(uint64_t value) {
    writeUInt32(value & 0xFFFFFFFF);
    writeUInt32((value >> 32) & 0xFFFFFFFF);
}

void Packet::writeFloat(float value) {
    writeUInt32(std::bit_cast<uint32_t>(value));
}

void Packet::writeString(const std::string& value) {
    flushBits();
    for (char c : value) {
        data.push_back(static_cast<uint8_t>(c));
    }
    data.push_back(0); // Null terminator
}

void Packet::writeBytes(const uint8_t* bytes, size_t length) {
    flushBits();
    data.insert(data.end(), bytes, bytes + length);
}

uint8_t Packet::readUInt8() {
    resetBitReader();
    if (readPos >= data.size()) return 0;
    return data[readPos++];
}

uint16_t Packet::readUInt16() {
    uint16_t value = 0;
    value |= readUInt8();
    value |= (readUInt8() << 8);
    return value;
}

uint32_t Packet::readUInt32() {
    uint32_t value = 0;
    value |= readUInt8();
    value |= (readUInt8() << 8);
    value |= (readUInt8() << 16);
    value |= (readUInt8() << 24);
    return value;
}

uint64_t Packet::readUInt64() {
    uint64_t value = readUInt32();
    value |= (static_cast<uint64_t>(readUInt32()) << 32);
    return value;
}

float Packet::readFloat() {
    return std::bit_cast<float>(readUInt32());
}

uint64_t Packet::readPackedGuid() {
    uint8_t mask = readUInt8();
    if (mask == 0) return 0;
    uint64_t guid = 0;
    for (int i = 0; i < 8; ++i) {
        if (mask & (1 << i))
            guid |= static_cast<uint64_t>(readUInt8()) << (i * 8);
    }
    return guid;
}

void Packet::writePackedGuid(uint64_t guid) {
    uint8_t mask = 0;
    uint8_t guidBytes[8];
    int count = 0;
    for (int i = 0; i < 8; ++i) {
        uint8_t byte = static_cast<uint8_t>((guid >> (i * 8)) & 0xFF);
        if (byte != 0) {
            mask |= (1 << i);
            guidBytes[count++] = byte;
        }
    }
    writeUInt8(mask);
    for (int i = 0; i < count; ++i)
        writeUInt8(guidBytes[i]);
}

std::string Packet::readString() {
    resetBitReader();
    std::string result;
    while (readPos < data.size()) {
        uint8_t c = data[readPos++];
        if (c == 0) break;
        result += static_cast<char>(c);
    }
    return result;
}

bool Packet::readSizedString(std::string& out, uint32_t maxLength) {
    const size_t mark = readPos;
    const uint32_t length = readUInt32();
    if (length == 0 || length >= maxLength || length > getRemainingSize()) {
        readPos = mark;
        return false;
    }
    out.resize(length);
    for (uint32_t i = 0; i < length; ++i) {
        out[i] = static_cast<char>(readUInt8());
    }
    // The length counts the terminator, so it is in the string and has to come
    // back off.
    if (!out.empty() && out.back() == '\0') out.pop_back();
    return true;
}


// ---------------------------------------------------------------------------
// Bit-level access. See the contract on packet.hpp.
// ---------------------------------------------------------------------------

void Packet::writeBit(bool value) {
    --writeBitPos;
    if (value) writeBitValue |= static_cast<uint8_t>(1u << writeBitPos);
    if (writeBitPos == 0) {
        // Emitted directly rather than through writeUInt8, which would flush
        // this byte a second time.
        data.push_back(writeBitValue);
        writeBitValue = 0;
        writeBitPos = 8;
    }
}

void Packet::writeBits(uint64_t value, int count) {
    if (count < 1 || count > 64) return;
    for (int i = count - 1; i >= 0; --i)
        writeBit(((value >> i) & 1u) != 0);
}

void Packet::flushBits() {
    if (writeBitPos == 8) return;
    const uint8_t pending = writeBitValue;
    writeBitValue = 0;
    writeBitPos = 8;
    data.push_back(pending);
}

bool Packet::readBit() {
    if (readBitPos == 8) {
        if (readPos >= data.size()) return false;
        readBitValue = data[readPos++];
        readBitPos = 0;
    }
    const bool value = ((readBitValue >> (7 - readBitPos)) & 1u) != 0;
    ++readBitPos;
    return value;
}

uint64_t Packet::readBits(int count) {
    if (count < 1 || count > 64) return 0;
    uint64_t value = 0;
    for (int i = count - 1; i >= 0; --i)
        if (readBit()) value |= (static_cast<uint64_t>(1) << i);
    return value;
}

void Packet::resetBitReader() {
    // Guarded so the byte readers, which call this on every field of every
    // packet, pay a compare rather than two stores.
    if (readBitPos == 8) return;
    readBitValue = 0;
    readBitPos = 8;
}

void Packet::writeGuidMask(uint64_t guid, const uint8_t* order, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const uint8_t index = order[i];
        if (index >= 8) continue;
        writeBit(((guid >> (index * 8)) & 0xFFu) != 0);
    }
}

void Packet::writeGuidBytes(uint64_t guid, const uint8_t* order, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const uint8_t index = order[i];
        if (index >= 8) continue;
        const uint8_t byte = static_cast<uint8_t>((guid >> (index * 8)) & 0xFFu);
        // The byte travels XOR'd with 1, and a zero byte does not travel at
        // all - the mask bit already said so.
        if (byte != 0) writeUInt8(static_cast<uint8_t>(byte ^ 1u));
    }
}

void Packet::readGuidMask(GuidBytes& guid, const uint8_t* order, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const uint8_t index = order[i];
        if (index >= 8) continue;
        guid[index] = readBit() ? 1u : 0u;
    }
}

void Packet::readGuidBytes(GuidBytes& guid, const uint8_t* order, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const uint8_t index = order[i];
        if (index >= 8) continue;
        // A mask entry of 1 means a byte follows; XOR undoes the writer's.
        if (guid[index] != 0) guid[index] ^= readUInt8();
    }
}

uint64_t Packet::guidFromBytes(const GuidBytes& guid) {
    uint64_t value = 0;
    for (size_t i = 0; i < guid.size(); ++i)
        value |= static_cast<uint64_t>(guid[i]) << (i * 8);
    return value;
}

} // namespace network
} // namespace wowee
