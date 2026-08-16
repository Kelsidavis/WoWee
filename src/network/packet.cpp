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
    data.push_back(value);
}

void Packet::writeUInt16(uint16_t value) {
    data.push_back(value & 0xFF);
    data.push_back((value >> 8) & 0xFF);
}

void Packet::writeUInt32(uint32_t value) {
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
    for (char c : value) {
        data.push_back(static_cast<uint8_t>(c));
    }
    data.push_back(0); // Null terminator
}

void Packet::writeBytes(const uint8_t* bytes, size_t length) {
    data.insert(data.end(), bytes, bytes + length);
}

uint8_t Packet::readUInt8() {
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

} // namespace network
} // namespace wowee
