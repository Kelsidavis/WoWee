#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include <cctype>
#include <cstring>
#include <set>
#include <sstream>
#include <string>

namespace wowee {
namespace pipeline {

namespace {
std::string trimAscii(std::string s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
        ++b;
    }
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }
    return s.substr(b, e - b);
}
} // namespace

DBCFile::DBCFile() = default;
DBCFile::~DBCFile() = default;

bool DBCFile::load(const std::vector<uint8_t>& dbcData) {
    if (dbcData.empty()) {
        LOG_ERROR("DBC data is empty");
        return false;
    }

    // Detect metadata CSV format: starts with '#'
    if (dbcData[0] == '#') {
        return loadCSV(dbcData);
    }

    // Detect plain CSV format (column-name header row, e.g. "id,raceId,...\n21,1,0\n")
    // Guard: binary WDBC files start with the four printable bytes "WDBC", so
    // we must explicitly exclude them before checking for plain-text content.
    if (std::isalpha(static_cast<unsigned char>(dbcData[0])) &&
        (dbcData.size() < 4 || std::memcmp(dbcData.data(), "WDBC", 4) != 0)) {
        bool isPlainText = true;
        size_t checkLen = std::min(dbcData.size(), size_t(256));
        for (size_t i = 0; i < checkLen; ++i) {
            uint8_t c = dbcData[i];
            if (c == '\n' || c == '\r') break;
            if (c < 0x20 || c > 0x7E) { isPlainText = false; break; }
        }
        if (isPlainText) {
            return loadPlainCSV(dbcData);
        }
    }

    if (dbcData.size() < sizeof(DBCHeader)) {
        LOG_ERROR("DBC data too small for header");
        return false;
    }

    // Read header
    const DBCHeader* header = reinterpret_cast<const DBCHeader*>(dbcData.data());

    // Verify magic
    if (std::memcmp(header->magic, "WDBC", 4) != 0) {
        LOG_ERROR("Invalid DBC magic: ", std::string(header->magic, 4));
        return false;
    }

    recordCount = header->recordCount;
    fieldCount = header->fieldCount;
    recordSize = header->recordSize;
    stringBlockSize = header->stringBlockSize;

    // Validate sizes
    uint32_t expectedSize = sizeof(DBCHeader) + (recordCount * recordSize) + stringBlockSize;
    if (dbcData.size() < expectedSize) {
        LOG_ERROR("DBC file truncated: expected ", expectedSize, " bytes, got ", dbcData.size());
        return false;
    }

    // Validate record size matches field count
    if (recordSize != fieldCount * 4) {
        LOG_WARNING("DBC record size mismatch: recordSize=", recordSize,
                    " but fieldCount*4=", fieldCount * 4);
    }

    LOG_DEBUG("Loading DBC: ", recordCount, " records, ",
              fieldCount, " fields, ", recordSize, " bytes/record, ",
              stringBlockSize, " string bytes");

    // Copy record data
    const uint8_t* recordStart = dbcData.data() + sizeof(DBCHeader);
    uint32_t totalRecordSize = recordCount * recordSize;
    recordData.resize(totalRecordSize);
    std::memcpy(recordData.data(), recordStart, totalRecordSize);

    // Copy string block
    const uint8_t* stringStart = recordStart + totalRecordSize;
    stringBlock.resize(stringBlockSize);
    if (stringBlockSize > 0) {
        std::memcpy(stringBlock.data(), stringStart, stringBlockSize);
    }

    loaded = true;
    idCacheBuilt = false;
    idToIndexCache.clear();

    return true;
}

const uint8_t* DBCFile::getRecord(uint32_t index) const {
    if (!loaded || index >= recordCount) {
        return nullptr;
    }

    return recordData.data() + (index * recordSize);
}

uint32_t DBCFile::getUInt32(uint32_t recordIndex, uint32_t fieldIndex) const {
    if (!loaded || recordIndex >= recordCount || fieldIndex >= fieldCount) {
        return 0;
    }

    const uint8_t* record = getRecord(recordIndex);
    if (!record) {
        return 0;
    }

    const uint32_t* field = reinterpret_cast<const uint32_t*>(record + (fieldIndex * 4));
    return *field;
}

int32_t DBCFile::getInt32(uint32_t recordIndex, uint32_t fieldIndex) const {
    return static_cast<int32_t>(getUInt32(recordIndex, fieldIndex));
}

float DBCFile::getFloat(uint32_t recordIndex, uint32_t fieldIndex) const {
    if (!loaded || recordIndex >= recordCount || fieldIndex >= fieldCount) {
        return 0.0f;
    }

    const uint8_t* record = getRecord(recordIndex);
    if (!record) {
        return 0.0f;
    }

    const float* field = reinterpret_cast<const float*>(record + (fieldIndex * 4));
    return *field;
}

std::string DBCFile::getString(uint32_t recordIndex, uint32_t fieldIndex) const {
    uint32_t offset = getUInt32(recordIndex, fieldIndex);
    return getStringByOffset(offset);
}

std::string DBCFile::getStringByOffset(uint32_t offset) const {
    if (!loaded || offset >= stringBlockSize) {
        return "";
    }

    // Find null terminator
    const char* str = reinterpret_cast<const char*>(stringBlock.data() + offset);
    const char* end = reinterpret_cast<const char*>(stringBlock.data() + stringBlockSize);

    // Find string length (up to null terminator or end of block)
    size_t length = 0;
    while (str + length < end && str[length] != '\0') {
        length++;
    }

    return std::string(str, length);
}

int32_t DBCFile::findRecordById(uint32_t id) const {
    if (!loaded) {
        return -1;
    }

    // Build ID cache if not already built
    if (!idCacheBuilt) {
        buildIdCache();
    }

    auto it = idToIndexCache.find(id);
    if (it != idToIndexCache.end()) {
        return static_cast<int32_t>(it->second);
    }

    return -1;
}

void DBCFile::buildIdCache() const {
    idToIndexCache.clear();

    for (uint32_t i = 0; i < recordCount; i++) {
        uint32_t id = getUInt32(i, 0);  // Assume first field is ID
        idToIndexCache[id] = i;
    }

    idCacheBuilt = true;
    LOG_DEBUG("Built DBC ID cache with ", idToIndexCache.size(), " entries");
}

bool DBCFile::loadCSV(const std::vector<uint8_t>& csvData) {
    std::string text(reinterpret_cast<const char*>(csvData.data()), csvData.size());
    std::istringstream stream(text);
    std::string line;

    // --- Parse metadata line: # fields=N strings=I,J,K ---
    if (!std::getline(stream, line) || line.empty() || line[0] != '#') {
        LOG_ERROR("CSV DBC: missing metadata line");
        return false;
    }

    fieldCount = 0;
    std::set<uint32_t> stringCols;

    // Parse "fields=N"
    auto fieldsPos = line.find("fields=");
    if (fieldsPos != std::string::npos) {
        try {
            fieldCount = static_cast<uint32_t>(std::stoul(line.substr(fieldsPos + 7)));
        } catch (...) {
            fieldCount = 0;
        }
    }
    if (fieldCount == 0) {
        LOG_ERROR("CSV DBC: invalid field count");
        return false;
    }

    // Parse "strings=I,J,K"
    auto stringsPos = line.find("strings=");
    if (stringsPos != std::string::npos) {
        std::istringstream ss(line.substr(stringsPos + 8));
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            tok = trimAscii(tok);
            if (!tok.empty()) {
                try {
                    stringCols.insert(static_cast<uint32_t>(std::stoul(tok)));
                } catch (...) {
                    LOG_WARNING("CSV DBC: invalid string column index token: '", tok, "'");
                }
            }
        }
    }

    // Field 0 is always the numeric record ID in DBC files — never a string.
    // Some CSV exports incorrectly mark it as a string column; force-remove it.
    if (stringCols.erase(0) > 0) {
        LOG_DEBUG("CSV DBC: removed field 0 from string columns (always numeric ID)");
    }

    recordSize = fieldCount * 4;

    // --- Build string block with initial null byte ---
    stringBlock.clear();
    stringBlock.push_back(0); // offset 0 = empty string

    // --- Parse data rows ---
    struct RowData {
        std::vector<uint32_t> fields;
    };
    std::vector<RowData> rows;

    while (std::getline(stream, line)) {
        if (line.empty()) continue;

        RowData row;
        row.fields.resize(fieldCount, 0);

        uint32_t col = 0;
        size_t pos = 0;

        while (col < fieldCount && pos < line.size()) {
            if (stringCols.count(col) && pos < line.size() && line[pos] == '"') {
                // Quoted string field
                pos++; // skip opening quote
                std::string str;
                while (pos < line.size()) {
                    if (line[pos] == '"') {
                        if (pos + 1 < line.size() && line[pos + 1] == '"') {
                            str += '"'; // escaped quote
                            pos += 2;
                        } else {
                            pos++; // closing quote
                            break;
                        }
                    } else {
                        str += line[pos++];
                    }
                }
                // Skip comma after closing quote
                if (pos < line.size() && line[pos] == ',') pos++;

                // Store string in string block
                if (str.empty()) {
                    row.fields[col] = 0; // points to empty string at offset 0
                } else {
                    uint32_t offset = static_cast<uint32_t>(stringBlock.size());
                    stringBlock.insert(stringBlock.end(), str.begin(), str.end());
                    stringBlock.push_back(0); // null terminator
                    row.fields[col] = offset;
                }
            } else {
                // Numeric field — read until comma or end of line
                size_t end = line.find(',', pos);
                if (end == std::string::npos) end = line.size();
                std::string tok = line.substr(pos, end - pos);
                if (!tok.empty()) {
                    try {
                        row.fields[col] = static_cast<uint32_t>(std::stoul(tok));
                    } catch (...) {
                        row.fields[col] = 0; // non-numeric value in numeric field
                    }
                }
                pos = (end < line.size()) ? end + 1 : line.size();
            }
            col++;
        }

        rows.push_back(std::move(row));
    }

    // --- Build record data (binary layout identical to WDBC) ---
    recordCount = static_cast<uint32_t>(rows.size());
    stringBlockSize = static_cast<uint32_t>(stringBlock.size());

    recordData.resize(static_cast<size_t>(recordCount) * recordSize);
    for (uint32_t i = 0; i < recordCount; ++i) {
        uint8_t* dst = recordData.data() + static_cast<size_t>(i) * recordSize;
        for (uint32_t f = 0; f < fieldCount; ++f) {
            uint32_t val = rows[i].fields[f];
            std::memcpy(dst + f * 4, &val, 4);
        }
    }

    loaded = true;
    idCacheBuilt = false;
    idToIndexCache.clear();

    LOG_DEBUG("Loaded CSV DBC: ", recordCount, " records, ",
              fieldCount, " fields, ", stringCols.size(), " string cols, ",
              stringBlockSize, " string bytes");
    return true;
}

bool DBCFile::loadPlainCSV(const std::vector<uint8_t>& csvData) {
    std::string text(reinterpret_cast<const char*>(csvData.data()), csvData.size());
    std::istringstream stream(text);
    std::string line;

    // First line is the column-name header — use it only to count fields.
    if (!std::getline(stream, line) || line.empty()) {
        LOG_ERROR("Plain CSV DBC: missing header line");
        return false;
    }
    if (!line.empty() && line.back() == '\r') line.pop_back();

    fieldCount = 1;
    for (char c : line) {
        if (c == ',') ++fieldCount;
    }
    if (fieldCount == 0) {
        LOG_ERROR("Plain CSV DBC: invalid field count");
        return false;
    }

    recordSize = fieldCount * 4;

    // Build a string block on the fly.  Offset 0 = empty string (always present).
    std::vector<uint8_t> strBlock(1, 0);

    // Helper: intern a string into strBlock, return its offset.
    auto internString = [&](const std::string& s) -> uint32_t {
        if (s.empty()) return 0;
        uint32_t offset = static_cast<uint32_t>(strBlock.size());
        for (char c : s) strBlock.push_back(static_cast<uint8_t>(c));
        strBlock.push_back(0); // null terminator
        return offset;
    };

    // Parse data rows.  Non-numeric cells are stored in strBlock.
    std::vector<std::vector<uint32_t>> rows;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        std::vector<uint32_t> row(fieldCount, 0);
        uint32_t col = 0;
        size_t pos = 0;
        while (col < fieldCount && pos <= line.size()) {
            size_t end = line.find(',', pos);
            if (end == std::string::npos) end = line.size();
            std::string tok = trimAscii(line.substr(pos, end - pos));
            if (!tok.empty()) {
                bool isNumeric = false;
                try {
                    int64_t v = std::stoll(tok);
                    row[col] = static_cast<uint32_t>(static_cast<int32_t>(v));
                    isNumeric = true;
                } catch (...) {}
                if (!isNumeric) {
                    // Non-numeric → store as string in the string block.
                    row[col] = internString(tok);
                }
            }
            pos = (end < line.size()) ? end + 1 : line.size() + 1;
            ++col;
        }
        rows.push_back(std::move(row));
    }

    stringBlock     = std::move(strBlock);
    stringBlockSize = static_cast<uint32_t>(stringBlock.size());

    recordCount = static_cast<uint32_t>(rows.size());
    recordData.resize(static_cast<size_t>(recordCount) * recordSize);
    for (uint32_t i = 0; i < recordCount; ++i) {
        uint8_t* dst = recordData.data() + static_cast<size_t>(i) * recordSize;
        for (uint32_t f = 0; f < fieldCount; ++f) {
            uint32_t val = rows[i][f];
            std::memcpy(dst + f * 4, &val, 4);
        }
    }

    loaded = true;
    idCacheBuilt = false;
    idToIndexCache.clear();

    LOG_DEBUG("Loaded plain CSV DBC: ", recordCount, " records, ", fieldCount,
              " fields, ", stringBlockSize, " string bytes");
    return true;
}

} // namespace pipeline
} // namespace wowee
