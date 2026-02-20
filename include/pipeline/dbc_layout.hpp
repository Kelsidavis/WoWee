#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace wowee {
namespace pipeline {

/**
 * Maps DBC field names to column indices for a single DBC file.
 * Column indices vary between WoW expansions.
 */
struct DBCFieldMap {
    std::unordered_map<std::string, uint32_t> fields;

    /** Get column index by field name. Returns 0xFFFFFFFF if unknown. */
    uint32_t field(const std::string& name) const {
        auto it = fields.find(name);
        return (it != fields.end()) ? it->second : 0xFFFFFFFF;
    }

    /** Convenience operator for shorter syntax: layout["Name"] */
    uint32_t operator[](const std::string& name) const { return field(name); }
};

/**
 * Maps DBC file names to their field layouts.
 * Loaded from JSON (e.g. Data/expansions/wotlk/dbc_layouts.json).
 */
class DBCLayout {
public:
    /** Load from JSON file. Returns true if successful. */
    bool loadFromJson(const std::string& path);

    /** Get the field map for a DBC file. Returns nullptr if unknown. */
    const DBCFieldMap* getLayout(const std::string& dbcName) const;

    /** Number of DBC layouts loaded. */
    size_t size() const { return layouts_.size(); }

private:
    std::unordered_map<std::string, DBCFieldMap> layouts_;
};

/**
 * Global active DBC layout (set by Application at startup).
 */
void setActiveDBCLayout(const DBCLayout* layout);
const DBCLayout* getActiveDBCLayout();

/** Convenience: get field index for a DBC field. */
inline uint32_t dbcField(const std::string& dbcName, const std::string& fieldName) {
    const auto* l = getActiveDBCLayout();
    if (!l) return 0xFFFFFFFF;
    const auto* fm = l->getLayout(dbcName);
    return fm ? fm->field(fieldName) : 0xFFFFFFFF;
}

} // namespace pipeline
} // namespace wowee
