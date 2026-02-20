#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <sstream>

namespace wowee {
namespace pipeline {

static const DBCLayout* g_activeDBCLayout = nullptr;

void setActiveDBCLayout(const DBCLayout* layout) { g_activeDBCLayout = layout; }
const DBCLayout* getActiveDBCLayout() { return g_activeDBCLayout; }

bool DBCLayout::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("DBCLayout: cannot open ", path);
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    layouts_.clear();
    size_t loaded = 0;
    size_t pos = 0;

    // Parse top-level object: { "DbcName": { "FieldName": index, ... }, ... }
    // Find the first '{'
    pos = json.find('{', pos);
    if (pos == std::string::npos) return false;
    ++pos;

    while (pos < json.size()) {
        // Find DBC name key
        size_t dbcKeyStart = json.find('"', pos);
        if (dbcKeyStart == std::string::npos) break;
        size_t dbcKeyEnd = json.find('"', dbcKeyStart + 1);
        if (dbcKeyEnd == std::string::npos) break;
        std::string dbcName = json.substr(dbcKeyStart + 1, dbcKeyEnd - dbcKeyStart - 1);

        // Find the nested object '{'
        size_t objStart = json.find('{', dbcKeyEnd);
        if (objStart == std::string::npos) break;

        // Find the matching '}'
        size_t objEnd = json.find('}', objStart);
        if (objEnd == std::string::npos) break;

        // Parse the inner object
        std::string inner = json.substr(objStart + 1, objEnd - objStart - 1);
        DBCFieldMap fieldMap;
        size_t ipos = 0;
        while (ipos < inner.size()) {
            size_t fkStart = inner.find('"', ipos);
            if (fkStart == std::string::npos) break;
            size_t fkEnd = inner.find('"', fkStart + 1);
            if (fkEnd == std::string::npos) break;
            std::string fieldName = inner.substr(fkStart + 1, fkEnd - fkStart - 1);

            size_t colon = inner.find(':', fkEnd);
            if (colon == std::string::npos) break;
            size_t valStart = colon + 1;
            while (valStart < inner.size() && (inner[valStart] == ' ' || inner[valStart] == '\t' ||
                   inner[valStart] == '\r' || inner[valStart] == '\n'))
                ++valStart;
            size_t valEnd = inner.find_first_of(",}\r\n", valStart);
            if (valEnd == std::string::npos) valEnd = inner.size();
            std::string valStr = inner.substr(valStart, valEnd - valStart);
            while (!valStr.empty() && (valStr.back() == ' ' || valStr.back() == '\t'))
                valStr.pop_back();

            try {
                uint32_t idx = static_cast<uint32_t>(std::stoul(valStr));
                fieldMap.fields[fieldName] = idx;
            } catch (...) {}

            ipos = valEnd + 1;
        }

        if (!fieldMap.fields.empty()) {
            layouts_[dbcName] = std::move(fieldMap);
            ++loaded;
        }

        pos = objEnd + 1;
    }

    LOG_INFO("DBCLayout: loaded ", loaded, " layouts from ", path);
    return loaded > 0;
}

const DBCFieldMap* DBCLayout::getLayout(const std::string& dbcName) const {
    auto it = layouts_.find(dbcName);
    return (it != layouts_.end()) ? &it->second : nullptr;
}

} // namespace pipeline
} // namespace wowee
