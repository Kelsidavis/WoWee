#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace wowee::addons {

struct TocFile {
    std::string addonName;
    std::string basePath;

    std::unordered_map<std::string, std::string> directives;
    std::vector<std::string> files;

    [[nodiscard]] std::string getTitle() const;
    [[nodiscard]] bool isLoadOnDemand() const;
    [[nodiscard]] std::vector<std::string> getSavedVariables() const;
    [[nodiscard]] std::vector<std::string> getSavedVariablesPerCharacter() const;
};

std::optional<TocFile> parseTocFile(const std::string& tocPath);

} // namespace wowee::addons
