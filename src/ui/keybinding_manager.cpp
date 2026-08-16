#include <algorithm>
#include <functional>
#include <vector>
#include "ui/keybinding_manager.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <sstream>

namespace wowee::ui {

static bool isReservedMovementKey(ImGuiKey key) {
    return key == ImGuiKey_W || key == ImGuiKey_A || key == ImGuiKey_S ||
           key == ImGuiKey_D || key == ImGuiKey_Q || key == ImGuiKey_E;
}

KeybindingManager& KeybindingManager::getInstance() {
    static KeybindingManager instance;
    return instance;
}

KeybindingManager::KeybindingManager() {
    initializeDefaults();
}

void KeybindingManager::initializeDefaults() {
    // Set default keybindings
    bindings_[static_cast<int>(Action::TOGGLE_CHARACTER_SCREEN)] = ImGuiKey_C;
    bindings_[static_cast<int>(Action::TOGGLE_INVENTORY)] = ImGuiKey_I;
    bindings_[static_cast<int>(Action::TOGGLE_BAGS)] = ImGuiKey_B;
    bindings_[static_cast<int>(Action::TOGGLE_SPELLBOOK)] = ImGuiKey_P;  // WoW standard key
    bindings_[static_cast<int>(Action::TOGGLE_TALENTS)] = ImGuiKey_N;     // WoW standard key
    bindings_[static_cast<int>(Action::TOGGLE_QUESTS)] = ImGuiKey_L;
    bindings_[static_cast<int>(Action::TOGGLE_MINIMAP)] = ImGuiKey_None;  // minimap is always visible; no default toggle
    bindings_[static_cast<int>(Action::TOGGLE_SETTINGS)] = ImGuiKey_Escape;
    bindings_[static_cast<int>(Action::TOGGLE_CHAT)] = ImGuiKey_Enter;
    bindings_[static_cast<int>(Action::TOGGLE_GUILD_ROSTER)] = ImGuiKey_O;
    bindings_[static_cast<int>(Action::TOGGLE_DUNGEON_FINDER)] = ImGuiKey_J;  // Originally I, reassigned to avoid conflict
    bindings_[static_cast<int>(Action::TOGGLE_WORLD_MAP)] = ImGuiKey_M;  // WoW standard: M opens world map
    bindings_[static_cast<int>(Action::TOGGLE_NAMEPLATES)] = ImGuiKey_V;
    bindings_[static_cast<int>(Action::TOGGLE_RAID_FRAMES)] = ImGuiKey_F;  // Reassigned from R (now camera reset)
    bindings_[static_cast<int>(Action::TOGGLE_ACHIEVEMENTS)] = ImGuiKey_Y;  // WoW standard key (Shift+Y in retail)
    bindings_[static_cast<int>(Action::TOGGLE_SKILLS)]      = ImGuiKey_K;  // WoW standard: K opens Skills/Professions
}

bool KeybindingManager::isActionPressed(Action action, bool repeat) {
    auto it = bindings_.find(static_cast<int>(action));
    if (it == bindings_.end()) return false;
    ImGuiKey key = it->second;
    if (key == ImGuiKey_None) return false;

    // Someone typing into a FrameXML edit box gets no bindings at all, which is
    // what the real client does and what the event path here already did - it
    // hands the key to the box and stops. This is the other way in: every panel
    // polls its key from inside its own draw, and a poll never passed through
    // that path, so the key arrived twice.
    if (interfaceTakingTypedInput()) return false;

    // When typing in a text field (e.g. chat input), never treat A-Z or 0-9 as shortcuts.
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) {
        if ((key >= ImGuiKey_A && key <= ImGuiKey_Z) ||
            (key >= ImGuiKey_0 && key <= ImGuiKey_9)) {
            return false;
        }
    }

    return ImGui::IsKeyPressed(key, repeat);
}

namespace {
/// One probe for the whole client, set while the interface is built and read
/// from every path that has to know whether someone is typing.
std::function<bool()>& typedInputProbe() {
    static std::function<bool()> probe;
    return probe;
}
}  // namespace

bool interfaceTakingTypedInput() {
    const auto& probe = typedInputProbe();
    return probe && probe();
}

void setTypedInputProbe(std::function<bool()> probe) {
    typedInputProbe() = std::move(probe);
}

namespace {
/// Set by the pump, read by the poll, cleared between iterations. A handful of
/// keys at most - only the ones whose handling lets go of the box.
std::vector<ImGuiKey>& consumedKeys() {
    static std::vector<ImGuiKey> keys;
    return keys;
}
}  // namespace

bool interfaceConsumedKey(ImGuiKey key) {
    const auto& keys = consumedKeys();
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

void noteInterfaceConsumedKey(ImGuiKey key) {
    if (!interfaceConsumedKey(key)) consumedKeys().push_back(key);
}

void clearInterfaceConsumedKeys() { consumedKeys().clear(); }

ImGuiKey KeybindingManager::getKeyForAction(Action action) const {
    auto it = bindings_.find(static_cast<int>(action));
    if (it == bindings_.end()) return ImGuiKey_None;
    return it->second;
}

void KeybindingManager::setKeyForAction(Action action, ImGuiKey key) {
    // Reserve movement keys so they cannot be used as UI shortcuts.
    (void)action;
    if (isReservedMovementKey(key)) {
        key = ImGuiKey_None;
    }
    bindings_[static_cast<int>(action)] = key;
}

void KeybindingManager::resetToDefaults() {
    bindings_.clear();
    initializeDefaults();
}

const char* KeybindingManager::getActionName(Action action) {
    switch (action) {
        case Action::TOGGLE_CHARACTER_SCREEN: return "Character Screen";
        case Action::TOGGLE_INVENTORY: return "Inventory";
        case Action::TOGGLE_BAGS: return "Bags";
        case Action::TOGGLE_SPELLBOOK: return "Spellbook";
        case Action::TOGGLE_TALENTS: return "Talents";
        case Action::TOGGLE_QUESTS: return "Quests";
        case Action::TOGGLE_MINIMAP: return "Minimap";
        case Action::TOGGLE_SETTINGS: return "Settings";
        case Action::TOGGLE_CHAT: return "Chat";
        case Action::TOGGLE_GUILD_ROSTER: return "Guild Roster / Social";
        case Action::TOGGLE_DUNGEON_FINDER: return "Dungeon Finder";
        case Action::TOGGLE_WORLD_MAP: return "World Map";
        case Action::TOGGLE_NAMEPLATES: return "Nameplates";
        case Action::TOGGLE_RAID_FRAMES: return "Raid Frames";
        case Action::TOGGLE_ACHIEVEMENTS: return "Achievements";
        case Action::TOGGLE_SKILLS: return "Skills / Professions";
        case Action::ACTION_COUNT: break;
    }
    return "Unknown";
}

void KeybindingManager::loadFromConfigFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        LOG_ERROR("KeybindingManager: Failed to open config file: ", filePath);
        return;
    }

    std::string line;
    bool inKeybindingsSection = false;

    while (std::getline(file, line)) {
        // Trim whitespace
        size_t start = line.find_first_not_of(" \t\r\n");
        size_t end = line.find_last_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        line = line.substr(start, end - start + 1);

        // Check for section header
        if (line == "[Keybindings]") {
            inKeybindingsSection = true;
            continue;
        } if (line[0] == '[') {
            inKeybindingsSection = false;
            continue;
        }

        if (!inKeybindingsSection || line.empty() || line[0] == ';' || line[0] == '#') continue;

        // Parse key=value pair
        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos) continue;

        std::string action = line.substr(0, eqPos);
        std::string keyStr = line.substr(eqPos + 1);

        // Trim key string
        size_t kStart = keyStr.find_first_not_of(" \t");
        size_t kEnd = keyStr.find_last_not_of(" \t");
        if (kStart != std::string::npos) {
            keyStr = keyStr.substr(kStart, kEnd - kStart + 1);
        }

        // Map action name to enum (simplified mapping)
        int actionIdx = -1;
        if (action == "toggle_character_screen") actionIdx = static_cast<int>(Action::TOGGLE_CHARACTER_SCREEN);
        else if (action == "toggle_inventory") actionIdx = static_cast<int>(Action::TOGGLE_INVENTORY);
        else if (action == "toggle_bags") actionIdx = static_cast<int>(Action::TOGGLE_BAGS);
        else if (action == "toggle_spellbook") actionIdx = static_cast<int>(Action::TOGGLE_SPELLBOOK);
        else if (action == "toggle_talents") actionIdx = static_cast<int>(Action::TOGGLE_TALENTS);
        else if (action == "toggle_quests") actionIdx = static_cast<int>(Action::TOGGLE_QUESTS);
        else if (action == "toggle_minimap") actionIdx = static_cast<int>(Action::TOGGLE_MINIMAP);
        else if (action == "toggle_settings") actionIdx = static_cast<int>(Action::TOGGLE_SETTINGS);
        else if (action == "toggle_chat") actionIdx = static_cast<int>(Action::TOGGLE_CHAT);
        else if (action == "toggle_guild_roster") actionIdx = static_cast<int>(Action::TOGGLE_GUILD_ROSTER);
        else if (action == "toggle_dungeon_finder") actionIdx = static_cast<int>(Action::TOGGLE_DUNGEON_FINDER);
        else if (action == "toggle_world_map") actionIdx = static_cast<int>(Action::TOGGLE_WORLD_MAP);
        else if (action == "toggle_nameplates") actionIdx = static_cast<int>(Action::TOGGLE_NAMEPLATES);
        else if (action == "toggle_raid_frames") actionIdx = static_cast<int>(Action::TOGGLE_RAID_FRAMES);
        else if (action == "toggle_quest_log") actionIdx = static_cast<int>(Action::TOGGLE_QUESTS);  // legacy alias
        else if (action == "toggle_achievements") actionIdx = static_cast<int>(Action::TOGGLE_ACHIEVEMENTS);
        else if (action == "toggle_skills") actionIdx = static_cast<int>(Action::TOGGLE_SKILLS);

        if (actionIdx < 0) continue;

        // Parse key string to ImGuiKey (simple mapping of common keys)
        ImGuiKey key = ImGuiKey_None;
        if (keyStr.length() == 1) {
            // Single character key (A-Z, 0-9)
            char c = keyStr[0];
            if (c >= 'A' && c <= 'Z') {
                key = static_cast<ImGuiKey>(ImGuiKey_A + (c - 'A'));
            } else if (c >= '0' && c <= '9') {
                key = static_cast<ImGuiKey>(ImGuiKey_0 + (c - '0'));
            }
        } else if (keyStr == "Escape") {
            key = ImGuiKey_Escape;
        } else if (keyStr == "Enter") {
            key = ImGuiKey_Enter;
        } else if (keyStr == "Tab") {
            key = ImGuiKey_Tab;
        } else if (keyStr == "Backspace") {
            key = ImGuiKey_Backspace;
        } else if (keyStr == "Space") {
            key = ImGuiKey_Space;
        } else if (keyStr == "Delete") {
            key = ImGuiKey_Delete;
        } else if (keyStr == "Home") {
            key = ImGuiKey_Home;
        } else if (keyStr == "End") {
            key = ImGuiKey_End;
        } else if (keyStr.find("F") == 0 && keyStr.length() <= 3) {
            // F1-F12 keys
            try {
                int fNum = std::stoi(keyStr.substr(1));
                if (fNum >= 1 && fNum <= 12) {
                    key = static_cast<ImGuiKey>(ImGuiKey_F1 + (fNum - 1));
                }
            } catch (...) {}
        }

        if (key == ImGuiKey_None) continue;

        // Reserve movement keys so they cannot be used as UI shortcuts.
        if (isReservedMovementKey(key)) {
            continue;
        }

        bindings_[actionIdx] = key;
    }

    file.close();
    LOG_INFO("KeybindingManager: Loaded keybindings from ", filePath);
}

void KeybindingManager::saveToConfigFile(const std::string& filePath) const {
    std::ifstream inFile(filePath);
    std::string content;
    std::string line;

    // Read existing file, removing [Keybindings] section if it exists
    bool inKeybindingsSection = false;
    if (inFile.is_open()) {
        while (std::getline(inFile, line)) {
            if (line == "[Keybindings]") {
                inKeybindingsSection = true;
                continue;
            } if (line[0] == '[') {
                inKeybindingsSection = false;
            }

            if (!inKeybindingsSection) {
                content += line + "\n";
            }
        }
        inFile.close();
    }

    // Append new Keybindings section
    content += "[Keybindings]\n";

    static constexpr struct {
        Action action;
        const char* name;
    } actionMap[] = {
        {.action = Action::TOGGLE_CHARACTER_SCREEN, .name = "toggle_character_screen"},
        {.action = Action::TOGGLE_INVENTORY, .name = "toggle_inventory"},
        {.action = Action::TOGGLE_BAGS, .name = "toggle_bags"},
        {.action = Action::TOGGLE_SPELLBOOK, .name = "toggle_spellbook"},
        {.action = Action::TOGGLE_TALENTS, .name = "toggle_talents"},
        {.action = Action::TOGGLE_QUESTS, .name = "toggle_quests"},
        {.action = Action::TOGGLE_MINIMAP, .name = "toggle_minimap"},
        {.action = Action::TOGGLE_SETTINGS, .name = "toggle_settings"},
        {.action = Action::TOGGLE_CHAT, .name = "toggle_chat"},
        {.action = Action::TOGGLE_GUILD_ROSTER, .name = "toggle_guild_roster"},
        {.action = Action::TOGGLE_DUNGEON_FINDER, .name = "toggle_dungeon_finder"},
        {.action = Action::TOGGLE_WORLD_MAP, .name = "toggle_world_map"},
        {.action = Action::TOGGLE_NAMEPLATES, .name = "toggle_nameplates"},
        {.action = Action::TOGGLE_RAID_FRAMES, .name = "toggle_raid_frames"},
        {.action = Action::TOGGLE_ACHIEVEMENTS, .name = "toggle_achievements"},
        {.action = Action::TOGGLE_SKILLS, .name = "toggle_skills"},
    };

    for (const auto& [action, nameStr] : actionMap) {
        auto it = bindings_.find(static_cast<int>(action));
        if (it == bindings_.end()) continue;

        ImGuiKey key = it->second;
        std::string keyStr;

        // Convert ImGuiKey to string
        if (key >= ImGuiKey_A && key <= ImGuiKey_Z) {
            keyStr += static_cast<char>('A' + (key - ImGuiKey_A));
        } else if (key >= ImGuiKey_0 && key <= ImGuiKey_9) {
            keyStr += static_cast<char>('0' + (key - ImGuiKey_0));
        } else if (key == ImGuiKey_Escape) {
            keyStr = "Escape";
        } else if (key == ImGuiKey_Enter) {
            keyStr = "Enter";
        } else if (key == ImGuiKey_Tab) {
            keyStr = "Tab";
        } else if (key == ImGuiKey_Backspace) {
            keyStr = "Backspace";
        } else if (key == ImGuiKey_Space) {
            keyStr = "Space";
        } else if (key == ImGuiKey_Delete) {
            keyStr = "Delete";
        } else if (key == ImGuiKey_Home) {
            keyStr = "Home";
        } else if (key == ImGuiKey_End) {
            keyStr = "End";
        } else if (key >= ImGuiKey_F1 && key <= ImGuiKey_F12) {
            keyStr = "F" + std::to_string(1 + (key - ImGuiKey_F1));
        }

        if (!keyStr.empty()) {
            content += nameStr;
            content += "=";
            content += keyStr;
            content += "\n";
        }
    }

    // Write back to file
    std::ofstream outFile(filePath);
    if (outFile.is_open()) {
        outFile << content;
        outFile.close();
        LOG_INFO("KeybindingManager: Saved keybindings to ", filePath);
    } else {
        LOG_ERROR("KeybindingManager: Failed to write config file: ", filePath);
    }
}

}  // namespace wowee::ui
