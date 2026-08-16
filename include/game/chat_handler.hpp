#pragma once

#include <deque>

#include "game/chat_filters.hpp"

#include "game/world_packets.hpp"
#include "game/opcode_table.hpp"
#include "game/handler_types.hpp"
#include "network/packet.hpp"
#include <deque>
#include <fstream>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;

class ChatHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit ChatHandler(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // --- Public API (delegated from GameHandler) ---
    void sendChatMessage(ChatType type, const std::string& message, const std::string& target = "");
    void sendAddonMessage(ChatType type, const std::string& message, const std::string& target = "");
    void sendTextEmote(uint32_t textEmoteId, uint64_t targetGuid = 0);
    void joinChannel(const std::string& channelName, const std::string& password = "");
    void leaveChannel(const std::string& channelName);
    /// Ask the server for a channel's members; the answer arrives as
    /// SMSG_CHANNEL_LIST and fires CHANNEL_ROSTER_UPDATE.
    void requestChannelList(const std::string& channelName);
    [[nodiscard]] std::string getChannelByIndex(int index) const;
    [[nodiscard]] int getChannelIndex(const std::string& channelName) const;
    [[nodiscard]] const std::vector<std::string>& getJoinedChannels() const { return joinedChannels_; }
    /// Whether the player owns this channel - the one who created it, or who
    /// it passed to. SMSG_CHANNEL_NOTIFY says so with OWNER_CHANGED, which
    /// carries the new owner's guid: AzerothCore's Channel::MakeOwnerChanged
    /// writes _ownerGUID into it.
    [[nodiscard]] bool ownsChannel(const std::string& name) const {
        return ownedChannels_.count(name) != 0;
    }
    /// Who sent the line with this id, or zero if it is not remembered.
    [[nodiscard]] uint64_t chatLineSender(uint32_t lineId) const {
        for (const auto& [id, guid] : chatLineSenders_)
            if (id == lineId) return guid;
        return 0;
    }

    /// The roster of a channel by name, empty until a list is asked for. The
    /// list used to be printed to chat and dropped, so the channel panel had
    /// no members to show and reported every channel as having none.
    [[nodiscard]] const std::vector<ChannelMember>& getChannelRoster(const std::string& channel) const {
        static const std::vector<ChannelMember> empty;
        auto it = channelRosters_.find(channel);
        return (it != channelRosters_.end()) ? it->second : empty;
    }
    void autoJoinDefaultChannels();
    void addLocalChatMessage(const MessageChatData& msg);

    /// Put a line of a given kind into chat, locally.
    ///
    /// The kind matters: CHAT_MSG_MONEY, CHAT_MSG_COMBAT_XP_GAIN and their like
    /// are chat messages, and the interface's handler reads twelve arguments
    /// off every one of them - arg4 is a channel name and it calls strlen on it
    /// without checking. Firing one of these with just the text raised there and
    /// took the rest of the handler with it, so the line never appeared at all.
    void addLocalChatLine(ChatType type, const std::string& message);
    void addSystemChatMessage(const std::string& message);

    /// Announces a message as CHAT_MSG_<TYPE> with WoW's argument order. Both
    /// the server path and the local one go through here, because an interface
    /// cannot tell the difference and should not have to.
    void fireChatEvent(const MessageChatData& msg);
    void toggleAfk(const std::string& message);
    void toggleDnd(const std::string& message);
    void replyToLastWhisper(const std::string& message);

    // ---- Methods moved from GameHandler ----
    void submitGmTicket(const std::string& text);
    /// Replace the text of the ticket already open.
    void updateGmTicket(const std::string& text);
    void handleMotd(network::Packet& packet);

    // --- State accessors ---
    std::deque<MessageChatData>& getChatHistory() { return chatHistory_; }
    [[nodiscard]] const std::deque<MessageChatData>& getChatHistory() const { return chatHistory_; }
    [[nodiscard]] size_t getMaxChatHistory() const { return maxChatHistory_; }
    void setMaxChatHistory(size_t n) { maxChatHistory_ = n; }

    // Chat auto-join settings (aliased from handler_types.hpp)
    using ChatAutoJoin = game::ChatAutoJoin;
    ChatAutoJoin chatAutoJoin;

private:
    // --- Packet handlers ---
    void handleMessageChat(network::Packet& packet);
    void handleTextEmote(network::Packet& packet);
    void handleChannelNotify(network::Packet& packet);
    void handleChannelList(network::Packet& packet);
    void handleUserlistAdd(network::Packet& packet);
    void handleUserlistRemove(network::Packet& packet);
    void applyUserlistChange(const std::string& chanName, uint64_t guid,
                             uint8_t memberFlags, bool removing);
    void initializeChatLog();
    void logChatMessage(const MessageChatData& msg, const char* source);

    GameHandler& owner_;

    // --- State ---
    std::deque<MessageChatData> chatHistory_;
    /// What has just been said, for the spam filter. Short on purpose: this
    /// is looking for a line pasted again a moment later, not keeping a record
    /// of the conversation. See repeatsRecentLine in chat_filters.hpp.
    std::deque<RecentChatLine> recentChatLines_;
    size_t maxChatHistory_ = 100;
    uint64_t chatUidCounter_ = 0;  // monotonic uid for MessageChatData::uid
    std::vector<std::string> joinedChannels_;
    std::set<std::string> ownedChannels_;
    /// Who sent the message a chat line id names, newest last.
    ///
    /// Every chat event carries a line id as its eleventh argument, and
    /// FrameXML builds it into the player link on the name - so a right-click
    /// on that name hands the id back and expects the client to know which
    /// message it was. This client sent zero for all of them, which is one id
    /// shared by every line ever printed and therefore no id at all.
    ///
    /// Bounded, because a session's chat is unbounded and only recent lines can
    /// be reported anyway.
    std::deque<std::pair<uint32_t, uint64_t>> chatLineSenders_;
    uint32_t nextChatLineId_ = 1;
    std::unordered_map<std::string, std::vector<ChannelMember>> channelRosters_;
    bool chatLogEnabled_ = false;
    bool chatLogInitialized_ = false;
    std::string chatLogPath_;
    std::ofstream chatLogStream_;

    // Track senders we've already auto-replied to (AFK/DND) this session
    // to prevent infinite reply loops. Cleared when AFK/DND is toggled off.
    std::unordered_set<std::string> afkAutoRepliedSenders_;
};

} // namespace game
} // namespace wowee
