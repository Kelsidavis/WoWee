#include "rendering/renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/scene.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/performance_hud.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/skybox.hpp"
#include "rendering/celestial.hpp"
#include "rendering/starfield.hpp"
#include "rendering/clouds.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/weather.hpp"
#include "rendering/lighting_manager.hpp"
#include "rendering/sky_system.hpp"
#include "rendering/swim_effects.hpp"
#include "rendering/mount_dust.hpp"
#include "rendering/charge_effect.hpp"
#include "rendering/levelup_effect.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/world_map.hpp"
#include "rendering/quest_marker_renderer.hpp"
#include "rendering/shader.hpp"
#include "game/game_handler.hpp"
#include "pipeline/m2_loader.hpp"
#include <algorithm>
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/terrain_mesh.hpp"
#include "core/application.hpp"
#include "core/window.hpp"
#include "core/logger.hpp"
#include "game/world.hpp"
#include "game/zone_manager.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_utils.hpp"
#include <imgui.h>
#include <imgui_impl_vulkan.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cctype>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <set>

namespace wowee {
namespace rendering {

struct EmoteInfo {
    uint32_t animId = 0;
    uint32_t dbcId = 0;  // EmotesText.dbc record ID (for CMSG_TEXT_EMOTE)
    bool loop = false;
    std::string textNoTarget;       // sender sees, no target: "You dance."
    std::string textTarget;         // sender sees, with target: "You dance with %s."
    std::string othersNoTarget;     // others see, no target: "%s dances."
    std::string othersTarget;       // others see, with target: "%s dances with %s."
    std::string command;
};

static std::unordered_map<std::string, EmoteInfo> EMOTE_TABLE;
static std::unordered_map<uint32_t, const EmoteInfo*> EMOTE_BY_DBCID; // reverse lookup: dbcId → EmoteInfo*
static bool emoteTableLoaded = false;

static bool envFlagEnabled(const char* key, bool defaultValue) {
    const char* raw = std::getenv(key);
    if (!raw || !*raw) return defaultValue;
    std::string v(raw);
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return !(v == "0" || v == "false" || v == "off" || v == "no");
}

static int envIntOrDefault(const char* key, int defaultValue) {
    const char* raw = std::getenv(key);
    if (!raw || !*raw) return defaultValue;
    char* end = nullptr;
    long n = std::strtol(raw, &end, 10);
    if (end == raw) return defaultValue;
    return static_cast<int>(n);
}

static std::vector<std::string> parseEmoteCommands(const std::string& raw) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : raw) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
            cur.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        } else if (!cur.empty()) {
            out.push_back(cur);
            cur.clear();
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static bool isLoopingEmote(const std::string& command) {
    static const std::unordered_set<std::string> kLooping = {
        "dance",
        "train",
    };
    return kLooping.find(command) != kLooping.end();
}

static void loadFallbackEmotes() {
    if (!EMOTE_TABLE.empty()) return;
    EMOTE_TABLE = {
        {"wave",    {67,  0, false, "You wave.", "You wave at %s.", "%s waves.", "%s waves at %s.", "wave"}},
        {"bow",     {66,  0, false, "You bow down graciously.", "You bow down before %s.", "%s bows down graciously.", "%s bows down before %s.", "bow"}},
        {"laugh",   {70,  0, false, "You laugh.", "You laugh at %s.", "%s laughs.", "%s laughs at %s.", "laugh"}},
        {"point",   {84,  0, false, "You point over yonder.", "You point at %s.", "%s points over yonder.", "%s points at %s.", "point"}},
        {"cheer",   {68,  0, false, "You cheer!", "You cheer at %s.", "%s cheers!", "%s cheers at %s.", "cheer"}},
        {"dance",   {69,  0, true,  "You burst into dance.", "You dance with %s.", "%s bursts into dance.", "%s dances with %s.", "dance"}},
        {"kneel",   {75,  0, false, "You kneel down.", "You kneel before %s.", "%s kneels down.", "%s kneels before %s.", "kneel"}},
        {"applaud", {80,  0, false, "You applaud. Bravo!", "You applaud at %s. Bravo!", "%s applauds. Bravo!", "%s applauds at %s. Bravo!", "applaud"}},
        {"shout",   {81,  0, false, "You shout.", "You shout at %s.", "%s shouts.", "%s shouts at %s.", "shout"}},
        {"chicken", {78,  0, false, "With arms flapping, you strut around. Cluck, Cluck, Chicken!",
                     "With arms flapping, you strut around %s. Cluck, Cluck, Chicken!",
                     "%s struts around. Cluck, Cluck, Chicken!", "%s struts around %s. Cluck, Cluck, Chicken!", "chicken"}},
        {"cry",     {77,  0, false, "You cry.", "You cry on %s's shoulder.", "%s cries.", "%s cries on %s's shoulder.", "cry"}},
        {"kiss",    {76,  0, false, "You blow a kiss into the wind.", "You blow a kiss to %s.", "%s blows a kiss into the wind.", "%s blows a kiss to %s.", "kiss"}},
        {"roar",    {74,  0, false, "You roar with bestial vigor. So fierce!", "You roar with bestial vigor at %s. So fierce!", "%s roars with bestial vigor. So fierce!", "%s roars with bestial vigor at %s. So fierce!", "roar"}},
        {"salute",  {113, 0, false, "You salute.", "You salute %s with respect.", "%s salutes.", "%s salutes %s with respect.", "salute"}},
        {"rude",    {73,  0, false, "You make a rude gesture.", "You make a rude gesture at %s.", "%s makes a rude gesture.", "%s makes a rude gesture at %s.", "rude"}},
        {"flex",    {82,  0, false, "You flex your muscles. Oooooh so strong!", "You flex at %s. Oooooh so strong!", "%s flexes. Oooooh so strong!", "%s flexes at %s. Oooooh so strong!", "flex"}},
        {"shy",     {83,  0, false, "You smile shyly.", "You smile shyly at %s.", "%s smiles shyly.", "%s smiles shyly at %s.", "shy"}},
        {"beg",     {79,  0, false, "You beg everyone around you. How pathetic.", "You beg %s. How pathetic.", "%s begs everyone around. How pathetic.", "%s begs %s. How pathetic.", "beg"}},
        {"eat",     {61,  0, false, "You begin to eat.", "You begin to eat in front of %s.", "%s begins to eat.", "%s begins to eat in front of %s.", "eat"}},
    };
}

static std::string replacePlaceholders(const std::string& text, const std::string* targetName) {
    if (text.empty()) return text;
    std::string out;
    out.reserve(text.size() + 16);
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '%' && i + 1 < text.size() && text[i + 1] == 's') {
            if (targetName && !targetName->empty()) out += *targetName;
            i++;
        } else {
            out.push_back(text[i]);
        }
    }
    return out;
}

static void loadEmotesFromDbc() {
    if (emoteTableLoaded) return;
    emoteTableLoaded = true;

    auto* assetManager = core::Application::getInstance().getAssetManager();
    if (!assetManager) {
        LOG_WARNING("Emotes: no AssetManager");
        loadFallbackEmotes();
        return;
    }

    auto emotesTextDbc = assetManager->loadDBC("EmotesText.dbc");
    auto emotesTextDataDbc = assetManager->loadDBC("EmotesTextData.dbc");
    if (!emotesTextDbc || !emotesTextDataDbc || !emotesTextDbc->isLoaded() || !emotesTextDataDbc->isLoaded()) {
        LOG_WARNING("Emotes: DBCs not available (EmotesText/EmotesTextData)");
        loadFallbackEmotes();
        return;
    }

    const auto* activeLayout = pipeline::getActiveDBCLayout();
    const auto* etdL = activeLayout ? activeLayout->getLayout("EmotesTextData") : nullptr;
    const auto* emL  = activeLayout ? activeLayout->getLayout("Emotes") : nullptr;
    const auto* etL  = activeLayout ? activeLayout->getLayout("EmotesText") : nullptr;

    std::unordered_map<uint32_t, std::string> textData;
    textData.reserve(emotesTextDataDbc->getRecordCount());
    for (uint32_t r = 0; r < emotesTextDataDbc->getRecordCount(); ++r) {
        uint32_t id = emotesTextDataDbc->getUInt32(r, etdL ? (*etdL)["ID"] : 0);
        std::string text = emotesTextDataDbc->getString(r, etdL ? (*etdL)["Text"] : 1);
        if (!text.empty()) textData.emplace(id, std::move(text));
    }

    std::unordered_map<uint32_t, uint32_t> emoteIdToAnim;
    if (auto emotesDbc = assetManager->loadDBC("Emotes.dbc"); emotesDbc && emotesDbc->isLoaded()) {
        emoteIdToAnim.reserve(emotesDbc->getRecordCount());
        for (uint32_t r = 0; r < emotesDbc->getRecordCount(); ++r) {
            uint32_t emoteId = emotesDbc->getUInt32(r, emL ? (*emL)["ID"] : 0);
            uint32_t animId = emotesDbc->getUInt32(r, emL ? (*emL)["AnimID"] : 2);
            if (animId != 0) emoteIdToAnim[emoteId] = animId;
        }
    }

    EMOTE_TABLE.clear();
    EMOTE_TABLE.reserve(emotesTextDbc->getRecordCount());
    for (uint32_t r = 0; r < emotesTextDbc->getRecordCount(); ++r) {
        uint32_t recordId = emotesTextDbc->getUInt32(r, etL ? (*etL)["ID"] : 0);
        std::string cmdRaw = emotesTextDbc->getString(r, etL ? (*etL)["Command"] : 1);
        if (cmdRaw.empty()) continue;

        uint32_t emoteRef = emotesTextDbc->getUInt32(r, etL ? (*etL)["EmoteRef"] : 2);
        uint32_t animId = 0;
        auto animIt = emoteIdToAnim.find(emoteRef);
        if (animIt != emoteIdToAnim.end()) {
            animId = animIt->second;
        } else {
            animId = emoteRef;  // fallback if EmotesText stores animation id directly
        }

        uint32_t senderTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["SenderTargetTextID"] : 5);
        uint32_t senderNoTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["SenderNoTargetTextID"] : 9);
        uint32_t othersTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["OthersTargetTextID"] : 3);
        uint32_t othersNoTargetTextId = emotesTextDbc->getUInt32(r, etL ? (*etL)["OthersNoTargetTextID"] : 7);

        std::string textTarget, textNoTarget, oTarget, oNoTarget;
        if (auto it = textData.find(senderTargetTextId); it != textData.end()) textTarget = it->second;
        if (auto it = textData.find(senderNoTargetTextId); it != textData.end()) textNoTarget = it->second;
        if (auto it = textData.find(othersTargetTextId); it != textData.end()) oTarget = it->second;
        if (auto it = textData.find(othersNoTargetTextId); it != textData.end()) oNoTarget = it->second;

        for (const std::string& cmd : parseEmoteCommands(cmdRaw)) {
            if (cmd.empty()) continue;
            EmoteInfo info;
            info.animId = animId;
            info.dbcId = recordId;
            info.loop = isLoopingEmote(cmd);
            info.textNoTarget = textNoTarget;
            info.textTarget = textTarget;
            info.othersNoTarget = oNoTarget;
            info.othersTarget = oTarget;
            info.command = cmd;
            EMOTE_TABLE.emplace(cmd, std::move(info));
        }
    }

    if (EMOTE_TABLE.empty()) {
        LOG_WARNING("Emotes: DBC loaded but no commands parsed, using fallback list");
        loadFallbackEmotes();
    } else {
        LOG_INFO("Emotes: loaded ", EMOTE_TABLE.size(), " commands from DBC");
    }

    // Build reverse lookup by dbcId (only first command per emote needed)
    EMOTE_BY_DBCID.clear();
    for (auto& [cmd, info] : EMOTE_TABLE) {
        if (info.dbcId != 0) {
            EMOTE_BY_DBCID.emplace(info.dbcId, &info);
        }
    }
}

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

bool Renderer::createPerFrameResources() {
    VkDevice device = vkCtx->getDevice();

    // --- Create shadow depth image ---
    VkImageCreateInfo imgCI{};
    imgCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgCI.imageType = VK_IMAGE_TYPE_2D;
    imgCI.format = VK_FORMAT_D32_SFLOAT;
    imgCI.extent = {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, 1};
    imgCI.mipLevels = 1;
    imgCI.arrayLayers = 1;
    imgCI.samples = VK_SAMPLE_COUNT_1_BIT;
    imgCI.tiling = VK_IMAGE_TILING_OPTIMAL;
    imgCI.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    VmaAllocationCreateInfo imgAllocCI{};
    imgAllocCI.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    if (vmaCreateImage(vkCtx->getAllocator(), &imgCI, &imgAllocCI,
            &shadowDepthImage, &shadowDepthAlloc, nullptr) != VK_SUCCESS) {
        LOG_ERROR("Failed to create shadow depth image");
        return false;
    }
    shadowDepthLayout_ = VK_IMAGE_LAYOUT_UNDEFINED;

    // --- Create shadow depth image view ---
    VkImageViewCreateInfo viewCI{};
    viewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewCI.image = shadowDepthImage;
    viewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewCI.format = VK_FORMAT_D32_SFLOAT;
    viewCI.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
    if (vkCreateImageView(device, &viewCI, nullptr, &shadowDepthView) != VK_SUCCESS) {
        LOG_ERROR("Failed to create shadow depth image view");
        return false;
    }

    // --- Create shadow sampler ---
    VkSamplerCreateInfo sampCI{};
    sampCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampCI.magFilter = VK_FILTER_LINEAR;
    sampCI.minFilter = VK_FILTER_LINEAR;
    sampCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sampCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    sampCI.compareEnable = VK_TRUE;
    sampCI.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    if (vkCreateSampler(device, &sampCI, nullptr, &shadowSampler) != VK_SUCCESS) {
        LOG_ERROR("Failed to create shadow sampler");
        return false;
    }

    // --- Create shadow render pass (depth-only) ---
    VkAttachmentDescription depthAtt{};
    depthAtt.format = VK_FORMAT_D32_SFLOAT;
    depthAtt.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAtt.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAtt.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAtt.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAtt.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAtt.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAtt.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthRef{};
    depthRef.attachment = 0;
    depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
    dep.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo rpCI{};
    rpCI.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpCI.attachmentCount = 1;
    rpCI.pAttachments = &depthAtt;
    rpCI.subpassCount = 1;
    rpCI.pSubpasses = &subpass;
    rpCI.dependencyCount = 1;
    rpCI.pDependencies = &dep;
    if (vkCreateRenderPass(device, &rpCI, nullptr, &shadowRenderPass) != VK_SUCCESS) {
        LOG_ERROR("Failed to create shadow render pass");
        return false;
    }

    // --- Create shadow framebuffer ---
    VkFramebufferCreateInfo fbCI{};
    fbCI.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbCI.renderPass = shadowRenderPass;
    fbCI.attachmentCount = 1;
    fbCI.pAttachments = &shadowDepthView;
    fbCI.width = SHADOW_MAP_SIZE;
    fbCI.height = SHADOW_MAP_SIZE;
    fbCI.layers = 1;
    if (vkCreateFramebuffer(device, &fbCI, nullptr, &shadowFramebuffer) != VK_SUCCESS) {
        LOG_ERROR("Failed to create shadow framebuffer");
        return false;
    }

    // --- Create descriptor set layout for set 0 (per-frame UBO + shadow sampler) ---
    VkDescriptorSetLayoutBinding bindings[2]{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = bindings;

    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &perFrameSetLayout) != VK_SUCCESS) {
        LOG_ERROR("Failed to create per-frame descriptor set layout");
        return false;
    }

    // --- Create descriptor pool for UBO + image sampler (normal frames + reflection) ---
    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = MAX_FRAMES + 1; // +1 for reflection perFrame UBO
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = MAX_FRAMES + 1;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = MAX_FRAMES + 1; // +1 for reflection descriptor set
    poolInfo.poolSizeCount = 2;
    poolInfo.pPoolSizes = poolSizes;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &sceneDescriptorPool) != VK_SUCCESS) {
        LOG_ERROR("Failed to create scene descriptor pool");
        return false;
    }

    // --- Create per-frame UBOs and descriptor sets ---
    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        // Create mapped UBO
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = sizeof(GPUPerFrameData);
        bufInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo mapInfo{};
        if (vmaCreateBuffer(vkCtx->getAllocator(), &bufInfo, &allocInfo,
                &perFrameUBOs[i], &perFrameUBOAllocs[i], &mapInfo) != VK_SUCCESS) {
            LOG_ERROR("Failed to create per-frame UBO ", i);
            return false;
        }
        perFrameUBOMapped[i] = mapInfo.pMappedData;

        // Allocate descriptor set
        VkDescriptorSetAllocateInfo setAlloc{};
        setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        setAlloc.descriptorPool = sceneDescriptorPool;
        setAlloc.descriptorSetCount = 1;
        setAlloc.pSetLayouts = &perFrameSetLayout;

        if (vkAllocateDescriptorSets(device, &setAlloc, &perFrameDescSets[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to allocate per-frame descriptor set ", i);
            return false;
        }

        // Write binding 0 (UBO) and binding 1 (shadow sampler)
        VkDescriptorBufferInfo descBuf{};
        descBuf.buffer = perFrameUBOs[i];
        descBuf.offset = 0;
        descBuf.range = sizeof(GPUPerFrameData);

        VkDescriptorImageInfo shadowImgInfo{};
        shadowImgInfo.sampler = shadowSampler;
        shadowImgInfo.imageView = shadowDepthView;
        shadowImgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = perFrameDescSets[i];
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].pBufferInfo = &descBuf;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = perFrameDescSets[i];
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].pImageInfo = &shadowImgInfo;

        vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
    }

    // --- Create reflection per-frame UBO and descriptor set ---
    {
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = sizeof(GPUPerFrameData);
        bufInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo mapInfo{};
        if (vmaCreateBuffer(vkCtx->getAllocator(), &bufInfo, &allocInfo,
                &reflPerFrameUBO, &reflPerFrameUBOAlloc, &mapInfo) != VK_SUCCESS) {
            LOG_ERROR("Failed to create reflection per-frame UBO");
            return false;
        }
        reflPerFrameUBOMapped = mapInfo.pMappedData;

        VkDescriptorSetAllocateInfo setAlloc{};
        setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        setAlloc.descriptorPool = sceneDescriptorPool;
        setAlloc.descriptorSetCount = 1;
        setAlloc.pSetLayouts = &perFrameSetLayout;

        if (vkAllocateDescriptorSets(device, &setAlloc, &reflPerFrameDescSet) != VK_SUCCESS) {
            LOG_ERROR("Failed to allocate reflection per-frame descriptor set");
            return false;
        }

        VkDescriptorBufferInfo descBuf{};
        descBuf.buffer = reflPerFrameUBO;
        descBuf.offset = 0;
        descBuf.range = sizeof(GPUPerFrameData);

        VkDescriptorImageInfo shadowImgInfo{};
        shadowImgInfo.sampler = shadowSampler;
        shadowImgInfo.imageView = shadowDepthView;
        shadowImgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = reflPerFrameDescSet;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].pBufferInfo = &descBuf;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = reflPerFrameDescSet;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].pImageInfo = &shadowImgInfo;

        vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
    }

    LOG_INFO("Per-frame Vulkan resources created (shadow map ", SHADOW_MAP_SIZE, "x", SHADOW_MAP_SIZE, ")");
    return true;
}

void Renderer::destroyPerFrameResources() {
    if (!vkCtx) return;
    vkDeviceWaitIdle(vkCtx->getDevice());
    VkDevice device = vkCtx->getDevice();

    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        if (perFrameUBOs[i]) {
            vmaDestroyBuffer(vkCtx->getAllocator(), perFrameUBOs[i], perFrameUBOAllocs[i]);
            perFrameUBOs[i] = VK_NULL_HANDLE;
        }
    }
    if (reflPerFrameUBO) {
        vmaDestroyBuffer(vkCtx->getAllocator(), reflPerFrameUBO, reflPerFrameUBOAlloc);
        reflPerFrameUBO = VK_NULL_HANDLE;
        reflPerFrameUBOMapped = nullptr;
    }
    if (sceneDescriptorPool) {
        vkDestroyDescriptorPool(device, sceneDescriptorPool, nullptr);
        sceneDescriptorPool = VK_NULL_HANDLE;
    }
    if (perFrameSetLayout) {
        vkDestroyDescriptorSetLayout(device, perFrameSetLayout, nullptr);
        perFrameSetLayout = VK_NULL_HANDLE;
    }

    // Destroy shadow resources
    if (shadowFramebuffer) { vkDestroyFramebuffer(device, shadowFramebuffer, nullptr); shadowFramebuffer = VK_NULL_HANDLE; }
    if (shadowRenderPass) { vkDestroyRenderPass(device, shadowRenderPass, nullptr); shadowRenderPass = VK_NULL_HANDLE; }
    if (shadowDepthView) { vkDestroyImageView(device, shadowDepthView, nullptr); shadowDepthView = VK_NULL_HANDLE; }
    if (shadowDepthImage) { vmaDestroyImage(vkCtx->getAllocator(), shadowDepthImage, shadowDepthAlloc); shadowDepthImage = VK_NULL_HANDLE; shadowDepthAlloc = VK_NULL_HANDLE; }
    if (shadowSampler) { vkDestroySampler(device, shadowSampler, nullptr); shadowSampler = VK_NULL_HANDLE; }
    shadowDepthLayout_ = VK_IMAGE_LAYOUT_UNDEFINED;
}

void Renderer::updatePerFrameUBO() {
    if (!camera) return;

    currentFrameData.view = camera->getViewMatrix();
    currentFrameData.projection = camera->getProjectionMatrix();
    currentFrameData.viewPos = glm::vec4(camera->getPosition(), 1.0f);
    currentFrameData.fogParams.z = globalTime;

    // Lighting from LightingManager
    if (lightingManager) {
        const auto& lp = lightingManager->getLightingParams();
        currentFrameData.lightDir = glm::vec4(lp.directionalDir, 0.0f);
        currentFrameData.lightColor = glm::vec4(lp.diffuseColor, 1.0f);
        currentFrameData.ambientColor = glm::vec4(lp.ambientColor, 1.0f);
        currentFrameData.fogColor = glm::vec4(lp.fogColor, 1.0f);
        currentFrameData.fogParams.x = lp.fogStart;
        currentFrameData.fogParams.y = lp.fogEnd;

        // Shift fog to blue when camera is significantly underwater (terrain water only).
        if (waterRenderer && camera) {
            glm::vec3 camPos = camera->getPosition();
            auto waterH = waterRenderer->getNearestWaterHeightAt(camPos.x, camPos.y, camPos.z);
            constexpr float MIN_SUBMERSION = 2.0f;
            if (waterH && camPos.z < (*waterH - MIN_SUBMERSION)
                       && !waterRenderer->isWmoWaterAt(camPos.x, camPos.y)) {
                float depth = *waterH - camPos.z - MIN_SUBMERSION;
                float blend = glm::clamp(1.0f - std::exp(-depth * 0.08f), 0.0f, 0.7f);
                glm::vec3 underwaterFog(0.03f, 0.09f, 0.18f);
                glm::vec3 blendedFog = glm::mix(lp.fogColor, underwaterFog, blend);
                currentFrameData.fogColor = glm::vec4(blendedFog, 1.0f);
                currentFrameData.fogParams.x = glm::mix(lp.fogStart, 20.0f, blend);
                currentFrameData.fogParams.y = glm::mix(lp.fogEnd, 200.0f, blend);
            }
        }
    }

    currentFrameData.lightSpaceMatrix = lightSpaceMatrix;
    currentFrameData.shadowParams = glm::vec4(shadowsEnabled ? 1.0f : 0.0f, 0.8f, 0.0f, 0.0f);

    // Player water ripple data: pack player XY into shadowParams.zw, ripple strength into fogParams.w
    if (cameraController) {
        currentFrameData.shadowParams.z = characterPosition.x;
        currentFrameData.shadowParams.w = characterPosition.y;
        bool inWater = cameraController->isSwimming();
        bool moving = cameraController->isMoving();
        currentFrameData.fogParams.w = (inWater && moving) ? 1.0f : 0.0f;
    } else {
        currentFrameData.fogParams.w = 0.0f;
    }

    // Copy to current frame's mapped UBO
    uint32_t frame = vkCtx->getCurrentFrame();
    std::memcpy(perFrameUBOMapped[frame], &currentFrameData, sizeof(GPUPerFrameData));
}

bool Renderer::initialize(core::Window* win) {
    window = win;
    vkCtx = win->getVkContext();
    deferredWorldInitEnabled_ = envFlagEnabled("WOWEE_DEFER_WORLD_SYSTEMS", true);
    LOG_INFO("Initializing renderer (Vulkan)");

    // Create camera (in front of Stormwind gate, looking north)
    camera = std::make_unique<Camera>();
    camera->setPosition(glm::vec3(-8900.0f, -170.0f, 150.0f));
    camera->setRotation(0.0f, -5.0f);
    camera->setAspectRatio(window->getAspectRatio());
    camera->setFov(60.0f);

    // Create camera controller
    cameraController = std::make_unique<CameraController>(camera.get());
    cameraController->setUseWoWSpeed(true);  // Use realistic WoW movement speed
    cameraController->setMouseSensitivity(0.15f);

    // Create scene
    scene = std::make_unique<Scene>();

    // Create performance HUD
    performanceHUD = std::make_unique<PerformanceHUD>();
    performanceHUD->setPosition(PerformanceHUD::Position::TOP_LEFT);

    // Create per-frame UBO and descriptor sets
    if (!createPerFrameResources()) {
        LOG_ERROR("Failed to create per-frame Vulkan resources");
        return false;
    }

    // Initialize Vulkan sub-renderers (Phase 3)

    // Sky system (owns skybox, starfield, celestial, clouds, lens flare)
    skySystem = std::make_unique<SkySystem>();
    if (!skySystem->initialize(vkCtx, perFrameSetLayout)) {
        LOG_ERROR("Failed to initialize sky system");
        return false;
    }
    // Expose sub-components via renderer accessors
    skybox = nullptr;  // Owned by skySystem; access via skySystem->getSkybox()
    celestial = nullptr;
    starField = nullptr;
    clouds = nullptr;
    lensFlare = nullptr;

    weather = std::make_unique<Weather>();
    weather->initialize(vkCtx, perFrameSetLayout);

    swimEffects = std::make_unique<SwimEffects>();
    swimEffects->initialize(vkCtx, perFrameSetLayout);

    mountDust = std::make_unique<MountDust>();
    mountDust->initialize(vkCtx, perFrameSetLayout);

    chargeEffect = std::make_unique<ChargeEffect>();
    chargeEffect->initialize(vkCtx, perFrameSetLayout);

    levelUpEffect = std::make_unique<LevelUpEffect>();

    LOG_INFO("Vulkan sub-renderers initialized (Phase 3)");

    // LightingManager doesn't use GL — initialize for data-only use
    lightingManager = std::make_unique<LightingManager>();
    [[maybe_unused]] auto* assetManager = core::Application::getInstance().getAssetManager();

    // Create zone manager
    zoneManager = std::make_unique<game::ZoneManager>();
    zoneManager->initialize();

    // Initialize AudioEngine (singleton)
    if (!audio::AudioEngine::instance().initialize()) {
        LOG_WARNING("Failed to initialize AudioEngine - audio will be disabled");
    }

    // Create music manager (initialized later with asset manager)
    musicManager = std::make_unique<audio::MusicManager>();
    footstepManager = std::make_unique<audio::FootstepManager>();
    activitySoundManager = std::make_unique<audio::ActivitySoundManager>();
    mountSoundManager = std::make_unique<audio::MountSoundManager>();
    npcVoiceManager = std::make_unique<audio::NpcVoiceManager>();
    ambientSoundManager = std::make_unique<audio::AmbientSoundManager>();
    uiSoundManager = std::make_unique<audio::UiSoundManager>();
    combatSoundManager = std::make_unique<audio::CombatSoundManager>();
    spellSoundManager = std::make_unique<audio::SpellSoundManager>();
    movementSoundManager = std::make_unique<audio::MovementSoundManager>();

    // TODO Phase 6: Vulkan underwater overlay, post-process, and shadow map
    // GL versions stubbed during migration

    LOG_INFO("Renderer initialized");
    return true;
}

void Renderer::shutdown() {
    if (terrainManager) {
        terrainManager->unloadAll();
        terrainManager.reset();
    }

    if (terrainRenderer) {
        terrainRenderer->shutdown();
        terrainRenderer.reset();
    }

    if (waterRenderer) {
        waterRenderer->shutdown();
        waterRenderer.reset();
    }

    if (minimap) {
        minimap->shutdown();
        minimap.reset();
    }

    if (worldMap) {
        worldMap->shutdown();
        worldMap.reset();
    }

    if (skySystem) {
        skySystem->shutdown();
        skySystem.reset();
    }

    // Individual sky components are owned by skySystem; just null the aliases
    skybox = nullptr;
    celestial = nullptr;
    starField = nullptr;
    clouds = nullptr;
    lensFlare = nullptr;

    if (weather) {
        weather.reset();
    }

    if (swimEffects) {
        swimEffects->shutdown();
        swimEffects.reset();
    }

    if (characterRenderer) {
        characterRenderer->shutdown();
        characterRenderer.reset();
    }

    if (wmoRenderer) {
        wmoRenderer->shutdown();
        wmoRenderer.reset();
    }

    if (m2Renderer) {
        m2Renderer->shutdown();
        m2Renderer.reset();
    }

    if (musicManager) {
        musicManager->shutdown();
        musicManager.reset();
    }
    if (footstepManager) {
        footstepManager->shutdown();
        footstepManager.reset();
    }
    if (activitySoundManager) {
        activitySoundManager->shutdown();
        activitySoundManager.reset();
    }

    // Shutdown AudioEngine singleton
    audio::AudioEngine::instance().shutdown();

    // Cleanup Vulkan selection circle resources
    if (vkCtx) {
        VkDevice device = vkCtx->getDevice();
        if (selCirclePipeline) { vkDestroyPipeline(device, selCirclePipeline, nullptr); selCirclePipeline = VK_NULL_HANDLE; }
        if (selCirclePipelineLayout) { vkDestroyPipelineLayout(device, selCirclePipelineLayout, nullptr); selCirclePipelineLayout = VK_NULL_HANDLE; }
        if (selCircleVertBuf) { vmaDestroyBuffer(vkCtx->getAllocator(), selCircleVertBuf, selCircleVertAlloc); selCircleVertBuf = VK_NULL_HANDLE; selCircleVertAlloc = VK_NULL_HANDLE; }
        if (selCircleIdxBuf) { vmaDestroyBuffer(vkCtx->getAllocator(), selCircleIdxBuf, selCircleIdxAlloc); selCircleIdxBuf = VK_NULL_HANDLE; selCircleIdxAlloc = VK_NULL_HANDLE; }
        if (overlayPipeline) { vkDestroyPipeline(device, overlayPipeline, nullptr); overlayPipeline = VK_NULL_HANDLE; }
        if (overlayPipelineLayout) { vkDestroyPipelineLayout(device, overlayPipelineLayout, nullptr); overlayPipelineLayout = VK_NULL_HANDLE; }
    }

    destroyPerFrameResources();

    zoneManager.reset();

    performanceHUD.reset();
    scene.reset();
    cameraController.reset();
    camera.reset();

    LOG_INFO("Renderer shutdown");
}

void Renderer::registerPreview(CharacterPreview* preview) {
    if (!preview) return;
    auto it = std::find(activePreviews_.begin(), activePreviews_.end(), preview);
    if (it == activePreviews_.end()) {
        activePreviews_.push_back(preview);
    }
}

void Renderer::unregisterPreview(CharacterPreview* preview) {
    auto it = std::find(activePreviews_.begin(), activePreviews_.end(), preview);
    if (it != activePreviews_.end()) {
        activePreviews_.erase(it);
    }
}

void Renderer::setMsaaSamples(VkSampleCountFlagBits samples) {
    if (!vkCtx) return;

    // Clamp to device maximum
    VkSampleCountFlagBits maxSamples = vkCtx->getMaxUsableSampleCount();
    if (samples > maxSamples) samples = maxSamples;

    if (samples == vkCtx->getMsaaSamples()) return;

    // Defer to between frames — cannot destroy render pass/framebuffers mid-frame
    pendingMsaaSamples_ = samples;
    msaaChangePending_ = true;
}

void Renderer::applyMsaaChange() {
    VkSampleCountFlagBits samples = pendingMsaaSamples_;
    msaaChangePending_ = false;

    VkSampleCountFlagBits current = vkCtx->getMsaaSamples();
    if (samples == current) return;

    LOG_INFO("Changing MSAA from ", static_cast<int>(current), "x to ", static_cast<int>(samples), "x");

    // Single GPU wait — all subsequent operations are CPU-side object creation
    vkDeviceWaitIdle(vkCtx->getDevice());

    // Set new MSAA and recreate swapchain (render pass, depth, MSAA image, framebuffers)
    vkCtx->setMsaaSamples(samples);
    if (!vkCtx->recreateSwapchain(window->getWidth(), window->getHeight())) {
        LOG_ERROR("MSAA change failed — reverting to 1x");
        vkCtx->setMsaaSamples(VK_SAMPLE_COUNT_1_BIT);
        vkCtx->recreateSwapchain(window->getWidth(), window->getHeight());
    }

    // Recreate all sub-renderer pipelines (they embed sample count from render pass)
    if (terrainRenderer) terrainRenderer->recreatePipelines();
    if (waterRenderer) {
        waterRenderer->recreatePipelines();
        if (vkCtx->getMsaaSamples() != VK_SAMPLE_COUNT_1_BIT) {
            waterRenderer->destroyWater1xResources();
            setupWater1xPass();
        } else {
            waterRenderer->destroyWater1xResources();
        }
    }
    if (wmoRenderer) wmoRenderer->recreatePipelines();
    if (m2Renderer) m2Renderer->recreatePipelines();
    if (characterRenderer) characterRenderer->recreatePipelines();
    if (questMarkerRenderer) questMarkerRenderer->recreatePipelines();
    if (weather) weather->recreatePipelines();
    if (swimEffects) swimEffects->recreatePipelines();
    if (mountDust) mountDust->recreatePipelines();
    if (chargeEffect) chargeEffect->recreatePipelines();

    // Sky system sub-renderers
    if (skySystem) {
        if (auto* sb = skySystem->getSkybox()) sb->recreatePipelines();
        if (auto* sf = skySystem->getStarField()) sf->recreatePipelines();
        if (auto* ce = skySystem->getCelestial()) ce->recreatePipelines();
        if (auto* cl = skySystem->getClouds()) cl->recreatePipelines();
        if (auto* lf = skySystem->getLensFlare()) lf->recreatePipelines();
    }

    if (minimap) minimap->recreatePipelines();

    // Selection circle + overlay use lazy init, just destroy them
    VkDevice device = vkCtx->getDevice();
    if (selCirclePipeline) { vkDestroyPipeline(device, selCirclePipeline, nullptr); selCirclePipeline = VK_NULL_HANDLE; }
    if (overlayPipeline) { vkDestroyPipeline(device, overlayPipeline, nullptr); overlayPipeline = VK_NULL_HANDLE; }

    // Reinitialize ImGui Vulkan backend with new MSAA sample count
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.ApiVersion = VK_API_VERSION_1_1;
    initInfo.Instance = vkCtx->getInstance();
    initInfo.PhysicalDevice = vkCtx->getPhysicalDevice();
    initInfo.Device = vkCtx->getDevice();
    initInfo.QueueFamily = vkCtx->getGraphicsQueueFamily();
    initInfo.Queue = vkCtx->getGraphicsQueue();
    initInfo.DescriptorPool = vkCtx->getImGuiDescriptorPool();
    initInfo.MinImageCount = 2;
    initInfo.ImageCount = vkCtx->getSwapchainImageCount();
    initInfo.PipelineInfoMain.RenderPass = vkCtx->getImGuiRenderPass();
    initInfo.PipelineInfoMain.MSAASamples = vkCtx->getMsaaSamples();
    ImGui_ImplVulkan_Init(&initInfo);

    LOG_INFO("MSAA change complete");
}

void Renderer::beginFrame() {
    if (!vkCtx) return;

    // Apply deferred MSAA change between frames (before any rendering state is used)
    if (msaaChangePending_) {
        applyMsaaChange();
    }

    // Handle swapchain recreation if needed
    if (vkCtx->isSwapchainDirty()) {
        vkCtx->recreateSwapchain(window->getWidth(), window->getHeight());
        // Rebuild water resources that reference swapchain extent/views
        if (waterRenderer) {
            waterRenderer->recreatePipelines();
            if (waterRenderer->hasWater1xPass()
                && vkCtx->getMsaaSamples() != VK_SAMPLE_COUNT_1_BIT) {
                waterRenderer->destroyWater1xResources();
                setupWater1xPass();
            }
        }
    }

    // Acquire swapchain image and begin command buffer
    currentCmd = vkCtx->beginFrame(currentImageIndex);
    if (currentCmd == VK_NULL_HANDLE) {
        // Swapchain out of date, will retry next frame
        return;
    }

    // Update per-frame UBO with current camera/lighting state
    updatePerFrameUBO();

    // --- Off-screen pre-passes (before main render pass) ---
    // Minimap composite (renders 3x3 tile grid into 768x768 render target)
    if (minimap && minimap->isEnabled() && camera) {
        glm::vec3 minimapCenter = camera->getPosition();
        if (cameraController && cameraController->isThirdPerson())
            minimapCenter = characterPosition;
        minimap->compositePass(currentCmd, minimapCenter);
    }
    // World map composite (renders zone tiles into 1024x768 render target)
    if (worldMap) {
        worldMap->compositePass(currentCmd);
    }

    // Character preview composite passes
    for (auto* preview : activePreviews_) {
        if (preview && preview->isModelLoaded()) {
            preview->compositePass(currentCmd, vkCtx->getCurrentFrame());
        }
    }

    // Shadow pre-pass (before main render pass)
    if (shadowsEnabled && shadowDepthImage != VK_NULL_HANDLE) {
        renderShadowPass();
    }

    // Water reflection pre-pass (renders scene from mirrored camera into 512x512 texture)
    renderReflectionPass();

    // --- Begin main render pass (clear color + depth) ---
    VkRenderPassBeginInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpInfo.renderPass = vkCtx->getImGuiRenderPass();
    rpInfo.framebuffer = vkCtx->getSwapchainFramebuffers()[currentImageIndex];
    rpInfo.renderArea.offset = {0, 0};
    rpInfo.renderArea.extent = vkCtx->getSwapchainExtent();

    // MSAA render pass has 3 attachments (color, depth, resolve), non-MSAA has 2
    VkClearValue clearValues[3]{};
    clearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    clearValues[1].depthStencil = {1.0f, 0};
    clearValues[2].color = {{0.0f, 0.0f, 0.0f, 1.0f}};  // resolve (DONT_CARE, but count must match)
    bool msaaOn = (vkCtx->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT);
    rpInfo.clearValueCount = msaaOn ? 3 : 2;
    rpInfo.pClearValues = clearValues;

    vkCmdBeginRenderPass(currentCmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

    // Set dynamic viewport and scissor
    VkExtent2D extent = vkCtx->getSwapchainExtent();
    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(currentCmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = extent;
    vkCmdSetScissor(currentCmd, 0, 1, &scissor);
}

void Renderer::endFrame() {
    if (!vkCtx || currentCmd == VK_NULL_HANDLE) return;

    // ImGui always renders in the main pass (its pipeline matches the main render pass)
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), currentCmd);

    vkCmdEndRenderPass(currentCmd);

    if (waterRenderer && currentImageIndex < vkCtx->getSwapchainImages().size()) {
        waterRenderer->captureSceneHistory(
            currentCmd,
            vkCtx->getSwapchainImages()[currentImageIndex],
            vkCtx->getDepthCopySourceImage(),
            vkCtx->getSwapchainExtent(),
            vkCtx->isDepthCopySourceMsaa());
    }

    // Render water in separate 1x pass after MSAA resolve + scene capture
    bool waterDeferred = waterRenderer && waterRenderer->hasWater1xPass()
                         && vkCtx->getMsaaSamples() != VK_SAMPLE_COUNT_1_BIT;
    if (waterDeferred && camera) {
        VkExtent2D ext = vkCtx->getSwapchainExtent();
        uint32_t frame = vkCtx->getCurrentFrame();
        if (waterRenderer->beginWater1xPass(currentCmd, currentImageIndex, ext)) {
            waterRenderer->render(currentCmd, perFrameDescSets[frame], *camera, globalTime, true);
            waterRenderer->endWater1xPass(currentCmd);
        }
    }

    // Submit and present
    vkCtx->endFrame(currentCmd, currentImageIndex);
    currentCmd = VK_NULL_HANDLE;
}

void Renderer::setCharacterFollow(uint32_t instanceId) {
    characterInstanceId = instanceId;
    if (cameraController && instanceId > 0) {
        cameraController->setFollowTarget(&characterPosition);
    }
}

void Renderer::setMounted(uint32_t mountInstId, uint32_t mountDisplayId, float heightOffset, const std::string& modelPath) {
    mountInstanceId_ = mountInstId;
    mountHeightOffset_ = heightOffset;
    mountSeatAttachmentId_ = -1;
    smoothedMountSeatPos_ = characterPosition;
    mountSeatSmoothingInit_ = false;
    mountAction_ = MountAction::None;  // Clear mount action state
    mountActionPhase_ = 0;
    charAnimState = CharAnimState::MOUNT;
    if (cameraController) {
        cameraController->setMounted(true);
        cameraController->setMountHeightOffset(heightOffset);
    }

    // Debug: dump available mount animations
    if (characterRenderer && mountInstId > 0) {
        characterRenderer->dumpAnimations(mountInstId);
    }

    // Discover mount animation capabilities (property-based, not hardcoded IDs)
    LOG_DEBUG("=== Mount Animation Dump (Display ID ", mountDisplayId, ") ===");
    characterRenderer->dumpAnimations(mountInstId);

    // Get all sequences for property-based analysis
    std::vector<pipeline::M2Sequence> sequences;
    if (!characterRenderer->getAnimationSequences(mountInstId, sequences)) {
        LOG_WARNING("Failed to get animation sequences for mount, using fallback IDs");
        sequences.clear();
    }

    // Helper: ID-based fallback finder
    auto findFirst = [&](std::initializer_list<uint32_t> candidates) -> uint32_t {
        for (uint32_t id : candidates) {
            if (characterRenderer->hasAnimation(mountInstId, id)) {
                return id;
            }
        }
        return 0;
    };

    // Property-based jump animation discovery with chain-based scoring
    auto discoverJumpSet = [&]() {
        // Debug: log all sequences for analysis
        LOG_DEBUG("=== Full sequence table for mount ===");
        for (const auto& seq : sequences) {
            LOG_DEBUG("SEQ id=", seq.id,
                     " dur=", seq.duration,
                     " flags=0x", std::hex, seq.flags, std::dec,
                     " moveSpd=", seq.movingSpeed,
                     " blend=", seq.blendTime,
                     " next=", seq.nextAnimation,
                     " alias=", seq.aliasNext);
        }
        LOG_DEBUG("=== End sequence table ===");

        // Known combat/bad animation IDs to avoid
        std::set<uint32_t> forbiddenIds = {53, 54, 16};  // jumpkick, attack

        auto scoreNear = [](int a, int b) -> int {
            int d = std::abs(a - b);
            return (d <= 8) ? (20 - d) : 0; // within 8 IDs gets points
        };

        auto isForbidden = [&](uint32_t id) {
            return forbiddenIds.count(id) != 0;
        };

        auto findSeqById = [&](uint32_t id) -> const pipeline::M2Sequence* {
            for (const auto& s : sequences) {
                if (s.id == id) return &s;
            }
            return nullptr;
        };

        uint32_t runId = findFirst({5, 4});
        uint32_t standId = findFirst({0});

        // Step A: Find loop candidates
        std::vector<uint32_t> loops;
        for (const auto& seq : sequences) {
            if (isForbidden(seq.id)) continue;
            // Bit 0x01 NOT set = loops (0x20, 0x60), bit 0x01 set = non-looping (0x21, 0x61)
            bool isLoop = (seq.flags & 0x01) == 0;
            if (isLoop && seq.duration >= 350 && seq.duration <= 1000 &&
                seq.id != runId && seq.id != standId) {
                loops.push_back(seq.id);
            }
        }

        // Choose loop: prefer one near known classic IDs (38), else best duration
        uint32_t loop = 0;
        if (!loops.empty()) {
            uint32_t best = loops[0];
            int bestScore = -999;
            for (uint32_t id : loops) {
                int sc = 0;
                sc += scoreNear((int)id, 38);  // classic hint
                const auto* s = findSeqById(id);
                if (s) sc += (s->duration >= 500 && s->duration <= 800) ? 5 : 0;
                if (sc > bestScore) {
                    bestScore = sc;
                    best = id;
                }
            }
            loop = best;
        }

        // Step B: Score start/end candidates
        uint32_t start = 0, end = 0;
        int bestStart = -999, bestEnd = -999;

        for (const auto& seq : sequences) {
            if (isForbidden(seq.id)) continue;
            // Only consider non-looping animations for start/end
            bool isLoop = (seq.flags & 0x01) == 0;
            if (isLoop) continue;

            // Start window
            if (seq.duration >= 450 && seq.duration <= 1100) {
                int sc = 0;
                if (loop) sc += scoreNear((int)seq.id, (int)loop);
                // Chain bonus: if this start points at loop or near it
                if (loop && (seq.nextAnimation == (int16_t)loop || seq.aliasNext == loop)) sc += 30;
                if (loop && scoreNear(seq.nextAnimation, (int)loop) > 0) sc += 10;
                // Penalize "stop/brake-ish": very long blendTime can be a stop transition
                if (seq.blendTime > 400) sc -= 5;

                if (sc > bestStart) {
                    bestStart = sc;
                    start = seq.id;
                }
            }

            // End window
            if (seq.duration >= 650 && seq.duration <= 1600) {
                int sc = 0;
                if (loop) sc += scoreNear((int)seq.id, (int)loop);
                // Chain bonus: end often points to run/stand or has no next
                if (seq.nextAnimation == (int16_t)runId || seq.nextAnimation == (int16_t)standId) sc += 10;
                if (seq.nextAnimation < 0) sc += 5; // no chain sometimes = terminal
                if (sc > bestEnd) {
                    bestEnd = sc;
                    end = seq.id;
                }
            }
        }

        LOG_DEBUG("Property-based jump discovery: start=", start, " loop=", loop, " end=", end,
                 " scores: start=", bestStart, " end=", bestEnd);
        return std::make_tuple(start, loop, end);
    };

    auto [discoveredStart, discoveredLoop, discoveredEnd] = discoverJumpSet();

    // Use discovered animations, fallback to known IDs if discovery fails
    mountAnims_.jumpStart = discoveredStart > 0 ? discoveredStart : findFirst({40, 37});
    mountAnims_.jumpLoop  = discoveredLoop > 0 ? discoveredLoop : findFirst({38});
    mountAnims_.jumpEnd   = discoveredEnd > 0 ? discoveredEnd : findFirst({39});
    mountAnims_.rearUp    = findFirst({94, 92, 40}); // RearUp/Special
    mountAnims_.run       = findFirst({5, 4});       // Run/Walk
    mountAnims_.stand     = findFirst({0});          // Stand (almost always 0)

    // Discover idle fidget animations using proper WoW M2 metadata (frequency, replay timers)
    mountAnims_.fidgets.clear();
    core::Logger::getInstance().debug("Scanning for fidget animations in ", sequences.size(), " sequences");

    // DEBUG: Log ALL non-looping, short, stationary animations to identify stamps/tosses
    core::Logger::getInstance().debug("=== ALL potential fidgets (no metadata filter) ===");
    for (const auto& seq : sequences) {
        bool isLoop = (seq.flags & 0x01) == 0;
        bool isStationary = std::abs(seq.movingSpeed) < 0.05f;
        bool reasonableDuration = seq.duration >= 400 && seq.duration <= 2500;

        if (!isLoop && reasonableDuration && isStationary) {
            core::Logger::getInstance().debug("  ALL: id=", seq.id,
                " dur=", seq.duration, "ms",
                " freq=", seq.frequency,
                " replay=", seq.replayMin, "-", seq.replayMax,
                " flags=0x", std::hex, seq.flags, std::dec,
                " next=", seq.nextAnimation);
        }
    }

    // Proper fidget discovery: frequency > 0 + replay timers indicate random idle animations
    for (const auto& seq : sequences) {
        bool isLoop = (seq.flags & 0x01) == 0;
        bool hasFrequency = seq.frequency > 0;
        bool hasReplay = seq.replayMax > 0;
        bool isStationary = std::abs(seq.movingSpeed) < 0.05f;
        bool reasonableDuration = seq.duration >= 400 && seq.duration <= 2500;

        // Log candidates with metadata
        if (!isLoop && reasonableDuration && isStationary && (hasFrequency || hasReplay)) {
            core::Logger::getInstance().debug("  Candidate: id=", seq.id,
                " dur=", seq.duration, "ms",
                " freq=", seq.frequency,
                " replay=", seq.replayMin, "-", seq.replayMax,
                " next=", seq.nextAnimation,
                " speed=", seq.movingSpeed);
        }

        // Exclude known problematic animations: death (5-6), wounds (7-9), combat (16-21), attacks (11-15)
        bool isDeathOrWound = (seq.id >= 5 && seq.id <= 9);
        bool isAttackOrCombat = (seq.id >= 11 && seq.id <= 21);
        bool isSpecial = (seq.id == 2 || seq.id == 3);  // Often aggressive specials

        // Select fidgets: (frequency OR replay) + exclude problematic ID ranges
        // Relaxed back to OR since some mounts may only have one metadata marker
        if (!isLoop && (hasFrequency || hasReplay) && isStationary && reasonableDuration &&
            !isDeathOrWound && !isAttackOrCombat && !isSpecial) {
            // Bonus: chains back to stand (indicates idle behavior)
            bool chainsToStand = (seq.nextAnimation == (int16_t)mountAnims_.stand) ||
                                 (seq.aliasNext == mountAnims_.stand) ||
                                 (seq.nextAnimation == -1);

            mountAnims_.fidgets.push_back(seq.id);
            core::Logger::getInstance().debug("  >> Selected fidget: id=", seq.id,
                (chainsToStand ? " (chains to stand)" : ""));
        }
    }

    // Ensure we have fallbacks for movement
    if (mountAnims_.stand == 0) mountAnims_.stand = 0;  // Force 0 even if not found
    if (mountAnims_.run == 0) mountAnims_.run = mountAnims_.stand;  // Fallback to stand if no run

    core::Logger::getInstance().debug("Mount animation set: jumpStart=", mountAnims_.jumpStart,
        " jumpLoop=", mountAnims_.jumpLoop,
        " jumpEnd=", mountAnims_.jumpEnd,
        " rearUp=", mountAnims_.rearUp,
        " run=", mountAnims_.run,
        " stand=", mountAnims_.stand,
        " fidgets=", mountAnims_.fidgets.size());

    // Notify mount sound manager
    if (mountSoundManager) {
        bool isFlying = taxiFlight_;  // Taxi flights are flying mounts
        mountSoundManager->onMount(mountDisplayId, isFlying, modelPath);
    }
}

void Renderer::clearMount() {
    mountInstanceId_ = 0;
    mountHeightOffset_ = 0.0f;
    mountPitch_ = 0.0f;
    mountRoll_ = 0.0f;
    mountSeatAttachmentId_ = -1;
    smoothedMountSeatPos_ = glm::vec3(0.0f);
    mountSeatSmoothingInit_ = false;
    mountAction_ = MountAction::None;
    mountActionPhase_ = 0;
    charAnimState = CharAnimState::IDLE;
    if (cameraController) {
        cameraController->setMounted(false);
        cameraController->setMountHeightOffset(0.0f);
    }

    // Notify mount sound manager
    if (mountSoundManager) {
        mountSoundManager->onDismount();
    }
}

uint32_t Renderer::resolveMeleeAnimId() {
    if (!characterRenderer || characterInstanceId == 0) {
        meleeAnimId = 0;
        meleeAnimDurationMs = 0.0f;
        return 0;
    }

    if (meleeAnimId != 0 && characterRenderer->hasAnimation(characterInstanceId, meleeAnimId)) {
        return meleeAnimId;
    }

    std::vector<pipeline::M2Sequence> sequences;
    if (!characterRenderer->getAnimationSequences(characterInstanceId, sequences)) {
        meleeAnimId = 0;
        meleeAnimDurationMs = 0.0f;
        return 0;
    }

    auto findDuration = [&](uint32_t id) -> float {
        for (const auto& seq : sequences) {
            if (seq.id == id && seq.duration > 0) {
                return static_cast<float>(seq.duration);
            }
        }
        return 0.0f;
    };

    // Select animation priority based on equipped weapon type
    // WoW inventory types: 17 = 2H weapon, 13/21 = 1H, 0 = unarmed
    // WoW anim IDs: 16 = unarmed, 17 = 1H attack, 18 = 2H attack
    const uint32_t* attackCandidates;
    size_t candidateCount;
    static const uint32_t candidates2H[] = {18, 17, 16, 19, 20, 21};
    static const uint32_t candidates1H[] = {17, 18, 16, 19, 20, 21};
    static const uint32_t candidatesUnarmed[] = {16, 17, 18, 19, 20, 21};
    if (equippedWeaponInvType_ == 17) { // INVTYPE_2HWEAPON
        attackCandidates = candidates2H;
        candidateCount = 6;
    } else if (equippedWeaponInvType_ == 0) {
        attackCandidates = candidatesUnarmed;
        candidateCount = 6;
    } else {
        attackCandidates = candidates1H;
        candidateCount = 6;
    }
    for (size_t ci = 0; ci < candidateCount; ci++) {
        uint32_t id = attackCandidates[ci];
        if (characterRenderer->hasAnimation(characterInstanceId, id)) {
            meleeAnimId = id;
            meleeAnimDurationMs = findDuration(id);
            return meleeAnimId;
        }
    }

    const uint32_t avoidIds[] = {0, 1, 4, 5, 11, 12, 13, 37, 38, 39, 41, 42, 97};
    auto isAvoid = [&](uint32_t id) -> bool {
        for (uint32_t avoid : avoidIds) {
            if (id == avoid) return true;
        }
        return false;
    };

    uint32_t bestId = 0;
    uint32_t bestDuration = 0;
    for (const auto& seq : sequences) {
        if (seq.duration == 0) continue;
        if (isAvoid(seq.id)) continue;
        if (seq.movingSpeed > 0.1f) continue;
        if (seq.duration < 150 || seq.duration > 2000) continue;
        if (bestId == 0 || seq.duration < bestDuration) {
            bestId = seq.id;
            bestDuration = seq.duration;
        }
    }

    if (bestId == 0) {
        for (const auto& seq : sequences) {
            if (seq.duration == 0) continue;
            if (isAvoid(seq.id)) continue;
            if (bestId == 0 || seq.duration < bestDuration) {
                bestId = seq.id;
                bestDuration = seq.duration;
            }
        }
    }

    meleeAnimId = bestId;
    meleeAnimDurationMs = static_cast<float>(bestDuration);
    return meleeAnimId;
}

void Renderer::updateCharacterAnimation() {
    // WoW WotLK AnimationData.dbc IDs
    constexpr uint32_t ANIM_STAND      = 0;
    constexpr uint32_t ANIM_WALK       = 4;
    constexpr uint32_t ANIM_RUN        = 5;
    // Candidate locomotion clips by common WotLK IDs.
    constexpr uint32_t ANIM_STRAFE_RUN_RIGHT  = 92;
    constexpr uint32_t ANIM_STRAFE_RUN_LEFT   = 93;
    constexpr uint32_t ANIM_STRAFE_WALK_LEFT  = 11;
    constexpr uint32_t ANIM_STRAFE_WALK_RIGHT = 12;
    constexpr uint32_t ANIM_BACKPEDAL         = 13;
    constexpr uint32_t ANIM_JUMP_START = 37;
    constexpr uint32_t ANIM_JUMP_MID   = 38;
    constexpr uint32_t ANIM_JUMP_END   = 39;
    constexpr uint32_t ANIM_SIT_DOWN   = 97;  // SitGround — transition to sitting
    constexpr uint32_t ANIM_SITTING    = 97;  // Hold on same animation (no separate idle)
    constexpr uint32_t ANIM_SWIM_IDLE  = 41;  // Treading water (SwimIdle)
    constexpr uint32_t ANIM_SWIM       = 42;  // Swimming forward (Swim)
    constexpr uint32_t ANIM_MOUNT      = 91;  // Seated on mount
    // Canonical player ready stances (AnimationData.dbc)
    constexpr uint32_t ANIM_READY_UNARMED = 22;  // ReadyUnarmed
    constexpr uint32_t ANIM_READY_1H      = 23;  // Ready1H
    constexpr uint32_t ANIM_READY_2H      = 24;  // Ready2H
    constexpr uint32_t ANIM_READY_2H_L    = 25;  // Ready2HL (some 2H left-handed rigs)
    constexpr uint32_t ANIM_FLY_IDLE   = 158; // Flying mount idle/hover
    constexpr uint32_t ANIM_FLY_FORWARD = 159; // Flying mount forward

    CharAnimState newState = charAnimState;

    bool moving = cameraController->isMoving();
    bool movingForward = cameraController->isMovingForward();
    bool movingBackward = cameraController->isMovingBackward();
    bool autoRunning = cameraController->isAutoRunning();
    bool strafeLeft = cameraController->isStrafingLeft();
    bool strafeRight = cameraController->isStrafingRight();
    // Strafe animation only plays during *pure* strafing (no forward/backward/autorun).
    // When forward+strafe are both held, the walk/run animation plays — same as the real client.
    bool pureStrafe = !movingForward && !movingBackward && !autoRunning;
    bool anyStrafeLeft = strafeLeft && !strafeRight && pureStrafe;
    bool anyStrafeRight = strafeRight && !strafeLeft && pureStrafe;
    bool grounded = cameraController->isGrounded();
    bool jumping = cameraController->isJumping();
    bool sprinting = cameraController->isSprinting();
    bool sitting = cameraController->isSitting();
    bool swim = cameraController->isSwimming();
    bool forceMelee = meleeSwingTimer > 0.0f && grounded && !swim;

    // When mounted, force MOUNT state and skip normal transitions
    if (isMounted()) {
        newState = CharAnimState::MOUNT;
        charAnimState = newState;

        // Play seated animation on player
        uint32_t currentAnimId = 0;
        float currentAnimTimeMs = 0.0f, currentAnimDurationMs = 0.0f;
        bool haveState = characterRenderer->getAnimationState(characterInstanceId, currentAnimId, currentAnimTimeMs, currentAnimDurationMs);
        if (!haveState || currentAnimId != ANIM_MOUNT) {
            characterRenderer->playAnimation(characterInstanceId, ANIM_MOUNT, true);
        }

        // Sync mount instance position and rotation
        float mountBob = 0.0f;
        float mountYawRad = glm::radians(characterYaw);
        if (mountInstanceId_ > 0) {
            characterRenderer->setInstancePosition(mountInstanceId_, characterPosition);

            // Procedural lean into turns (ground mounts only, optional enhancement)
            if (!taxiFlight_ && moving && lastDeltaTime_ > 0.0f) {
                float currentYawDeg = characterYaw;
                float turnRate = (currentYawDeg - prevMountYaw_) / lastDeltaTime_;
                // Normalize to [-180, 180] for wrap-around
                while (turnRate > 180.0f) turnRate -= 360.0f;
                while (turnRate < -180.0f) turnRate += 360.0f;

                float targetLean = glm::clamp(turnRate * 0.15f, -0.25f, 0.25f);
                mountRoll_ = glm::mix(mountRoll_, targetLean, lastDeltaTime_ * 6.0f);
                prevMountYaw_ = currentYawDeg;
            } else {
                // Return to upright when not turning
                mountRoll_ = glm::mix(mountRoll_, 0.0f, lastDeltaTime_ * 8.0f);
            }

            // Apply pitch (up/down), roll (banking), and yaw for realistic flight
            characterRenderer->setInstanceRotation(mountInstanceId_, glm::vec3(mountPitch_, mountRoll_, mountYawRad));

            // Drive mount model animation: idle when still, run when moving
            auto pickMountAnim = [&](std::initializer_list<uint32_t> candidates, uint32_t fallback) -> uint32_t {
                for (uint32_t id : candidates) {
                    if (characterRenderer->hasAnimation(mountInstanceId_, id)) {
                        return id;
                    }
                }
                return fallback;
            };

            uint32_t mountAnimId = ANIM_STAND;

            // Get current mount animation state (used throughout)
            uint32_t curMountAnim = 0;
            float curMountTime = 0, curMountDur = 0;
            bool haveMountState = characterRenderer->getAnimationState(mountInstanceId_, curMountAnim, curMountTime, curMountDur);

            // Taxi flight: use flying animations instead of ground movement
            if (taxiFlight_) {
                // Log available animations once when taxi starts
                if (!taxiAnimsLogged_) {
                    taxiAnimsLogged_ = true;
                    LOG_INFO("Taxi flight active: mountInstanceId_=", mountInstanceId_,
                             " curMountAnim=", curMountAnim, " haveMountState=", haveMountState);
                    std::vector<pipeline::M2Sequence> seqs;
                    if (characterRenderer->getAnimationSequences(mountInstanceId_, seqs)) {
                        std::string animList;
                        for (const auto& s : seqs) {
                            if (!animList.empty()) animList += ", ";
                            animList += std::to_string(s.id);
                        }
                        LOG_INFO("Taxi mount available animations: [", animList, "]");
                    }
                }

                // Try multiple flying animation IDs in priority order:
                // 159=FlyForward, 158=FlyIdle (WotLK flying mounts)
                // 234=FlyRun, 229=FlyStand (Vanilla creature fly anims)
                // 233=FlyWalk, 141=FlyMounted, 369=FlyRun (alternate IDs)
                // 6=Fly (classic creature fly)
                // Fallback: Run, then Stand (hover)
                uint32_t flyAnims[] = {ANIM_FLY_FORWARD, ANIM_FLY_IDLE, 234, 229, 233, 141, 369, 6, ANIM_RUN};
                mountAnimId = ANIM_STAND; // ultimate fallback: hover/idle
                for (uint32_t fa : flyAnims) {
                    if (characterRenderer->hasAnimation(mountInstanceId_, fa)) {
                        mountAnimId = fa;
                        break;
                    }
                }

                if (!haveMountState || curMountAnim != mountAnimId) {
                    LOG_INFO("Taxi mount: playing animation ", mountAnimId);
                    characterRenderer->playAnimation(mountInstanceId_, mountAnimId, true);
                }

                // Skip all ground mount logic (jumps, fidgets, etc.)
                goto taxi_mount_done;
            } else {
                taxiAnimsLogged_ = false;
            }

            // Check for jump trigger - use cached per-mount animation IDs
            if (cameraController->isJumpKeyPressed() && grounded && mountAction_ == MountAction::None) {
                if (moving && mountAnims_.jumpLoop > 0) {
                    // Moving: skip JumpStart (looks like stopping), go straight to airborne loop
                    LOG_DEBUG("Mount jump triggered while moving: using jumpLoop anim ", mountAnims_.jumpLoop);
                    characterRenderer->playAnimation(mountInstanceId_, mountAnims_.jumpLoop, true);
                    mountAction_ = MountAction::Jump;
                    mountActionPhase_ = 1;  // Start in airborne phase
                    mountAnimId = mountAnims_.jumpLoop;
                    if (mountSoundManager) {
                        mountSoundManager->playJumpSound();
                    }
                    if (cameraController) {
                        cameraController->triggerMountJump();
                    }
                } else if (!moving && mountAnims_.rearUp > 0) {
                    // Standing still: rear-up flourish
                    LOG_DEBUG("Mount rear-up triggered: playing rearUp anim ", mountAnims_.rearUp);
                    characterRenderer->playAnimation(mountInstanceId_, mountAnims_.rearUp, false);
                    mountAction_ = MountAction::RearUp;
                    mountActionPhase_ = 0;
                    mountAnimId = mountAnims_.rearUp;
                    // Trigger semantic rear-up sound
                    if (mountSoundManager) {
                        mountSoundManager->playRearUpSound();
                    }
                }
            }

            // Handle active mount actions (jump chaining or rear-up)
            if (mountAction_ != MountAction::None) {
                bool animFinished = haveMountState && curMountDur > 0.1f &&
                                   (curMountTime >= curMountDur - 0.05f);

                if (mountAction_ == MountAction::Jump) {
                    // Jump sequence: start → loop → end (physics-driven)
                    if (mountActionPhase_ == 0 && animFinished && mountAnims_.jumpLoop > 0) {
                        // JumpStart finished, go to JumpLoop (airborne)
                        LOG_DEBUG("Mount jump: phase 0→1 (JumpStart→JumpLoop anim ", mountAnims_.jumpLoop, ")");
                        characterRenderer->playAnimation(mountInstanceId_, mountAnims_.jumpLoop, true);
                        mountActionPhase_ = 1;
                        mountAnimId = mountAnims_.jumpLoop;
                    } else if (mountActionPhase_ == 0 && animFinished && mountAnims_.jumpLoop == 0) {
                        // No JumpLoop, go straight to airborne phase 1 (hold JumpStart pose)
                        LOG_DEBUG("Mount jump: phase 0→1 (no JumpLoop, holding JumpStart)");
                        mountActionPhase_ = 1;
                    } else if (mountActionPhase_ == 1 && grounded && mountAnims_.jumpEnd > 0) {
                        // Landed after airborne phase! Go to JumpEnd (grounded-triggered)
                        LOG_DEBUG("Mount jump: phase 1→2 (landed, JumpEnd anim ", mountAnims_.jumpEnd, ")");
                        characterRenderer->playAnimation(mountInstanceId_, mountAnims_.jumpEnd, false);
                        mountActionPhase_ = 2;
                        mountAnimId = mountAnims_.jumpEnd;
                        // Trigger semantic landing sound
                        if (mountSoundManager) {
                            mountSoundManager->playLandSound();
                        }
                    } else if (mountActionPhase_ == 1 && grounded && mountAnims_.jumpEnd == 0) {
                        // No JumpEnd animation, return directly to movement after landing
                        LOG_DEBUG("Mount jump: phase 1→done (landed, no JumpEnd, returning to ",
                                 moving ? "run" : "stand", " anim ", (moving ? mountAnims_.run : mountAnims_.stand), ")");
                        mountAction_ = MountAction::None;
                        mountAnimId = moving ? mountAnims_.run : mountAnims_.stand;
                        characterRenderer->playAnimation(mountInstanceId_, mountAnimId, true);
                    } else if (mountActionPhase_ == 2 && animFinished) {
                        // JumpEnd finished, return to movement
                        LOG_DEBUG("Mount jump: phase 2→done (JumpEnd finished, returning to ",
                                 moving ? "run" : "stand", " anim ", (moving ? mountAnims_.run : mountAnims_.stand), ")");
                        mountAction_ = MountAction::None;
                        mountAnimId = moving ? mountAnims_.run : mountAnims_.stand;
                        characterRenderer->playAnimation(mountInstanceId_, mountAnimId, true);
                    } else {
                        mountAnimId = curMountAnim;  // Keep current jump animation
                    }
                } else if (mountAction_ == MountAction::RearUp) {
                    // Rear-up: single animation, return to stand when done
                    if (animFinished) {
                        LOG_DEBUG("Mount rear-up: finished, returning to ",
                                 moving ? "run" : "stand", " anim ", (moving ? mountAnims_.run : mountAnims_.stand));
                        mountAction_ = MountAction::None;
                        mountAnimId = moving ? mountAnims_.run : mountAnims_.stand;
                        characterRenderer->playAnimation(mountInstanceId_, mountAnimId, true);
                    } else {
                        mountAnimId = curMountAnim;  // Keep current rear-up animation
                    }
                }
            } else if (moving) {
                // Normal movement animations
                if (anyStrafeLeft) {
                    mountAnimId = pickMountAnim({ANIM_STRAFE_RUN_LEFT, ANIM_STRAFE_WALK_LEFT, ANIM_RUN}, ANIM_RUN);
                } else if (anyStrafeRight) {
                    mountAnimId = pickMountAnim({ANIM_STRAFE_RUN_RIGHT, ANIM_STRAFE_WALK_RIGHT, ANIM_RUN}, ANIM_RUN);
                } else if (movingBackward) {
                    mountAnimId = pickMountAnim({ANIM_BACKPEDAL}, ANIM_RUN);
                } else {
                    mountAnimId = ANIM_RUN;
                }
            }

            // Cancel active fidget immediately if movement starts
            if (moving && mountActiveFidget_ != 0) {
                mountActiveFidget_ = 0;
                // Force play run animation to stop fidget immediately
                characterRenderer->playAnimation(mountInstanceId_, mountAnimId, true);
            }

            // Check if active fidget has completed (only when not moving)
            if (!moving && mountActiveFidget_ != 0) {
                uint32_t curAnim = 0;
                float curTime = 0.0f, curDur = 0.0f;
                if (characterRenderer->getAnimationState(mountInstanceId_, curAnim, curTime, curDur)) {
                    // If animation changed or completed, clear active fidget
                    if (curAnim != mountActiveFidget_ || curTime >= curDur * 0.95f) {
                        mountActiveFidget_ = 0;
                        LOG_DEBUG("Mount fidget completed");
                    }
                }
            }

            // Idle fidgets: random one-shot animations when standing still
            if (!moving && mountAction_ == MountAction::None && mountActiveFidget_ == 0 && !mountAnims_.fidgets.empty()) {
                mountIdleFidgetTimer_ += lastDeltaTime_;
                static float nextFidgetTime = 6.0f + (rand() % 7);  // 6-12 seconds

                if (mountIdleFidgetTimer_ >= nextFidgetTime) {
                    // Trigger random fidget animation
                    static std::mt19937 rng(std::random_device{}());
                    std::uniform_int_distribution<size_t> dist(0, mountAnims_.fidgets.size() - 1);
                    uint32_t fidgetAnim = mountAnims_.fidgets[dist(rng)];

                    characterRenderer->playAnimation(mountInstanceId_, fidgetAnim, false);
                    mountActiveFidget_ = fidgetAnim;  // Track active fidget
                    mountIdleFidgetTimer_ = 0.0f;
                    nextFidgetTime = 6.0f + (rand() % 7);  // Randomize next fidget time

                    LOG_DEBUG("Mount idle fidget: playing anim ", fidgetAnim);
                }
            }
            if (moving) {
                mountIdleFidgetTimer_ = 0.0f;  // Reset timer when moving
            }

            // Idle ambient sounds: snorts and whinnies only, infrequent
            if (!moving && mountSoundManager) {
                mountIdleSoundTimer_ += lastDeltaTime_;
                static float nextIdleSoundTime = 45.0f + (rand() % 46);  // 45-90 seconds

                if (mountIdleSoundTimer_ >= nextIdleSoundTime) {
                    mountSoundManager->playIdleSound();
                    mountIdleSoundTimer_ = 0.0f;
                    nextIdleSoundTime = 45.0f + (rand() % 46);  // Randomize next sound time
                }
            } else if (moving) {
                mountIdleSoundTimer_ = 0.0f;  // Reset timer when moving
            }

            // Only update animation if it changed and we're not in an action sequence or playing a fidget
            if (mountAction_ == MountAction::None && mountActiveFidget_ == 0 && (!haveMountState || curMountAnim != mountAnimId)) {
                bool loop = true;  // Normal movement animations loop
                characterRenderer->playAnimation(mountInstanceId_, mountAnimId, loop);
            }

            taxi_mount_done:
            // Rider bob: sinusoidal motion synced to mount's run animation (only used in fallback positioning)
            mountBob = 0.0f;
            if (moving && haveMountState && curMountDur > 1.0f) {
                float norm = std::fmod(curMountTime, curMountDur) / curMountDur;
                // One bounce per stride cycle
                float bobSpeed = taxiFlight_ ? 2.0f : 1.0f;
                mountBob = std::sin(norm * 2.0f * 3.14159f * bobSpeed) * 0.12f;
            }
        }

        // Use mount's attachment point for proper bone-driven rider positioning.
        if (taxiFlight_) {
            glm::mat4 mountSeatTransform(1.0f);
            bool haveSeat = false;
            static constexpr uint32_t kTaxiSeatAttachmentId = 0;  // deterministic rider seat
            if (mountSeatAttachmentId_ == -1) {
                mountSeatAttachmentId_ = static_cast<int>(kTaxiSeatAttachmentId);
            }
            if (mountSeatAttachmentId_ >= 0) {
                haveSeat = characterRenderer->getAttachmentTransform(
                    mountInstanceId_, static_cast<uint32_t>(mountSeatAttachmentId_), mountSeatTransform);
            }
            if (!haveSeat) {
                mountSeatAttachmentId_ = -2;
            }

            if (haveSeat) {
                glm::vec3 targetRiderPos = glm::vec3(mountSeatTransform[3]) + glm::vec3(0.0f, 0.0f, 0.02f);
                // Taxi passengers should be rigidly parented to mount attachment transforms.
                // Smoothing here introduces visible seat lag/drift on turns.
                mountSeatSmoothingInit_ = false;
                smoothedMountSeatPos_ = targetRiderPos;
                characterRenderer->setInstancePosition(characterInstanceId, targetRiderPos);
            } else {
                mountSeatSmoothingInit_ = false;
                glm::vec3 playerPos = characterPosition + glm::vec3(0.0f, 0.0f, mountHeightOffset_ + 0.10f);
                characterRenderer->setInstancePosition(characterInstanceId, playerPos);
            }

            float riderPitch = mountPitch_ * 0.35f;
            float riderRoll = mountRoll_ * 0.35f;
            characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(riderPitch, riderRoll, mountYawRad));
            return;
        }

        // Ground mounts: try a seat attachment first.
        glm::mat4 mountSeatTransform;
        bool haveSeat = false;
        if (mountSeatAttachmentId_ >= 0) {
            haveSeat = characterRenderer->getAttachmentTransform(
                mountInstanceId_, static_cast<uint32_t>(mountSeatAttachmentId_), mountSeatTransform);
        } else if (mountSeatAttachmentId_ == -1) {
            // Probe common rider seat attachment IDs once per mount.
            static constexpr uint32_t kSeatAttachments[] = {0, 5, 6, 7, 8};
            for (uint32_t attId : kSeatAttachments) {
                if (characterRenderer->getAttachmentTransform(mountInstanceId_, attId, mountSeatTransform)) {
                    mountSeatAttachmentId_ = static_cast<int>(attId);
                    haveSeat = true;
                    break;
                }
            }
            if (!haveSeat) {
                mountSeatAttachmentId_ = -2;
            }
        }

        if (haveSeat) {
            // Extract position from mount seat transform (attachment point already includes proper seat height)
            glm::vec3 mountSeatPos = glm::vec3(mountSeatTransform[3]);

            // Keep seat offset minimal; large offsets amplify visible bobble.
            glm::vec3 seatOffset = glm::vec3(0.0f, 0.0f, taxiFlight_ ? 0.04f : 0.08f);
            glm::vec3 targetRiderPos = mountSeatPos + seatOffset;
            // When moving, smoothing the seat position produces visible lag that looks like
            // the rider sliding toward the rump. Anchor rigidly while moving.
            if (moving) {
                mountSeatSmoothingInit_ = false;
                smoothedMountSeatPos_ = targetRiderPos;
            } else if (!mountSeatSmoothingInit_) {
                smoothedMountSeatPos_ = targetRiderPos;
                mountSeatSmoothingInit_ = true;
            } else {
                float smoothHz = taxiFlight_ ? 10.0f : 14.0f;
                float alpha = 1.0f - std::exp(-smoothHz * std::max(lastDeltaTime_, 0.001f));
                smoothedMountSeatPos_ = glm::mix(smoothedMountSeatPos_, targetRiderPos, alpha);
            }

            // Position rider at mount seat
            characterRenderer->setInstancePosition(characterInstanceId, smoothedMountSeatPos_);

            // Rider uses character facing yaw, not mount bone rotation
            // (rider faces character direction, seat bone only provides position)
            float yawRad = glm::radians(characterYaw);
            float riderPitch = taxiFlight_ ? mountPitch_ * 0.35f : 0.0f;
            float riderRoll = taxiFlight_ ? mountRoll_ * 0.35f : 0.0f;
            characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(riderPitch, riderRoll, yawRad));
        } else {
            // Fallback to old manual positioning if attachment not found
            mountSeatSmoothingInit_ = false;
            float yawRad = glm::radians(characterYaw);
            glm::mat4 mountRotation = glm::mat4(1.0f);
            mountRotation = glm::rotate(mountRotation, yawRad, glm::vec3(0.0f, 0.0f, 1.0f));
            mountRotation = glm::rotate(mountRotation, mountRoll_, glm::vec3(1.0f, 0.0f, 0.0f));
            mountRotation = glm::rotate(mountRotation, mountPitch_, glm::vec3(0.0f, 1.0f, 0.0f));
            glm::vec3 localOffset(0.0f, 0.0f, mountHeightOffset_ + mountBob);
            glm::vec3 worldOffset = glm::vec3(mountRotation * glm::vec4(localOffset, 0.0f));
            glm::vec3 playerPos = characterPosition + worldOffset;
            characterRenderer->setInstancePosition(characterInstanceId, playerPos);
            characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(mountPitch_, mountRoll_, yawRad));
        }
        return;
    }

    if (!forceMelee) switch (charAnimState) {
        case CharAnimState::IDLE:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (sitting && grounded) {
                newState = CharAnimState::SIT_DOWN;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            } else if (inCombat_ && grounded) {
                newState = CharAnimState::COMBAT_IDLE;
            }
            break;

        case CharAnimState::WALK:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (!moving) {
                newState = CharAnimState::IDLE;
            } else if (sprinting) {
                newState = CharAnimState::RUN;
            }
            break;

        case CharAnimState::RUN:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (!moving) {
                newState = CharAnimState::IDLE;
            } else if (!sprinting) {
                newState = CharAnimState::WALK;
            }
            break;

        case CharAnimState::JUMP_START:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (grounded) {
                newState = CharAnimState::JUMP_END;
            } else {
                newState = CharAnimState::JUMP_MID;
            }
            break;

        case CharAnimState::JUMP_MID:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (grounded) {
                newState = CharAnimState::JUMP_END;
            }
            break;

        case CharAnimState::JUMP_END:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            } else {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::SIT_DOWN:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (!sitting) {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::SITTING:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (!sitting) {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::EMOTE:
            if (swim) {
                cancelEmote();
                newState = CharAnimState::SWIM_IDLE;
            } else if (jumping || !grounded) {
                cancelEmote();
                newState = CharAnimState::JUMP_START;
            } else if (moving) {
                cancelEmote();
                newState = sprinting ? CharAnimState::RUN : CharAnimState::WALK;
            } else if (sitting) {
                cancelEmote();
                newState = CharAnimState::SIT_DOWN;
            }
            break;

        case CharAnimState::SWIM_IDLE:
            if (!swim) {
                newState = moving ? CharAnimState::WALK : CharAnimState::IDLE;
            } else if (moving) {
                newState = CharAnimState::SWIM;
            }
            break;

        case CharAnimState::SWIM:
            if (!swim) {
                newState = moving ? CharAnimState::WALK : CharAnimState::IDLE;
            } else if (!moving) {
                newState = CharAnimState::SWIM_IDLE;
            }
            break;

        case CharAnimState::MELEE_SWING:
            if (swim) {
                newState = CharAnimState::SWIM_IDLE;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            } else if (sitting) {
                newState = CharAnimState::SIT_DOWN;
            } else if (inCombat_) {
                newState = CharAnimState::COMBAT_IDLE;
            } else {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::MOUNT:
            // If we got here, the mount state was cleared externally but the
            // animation state hasn't been reset yet. Fall back to normal logic.
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (sitting && grounded) {
                newState = CharAnimState::SIT_DOWN;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            } else {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::COMBAT_IDLE:
            if (swim) {
                newState = moving ? CharAnimState::SWIM : CharAnimState::SWIM_IDLE;
            } else if (!grounded && jumping) {
                newState = CharAnimState::JUMP_START;
            } else if (!grounded) {
                newState = CharAnimState::JUMP_MID;
            } else if (moving && sprinting) {
                newState = CharAnimState::RUN;
            } else if (moving) {
                newState = CharAnimState::WALK;
            } else if (!inCombat_) {
                newState = CharAnimState::IDLE;
            }
            break;

        case CharAnimState::CHARGE:
            // Stay in CHARGE until charging_ is cleared
            break;
    }

    if (forceMelee) {
        newState = CharAnimState::MELEE_SWING;
    }

    if (charging_) {
        newState = CharAnimState::CHARGE;
    }

    if (newState != charAnimState) {
        charAnimState = newState;
    }

    auto pickFirstAvailable = [&](std::initializer_list<uint32_t> candidates, uint32_t fallback) -> uint32_t {
        for (uint32_t id : candidates) {
            if (characterRenderer->hasAnimation(characterInstanceId, id)) {
                return id;
            }
        }
        return fallback;
    };

    uint32_t animId = ANIM_STAND;
    bool loop = true;

    switch (charAnimState) {
        case CharAnimState::IDLE:       animId = ANIM_STAND;      loop = true;  break;
        case CharAnimState::WALK:
            if (movingBackward) {
                animId = pickFirstAvailable({ANIM_BACKPEDAL}, ANIM_WALK);
            } else if (anyStrafeLeft) {
                animId = pickFirstAvailable({ANIM_STRAFE_WALK_LEFT, ANIM_STRAFE_RUN_LEFT}, ANIM_WALK);
            } else if (anyStrafeRight) {
                animId = pickFirstAvailable({ANIM_STRAFE_WALK_RIGHT, ANIM_STRAFE_RUN_RIGHT}, ANIM_WALK);
            } else {
                animId = pickFirstAvailable({ANIM_WALK, ANIM_RUN}, ANIM_STAND);
            }
            loop = true;
            break;
        case CharAnimState::RUN:
            if (movingBackward) {
                animId = pickFirstAvailable({ANIM_BACKPEDAL}, ANIM_WALK);
            } else if (anyStrafeLeft) {
                animId = pickFirstAvailable({ANIM_STRAFE_RUN_LEFT}, ANIM_RUN);
            } else if (anyStrafeRight) {
                animId = pickFirstAvailable({ANIM_STRAFE_RUN_RIGHT}, ANIM_RUN);
            } else {
                animId = pickFirstAvailable({ANIM_RUN, ANIM_WALK}, ANIM_STAND);
            }
            loop = true;
            break;
        case CharAnimState::JUMP_START: animId = ANIM_JUMP_START; loop = false; break;
        case CharAnimState::JUMP_MID:   animId = ANIM_JUMP_MID;   loop = false; break;
        case CharAnimState::JUMP_END:   animId = ANIM_JUMP_END;   loop = false; break;
        case CharAnimState::SIT_DOWN:   animId = ANIM_SIT_DOWN;   loop = false; break;
        case CharAnimState::SITTING:    animId = ANIM_SITTING;    loop = true;  break;
        case CharAnimState::EMOTE:      animId = emoteAnimId;     loop = emoteLoop; break;
        case CharAnimState::SWIM_IDLE:  animId = ANIM_SWIM_IDLE;  loop = true;  break;
        case CharAnimState::SWIM:       animId = ANIM_SWIM;       loop = true;  break;
        case CharAnimState::MELEE_SWING:
            animId = resolveMeleeAnimId();
            if (animId == 0) {
                animId = ANIM_STAND;
            }
            loop = false;
            break;
        case CharAnimState::MOUNT:      animId = ANIM_MOUNT;      loop = true;  break;
        case CharAnimState::COMBAT_IDLE:
            animId = pickFirstAvailable(
                {ANIM_READY_1H, ANIM_READY_2H, ANIM_READY_2H_L, ANIM_READY_UNARMED},
                ANIM_STAND);
            loop = true;
            break;
        case CharAnimState::CHARGE:
            animId = ANIM_RUN;
            loop = true;
            break;
    }

    uint32_t currentAnimId = 0;
    float currentAnimTimeMs = 0.0f;
    float currentAnimDurationMs = 0.0f;
    bool haveState = characterRenderer->getAnimationState(characterInstanceId, currentAnimId, currentAnimTimeMs, currentAnimDurationMs);
    if (!haveState || currentAnimId != animId) {
        characterRenderer->playAnimation(characterInstanceId, animId, loop);
    }
}

void Renderer::playEmote(const std::string& emoteName) {
    loadEmotesFromDbc();
    auto it = EMOTE_TABLE.find(emoteName);
    if (it == EMOTE_TABLE.end()) return;

    const auto& info = it->second;
    if (info.animId == 0) return;
    emoteActive = true;
    emoteAnimId = info.animId;
    emoteLoop = info.loop;
    charAnimState = CharAnimState::EMOTE;

    if (characterRenderer && characterInstanceId > 0) {
        characterRenderer->playAnimation(characterInstanceId, emoteAnimId, emoteLoop);
    }
}

void Renderer::cancelEmote() {
    emoteActive = false;
    emoteAnimId = 0;
    emoteLoop = false;
}

void Renderer::triggerLevelUpEffect(const glm::vec3& position) {
    if (!levelUpEffect) return;

    // Lazy-load the M2 model on first trigger
    if (!levelUpEffect->isModelLoaded() && m2Renderer) {
        if (!cachedAssetManager) {
            cachedAssetManager = core::Application::getInstance().getAssetManager();
        }
        if (!cachedAssetManager) {
            LOG_WARNING("LevelUpEffect: no asset manager available");
        } else {
            auto m2Data = cachedAssetManager->readFile("Spells\\LevelUp\\LevelUp.m2");
            auto skinData = cachedAssetManager->readFile("Spells\\LevelUp\\LevelUp00.skin");
            LOG_INFO("LevelUpEffect: m2Data=", m2Data.size(), " skinData=", skinData.size());
            if (!m2Data.empty()) {
                levelUpEffect->loadModel(m2Renderer.get(), m2Data, skinData);
            } else {
                LOG_WARNING("LevelUpEffect: failed to read Spell\\LevelUp\\LevelUp.m2");
            }
        }
    }

    levelUpEffect->trigger(position);
}

void Renderer::startChargeEffect(const glm::vec3& position, const glm::vec3& direction) {
    if (!chargeEffect) return;

    // Lazy-load M2 models on first use
    if (!chargeEffect->isActive() && m2Renderer) {
        if (!cachedAssetManager) {
            cachedAssetManager = core::Application::getInstance().getAssetManager();
        }
        if (cachedAssetManager) {
            chargeEffect->tryLoadM2Models(m2Renderer.get(), cachedAssetManager);
        }
    }

    chargeEffect->start(position, direction);
}

void Renderer::emitChargeEffect(const glm::vec3& position, const glm::vec3& direction) {
    if (chargeEffect) {
        chargeEffect->emit(position, direction);
    }
}

void Renderer::stopChargeEffect() {
    if (chargeEffect) {
        chargeEffect->stop();
    }
}

void Renderer::triggerMeleeSwing() {
    if (!characterRenderer || characterInstanceId == 0) return;
    if (meleeSwingCooldown > 0.0f) return;
    if (emoteActive) {
        cancelEmote();
    }
    resolveMeleeAnimId();
    meleeSwingCooldown = 0.1f;
    float durationSec = meleeAnimDurationMs > 0.0f ? meleeAnimDurationMs / 1000.0f : 0.6f;
    if (durationSec < 0.25f) durationSec = 0.25f;
    if (durationSec > 1.0f) durationSec = 1.0f;
    meleeSwingTimer = durationSec;
    if (activitySoundManager) {
        activitySoundManager->playMeleeSwing();
    }
}

std::string Renderer::getEmoteText(const std::string& emoteName, const std::string* targetName) {
    loadEmotesFromDbc();
    auto it = EMOTE_TABLE.find(emoteName);
    if (it != EMOTE_TABLE.end()) {
        const auto& info = it->second;
        const std::string& base = (targetName ? info.textTarget : info.textNoTarget);
        if (!base.empty()) {
            return replacePlaceholders(base, targetName);
        }
        if (targetName && !targetName->empty()) {
            return "You " + info.command + " at " + *targetName + ".";
        }
        return "You " + info.command + ".";
    }
    return "";
}

uint32_t Renderer::getEmoteDbcId(const std::string& emoteName) {
    loadEmotesFromDbc();
    auto it = EMOTE_TABLE.find(emoteName);
    if (it != EMOTE_TABLE.end()) {
        return it->second.dbcId;
    }
    return 0;
}

std::string Renderer::getEmoteTextByDbcId(uint32_t dbcId, const std::string& senderName,
                                           const std::string* targetName) {
    loadEmotesFromDbc();
    auto it = EMOTE_BY_DBCID.find(dbcId);
    if (it == EMOTE_BY_DBCID.end()) return "";

    const EmoteInfo& info = *it->second;

    // Use "others see" text templates: "%s dances." / "%s dances with %s."
    if (targetName && !targetName->empty()) {
        if (!info.othersTarget.empty()) {
            // Replace first %s with sender, second %s with target
            std::string out;
            out.reserve(info.othersTarget.size() + senderName.size() + targetName->size());
            bool firstReplaced = false;
            for (size_t i = 0; i < info.othersTarget.size(); ++i) {
                if (info.othersTarget[i] == '%' && i + 1 < info.othersTarget.size() && info.othersTarget[i + 1] == 's') {
                    out += firstReplaced ? *targetName : senderName;
                    firstReplaced = true;
                    ++i;
                } else {
                    out.push_back(info.othersTarget[i]);
                }
            }
            return out;
        }
        return senderName + " " + info.command + "s at " + *targetName + ".";
    } else {
        if (!info.othersNoTarget.empty()) {
            return replacePlaceholders(info.othersNoTarget, &senderName);
        }
        return senderName + " " + info.command + "s.";
    }
}

uint32_t Renderer::getEmoteAnimByDbcId(uint32_t dbcId) {
    loadEmotesFromDbc();
    auto it = EMOTE_BY_DBCID.find(dbcId);
    if (it != EMOTE_BY_DBCID.end()) {
        return it->second->animId;
    }
    return 0;
}

void Renderer::setTargetPosition(const glm::vec3* pos) {
    targetPosition = pos;
}

bool Renderer::isMoving() const {
    return cameraController && cameraController->isMoving();
}

bool Renderer::isFootstepAnimationState() const {
    return charAnimState == CharAnimState::WALK || charAnimState == CharAnimState::RUN;
}

bool Renderer::shouldTriggerFootstepEvent(uint32_t animationId, float animationTimeMs, float animationDurationMs) {
    if (animationDurationMs <= 1.0f) {
        footstepNormInitialized = false;
        return false;
    }

    float norm = std::fmod(animationTimeMs, animationDurationMs) / animationDurationMs;
    if (norm < 0.0f) norm += 1.0f;

    if (animationId != footstepLastAnimationId) {
        footstepLastAnimationId = animationId;
        footstepLastNormTime = norm;
        footstepNormInitialized = true;
        return false;
    }

    if (!footstepNormInitialized) {
        footstepNormInitialized = true;
        footstepLastNormTime = norm;
        return false;
    }

    auto crossed = [&](float eventNorm) {
        if (footstepLastNormTime <= norm) {
            return footstepLastNormTime < eventNorm && eventNorm <= norm;
        }
        return footstepLastNormTime < eventNorm || eventNorm <= norm;
    };

    bool trigger = crossed(0.22f) || crossed(0.72f);
    footstepLastNormTime = norm;
    return trigger;
}

audio::FootstepSurface Renderer::resolveFootstepSurface() const {
    if (!cameraController || !cameraController->isThirdPerson()) {
        return audio::FootstepSurface::STONE;
    }

    const glm::vec3& p = characterPosition;

    // Cache footstep surface to avoid expensive queries every step
    // Only update if moved >1.5 units or timer expired (0.5s)
    float distSq = glm::dot(p - cachedFootstepPosition, p - cachedFootstepPosition);
    if (distSq < 2.25f && cachedFootstepUpdateTimer < 0.5f) {
        return cachedFootstepSurface;
    }

    // Update cache
    cachedFootstepPosition = p;
    cachedFootstepUpdateTimer = 0.0f;

    if (cameraController->isSwimming()) {
        cachedFootstepSurface = audio::FootstepSurface::WATER;
        return audio::FootstepSurface::WATER;
    }

    if (waterRenderer) {
        auto waterH = waterRenderer->getWaterHeightAt(p.x, p.y);
        if (waterH && p.z < (*waterH + 0.25f)) {
            cachedFootstepSurface = audio::FootstepSurface::WATER;
            return audio::FootstepSurface::WATER;
        }
    }

    if (wmoRenderer) {
        auto wmoFloor = wmoRenderer->getFloorHeight(p.x, p.y, p.z + 1.5f);
        auto terrainFloor = terrainManager ? terrainManager->getHeightAt(p.x, p.y) : std::nullopt;
        if (wmoFloor && (!terrainFloor || *wmoFloor >= *terrainFloor - 0.1f)) {
            cachedFootstepSurface = audio::FootstepSurface::STONE;
            return audio::FootstepSurface::STONE;
        }
    }

    // Determine surface type (expensive - only done when cache needs update)
    audio::FootstepSurface surface = audio::FootstepSurface::STONE;

    if (terrainManager) {
        auto texture = terrainManager->getDominantTextureAt(p.x, p.y);
        if (texture) {
            std::string t = *texture;
            for (char& c : t) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (t.find("snow") != std::string::npos || t.find("ice") != std::string::npos) surface = audio::FootstepSurface::SNOW;
            else if (t.find("grass") != std::string::npos || t.find("moss") != std::string::npos || t.find("leaf") != std::string::npos) surface = audio::FootstepSurface::GRASS;
            else if (t.find("sand") != std::string::npos || t.find("dirt") != std::string::npos || t.find("mud") != std::string::npos) surface = audio::FootstepSurface::DIRT;
            else if (t.find("wood") != std::string::npos || t.find("timber") != std::string::npos) surface = audio::FootstepSurface::WOOD;
            else if (t.find("metal") != std::string::npos || t.find("iron") != std::string::npos) surface = audio::FootstepSurface::METAL;
            else if (t.find("stone") != std::string::npos || t.find("rock") != std::string::npos || t.find("cobble") != std::string::npos || t.find("brick") != std::string::npos) surface = audio::FootstepSurface::STONE;
        }
    }

    cachedFootstepSurface = surface;
    return surface;
}

void Renderer::update(float deltaTime) {
    globalTime += deltaTime;
    if (musicSwitchCooldown_ > 0.0f) {
        musicSwitchCooldown_ = std::max(0.0f, musicSwitchCooldown_ - deltaTime);
    }
    runDeferredWorldInitStep(deltaTime);

    auto updateStart = std::chrono::steady_clock::now();
    lastDeltaTime_ = deltaTime;  // Cache for use in updateCharacterAnimation()

    if (wmoRenderer) wmoRenderer->resetQueryStats();
    if (m2Renderer) m2Renderer->resetQueryStats();

    if (cameraController) {
        auto cameraStart = std::chrono::steady_clock::now();
        cameraController->update(deltaTime);
        auto cameraEnd = std::chrono::steady_clock::now();
        lastCameraUpdateMs = std::chrono::duration<double, std::milli>(cameraEnd - cameraStart).count();

        // Update 3D audio listener position/orientation to match camera
        if (camera) {
            audio::AudioEngine::instance().setListenerPosition(camera->getPosition());
            audio::AudioEngine::instance().setListenerOrientation(camera->getForward(), camera->getUp());
        }
    } else {
        lastCameraUpdateMs = 0.0;
    }

    // Visibility hardening: ensure player instance cannot stay hidden after
    // taxi/camera transitions, but preserve first-person self-hide.
    if (characterRenderer && characterInstanceId > 0 && cameraController) {
        if ((cameraController->isThirdPerson() && !cameraController->isFirstPersonView()) || taxiFlight_) {
            characterRenderer->setInstanceVisible(characterInstanceId, true);
        }
    }

    // Update lighting system
    if (lightingManager) {
        const auto* gh = core::Application::getInstance().getGameHandler();
        uint32_t mapId    = gh ? gh->getCurrentMapId() : 0;
        float gameTime    = gh ? gh->getGameTime() : -1.0f;
        bool isRaining    = gh ? gh->isRaining() : false;
        bool isUnderwater = cameraController ? cameraController->isSwimming() : false;

        lightingManager->update(characterPosition, mapId, gameTime, isRaining, isUnderwater);

        // Sync weather visual renderer with game state
        if (weather && gh) {
            uint32_t wType = gh->getWeatherType();
            float wInt = gh->getWeatherIntensity();
            if (wType != 0) {
                // Server-driven weather (SMSG_WEATHER) — authoritative
                if (wType == 1)      weather->setWeatherType(Weather::Type::RAIN);
                else if (wType == 2) weather->setWeatherType(Weather::Type::SNOW);
                else                 weather->setWeatherType(Weather::Type::NONE);
                weather->setIntensity(wInt);
            } else {
                // No server weather — use zone-based weather configuration
                weather->updateZoneWeather(currentZoneId, deltaTime);
            }
            weather->setEnabled(true);
        } else if (weather) {
            // No game handler (single-player without network) — zone weather only
            weather->updateZoneWeather(currentZoneId, deltaTime);
            weather->setEnabled(true);
        }
    }

    // Sync character model position/rotation and animation with follow target
    if (characterInstanceId > 0 && characterRenderer && cameraController) {
        if (meleeSwingCooldown > 0.0f) {
            meleeSwingCooldown = std::max(0.0f, meleeSwingCooldown - deltaTime);
        }
        if (meleeSwingTimer > 0.0f) {
            meleeSwingTimer = std::max(0.0f, meleeSwingTimer - deltaTime);
        }

        characterRenderer->setInstancePosition(characterInstanceId, characterPosition);

        // Movement-facing comes from camera controller and is decoupled from LMB orbit.
        // During taxi flights, orientation is controlled by the flight path (not player input)
        if (taxiFlight_) {
            // Taxi flight: use orientation from flight path
            characterYaw = cameraController->getFacingYaw();
        } else if (cameraController->isMoving() || cameraController->isRightMouseHeld()) {
            characterYaw = cameraController->getFacingYaw();
        } else if (inCombat_ && targetPosition && !emoteActive && !isMounted()) {
            // Face target when in combat and idle
            glm::vec3 toTarget = *targetPosition - characterPosition;
            if (glm::length(glm::vec2(toTarget.x, toTarget.y)) > 0.1f) {
                float targetYaw = glm::degrees(std::atan2(toTarget.y, toTarget.x));
                float diff = targetYaw - characterYaw;
                while (diff > 180.0f) diff -= 360.0f;
                while (diff < -180.0f) diff += 360.0f;
                float rotSpeed = 360.0f * deltaTime;
                if (std::abs(diff) < rotSpeed) {
                    characterYaw = targetYaw;
                } else {
                    characterYaw += (diff > 0 ? rotSpeed : -rotSpeed);
                }
            }
        }
        float yawRad = glm::radians(characterYaw);
        characterRenderer->setInstanceRotation(characterInstanceId, glm::vec3(0.0f, 0.0f, yawRad));

        // Update animation based on movement state
        updateCharacterAnimation();
    }

    // Update terrain streaming
    if (terrainManager && camera) {
        terrainManager->update(*camera, deltaTime);
    }

    // Update sky system (skybox time, star twinkle, clouds, celestial moon phases)
    if (skySystem) {
        skySystem->update(deltaTime);
    }

    // Update weather particles
    if (weather && camera) {
        weather->update(*camera, deltaTime);
    }

    // Update swim effects
    if (swimEffects && camera && cameraController && waterRenderer) {
        swimEffects->update(*camera, *cameraController, *waterRenderer, deltaTime);
    }

    // Update mount dust effects
    if (mountDust) {
        mountDust->update(deltaTime);

        // Spawn dust when mounted and moving on ground
        if (isMounted() && camera && cameraController && !taxiFlight_) {
            bool isMoving = cameraController->isMoving();
            bool onGround = cameraController->isGrounded();

            if (isMoving && onGround) {
                // Calculate velocity from camera direction and speed
                glm::vec3 forward = camera->getForward();
                float speed = cameraController->getMovementSpeed();
                glm::vec3 velocity = forward * speed;
                velocity.z = 0.0f;  // Ignore vertical component

                // Spawn dust at mount's feet (slightly below character position)
                glm::vec3 dustPos = characterPosition - glm::vec3(0.0f, 0.0f, mountHeightOffset_ * 0.8f);
                mountDust->spawnDust(dustPos, velocity, isMoving);
            }
        }
    }
    // Update level-up effect
    if (levelUpEffect) {
        levelUpEffect->update(deltaTime);
    }
    // Update charge effect
    if (chargeEffect) {
        chargeEffect->update(deltaTime);
    }


    // Update character animations
    if (characterRenderer && camera) {
        characterRenderer->update(deltaTime, camera->getPosition());
    }

    // Update AudioEngine (cleanup finished sounds, etc.)
    audio::AudioEngine::instance().update(deltaTime);

    // Footsteps: animation-event driven + surface query at event time.
    if (footstepManager) {
        footstepManager->update(deltaTime);
        cachedFootstepUpdateTimer += deltaTime;  // Update surface cache timer
        bool canPlayFootsteps = characterRenderer && characterInstanceId > 0 &&
            cameraController && cameraController->isThirdPerson() &&
            cameraController->isGrounded() && !cameraController->isSwimming();

        if (canPlayFootsteps && isMounted() && mountInstanceId_ > 0 && !taxiFlight_) {
            // Mount footsteps: use mount's animation for timing
            uint32_t animId = 0;
            float animTimeMs = 0.0f, animDurationMs = 0.0f;
            if (characterRenderer->getAnimationState(mountInstanceId_, animId, animTimeMs, animDurationMs) &&
                animDurationMs > 1.0f && cameraController->isMoving()) {
                float norm = std::fmod(animTimeMs, animDurationMs) / animDurationMs;
                if (norm < 0.0f) norm += 1.0f;

                if (animId != mountFootstepLastAnimId) {
                    mountFootstepLastAnimId = animId;
                    mountFootstepLastNormTime = norm;
                    mountFootstepNormInitialized = true;
                } else if (!mountFootstepNormInitialized) {
                    mountFootstepNormInitialized = true;
                    mountFootstepLastNormTime = norm;
                } else {
                    // Mount gait: 2 hoofbeats per cycle (synced with animation)
                    auto crossed = [&](float eventNorm) {
                        if (mountFootstepLastNormTime <= norm) {
                            return mountFootstepLastNormTime < eventNorm && eventNorm <= norm;
                        }
                        return mountFootstepLastNormTime < eventNorm || eventNorm <= norm;
                    };
                    if (crossed(0.25f) || crossed(0.75f)) {
                        footstepManager->playFootstep(resolveFootstepSurface(), true);
                    }
                    mountFootstepLastNormTime = norm;
                }
            } else {
                mountFootstepNormInitialized = false;
            }
            footstepNormInitialized = false;  // Reset player footstep tracking
        } else if (canPlayFootsteps && isFootstepAnimationState()) {
            uint32_t animId = 0;
            float animTimeMs = 0.0f;
            float animDurationMs = 0.0f;
            if (characterRenderer->getAnimationState(characterInstanceId, animId, animTimeMs, animDurationMs) &&
                shouldTriggerFootstepEvent(animId, animTimeMs, animDurationMs)) {
                auto surface = resolveFootstepSurface();
                footstepManager->playFootstep(surface, cameraController->isSprinting());
                // Play additional splash sound and spawn foot splash particles when wading
                if (surface == audio::FootstepSurface::WATER) {
                    if (movementSoundManager) {
                        movementSoundManager->playWaterFootstep(audio::MovementSoundManager::CharacterSize::MEDIUM);
                    }
                    if (swimEffects && waterRenderer) {
                        auto wh = waterRenderer->getWaterHeightAt(characterPosition.x, characterPosition.y);
                        if (wh) {
                            swimEffects->spawnFootSplash(characterPosition, *wh);
                        }
                    }
                }
            }
            mountFootstepNormInitialized = false;
        } else {
            footstepNormInitialized = false;
            mountFootstepNormInitialized = false;
        }
    }

    // Activity SFX: animation/state-driven jump, landing, and swim loops/splashes.
    if (activitySoundManager) {
        activitySoundManager->update(deltaTime);
        if (cameraController && cameraController->isThirdPerson()) {
            bool grounded = cameraController->isGrounded();
            bool jumping = cameraController->isJumping();
            bool falling = cameraController->isFalling();
            bool swimming = cameraController->isSwimming();
            bool moving = cameraController->isMoving();

            if (!sfxStateInitialized) {
                sfxPrevGrounded = grounded;
                sfxPrevJumping = jumping;
                sfxPrevFalling = falling;
                sfxPrevSwimming = swimming;
                sfxStateInitialized = true;
            }

            if (jumping && !sfxPrevJumping && !swimming) {
                activitySoundManager->playJump();
            }

            if (grounded && !sfxPrevGrounded) {
                bool hardLanding = sfxPrevFalling;
                activitySoundManager->playLanding(resolveFootstepSurface(), hardLanding);
            }

            if (swimming && !sfxPrevSwimming) {
                activitySoundManager->playWaterEnter();
            } else if (!swimming && sfxPrevSwimming) {
                activitySoundManager->playWaterExit();
            }

            activitySoundManager->setSwimmingState(swimming, moving);

            // Fade music underwater
            if (musicManager) {
                musicManager->setUnderwaterMode(swimming);
            }

            sfxPrevGrounded = grounded;
            sfxPrevJumping = jumping;
            sfxPrevFalling = falling;
            sfxPrevSwimming = swimming;
        } else {
            activitySoundManager->setSwimmingState(false, false);
            // Restore music volume when activity sounds disabled
            if (musicManager) {
                musicManager->setUnderwaterMode(false);
            }
            sfxStateInitialized = false;
        }
    }

    // Mount ambient sounds: wing flaps, breathing, etc.
    if (mountSoundManager) {
        mountSoundManager->update(deltaTime);
        if (cameraController && isMounted()) {
            bool moving = cameraController->isMoving();
            bool flying = taxiFlight_ || !cameraController->isGrounded();  // Flying if taxi or airborne
            mountSoundManager->setMoving(moving);
            mountSoundManager->setFlying(flying);
        }
    }

    const bool canQueryWmo = (camera && wmoRenderer);
    const glm::vec3 camPos = camera ? camera->getPosition() : glm::vec3(0.0f);
    uint32_t insideWmoId = 0;
    const bool insideWmo = canQueryWmo &&
        wmoRenderer->isInsideWMO(camPos.x, camPos.y, camPos.z, &insideWmoId);

    // Ambient environmental sounds: fireplaces, water, birds, etc.
    if (ambientSoundManager && camera && wmoRenderer && cameraController) {
        bool isIndoor = insideWmo;
        bool isSwimming = cameraController->isSwimming();

        // Check if inside blacksmith (96048 = Goldshire blacksmith)
        bool isBlacksmith = (insideWmoId == 96048);

        // Sync weather audio with visual weather system
        if (weather) {
            auto weatherType = weather->getWeatherType();
            float intensity = weather->getIntensity();

            audio::AmbientSoundManager::WeatherType audioWeatherType = audio::AmbientSoundManager::WeatherType::NONE;

            if (weatherType == Weather::Type::RAIN) {
                if (intensity < 0.33f) {
                    audioWeatherType = audio::AmbientSoundManager::WeatherType::RAIN_LIGHT;
                } else if (intensity < 0.66f) {
                    audioWeatherType = audio::AmbientSoundManager::WeatherType::RAIN_MEDIUM;
                } else {
                    audioWeatherType = audio::AmbientSoundManager::WeatherType::RAIN_HEAVY;
                }
            } else if (weatherType == Weather::Type::SNOW) {
                if (intensity < 0.33f) {
                    audioWeatherType = audio::AmbientSoundManager::WeatherType::SNOW_LIGHT;
                } else if (intensity < 0.66f) {
                    audioWeatherType = audio::AmbientSoundManager::WeatherType::SNOW_MEDIUM;
                } else {
                    audioWeatherType = audio::AmbientSoundManager::WeatherType::SNOW_HEAVY;
                }
            }

            ambientSoundManager->setWeather(audioWeatherType);
        }

        ambientSoundManager->update(deltaTime, camPos, isIndoor, isSwimming, isBlacksmith);
    }

    // Update M2 doodad animations (pass camera for frustum-culling bone computation)
    if (m2Renderer && camera) {
        m2Renderer->update(deltaTime, camera->getPosition(),
                           camera->getProjectionMatrix() * camera->getViewMatrix());
    }

    // Helper: play zone music, dispatching local files (file: prefix) vs MPQ paths
    auto playZoneMusic = [&](const std::string& music) {
        if (music.empty()) return;
        if (music.rfind("file:", 0) == 0) {
            musicManager->crossfadeToFile(music.substr(5));
        } else {
            musicManager->crossfadeTo(music);
        }
    };

    // Update zone detection and music
    if (zoneManager && musicManager && terrainManager && camera) {
        // First check tile-based zone
        auto tile = terrainManager->getCurrentTile();
        uint32_t zoneId = zoneManager->getZoneId(tile.x, tile.y);

        bool insideTavern = false;
        bool insideBlacksmith = false;
        std::string tavernMusic;

        // Override with WMO-based detection (e.g., inside Stormwind, taverns, blacksmiths)
        if (wmoRenderer) {
            uint32_t wmoModelId = insideWmoId;
            if (insideWmo) {
                // Check if inside Stormwind WMO (model ID 10047)
                if (wmoModelId == 10047) {
                    zoneId = 1519;  // Stormwind City
                }

                // Detect taverns/inns/blacksmiths by WMO model ID
                // Log WMO ID for debugging
                static uint32_t lastLoggedWmoId = 0;
                if (wmoModelId != lastLoggedWmoId) {
                    LOG_INFO("Inside WMO model ID: ", wmoModelId);
                    lastLoggedWmoId = wmoModelId;
                }

                // Blacksmith detection
                if (wmoModelId == 96048) {  // Goldshire blacksmith
                    insideBlacksmith = true;
                    LOG_INFO("Detected blacksmith WMO ", wmoModelId);
                }

                // These IDs represent typical Alliance and Horde inn buildings
                if (wmoModelId == 191 ||    // Goldshire inn (old ID)
                    wmoModelId == 71414 ||  // Goldshire inn (actual)
                    wmoModelId == 190 ||    // Small inn (common)
                    wmoModelId == 220 ||    // Tavern building
                    wmoModelId == 221 ||    // Large tavern
                    wmoModelId == 5392 ||   // Horde inn
                    wmoModelId == 5393) {   // Another inn variant
                    insideTavern = true;
                    // WoW tavern music (cozy ambient tracks) - FIXED PATHS
                    static const std::vector<std::string> tavernTracks = {
                        "Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance01.mp3",
                        "Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance02.mp3",
                        "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern1A.mp3",
                        "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern2A.mp3",
                    };
                    static int tavernTrackIndex = 0;
                    tavernMusic = tavernTracks[tavernTrackIndex % tavernTracks.size()];
                    LOG_INFO("Detected tavern WMO ", wmoModelId, ", playing: ", tavernMusic);
                }
            }
        }

        // Handle tavern music transitions
        if (insideTavern) {
            if (!inTavern_ && !tavernMusic.empty()) {
                inTavern_ = true;
                LOG_INFO("Entered tavern");
                musicManager->playMusic(tavernMusic, true);  // Immediate playback, looping
                musicSwitchCooldown_ = 6.0f;
            }
        } else if (inTavern_) {
            // Exited tavern - restore zone music with crossfade
            inTavern_ = false;
            LOG_INFO("Exited tavern");
            auto* info = zoneManager->getZoneInfo(currentZoneId);
            if (info) {
                std::string music = zoneManager->getRandomMusic(currentZoneId);
                if (!music.empty()) {
                    playZoneMusic(music);
                    musicSwitchCooldown_ = 6.0f;
                }
            }
        }

        // Handle blacksmith music (stop music when entering blacksmith, let ambience play)
        if (insideBlacksmith) {
            if (!inBlacksmith_) {
                inBlacksmith_ = true;
                LOG_INFO("Entered blacksmith - stopping music");
                musicManager->stopMusic();
            }
        } else if (inBlacksmith_) {
            // Exited blacksmith - restore zone music with crossfade
            inBlacksmith_ = false;
            LOG_INFO("Exited blacksmith - restoring music");
            auto* info = zoneManager->getZoneInfo(currentZoneId);
            if (info) {
                std::string music = zoneManager->getRandomMusic(currentZoneId);
                if (!music.empty()) {
                    playZoneMusic(music);
                    musicSwitchCooldown_ = 6.0f;
                }
            }
        }

        // Handle normal zone transitions (only if not in tavern or blacksmith)
        if (!insideTavern && !insideBlacksmith && zoneId != currentZoneId && zoneId != 0) {
            currentZoneId = zoneId;
            auto* info = zoneManager->getZoneInfo(zoneId);
            if (info) {
                currentZoneName = info->name;
                LOG_INFO("Entered zone: ", info->name);
                if (musicSwitchCooldown_ <= 0.0f) {
                    std::string music = zoneManager->getRandomMusic(zoneId);
                    if (!music.empty()) {
                        playZoneMusic(music);
                        musicSwitchCooldown_ = 6.0f;
                    }
                }
            }
        }

        musicManager->update(deltaTime);

        // When a track finishes, pick a new random track from the current zone
        if (!musicManager->isPlaying() && !inTavern_ && !inBlacksmith_ &&
            currentZoneId != 0 && musicSwitchCooldown_ <= 0.0f) {
            std::string music = zoneManager->getRandomMusic(currentZoneId);
            if (!music.empty()) {
                playZoneMusic(music);
                musicSwitchCooldown_ = 2.0f;
            }
        }
    }

    // Update performance HUD
    if (performanceHUD) {
        performanceHUD->update(deltaTime);
    }

    // Periodic cache hygiene: drop model GPU data no longer referenced by active instances.
    static float modelCleanupTimer = 0.0f;
    modelCleanupTimer += deltaTime;
    if (modelCleanupTimer >= 5.0f) {
        if (wmoRenderer) {
            wmoRenderer->cleanupUnusedModels();
        }
        if (m2Renderer) {
            m2Renderer->cleanupUnusedModels();
        }
        modelCleanupTimer = 0.0f;
    }

    auto updateEnd = std::chrono::steady_clock::now();
    lastUpdateMs = std::chrono::duration<double, std::milli>(updateEnd - updateStart).count();
}

void Renderer::runDeferredWorldInitStep(float deltaTime) {
    if (!deferredWorldInitEnabled_ || !deferredWorldInitPending_ || !cachedAssetManager) return;
    if (deferredWorldInitCooldown_ > 0.0f) {
        deferredWorldInitCooldown_ = std::max(0.0f, deferredWorldInitCooldown_ - deltaTime);
        if (deferredWorldInitCooldown_ > 0.0f) return;
    }

    switch (deferredWorldInitStage_) {
        case 0:
            if (ambientSoundManager) {
                ambientSoundManager->initialize(cachedAssetManager);
            }
            if (terrainManager && ambientSoundManager) {
                terrainManager->setAmbientSoundManager(ambientSoundManager.get());
            }
            break;
        case 1:
            if (uiSoundManager) uiSoundManager->initialize(cachedAssetManager);
            break;
        case 2:
            if (combatSoundManager) combatSoundManager->initialize(cachedAssetManager);
            break;
        case 3:
            if (spellSoundManager) spellSoundManager->initialize(cachedAssetManager);
            break;
        case 4:
            if (movementSoundManager) movementSoundManager->initialize(cachedAssetManager);
            break;
        case 5:
            if (questMarkerRenderer) questMarkerRenderer->initialize(vkCtx, perFrameSetLayout, cachedAssetManager);
            break;
        default:
            deferredWorldInitPending_ = false;
            return;
    }

    deferredWorldInitStage_++;
    deferredWorldInitCooldown_ = 0.12f;
}

// ============================================================
// Selection Circle
// ============================================================

void Renderer::initSelectionCircle() {
    if (selCirclePipeline != VK_NULL_HANDLE) return;
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();

    // Load shaders
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/selection_circle.vert.spv")) {
        LOG_ERROR("initSelectionCircle: failed to load vertex shader");
        return;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/selection_circle.frag.spv")) {
        LOG_ERROR("initSelectionCircle: failed to load fragment shader");
        vertShader.destroy();
        return;
    }

    // Pipeline layout: push constants only (mat4 mvp=64 + vec4 color=16), VERTEX|FRAGMENT
    VkPushConstantRange pcRange{};
    pcRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pcRange.offset = 0;
    pcRange.size = 80;
    selCirclePipelineLayout = createPipelineLayout(device, {}, {pcRange});

    // Vertex input: binding 0, stride 12, vec3 at location 0
    VkVertexInputBindingDescription vertBind{0, 12, VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription vertAttr{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0};

    // Build disc geometry as TRIANGLE_LIST (replaces GL_TRIANGLE_FAN)
    // N=48 segments: center at origin + ring verts
    constexpr int SEGMENTS = 48;
    std::vector<float> verts;
    verts.reserve((SEGMENTS + 1) * 3);
    // Center vertex
    verts.insert(verts.end(), {0.0f, 0.0f, 0.0f});
    // Ring vertices
    for (int i = 0; i <= SEGMENTS; ++i) {
        float angle = 2.0f * 3.14159265f * static_cast<float>(i) / static_cast<float>(SEGMENTS);
        verts.push_back(std::cos(angle));
        verts.push_back(std::sin(angle));
        verts.push_back(0.0f);
    }

    // Build TRIANGLE_LIST indices: N triangles (center=0, ring[i]=i+1, ring[i+1]=i+2)
    std::vector<uint16_t> indices;
    indices.reserve(SEGMENTS * 3);
    for (int i = 0; i < SEGMENTS; ++i) {
        indices.push_back(0);
        indices.push_back(static_cast<uint16_t>(i + 1));
        indices.push_back(static_cast<uint16_t>(i + 2));
    }
    selCircleVertCount = SEGMENTS * 3; // index count for drawing

    // Upload vertex buffer
    AllocatedBuffer vbuf = uploadBuffer(*vkCtx, verts.data(),
        verts.size() * sizeof(float), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    selCircleVertBuf = vbuf.buffer;
    selCircleVertAlloc = vbuf.allocation;

    // Upload index buffer
    AllocatedBuffer ibuf = uploadBuffer(*vkCtx, indices.data(),
        indices.size() * sizeof(uint16_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    selCircleIdxBuf = ibuf.buffer;
    selCircleIdxAlloc = ibuf.allocation;

    // Build pipeline: alpha blend, no depth write/test, TRIANGLE_LIST, CULL_NONE
    selCirclePipeline = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({vertBind}, {vertAttr})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(selCirclePipelineLayout)
        .setRenderPass(vkCtx->getImGuiRenderPass())
        .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
        .build(device);

    vertShader.destroy();
    fragShader.destroy();

    if (!selCirclePipeline) {
        LOG_ERROR("initSelectionCircle: failed to build pipeline");
    }
}

void Renderer::setSelectionCircle(const glm::vec3& pos, float radius, const glm::vec3& color) {
    selCirclePos = pos;
    selCircleRadius = radius;
    selCircleColor = color;
    selCircleVisible = true;
}

void Renderer::clearSelectionCircle() {
    selCircleVisible = false;
}

void Renderer::renderSelectionCircle(const glm::mat4& view, const glm::mat4& projection) {
    if (!selCircleVisible) return;
    initSelectionCircle();
    if (selCirclePipeline == VK_NULL_HANDLE || currentCmd == VK_NULL_HANDLE) return;

    // Keep circle anchored near target foot Z. Accept nearby floor probes only,
    // so distant upper/lower WMO planes don't yank the ring away from feet.
    const float baseZ = selCirclePos.z;
    float floorZ = baseZ;
    auto considerFloor = [&](std::optional<float> sample) {
        if (!sample) return;
        const float h = *sample;
        // Ignore unrelated floors/ceilings far from target feet.
        if (h < baseZ - 1.25f || h > baseZ + 0.85f) return;
        floorZ = std::max(floorZ, h);
    };

    if (terrainManager) {
        considerFloor(terrainManager->getHeightAt(selCirclePos.x, selCirclePos.y));
    }
    if (wmoRenderer) {
        considerFloor(wmoRenderer->getFloorHeight(selCirclePos.x, selCirclePos.y, selCirclePos.z + 3.0f));
    }
    if (m2Renderer) {
        considerFloor(m2Renderer->getFloorHeight(selCirclePos.x, selCirclePos.y, selCirclePos.z + 2.0f));
    }

    glm::vec3 raisedPos = selCirclePos;
    raisedPos.z = floorZ + 0.17f;
    glm::mat4 model = glm::translate(glm::mat4(1.0f), raisedPos);
    model = glm::scale(model, glm::vec3(selCircleRadius));

    glm::mat4 mvp = projection * view * model;
    glm::vec4 color4(selCircleColor, 1.0f);

    vkCmdBindPipeline(currentCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, selCirclePipeline);
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(currentCmd, 0, 1, &selCircleVertBuf, &offset);
    vkCmdBindIndexBuffer(currentCmd, selCircleIdxBuf, 0, VK_INDEX_TYPE_UINT16);
    // Push mvp (64 bytes) at offset 0
    vkCmdPushConstants(currentCmd, selCirclePipelineLayout,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, 64, &mvp[0][0]);
    // Push color (16 bytes) at offset 64
    vkCmdPushConstants(currentCmd, selCirclePipelineLayout,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        64, 16, &color4[0]);
    vkCmdDrawIndexed(currentCmd, static_cast<uint32_t>(selCircleVertCount), 1, 0, 0, 0);
}

// ──────────────────────────────────────────────────────────────
// Fullscreen overlay pipeline (underwater tint, etc.)
// ──────────────────────────────────────────────────────────────

void Renderer::initOverlayPipeline() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();

    // Push constant: vec4 color (16 bytes), visible to both stages
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pc.offset = 0;
    pc.size = 16;

    VkPipelineLayoutCreateInfo plCI{};
    plCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plCI.pushConstantRangeCount = 1;
    plCI.pPushConstantRanges = &pc;
    vkCreatePipelineLayout(device, &plCI, nullptr, &overlayPipelineLayout);

    VkShaderModule vertMod, fragMod;
    if (!vertMod.loadFromFile(device, "assets/shaders/postprocess.vert.spv") ||
        !fragMod.loadFromFile(device, "assets/shaders/overlay.frag.spv")) {
        LOG_ERROR("Renderer: failed to load overlay shaders");
        vertMod.destroy(); fragMod.destroy();
        return;
    }

    overlayPipeline = PipelineBuilder()
        .setShaders(vertMod.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragMod.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({}, {})                             // fullscreen triangle, no VBOs
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(overlayPipelineLayout)
        .setRenderPass(vkCtx->getImGuiRenderPass())
        .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
        .build(device);

    vertMod.destroy(); fragMod.destroy();

    if (overlayPipeline) LOG_INFO("Renderer: overlay pipeline initialized");
}

void Renderer::renderOverlay(const glm::vec4& color) {
    if (!overlayPipeline) initOverlayPipeline();
    if (!overlayPipeline || currentCmd == VK_NULL_HANDLE) return;
    vkCmdBindPipeline(currentCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, overlayPipeline);
    vkCmdPushConstants(currentCmd, overlayPipelineLayout,
                       VK_SHADER_STAGE_FRAGMENT_BIT, 0, 16, &color[0]);
    vkCmdDraw(currentCmd, 3, 1, 0, 0); // fullscreen triangle
}

void Renderer::renderWorld(game::World* world, game::GameHandler* gameHandler) {
    (void)world;

    auto renderStart = std::chrono::steady_clock::now();
    lastTerrainRenderMs = 0.0;
    lastWMORenderMs = 0.0;
    lastM2RenderMs = 0.0;

    uint32_t frameIdx = vkCtx->getCurrentFrame();
    VkDescriptorSet perFrameSet = perFrameDescSets[frameIdx];
    const glm::mat4& view = camera ? camera->getViewMatrix() : glm::mat4(1.0f);
    const glm::mat4& projection = camera ? camera->getProjectionMatrix() : glm::mat4(1.0f);

    // Get time of day for sky-related rendering
    float timeOfDay = (skySystem && skySystem->getSkybox()) ? skySystem->getSkybox()->getTimeOfDay() : 12.0f;

    // Render sky system (unified coordinator for skybox, stars, celestial, clouds, lens flare)
    if (skySystem && camera) {
        rendering::SkyParams skyParams;
        skyParams.timeOfDay = timeOfDay;
        skyParams.gameTime = gameHandler ? gameHandler->getGameTime() : -1.0f;

        if (lightingManager) {
            const auto& lighting = lightingManager->getLightingParams();
            skyParams.directionalDir = lighting.directionalDir;
            skyParams.sunColor = lighting.diffuseColor;
            skyParams.skyTopColor = lighting.skyTopColor;
            skyParams.skyMiddleColor = lighting.skyMiddleColor;
            skyParams.skyBand1Color = lighting.skyBand1Color;
            skyParams.skyBand2Color = lighting.skyBand2Color;
            skyParams.cloudDensity = lighting.cloudDensity;
            skyParams.fogDensity = lighting.fogDensity;
            skyParams.horizonGlow = lighting.horizonGlow;
        }

        // Weather attenuation for lens flare
        if (gameHandler) {
            skyParams.weatherIntensity = gameHandler->getWeatherIntensity();
        }

        skyParams.skyboxModelId = 0;
        skyParams.skyboxHasStars = false;

        skySystem->render(currentCmd, perFrameSet, *camera, skyParams);
    }

    // Terrain (opaque pass)
    if (terrainRenderer && camera && terrainEnabled) {
        auto terrainStart = std::chrono::steady_clock::now();
        terrainRenderer->render(currentCmd, perFrameSet, *camera);
        lastTerrainRenderMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - terrainStart).count();
    }

    // WMO buildings (opaque, drawn before characters so selection circle sits on top)
    if (wmoRenderer && camera) {
        auto wmoStart = std::chrono::steady_clock::now();
        wmoRenderer->render(currentCmd, perFrameSet, *camera);
        lastWMORenderMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - wmoStart).count();
    }

    // Selection circle (drawn after WMO, before characters)
    renderSelectionCircle(view, projection);

    // Characters (after selection circle so units draw over the ring)
    if (characterRenderer && camera) {
        characterRenderer->render(currentCmd, perFrameSet, *camera);
    }

    // M2 doodads, creatures, glow sprites, particles
    if (m2Renderer && camera) {
        if (cameraController) {
            m2Renderer->setInsideInterior(cameraController->isInsideWMO());
            m2Renderer->setOnTaxi(cameraController->isOnTaxi());
        }
        auto m2Start = std::chrono::steady_clock::now();
        m2Renderer->render(currentCmd, perFrameSet, *camera);
        m2Renderer->renderSmokeParticles(currentCmd, perFrameSet);
        m2Renderer->renderM2Particles(currentCmd, perFrameSet);
        lastM2RenderMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - m2Start).count();
    }

    // Water (transparent, after all opaques)
    // When MSAA is on and 1x pass is available, water renders after main pass ends
    bool waterDeferred = waterRenderer && waterRenderer->hasWater1xPass()
                         && vkCtx->getMsaaSamples() != VK_SAMPLE_COUNT_1_BIT;
    if (waterRenderer && camera && !waterDeferred) {
        waterRenderer->render(currentCmd, perFrameSet, *camera, globalTime);
    }

    // Weather particles
    if (weather && camera) {
        weather->render(currentCmd, perFrameSet);
    }

    // Swim effects (ripples, bubbles)
    if (swimEffects && camera) {
        swimEffects->render(currentCmd, perFrameSet);
    }

    // Mount dust
    if (mountDust && camera) {
        mountDust->render(currentCmd, perFrameSet);
    }

    // Charge effect
    if (chargeEffect && camera) {
        chargeEffect->render(currentCmd, perFrameSet);
    }

    // Quest markers (billboards above NPCs)
    if (questMarkerRenderer && camera) {
        questMarkerRenderer->render(currentCmd, perFrameSet, *camera);
    }

    // Underwater blue fog overlay — only for terrain water, not WMO water.
    if (overlayPipeline && waterRenderer && camera) {
        glm::vec3 camPos = camera->getPosition();
        auto waterH = waterRenderer->getNearestWaterHeightAt(camPos.x, camPos.y, camPos.z);
        constexpr float MIN_SUBMERSION_OVERLAY = 1.5f;
        if (waterH && camPos.z < (*waterH - MIN_SUBMERSION_OVERLAY)
                   && !waterRenderer->isWmoWaterAt(camPos.x, camPos.y)) {
            float depth = *waterH - camPos.z - MIN_SUBMERSION_OVERLAY;

            // Check for canal (liquid type 5, 13, 17) — denser/darker fog
            bool canal = false;
            if (auto lt = waterRenderer->getWaterTypeAt(camPos.x, camPos.y))
                canal = (*lt == 5 || *lt == 13 || *lt == 17);

            // Fog opacity increases with depth: thin at surface, thick deep down
            float fogStrength = 1.0f - std::exp(-depth * (canal ? 0.25f : 0.12f));
            fogStrength = glm::clamp(fogStrength, 0.0f, 0.75f);

            glm::vec4 tint = canal
                ? glm::vec4(0.01f, 0.04f, 0.10f, fogStrength)
                : glm::vec4(0.03f, 0.09f, 0.18f, fogStrength);
            renderOverlay(tint);
        }
    }

    // Minimap overlay
    if (minimap && minimap->isEnabled() && camera && window) {
        glm::vec3 minimapCenter = camera->getPosition();
        if (cameraController && cameraController->isThirdPerson())
            minimapCenter = characterPosition;
        float minimapPlayerOrientation = 0.0f;
        bool hasMinimapPlayerOrientation = false;
        if (cameraController) {
            // Use the same yaw that drives character model rendering so minimap
            // orientation cannot drift by a different axis/sign convention.
            float facingRad = glm::radians(characterYaw);
            glm::vec3 facingFwd(std::cos(facingRad), std::sin(facingRad), 0.0f);
            minimapPlayerOrientation = std::atan2(-facingFwd.x, facingFwd.y);
            hasMinimapPlayerOrientation = true;
        } else if (gameHandler) {
            minimapPlayerOrientation = gameHandler->getMovementInfo().orientation;
            hasMinimapPlayerOrientation = true;
        }
        minimap->render(currentCmd, *camera, minimapCenter,
                        window->getWidth(), window->getHeight(),
                        minimapPlayerOrientation, hasMinimapPlayerOrientation);
    }

    auto renderEnd = std::chrono::steady_clock::now();
    lastRenderMs = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();

}

// initPostProcess(), resizePostProcess(), shutdownPostProcess() removed —
// post-process pipeline is now handled by Vulkan (Phase 6 cleanup).

bool Renderer::loadTestTerrain(pipeline::AssetManager* assetManager, const std::string& adtPath) {
    if (!assetManager) {
        LOG_ERROR("Asset manager is null");
        return false;
    }

    LOG_INFO("Loading test terrain: ", adtPath);

    // Create terrain renderer if not already created
    if (!terrainRenderer) {
        terrainRenderer = std::make_unique<TerrainRenderer>();
        if (!terrainRenderer->initialize(vkCtx, perFrameSetLayout, assetManager)) {
            LOG_ERROR("Failed to initialize terrain renderer");
            terrainRenderer.reset();
            return false;
        }
    }

    // Create water renderer if not already created
    if (!waterRenderer) {
        waterRenderer = std::make_unique<WaterRenderer>();
        if (!waterRenderer->initialize(vkCtx, perFrameSetLayout)) {
            LOG_ERROR("Failed to initialize water renderer");
            waterRenderer.reset();
        } else if (vkCtx->getMsaaSamples() != VK_SAMPLE_COUNT_1_BIT) {
            setupWater1xPass();
        }
    }

    // Create minimap if not already created
    if (!minimap) {
        minimap = std::make_unique<Minimap>();
        if (!minimap->initialize(vkCtx, perFrameSetLayout)) {
            LOG_ERROR("Failed to initialize minimap");
            minimap.reset();
        }
    }

    // Create world map if not already created
    if (!worldMap) {
        worldMap = std::make_unique<WorldMap>();
        if (!worldMap->initialize(vkCtx, assetManager)) {
            LOG_ERROR("Failed to initialize world map");
            worldMap.reset();
        }
    }

    // Create M2, WMO, and Character renderers
    if (!m2Renderer) {
        m2Renderer = std::make_unique<M2Renderer>();
        m2Renderer->initialize(vkCtx, perFrameSetLayout, assetManager);
        if (swimEffects) {
            swimEffects->setM2Renderer(m2Renderer.get());
        }
    }
    if (!wmoRenderer) {
        wmoRenderer = std::make_unique<WMORenderer>();
        wmoRenderer->initialize(vkCtx, perFrameSetLayout, assetManager);
    }

    // Initialize shadow pipelines (Phase 7/8)
    if (wmoRenderer && shadowRenderPass != VK_NULL_HANDLE) {
        wmoRenderer->initializeShadow(shadowRenderPass);
    }
    if (m2Renderer && shadowRenderPass != VK_NULL_HANDLE) {
        m2Renderer->initializeShadow(shadowRenderPass);
    }
    if (!characterRenderer) {
        characterRenderer = std::make_unique<CharacterRenderer>();
        characterRenderer->initialize(vkCtx, perFrameSetLayout, assetManager);
    }
    if (characterRenderer && shadowRenderPass != VK_NULL_HANDLE) {
        characterRenderer->initializeShadow(shadowRenderPass);
    }

    // Create and initialize terrain manager
    if (!terrainManager) {
        terrainManager = std::make_unique<TerrainManager>();
        if (!terrainManager->initialize(assetManager, terrainRenderer.get())) {
            LOG_ERROR("Failed to initialize terrain manager");
            terrainManager.reset();
            return false;
        }
        // Set water renderer for terrain streaming
        if (waterRenderer) {
            terrainManager->setWaterRenderer(waterRenderer.get());
        }
        // Set M2 renderer for doodad loading during streaming
        if (m2Renderer) {
            terrainManager->setM2Renderer(m2Renderer.get());
        }
        // Set WMO renderer for building loading during streaming
        if (wmoRenderer) {
            terrainManager->setWMORenderer(wmoRenderer.get());
        }
        // Set ambient sound manager for environmental audio emitters
        if (ambientSoundManager) {
            terrainManager->setAmbientSoundManager(ambientSoundManager.get());
        }
        // Pass asset manager to character renderer for texture loading
        if (characterRenderer) {
            characterRenderer->setAssetManager(assetManager);
        }
        // Wire asset manager to minimap for tile texture loading
        if (minimap) {
            minimap->setAssetManager(assetManager);
        }
        // Wire terrain manager, WMO renderer, and water renderer to camera controller
        if (cameraController) {
            cameraController->setTerrainManager(terrainManager.get());
            if (wmoRenderer) {
                cameraController->setWMORenderer(wmoRenderer.get());
            }
            if (m2Renderer) {
                cameraController->setM2Renderer(m2Renderer.get());
            }
            if (waterRenderer) {
                cameraController->setWaterRenderer(waterRenderer.get());
            }
        }
    }

    // Parse tile coordinates from ADT path
    // Format: World\Maps\{MapName}\{MapName}_{X}_{Y}.adt
    int tileX = 32, tileY = 49;  // defaults
    {
        // Find last path separator
        size_t lastSep = adtPath.find_last_of("\\/");
        if (lastSep != std::string::npos) {
            std::string filename = adtPath.substr(lastSep + 1);
            // Find first underscore after map name
            size_t firstUnderscore = filename.find('_');
            if (firstUnderscore != std::string::npos) {
                size_t secondUnderscore = filename.find('_', firstUnderscore + 1);
                if (secondUnderscore != std::string::npos) {
                    size_t dot = filename.find('.', secondUnderscore);
                    if (dot != std::string::npos) {
                        tileX = std::stoi(filename.substr(firstUnderscore + 1, secondUnderscore - firstUnderscore - 1));
                        tileY = std::stoi(filename.substr(secondUnderscore + 1, dot - secondUnderscore - 1));
                    }
                }
            }
            // Extract map name
            std::string mapName = filename.substr(0, firstUnderscore != std::string::npos ? firstUnderscore : filename.size());
            terrainManager->setMapName(mapName);
            if (minimap) {
                minimap->setMapName(mapName);
            }
            if (worldMap) {
                worldMap->setMapName(mapName);
            }
        }
    }

    LOG_INFO("Enqueuing initial tile [", tileX, ",", tileY, "] via terrain manager");

    // Enqueue the initial tile for async loading (avoids long sync stalls)
    if (!terrainManager->enqueueTile(tileX, tileY)) {
        LOG_ERROR("Failed to enqueue initial tile [", tileX, ",", tileY, "]");
        return false;
    }

    terrainLoaded = true;

    // Initialize music manager with asset manager
    if (musicManager && assetManager && !cachedAssetManager) {
        audio::AudioEngine::instance().setAssetManager(assetManager);
        musicManager->initialize(assetManager);
        if (footstepManager) {
            footstepManager->initialize(assetManager);
        }
        if (activitySoundManager) {
            activitySoundManager->initialize(assetManager);
        }
        if (mountSoundManager) {
            mountSoundManager->initialize(assetManager);
        }
        if (npcVoiceManager) {
            npcVoiceManager->initialize(assetManager);
        }
        if (!deferredWorldInitEnabled_) {
            if (ambientSoundManager) {
                ambientSoundManager->initialize(assetManager);
            }
            if (uiSoundManager) {
                uiSoundManager->initialize(assetManager);
            }
            if (combatSoundManager) {
                combatSoundManager->initialize(assetManager);
            }
            if (spellSoundManager) {
                spellSoundManager->initialize(assetManager);
            }
            if (movementSoundManager) {
                movementSoundManager->initialize(assetManager);
            }
            if (questMarkerRenderer) {
                questMarkerRenderer->initialize(vkCtx, perFrameSetLayout, assetManager);
            }

            if (envFlagEnabled("WOWEE_PREWARM_ZONE_MUSIC", false)) {
                if (zoneManager) {
                    for (const auto& musicPath : zoneManager->getAllMusicPaths()) {
                        musicManager->preloadMusic(musicPath);
                    }
                }
                static const std::vector<std::string> tavernTracks = {
                    "Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance01.mp3",
                    "Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance02.mp3",
                    "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern1A.mp3",
                    "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern2A.mp3",
                };
                for (const auto& musicPath : tavernTracks) {
                    musicManager->preloadMusic(musicPath);
                }
            }
        } else {
            deferredWorldInitPending_ = true;
            deferredWorldInitStage_ = 0;
            deferredWorldInitCooldown_ = 0.25f;
        }

        cachedAssetManager = assetManager;
    }

    // Snap camera to ground now that terrain is loaded
    if (cameraController) {
        cameraController->reset();
    }

    LOG_INFO("Test terrain loaded successfully!");
    LOG_INFO("  Chunks: ", terrainRenderer->getChunkCount());
    LOG_INFO("  Triangles: ", terrainRenderer->getTriangleCount());

    return true;
}

void Renderer::setWireframeMode(bool enabled) {
    if (terrainRenderer) {
        terrainRenderer->setWireframe(enabled);
    }
}

bool Renderer::loadTerrainArea(const std::string& mapName, int centerX, int centerY, int radius) {
    // Create terrain renderer if not already created
    if (!terrainRenderer) {
        LOG_ERROR("Terrain renderer not initialized");
        return false;
    }

    // Create terrain manager if not already created
    if (!terrainManager) {
        terrainManager = std::make_unique<TerrainManager>();
        // Wire terrain manager to camera controller for grounding
        if (cameraController) {
            cameraController->setTerrainManager(terrainManager.get());
        }
    }

    LOG_INFO("Loading terrain area: ", mapName, " [", centerX, ",", centerY, "] radius=", radius);

    terrainManager->setMapName(mapName);
    terrainManager->setLoadRadius(radius);
    terrainManager->setUnloadRadius(radius + 1);

    // Load tiles in radius
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            int tileX = centerX + dx;
            int tileY = centerY + dy;

            if (tileX >= 0 && tileX <= 63 && tileY >= 0 && tileY <= 63) {
                terrainManager->loadTile(tileX, tileY);
            }
        }
    }

    terrainLoaded = true;

    // Get asset manager from Application if not cached yet
    if (!cachedAssetManager) {
        cachedAssetManager = core::Application::getInstance().getAssetManager();
    }

    // Initialize music manager with asset manager
    if (musicManager && cachedAssetManager) {
        if (!musicManager->isInitialized()) {
            musicManager->initialize(cachedAssetManager);
        }
    }
    if (footstepManager && cachedAssetManager) {
        if (!footstepManager->isInitialized()) {
            footstepManager->initialize(cachedAssetManager);
        }
    }
    if (activitySoundManager && cachedAssetManager) {
        if (!activitySoundManager->isInitialized()) {
            activitySoundManager->initialize(cachedAssetManager);
        }
    }
    if (mountSoundManager && cachedAssetManager) {
        mountSoundManager->initialize(cachedAssetManager);
    }
    if (npcVoiceManager && cachedAssetManager) {
        npcVoiceManager->initialize(cachedAssetManager);
    }
    if (!deferredWorldInitEnabled_) {
        if (ambientSoundManager && cachedAssetManager) {
            ambientSoundManager->initialize(cachedAssetManager);
        }
        if (uiSoundManager && cachedAssetManager) {
            uiSoundManager->initialize(cachedAssetManager);
        }
        if (combatSoundManager && cachedAssetManager) {
            combatSoundManager->initialize(cachedAssetManager);
        }
        if (spellSoundManager && cachedAssetManager) {
            spellSoundManager->initialize(cachedAssetManager);
        }
        if (movementSoundManager && cachedAssetManager) {
            movementSoundManager->initialize(cachedAssetManager);
        }
        if (questMarkerRenderer && cachedAssetManager) {
            questMarkerRenderer->initialize(vkCtx, perFrameSetLayout, cachedAssetManager);
        }
    } else {
        deferredWorldInitPending_ = true;
        deferredWorldInitStage_ = 0;
        deferredWorldInitCooldown_ = 0.1f;
    }

    // Wire ambient sound manager to terrain manager for emitter registration
    if (terrainManager && ambientSoundManager) {
        terrainManager->setAmbientSoundManager(ambientSoundManager.get());
    }

    // Wire WMO, M2, and water renderer to camera controller
    if (cameraController && wmoRenderer) {
        cameraController->setWMORenderer(wmoRenderer.get());
    }
    if (cameraController && m2Renderer) {
        cameraController->setM2Renderer(m2Renderer.get());
    }
    if (cameraController && waterRenderer) {
        cameraController->setWaterRenderer(waterRenderer.get());
    }

    // Snap camera to ground now that terrain is loaded
    if (cameraController) {
        cameraController->reset();
    }

    LOG_INFO("Terrain area loaded: ", terrainManager->getLoadedTileCount(), " tiles");

    return true;
}

void Renderer::setTerrainStreaming(bool enabled) {
    if (terrainManager) {
        terrainManager->setStreamingEnabled(enabled);
        LOG_INFO("Terrain streaming: ", enabled ? "ON" : "OFF");
    }
}

void Renderer::renderHUD() {
    if (performanceHUD && camera) {
        performanceHUD->render(this, camera.get());
    }
}

// ──────────────────────────────────────────────────────
// Shadow mapping helpers
// ──────────────────────────────────────────────────────

// initShadowMap() and compileShadowShader() removed — shadow resources now created
// in createPerFrameResources() as part of the Vulkan shadow infrastructure.

glm::mat4 Renderer::computeLightSpaceMatrix() {
    constexpr float kShadowHalfExtent = 180.0f;
    constexpr float kShadowLightDistance = 280.0f;
    constexpr float kShadowNearPlane = 1.0f;
    constexpr float kShadowFarPlane = 600.0f;

    // Use active lighting direction so shadow projection matches main shading.
    // Fragment shaders derive lighting with `ldir = normalize(-lightDir.xyz)`,
    // therefore shadow rays must use -directionalDir to stay aligned.
    glm::vec3 sunDir = glm::normalize(glm::vec3(-0.3f, -0.7f, -0.6f));
    if (lightingManager) {
        const auto& lighting = lightingManager->getLightingParams();
        if (glm::length(lighting.directionalDir) > 0.001f) {
            sunDir = glm::normalize(-lighting.directionalDir);
        }
    }
    // Shadow camera expects light rays pointing downward in render space (Z up).
    // Some profiles/opcode paths provide the opposite convention; normalize here.
    if (sunDir.z > 0.0f) {
        sunDir = -sunDir;
    }
    // Keep a minimum downward component so the frustum doesn't collapse at grazing angles.
    if (sunDir.z > -0.08f) {
        sunDir.z = -0.08f;
        sunDir = glm::normalize(sunDir);
    }

    // Shadow center follows the player directly; texel snapping below
    // prevents shimmer without needing to freeze the projection.
    glm::vec3 desiredCenter = characterPosition;
    if (!shadowCenterInitialized) {
        if (glm::dot(desiredCenter, desiredCenter) < 1.0f) {
            return glm::mat4(0.0f);
        }
        shadowCenterInitialized = true;
    }
    shadowCenter = desiredCenter;
    glm::vec3 center = shadowCenter;

    // Snap to shadow texel grid to keep projection stable while moving.
    float halfExtent = kShadowHalfExtent;
    float texelWorld = (2.0f * halfExtent) / static_cast<float>(SHADOW_MAP_SIZE);

    // Build light view to get stable axes
    glm::vec3 up(0.0f, 0.0f, 1.0f);
    // If sunDir is nearly parallel to up, pick a different up vector
    if (std::abs(glm::dot(sunDir, up)) > 0.99f) {
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    glm::mat4 lightView = glm::lookAt(center - sunDir * kShadowLightDistance, center, up);

    // Stable texel snapping in light space removes movement shimmer.
    glm::vec4 centerLS = lightView * glm::vec4(center, 1.0f);
    centerLS.x = std::round(centerLS.x / texelWorld) * texelWorld;
    centerLS.y = std::round(centerLS.y / texelWorld) * texelWorld;
    glm::vec4 snappedCenter = glm::inverse(lightView) * centerLS;
    center = glm::vec3(snappedCenter);
    shadowCenter = center;
    lightView = glm::lookAt(center - sunDir * kShadowLightDistance, center, up);

    glm::mat4 lightProj = glm::ortho(-halfExtent, halfExtent, -halfExtent, halfExtent,
                                     kShadowNearPlane, kShadowFarPlane);
    lightProj[1][1] *= -1.0f; // Vulkan Y-flip for shadow pass

    return lightProj * lightView;
}

void Renderer::setupWater1xPass() {
    if (!waterRenderer || !vkCtx) return;
    VkImageView depthView = vkCtx->getDepthResolveImageView();
    if (!depthView) {
        LOG_WARNING("No depth resolve image available - cannot create 1x water pass");
        return;
    }

    waterRenderer->createWater1xPass(vkCtx->getSwapchainFormat(), vkCtx->getDepthFormat());
    waterRenderer->createWater1xFramebuffers(
        vkCtx->getSwapchainImageViews(), depthView, vkCtx->getSwapchainExtent());
}

void Renderer::renderReflectionPass() {
    if (!waterRenderer || !camera || !waterRenderer->hasReflectionPass() || !waterRenderer->hasSurfaces()) return;
    if (currentCmd == VK_NULL_HANDLE || !reflPerFrameUBOMapped) return;

    // Reflection pass uses 1x MSAA. Scene pipelines must be render-pass-compatible,
    // which requires matching sample counts. Only render scene into reflection when MSAA is off.
    bool canRenderScene = (vkCtx->getMsaaSamples() == VK_SAMPLE_COUNT_1_BIT);

    // Find dominant water height near camera
    auto waterH = waterRenderer->getDominantWaterHeight(camera->getPosition());
    if (!waterH) return;

    float waterHeight = *waterH;

    // Skip reflection if camera is underwater (Z is up)
    if (camera->getPosition().z < waterHeight + 0.5f) return;

    // Compute reflected view and oblique projection
    glm::mat4 reflView = WaterRenderer::computeReflectedView(*camera, waterHeight);
    glm::mat4 reflProj = WaterRenderer::computeObliqueProjection(
        camera->getProjectionMatrix(), reflView, waterHeight);

    // Update water renderer's reflection UBO with the reflected viewProj
    waterRenderer->updateReflectionUBO(reflProj * reflView);

    // Fill the reflection per-frame UBO (same as normal but with reflected matrices)
    GPUPerFrameData reflData = currentFrameData;
    reflData.view = reflView;
    reflData.projection = reflProj;
    // Reflected camera position (Z is up)
    glm::vec3 reflPos = camera->getPosition();
    reflPos.z = 2.0f * waterHeight - reflPos.z;
    reflData.viewPos = glm::vec4(reflPos, 1.0f);
    std::memcpy(reflPerFrameUBOMapped, &reflData, sizeof(GPUPerFrameData));

    // Begin reflection render pass (clears to black; scene rendered if pipeline-compatible)
    if (!waterRenderer->beginReflectionPass(currentCmd)) return;

    if (canRenderScene) {
        // Render scene into reflection texture (sky + terrain + WMO only for perf)
        if (skySystem) {
            rendering::SkyParams skyParams;
            skyParams.timeOfDay = (skySystem->getSkybox()) ? skySystem->getSkybox()->getTimeOfDay() : 12.0f;
            if (lightingManager) {
                const auto& lp = lightingManager->getLightingParams();
                skyParams.directionalDir = lp.directionalDir;
                skyParams.sunColor = lp.diffuseColor;
                skyParams.skyTopColor = lp.skyTopColor;
                skyParams.skyMiddleColor = lp.skyMiddleColor;
                skyParams.skyBand1Color = lp.skyBand1Color;
                skyParams.skyBand2Color = lp.skyBand2Color;
                skyParams.cloudDensity = lp.cloudDensity;
                skyParams.fogDensity = lp.fogDensity;
                skyParams.horizonGlow = lp.horizonGlow;
            }
            // weatherIntensity left at default 0 for reflection pass (no game handler in scope)
            skySystem->render(currentCmd, reflPerFrameDescSet, *camera, skyParams);
        }
        if (terrainRenderer && terrainEnabled) {
            terrainRenderer->render(currentCmd, reflPerFrameDescSet, *camera);
        }
        if (wmoRenderer) {
            wmoRenderer->render(currentCmd, reflPerFrameDescSet, *camera);
        }
    }

    waterRenderer->endReflectionPass(currentCmd);
}

void Renderer::renderShadowPass() {
    if (!shadowsEnabled || shadowDepthImage == VK_NULL_HANDLE) return;
    if (currentCmd == VK_NULL_HANDLE) return;

    const int baseInterval = std::max(1, envIntOrDefault("WOWEE_SHADOW_INTERVAL", 1));
    const int denseInterval = std::max(baseInterval, envIntOrDefault("WOWEE_SHADOW_INTERVAL_DENSE", 3));
    const uint32_t denseCharThreshold = static_cast<uint32_t>(std::max(1, envIntOrDefault("WOWEE_DENSE_CHAR_THRESHOLD", 120)));
    const uint32_t denseM2Threshold = static_cast<uint32_t>(std::max(1, envIntOrDefault("WOWEE_DENSE_M2_THRESHOLD", 900)));
    const bool denseScene =
        (characterRenderer && characterRenderer->getInstanceCount() >= denseCharThreshold) ||
        (m2Renderer && m2Renderer->getInstanceCount() >= denseM2Threshold);
    const int shadowInterval = denseScene ? denseInterval : baseInterval;
    if (++shadowFrameCounter_ < static_cast<uint32_t>(shadowInterval)) {
        return;
    }
    shadowFrameCounter_ = 0;

    // Compute and store light space matrix; write to per-frame UBO
    lightSpaceMatrix = computeLightSpaceMatrix();
    // Zero matrix means character position isn't set yet — skip shadow pass entirely.
    if (lightSpaceMatrix == glm::mat4(0.0f)) return;
    uint32_t frame = vkCtx->getCurrentFrame();
    auto* ubo = reinterpret_cast<GPUPerFrameData*>(perFrameUBOMapped[frame]);
    if (ubo) {
        ubo->lightSpaceMatrix = lightSpaceMatrix;
        ubo->shadowParams.x = shadowsEnabled ? 1.0f : 0.0f;
        ubo->shadowParams.y = 0.8f;
    }

    // Barrier 1: transition shadow map into writable depth layout.
    VkImageMemoryBarrier b1{};
    b1.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    b1.oldLayout = shadowDepthLayout_;
    b1.newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    b1.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b1.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b1.srcAccessMask = (shadowDepthLayout_ == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        ? VK_ACCESS_SHADER_READ_BIT
        : 0;
    b1.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                       VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    b1.image = shadowDepthImage;
    b1.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
    VkPipelineStageFlags srcStage = (shadowDepthLayout_ == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        ? VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT
        : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
    vkCmdPipelineBarrier(currentCmd,
        srcStage, VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
        0, 0, nullptr, 0, nullptr, 1, &b1);

    // Begin shadow render pass
    VkRenderPassBeginInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpInfo.renderPass = shadowRenderPass;
    rpInfo.framebuffer = shadowFramebuffer;
    rpInfo.renderArea = {{0, 0}, {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE}};
    VkClearValue clear{};
    clear.depthStencil = {1.0f, 0};
    rpInfo.clearValueCount = 1;
    rpInfo.pClearValues = &clear;
    vkCmdBeginRenderPass(currentCmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport vp{0, 0, static_cast<float>(SHADOW_MAP_SIZE), static_cast<float>(SHADOW_MAP_SIZE), 0.0f, 1.0f};
    vkCmdSetViewport(currentCmd, 0, 1, &vp);
    VkRect2D sc{{0, 0}, {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE}};
    vkCmdSetScissor(currentCmd, 0, 1, &sc);

    // Phase 7/8: render shadow casters
    const float baseShadowCullRadius = static_cast<float>(std::max(40, envIntOrDefault("WOWEE_SHADOW_CULL_RADIUS", 180)));
    const float denseShadowCullRadius = static_cast<float>(std::max(30, envIntOrDefault("WOWEE_SHADOW_CULL_RADIUS_DENSE", 90)));
    const float shadowCullRadius = denseScene ? std::min(baseShadowCullRadius, denseShadowCullRadius) : baseShadowCullRadius;
    if (wmoRenderer) {
        wmoRenderer->renderShadow(currentCmd, lightSpaceMatrix, shadowCenter, shadowCullRadius);
    }
    if (m2Renderer) {
        m2Renderer->renderShadow(currentCmd, lightSpaceMatrix, globalTime, shadowCenter, shadowCullRadius);
    }
    if (characterRenderer) {
        characterRenderer->renderShadow(currentCmd, lightSpaceMatrix, shadowCenter, shadowCullRadius);
    }

    vkCmdEndRenderPass(currentCmd);

    // Barrier 2: DEPTH_STENCIL_ATTACHMENT_OPTIMAL → SHADER_READ_ONLY_OPTIMAL
    VkImageMemoryBarrier b2{};
    b2.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    b2.oldLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    b2.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    b2.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b2.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b2.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    b2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    b2.image = shadowDepthImage;
    b2.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
    vkCmdPipelineBarrier(currentCmd,
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0, 0, nullptr, 0, nullptr, 1, &b2);
    shadowDepthLayout_ = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
}

} // namespace rendering
} // namespace wowee
