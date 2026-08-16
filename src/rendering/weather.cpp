#include "rendering/weather.hpp"
#include "rendering/camera.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/vk_utils.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <random>
#include <cmath>
#include <cstring>

namespace wowee {
namespace rendering {

namespace {
// Seeded RNG for weather particle positions and cycle durations.
// Replaces bare rand() which defaults to seed 1 without srand(),
// producing identical weather patterns on every launch.
std::mt19937& weatherRng() {
    static std::mt19937 gen(std::random_device{}());
    return gen;
}
float weatherRandFloat() {
    return std::uniform_real_distribution<float>(0.0f, 1.0f)(weatherRng());
}
} // namespace

Weather::Weather() {
}

Weather::~Weather() {
    shutdown();
}

/// Builds the one pipeline this effect draws with, vertex layout and all.
///
/// initialize() and recreatePipelines() described it identically, which is two
/// statements of what a weather particle vertex is in one file.
void Weather::buildPipelines(VkDevice device,
                             const VkPipelineShaderStageCreateInfo& vertStage,
                             const VkPipelineShaderStageCreateInfo& fragStage) {
    // Vertex input: position only (vec3), stride = 3 * sizeof(float)
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = 3 * sizeof(float);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription posAttr{};
    posAttr.location = 0;
    posAttr.binding = 0;
    posAttr.format = VK_FORMAT_R32G32B32_SFLOAT;
    posAttr.offset = 0;

    // Dynamic viewport and scissor
    std::vector<VkDynamicState> dynamicStates = viewportAndScissorDynamic();

    pipeline = PipelineBuilder()
        .setShaders(vertStage, fragStage)
        .setVertexInput({binding}, {posAttr})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, false, VK_COMPARE_OP_LESS)  // depth test on, write off (transparent particles)
        .setColorBlendAttachment(PipelineBuilder::blendAlpha())
        .setMultisample(vkCtx->getMsaaSamples())
        .setLayout(pipelineLayout)
        .setRenderPass(vkCtx->getImGuiRenderPass())
        .setDynamicStates(dynamicStates)
        .build(device, vkCtx->getPipelineCache());
}

bool Weather::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout) {
    LOG_INFO("Initializing weather system");

    vkCtx = ctx;
    VkDevice device = vkCtx->getDevice();

    // Load SPIR-V shaders
    auto shaders = loadShaderPair(device, "assets/shaders/weather.vert.spv", "assets/shaders/weather.frag.spv", "weather");
    if (!shaders) return false;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    // Push constant range: { float particleSize; float pad0; float pad1; float pad2; vec4 particleColor; } = 32 bytes
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = 32;  // 4 floats + vec4

    // Create pipeline layout with perFrameLayout (set 0) + push constants
    pipelineLayout = createPipelineLayout(device, {perFrameLayout}, {pushRange});
    if (pipelineLayout == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create weather pipeline layout");
        return false;
    }

    buildPipelines(device, vertStage, fragStage);


    if (pipeline == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create weather pipeline");
        return false;
    }

    // Create a dynamic mapped vertex buffer large enough for MAX_PARTICLES
    dynamicVBSize = MAX_PARTICLES * sizeof(glm::vec3);
    AllocatedBuffer buf = createBuffer(vkCtx->getAllocator(), dynamicVBSize,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
    dynamicVB = buf.buffer;
    dynamicVBAlloc = buf.allocation;
    dynamicVBAllocInfo = buf.info;

    if (dynamicVB == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create weather dynamic vertex buffer");
        return false;
    }

    // Reserve space for particles
    particles.reserve(MAX_PARTICLES);
    particlePositions.reserve(MAX_PARTICLES);

    LOG_INFO("Weather system initialized");
    return true;
}

void Weather::recreatePipelines() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();

    destroy(device, pipeline);

    auto shaders = loadShaderPair(device, "assets/shaders/weather.vert.spv", "assets/shaders/weather.frag.spv", "weather");
    if (!shaders) return;
    const auto& vertStage = shaders.vertStage;
    const auto& fragStage = shaders.fragStage;

    buildPipelines(device, vertStage, fragStage);


    if (pipeline == VK_NULL_HANDLE) {
        LOG_ERROR("Weather::recreatePipelines: failed to create pipeline");
    }
}

void Weather::update(const Camera& camera, float deltaTime) {
    if (!enabled || weatherType == Type::NONE) {
        return;
    }

    // Initialize particles if needed
    if (particles.empty()) {
        resetParticles(camera);
    }

    // Calculate active particle count based on intensity
    int targetParticleCount =
        static_cast<int>(MAX_PARTICLES * intensity * densityScale_);

    // Adjust particle count
    while (static_cast<int>(particles.size()) < targetParticleCount) {
        Particle p;
        p.position = getRandomPosition(camera.getPosition());
        p.position.y = camera.getPosition().y + SPAWN_HEIGHT;
        p.lifetime = 0.0f;

        if (weatherType == Type::RAIN) {
            p.velocity = glm::vec3(0.0f, -50.0f, 0.0f);  // Fast downward
            p.maxLifetime = 5.0f;
        } else if (weatherType == Type::STORM) {
            // Storm: faster, angled rain with wind
            p.velocity = glm::vec3(15.0f, -70.0f, 8.0f);
            p.maxLifetime = 3.5f;
        } else {  // SNOW
            p.velocity = glm::vec3(0.0f, -5.0f, 0.0f);   // Slow downward
            p.maxLifetime = 10.0f;
        }

        particles.push_back(p);
    }

    while (static_cast<int>(particles.size()) > targetParticleCount) {
        particles.pop_back();
    }

    // Combined update + position copy. Hoist camera.getPosition() out of
    // the per-particle call (each was re-reading the camera member) and
    // fold the position-copy pass into the update loop so we only walk
    // the particle vector once.
    const glm::vec3 cameraPos = camera.getPosition();
    particlePositions.clear();
    particlePositions.reserve(particles.size());
    for (auto& particle : particles) {
        updateParticle(particle, cameraPos, deltaTime);
        particlePositions.push_back(particle.position);
    }
}

void Weather::updateParticle(Particle& particle, const glm::vec3& cameraPos, float deltaTime) {
    // Update lifetime
    particle.lifetime += deltaTime;

    // Reset if lifetime exceeded or too far from camera
    glm::vec3 toCamera = particle.position - cameraPos;
    float distSq = glm::dot(toCamera, toCamera);

    if (particle.lifetime >= particle.maxLifetime || distSq > SPAWN_VOLUME_SIZE * SPAWN_VOLUME_SIZE ||
        particle.position.y < cameraPos.y - 20.0f) {
        // Respawn at top
        particle.position = getRandomPosition(cameraPos);
        particle.position.y = cameraPos.y + SPAWN_HEIGHT;
        particle.lifetime = 0.0f;
    }

    // Add wind effect for snow
    if (weatherType == Type::SNOW) {
        float windX = std::sin(particle.lifetime * 0.5f) * 2.0f;
        float windZ = std::cos(particle.lifetime * 0.3f) * 2.0f;
        particle.velocity.x = windX;
        particle.velocity.z = windZ;
    }
    // Storm: gusty, turbulent wind with varying direction
    if (weatherType == Type::STORM) {
        float gust = std::sin(particle.lifetime * 1.5f + particle.position.x * 0.1f) * 5.0f;
        particle.velocity.x = 15.0f + gust;
        particle.velocity.z = 8.0f + std::cos(particle.lifetime * 2.0f) * 3.0f;
    }

    // Update position
    particle.position += particle.velocity * deltaTime;
}

void Weather::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet) {
    if (!enabled || weatherType == Type::NONE || particlePositions.empty() ||
        pipeline == VK_NULL_HANDLE) {
        return;
    }

    // Upload particle positions to mapped buffer
    VkDeviceSize uploadSize = particlePositions.size() * sizeof(glm::vec3);
    if (uploadSize > 0 && dynamicVBAllocInfo.pMappedData) {
        std::memcpy(dynamicVBAllocInfo.pMappedData, particlePositions.data(), uploadSize);
    }

    // Push constant data: { float particleSize; float pad0; float pad1; float pad2; vec4 particleColor; }
    struct WeatherPush {
        float particleSize;
        float pad0;
        float pad1;
        float pad2;
        glm::vec4 particleColor;
    };

    WeatherPush push{};
    if (weatherType == Type::RAIN) {
        push.particleSize = 3.0f;
        push.particleColor = glm::vec4(0.7f, 0.8f, 0.9f, 0.6f);
    } else if (weatherType == Type::STORM) {
        push.particleSize = 3.5f;
        push.particleColor = glm::vec4(0.6f, 0.65f, 0.75f, 0.7f);  // Darker, more opaque
    } else {  // SNOW
        push.particleSize = 8.0f;
        push.particleColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.9f);
    }

    // Bind pipeline
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    // Bind per-frame descriptor set (set 0 - camera UBO)
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
        0, 1, &perFrameSet, 0, nullptr);

    // Push constants
    vkCmdPushConstants(cmd, pipelineLayout,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, sizeof(push), &push);

    // Bind vertex buffer
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &dynamicVB, &offset);

    // Draw particles as points
    vkCmdDraw(cmd, static_cast<uint32_t>(particlePositions.size()), 1, 0, 0);
}

void Weather::resetParticles(const Camera& camera) {
    particles.clear();

    int particleCount = static_cast<int>(MAX_PARTICLES * intensity * densityScale_);
    glm::vec3 cameraPos = camera.getPosition();

    for (int i = 0; i < particleCount; ++i) {
        Particle p;
        p.position = getRandomPosition(cameraPos);
        p.position.y = cameraPos.y + SPAWN_HEIGHT * (weatherRandFloat());
        p.lifetime = 0.0f;

        if (weatherType == Type::RAIN) {
            p.velocity = glm::vec3(0.0f, -50.0f, 0.0f);
            p.maxLifetime = 5.0f;
        } else {  // SNOW
            p.velocity = glm::vec3(0.0f, -5.0f, 0.0f);
            p.maxLifetime = 10.0f;
        }

        particles.push_back(p);
    }
}

glm::vec3 Weather::getRandomPosition(const glm::vec3& center) const {
    // Reuse the shared weather RNG to avoid duplicate generator state
    static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    float x = center.x + dist(weatherRng()) * SPAWN_VOLUME_SIZE;
    float z = center.z + dist(weatherRng()) * SPAWN_VOLUME_SIZE;
    float y = center.y;

    return glm::vec3(x, y, z);
}

void Weather::setIntensity(float intensity) {
    this->intensity = glm::clamp(intensity, 0.0f, 1.0f);
}

int Weather::getParticleCount() const {
    return static_cast<int>(particles.size());
}

void Weather::shutdown() {
    if (vkCtx) {
        destroyParticleResources(vkCtx->getDevice(), vkCtx->getAllocator(),
                                 pipeline, pipelineLayout, dynamicVB,
                                 dynamicVBAlloc);
    }

    vkCtx = nullptr;
    particles.clear();
    particlePositions.clear();
}

// ---------------------------------------------------------------------------
// Zone-based weather configuration
// ---------------------------------------------------------------------------

void Weather::setZoneWeather(uint32_t zoneId, Type type, float minIntensity, float maxIntensity, float probability) {
    zoneWeatherTable_[zoneId] = {.type = type, .minIntensity = minIntensity, .maxIntensity = maxIntensity, .probability = probability};
}

void Weather::initializeZoneWeatherDefaults() {
    if (zoneWeatherInitialized_) return;
    zoneWeatherInitialized_ = true;

    // Eastern Kingdoms zones
    // Duskwood's persistent atmosphere is supplied by its lighting fog profile.
    // Do not synthesize rain here: the streak particles read as wind-blown fog.
    // Renderer also suppresses server rain in Duskwood so its fog stays legible.
    setZoneWeather(11,   Type::RAIN, 0.1f, 0.4f, 0.15f);  // Wetlands - moderate rain
    setZoneWeather(8,    Type::RAIN, 0.1f, 0.5f, 0.2f);   // Swamp of Sorrows
    setZoneWeather(33,   Type::RAIN, 0.2f, 0.7f, 0.25f);  // Stranglethorn Vale
    setZoneWeather(44,   Type::RAIN, 0.1f, 0.3f, 0.1f);   // Redridge Mountains - light rain
    setZoneWeather(36,   Type::RAIN, 0.1f, 0.4f, 0.15f);  // Alterac Mountains
    setZoneWeather(45,   Type::RAIN, 0.1f, 0.3f, 0.1f);   // Arathi Highlands
    setZoneWeather(267,  Type::RAIN, 0.2f, 0.5f, 0.2f);   // Hillsbrad Foothills
    setZoneWeather(28,   Type::RAIN, 0.1f, 0.3f, 0.1f);   // Western Plaguelands - occasional rain
    setZoneWeather(139,  Type::RAIN, 0.1f, 0.3f, 0.1f);   // Eastern Plaguelands

    // Snowy zones
    setZoneWeather(1,    Type::SNOW, 0.2f, 0.6f, 0.3f);   // Dun Morogh
    setZoneWeather(51,   Type::SNOW, 0.1f, 0.5f, 0.2f);   // Searing Gorge (occasional)
    setZoneWeather(41,   Type::SNOW, 0.1f, 0.4f, 0.15f);  // Deadwind Pass
    setZoneWeather(2817, Type::SNOW, 0.3f, 0.7f, 0.4f);   // Crystalsong Forest
    setZoneWeather(67,   Type::SNOW, 0.2f, 0.6f, 0.35f);  // Storm Peaks
    setZoneWeather(65,   Type::SNOW, 0.2f, 0.5f, 0.3f);   // Dragonblight
    setZoneWeather(394,  Type::SNOW, 0.1f, 0.4f, 0.2f);   // Grizzly Hills
    setZoneWeather(495,  Type::SNOW, 0.3f, 0.8f, 0.5f);   // Howling Fjord
    setZoneWeather(210,  Type::SNOW, 0.2f, 0.5f, 0.25f);  // Icecrown
    setZoneWeather(3537, Type::SNOW, 0.2f, 0.6f, 0.3f);   // Borean Tundra
    setZoneWeather(4742, Type::SNOW, 0.2f, 0.5f, 0.3f);   // Hrothgar's Landing

    // Kalimdor zones
    setZoneWeather(15,   Type::RAIN, 0.1f, 0.4f, 0.15f);  // Dustwallow Marsh
    setZoneWeather(16,   Type::RAIN, 0.1f, 0.3f, 0.1f);   // Azshara
    setZoneWeather(148,  Type::RAIN, 0.1f, 0.4f, 0.15f);  // Darkshore
    setZoneWeather(331,  Type::RAIN, 0.1f, 0.3f, 0.1f);   // Ashenvale
    setZoneWeather(405,  Type::RAIN, 0.1f, 0.3f, 0.1f);   // Desolace
    setZoneWeather(490,  Type::RAIN, 0.1f, 0.4f, 0.15f);  // Un'Goro Crater
    setZoneWeather(493,  Type::RAIN, 0.1f, 0.3f, 0.1f);   // Moonglade

    // Winterspring is snowy
    setZoneWeather(618,  Type::SNOW, 0.2f, 0.6f, 0.3f);   // Winterspring

    // Outland
    setZoneWeather(3483, Type::RAIN, 0.1f, 0.3f, 0.1f);   // Hellfire Peninsula (occasional)
    setZoneWeather(3521, Type::RAIN, 0.1f, 0.4f, 0.15f);  // Zangarmarsh
    setZoneWeather(3519, Type::RAIN, 0.1f, 0.3f, 0.1f);   // Terokkar Forest
}

void Weather::updateZoneWeather(uint32_t zoneId, float deltaTime) {
    if (!zoneWeatherInitialized_) {
        initializeZoneWeatherDefaults();
    }

    // Zone changed - reset weather cycle
    if (zoneId != currentWeatherZone_) {
        currentWeatherZone_ = zoneId;
        zoneWeatherTimer_ = 0.0f;

        auto it = zoneWeatherTable_.find(zoneId);
        if (it == zoneWeatherTable_.end()) {
            // Zone has no configured weather - clear gradually
            targetIntensity_ = 0.0f;
        } else {
            // Roll whether weather is active based on probability
            float roll = weatherRandFloat();
            zoneWeatherActive_ = (roll < it->second.probability);

            if (zoneWeatherActive_) {
                weatherType = it->second.type;
                // Random intensity within configured range
                float t = weatherRandFloat();
                targetIntensity_ = glm::mix(it->second.minIntensity, it->second.maxIntensity, t);
                // Random cycle duration: 3-8 minutes
                zoneWeatherCycleDuration_ = 180.0f + weatherRandFloat() * 300.0f;
            } else {
                targetIntensity_ = 0.0f;
                zoneWeatherCycleDuration_ = 120.0f + weatherRandFloat() * 180.0f;
            }
        }
    }

    // Smooth intensity transitions
    float transitionSpeed = 0.15f * deltaTime; // ~7 seconds to full transition
    if (intensity < targetIntensity_) {
        intensity = std::min(intensity + transitionSpeed, targetIntensity_);
    } else if (intensity > targetIntensity_) {
        intensity = std::max(intensity - transitionSpeed, targetIntensity_);
    }

    // If intensity reached zero and target is zero, clear weather type
    if (intensity <= 0.01f && targetIntensity_ <= 0.01f) {
        if (weatherType != Type::NONE) {
            weatherType = Type::NONE;
            particles.clear();
        }
    }

    // Weather cycling - periodically re-roll weather
    zoneWeatherTimer_ += deltaTime;
    if (zoneWeatherTimer_ >= zoneWeatherCycleDuration_ && zoneWeatherCycleDuration_ > 0.0f) {
        zoneWeatherTimer_ = 0.0f;

        auto it = zoneWeatherTable_.find(zoneId);
        if (it != zoneWeatherTable_.end()) {
            float roll = weatherRandFloat();
            zoneWeatherActive_ = (roll < it->second.probability);

            if (zoneWeatherActive_) {
                weatherType = it->second.type;
                float t = weatherRandFloat();
                targetIntensity_ = glm::mix(it->second.minIntensity, it->second.maxIntensity, t);
            } else {
                targetIntensity_ = 0.0f;
            }

            // New cycle duration
            zoneWeatherCycleDuration_ = 180.0f + weatherRandFloat() * 300.0f;
        }
    }
}

} // namespace rendering
} // namespace wowee
