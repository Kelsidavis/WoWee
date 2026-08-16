#define VMA_IMPLEMENTATION
#include <set>
#include <thread>
#include <mutex>
#include "rendering/vk_context.hpp"

#include <fstream>
#include "core/logger.hpp"
#include <VkBootstrap.h>
#include <SDL2/SDL_vulkan.h>
#include <imgui_impl_vulkan.h>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <string>

namespace wowee {
namespace rendering {

VkContext* VkContext::sInstance_ = nullptr;

// Hash a VkSamplerCreateInfo into a 64-bit key for the sampler cache.
// FNV-1a chosen for speed and low collision rate on small structured data.
// Constants from: http://www.isthe.com/chongo/tech/comp/fnv/
static constexpr uint64_t kFnv1aOffsetBasis = 14695981039346656037ULL;
static constexpr uint64_t kFnv1aPrime       = 1099511628211ULL;

static uint64_t hashSamplerCreateInfo(const VkSamplerCreateInfo& s) {
    uint64_t h = kFnv1aOffsetBasis;
    auto mix = [&](uint64_t v) {
        h ^= v;
        h *= kFnv1aPrime;
    };
    mix(static_cast<uint64_t>(s.minFilter));
    mix(static_cast<uint64_t>(s.magFilter));
    mix(static_cast<uint64_t>(s.mipmapMode));
    mix(static_cast<uint64_t>(s.addressModeU));
    mix(static_cast<uint64_t>(s.addressModeV));
    mix(static_cast<uint64_t>(s.addressModeW));
    mix(static_cast<uint64_t>(s.anisotropyEnable));
    // Bit-cast floats to uint32_t for hashing
    uint32_t aniso;
    std::memcpy(&aniso, &s.maxAnisotropy, sizeof(aniso));
    mix(static_cast<uint64_t>(aniso));
    uint32_t maxLodBits;
    std::memcpy(&maxLodBits, &s.maxLod, sizeof(maxLodBits));
    mix(static_cast<uint64_t>(maxLodBits));
    uint32_t minLodBits;
    std::memcpy(&minLodBits, &s.minLod, sizeof(minLodBits));
    mix(static_cast<uint64_t>(minLodBits));
    mix(static_cast<uint64_t>(s.compareEnable));
    mix(static_cast<uint64_t>(s.compareOp));
    mix(static_cast<uint64_t>(s.borderColor));
    uint32_t biasBits;
    std::memcpy(&biasBits, &s.mipLodBias, sizeof(biasBits));
    mix(static_cast<uint64_t>(biasBits));
    mix(static_cast<uint64_t>(s.unnormalizedCoordinates));
    return h;
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    [[maybe_unused]] VkDebugUtilsMessageTypeFlagsEXT type,
    const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
    [[maybe_unused]] void* userData)
{
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
        LOG_ERROR("Vulkan: ", callbackData->pMessage);
    } else if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        LOG_WARNING("Vulkan: ", callbackData->pMessage);
    }
    return VK_FALSE;
}

VkContext::~VkContext() {
    shutdown();
}

bool VkContext::initialize(SDL_Window* window) {
    LOG_INFO("Initializing Vulkan context");

    if (!createInstance(window)) return false;
    if (!createSurface(window)) return false;
    if (!selectPhysicalDevice()) return false;
    if (!createLogicalDevice()) return false;
    if (!createAllocator()) return false;

    // Pipeline cache: try to load from disk, fall back to empty cache.
    // Not fatal - if it fails we just skip caching.
    createPipelineCache();

    int w, h;
    SDL_Vulkan_GetDrawableSize(window, &w, &h);
    if (!createSwapchain(w, h)) return false;

    if (!createCommandPools()) return false;
    if (!createSyncObjects()) return false;
    if (!createImGuiResources()) return false;

    // Query anisotropy support from the physical device.
    VkPhysicalDeviceFeatures supportedFeatures{};
    vkGetPhysicalDeviceFeatures(physicalDevice, &supportedFeatures);
    samplerAnisotropySupported_ = (supportedFeatures.samplerAnisotropy == VK_TRUE);
    LOG_INFO("Sampler anisotropy supported: ", samplerAnisotropySupported_ ? "YES" : "NO");

    sInstance_ = this;

    LOG_INFO("Vulkan context initialized successfully");
    return true;
}

void VkContext::shutdown() {
    if (!device && !instance) return;  // Already shut down or never initialized

    LOG_DEBUG("VkContext::shutdown - vkDeviceWaitIdle...");
    if (device) {
        vkDeviceWaitIdle(device);
    }

    // Clear deferred cleanup queues WITHOUT executing them.  By this point the
    // sub-renderers (which own the descriptor pools/buffers these lambdas
    // reference) have already been destroyed, so running them would call
    // vkFreeDescriptorSets on invalid pools.  vkDestroyDevice reclaims all
    // device-child resources anyway.
    size_t droppedCleanups = 0;
    for (uint32_t fi = 0; fi < MAX_FRAMES_IN_FLIGHT; fi++) {
        droppedCleanups += deferredCleanup_[fi].size();
        deferredCleanup_[fi].clear();
    }
    // Said out loud, because these are exactly the objects vkDestroyDevice then
    // reports as leaked, and without the count there is no way to tell that
    // report apart from a resource nobody freed at all.
    if (droppedCleanups > 0) {
        LOG_INFO("shutdown: dropped ", droppedCleanups,
                 " deferred destructions unexecuted (their pools are already gone;"
                 " vkDestroyDevice reclaims them and validation counts them as leaked)");
    }

    LOG_DEBUG("VkContext::shutdown - destroyImGuiResources...");
    destroyImGuiResources();

    // Destroy sync objects
    if (frameTimeline_) {
        vkDestroySemaphore(device, frameTimeline_, nullptr);
        frameTimeline_ = VK_NULL_HANDLE;
    }
    for (auto& frame : frames) {
        if (frame.inFlightFence) vkDestroyFence(device, frame.inFlightFence, nullptr);
        if (frame.commandPool) vkDestroyCommandPool(device, frame.commandPool, nullptr);
        frame = {};
    }
    for (auto sem : imageAcquiredSemaphores_) { if (sem) vkDestroySemaphore(device, sem, nullptr); }
    imageAcquiredSemaphores_.clear();
    for (auto sem : renderFinishedSemaphores_) { if (sem) vkDestroySemaphore(device, sem, nullptr); }
    renderFinishedSemaphores_.clear();
    if (nextAcquireSemaphore_) { vkDestroySemaphore(device, nextAcquireSemaphore_, nullptr); nextAcquireSemaphore_ = VK_NULL_HANDLE; }

    // Clean up any in-flight async upload batches. waitAllUploads does the full
    // retirement -- fence, command buffer, VMA staging and the plainly
    // allocated staging -- and the device is already idle above, so its waits
    // return at once. Both command pools it frees from are destroyed below
    // this point, and so is the allocator.
    //
    // This used to destroy only the fence, on the grounds that the allocator
    // was about to be torn down anyway. That was never true of rawStaging,
    // which is vkCreateBuffer/vkAllocateMemory and belongs to no allocator, and
    // stopped being true of the rest once shutdown began destroying the VMA
    // allocator under validation. Every batch left in flight leaked a command
    // buffer, its staging buffers and their memory, which is most of what
    // vkDestroyDevice reported.
    if (!inFlightBatches_.empty() || !batchRawStaging_.empty() || !batchStagingBuffers_.empty()) {
        size_t rawInFlight = 0, vmaInFlight = 0;
        for (const auto& b : inFlightBatches_) {
            rawInFlight += b.rawStaging.size();
            vmaInFlight += b.stagingBuffers.size();
        }
        LOG_INFO("shutdown: retiring ", inFlightBatches_.size(), " upload batches (",
                 rawInFlight, " raw + ", vmaInFlight, " pooled staging), plus ",
                 batchRawStaging_.size(), " raw + ", batchStagingBuffers_.size(),
                 " pooled in the batch still being built");
    }
    waitAllUploads();

    // The batch still being accumulated has never been submitted, so
    // waitAllUploads does not see it. Its staging belongs to this context
    // alone -- no descriptor pool, no sub-renderer -- so unlike the deferred
    // queues above there is nothing here that could already be invalid.
    for (auto& raw : batchRawStaging_) {
        vkDestroyBuffer(device, raw.buffer, nullptr);
        vkFreeMemory(device, raw.memory, nullptr);
    }
    batchRawStaging_.clear();
    for (auto& staging : batchStagingBuffers_) {
        destroyBuffer(allocator, staging);
    }
    batchStagingBuffers_.clear();

    if (immFence) { vkDestroyFence(device, immFence, nullptr); immFence = VK_NULL_HANDLE; }
    // Destroying the pool implicitly frees immCmdBuf_; just drop the handle.
    if (immCommandPool) { vkDestroyCommandPool(device, immCommandPool, nullptr); immCommandPool = VK_NULL_HANDLE; }
    immCmdBuf_ = VK_NULL_HANDLE;
    if (transferCommandPool_) { vkDestroyCommandPool(device, transferCommandPool_, nullptr); transferCommandPool_ = VK_NULL_HANDLE; }

    // Persist pipeline cache to disk before tearing down the device.
    savePipelineCache();
    if (pipelineCache_) {
        vkDestroyPipelineCache(device, pipelineCache_, nullptr);
        pipelineCache_ = VK_NULL_HANDLE;
    }

    // Destroy all cached samplers.
    for (auto& [key, sampler] : samplerCache_) {
        if (sampler) vkDestroySampler(device, sampler, nullptr);
    }
    samplerCache_.clear();
    LOG_INFO("Sampler cache cleared");

    sInstance_ = nullptr;

    LOG_DEBUG("VkContext::shutdown - destroySwapchain...");
    destroySwapchain();

    // Normally skip vmaDestroyAllocator: it walks every allocation to free it,
    // which takes many seconds with thousands of loaded textures and models. The
    // driver reclaims all device memory when the device is destroyed and the OS
    // reclaims the rest at process exit, so skipping it makes shutdown instant.
    //
    // Under validation, tear it down properly. Whatever the caches still hold is
    // otherwise reported object by object at vkDestroyDevice - ninety thousand
    // errors in one run - and that flood buries any real problem the layers find.
    // Paying a few seconds on the way out is worth a usable validation signal,
    // and it also means a genuine leak still shows up rather than hiding in the
    // noise. Players never take this path.
    if (allocator) {
        if (validationActive_) {
            // What the allocator still holds, before it is torn down. Anything
            // counted here is a vmaCreateBuffer/Image whose owner never called
            // the matching vmaDestroy, and vmaDestroyAllocator frees the memory
            // blocks without destroying those handles -- so they are what
            // vkDestroyDevice then reports.
            VmaTotalStatistics stats{};
            vmaCalculateStatistics(allocator, &stats);
            LOG_INFO("shutdown: VMA still holds ",
                     stats.total.statistics.allocationCount, " allocations in ",
                     stats.total.statistics.blockCount, " blocks (",
                     stats.total.statistics.allocationBytes / (1024 * 1024), " MB)");
            // The detailed JSON names every surviving allocation's size and
            // memory type, which is what identifies the owner -- a handful of
            // distinctive sizes is usually enough to point at one subsystem.
            if (stats.total.statistics.allocationCount > 0) {
                char* statsJson = nullptr;
                vmaBuildStatsString(allocator, &statsJson, VK_TRUE);
                if (statsJson) {
                    const auto path = std::filesystem::temp_directory_path() / "wowee-vma-leak.json";
                    if (std::ofstream out(path); out) {
                        out << statsJson;
                        LOG_INFO("shutdown: surviving VMA allocations dumped to ", path.string());
                    }
                    vmaFreeStatsString(allocator, statsJson);
                }
            }
            LOG_INFO("Validation active - destroying VMA allocator (slow, but keeps the exit clean)");
            vmaDestroyAllocator(allocator);
        }
        allocator = VK_NULL_HANDLE;
    }

    LOG_DEBUG("VkContext::shutdown - vkDestroyDevice...");
    if (device) { vkDestroyDevice(device, nullptr); device = VK_NULL_HANDLE; }
    if (surface) { vkDestroySurfaceKHR(instance, surface, nullptr); surface = VK_NULL_HANDLE; }

    if (debugMessenger) {
        auto func = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));
        if (func) func(instance, debugMessenger, nullptr);
        debugMessenger = VK_NULL_HANDLE;
    }

    if (instance) { vkDestroyInstance(instance, nullptr); instance = VK_NULL_HANDLE; }

    LOG_DEBUG("Vulkan context shutdown complete");
}

void VkContext::deferAfterFrameFence(std::function<void()>&& fn) {
    deferredCleanup_[currentFrame].push_back(std::move(fn));
}

void VkContext::deferAfterAllFrameFences(std::function<void()>&& fn) {
    // Shared resources (material descriptor sets, vertex/index buffers) are
    // bound by every in-flight frame's command buffer.  deferAfterFrameFence
    // only waits for ONE slot's fence - the other slot may still be executing.
    // Add to every slot; a shared counter ensures the lambda runs exactly once,
    // after the LAST slot has been fenced.
    auto counter  = std::make_shared<uint32_t>(MAX_FRAMES_IN_FLIGHT);
    auto sharedFn = std::make_shared<std::function<void()>>(std::move(fn));
    for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        deferredCleanup_[i].push_back([counter, sharedFn]() {
            if (--(*counter) == 0) {
                (*sharedFn)();
            }
        });
    }
}

void VkContext::flushDeferredCleanup() {
    // Run every queued destruction now rather than waiting for the frame slots
    // to come around again. Subsystems defer destruction because in-flight
    // command buffers may still reference the resources, but during shutdown no
    // further frames are rendered, so anything queued would otherwise sit there
    // until VkContext::shutdown drops the queues unexecuted - which is how every
    // resident terrain chunk and WMO group ended up outliving the device.
    //
    // Call this while the subsystem's descriptor pools are still alive: the
    // queued lambdas free descriptor sets from them.
    for (uint32_t fi = 0; fi < MAX_FRAMES_IN_FLIGHT; fi++) {
        runDeferredCleanup(fi);
    }
}

void VkContext::runDeferredCleanup(uint32_t frameIndex) {
    auto& q = deferredCleanup_[frameIndex];
    if (q.empty()) return;
    for (auto& fn : q) {
        if (fn) fn();
    }
    q.clear();
}

VkSampler VkContext::getOrCreateSampler(const VkSamplerCreateInfo& info) {
    // Clamp anisotropy if the device doesn't support the feature.
    VkSamplerCreateInfo adjusted = info;
    if (!samplerAnisotropySupported_) {
        adjusted.anisotropyEnable = VK_FALSE;
        adjusted.maxAnisotropy = 1.0f;
    } else if (adjusted.maxAnisotropy > anisotropyLimit_) {
        // ...and to what the player asked for, which is the same kind of
        // ceiling: callers ask for the filtering they want and this is what
        // the client will actually give. Hashed with the rest of the state
        // below, so two requests that differ only above the ceiling now share
        // one sampler rather than making two identical ones.
        adjusted.maxAnisotropy = anisotropyLimit_;
        adjusted.anisotropyEnable = adjusted.maxAnisotropy > 1.0f ? VK_TRUE : VK_FALSE;
    }

    uint64_t key = hashSamplerCreateInfo(adjusted);

    {
        std::lock_guard<std::mutex> lock(samplerCacheMutex_);
        auto it = samplerCache_.find(key);
        if (it != samplerCache_.end()) {
            return it->second;
        }
    }

    // Create a new sampler outside the lock (vkCreateSampler is thread-safe
    // for distinct create infos, but we re-lock to insert).
    VkSampler sampler = VK_NULL_HANDLE;
    if (vkCreateSampler(device, &adjusted, nullptr, &sampler) != VK_SUCCESS) {
        LOG_ERROR("getOrCreateSampler: vkCreateSampler failed");
        return VK_NULL_HANDLE;
    }

    {
        std::lock_guard<std::mutex> lock(samplerCacheMutex_);
        // Double-check: another thread may have inserted while we were creating.
        auto [it, inserted] = samplerCache_.emplace(key, sampler);
        if (!inserted) {
            // Another thread won the race - destroy our duplicate and use theirs.
            vkDestroySampler(device, sampler, nullptr);
            return it->second;
        }
    }

    return sampler;
}

bool VkContext::createInstance(SDL_Window* window) {
    // Get required SDL extensions
    unsigned int sdlExtCount = 0;
    SDL_Vulkan_GetInstanceExtensions(window, &sdlExtCount, nullptr);
    std::vector<const char*> sdlExts(sdlExtCount);
    SDL_Vulkan_GetInstanceExtensions(window, &sdlExtCount, sdlExts.data());

    vkb::InstanceBuilder builder;
    builder.set_app_name("Wowee")
           .set_app_version(VK_MAKE_VERSION(1, 0, 0))
           .require_api_version(1, 2, 0)
           .set_minimum_instance_version(1, 1, 0);

    for (auto ext : sdlExts) {
        builder.enable_extension(ext);
    }

    // Allow turning validation on in a release build via env var, so the
    // Khronos validation layer's messages (e.g. the exact VK error behind an
    // FSR3 pipeline-creation failure) get routed to our log via debugCallback.
    bool enableValidationEffective = enableValidation;
    if (const char* v = std::getenv("WOWEE_VULKAN_VALIDATION")) {
        if (v[0] && v[0] != '0') enableValidationEffective = true;
    }
    if (enableValidationEffective) {
        builder.request_validation_layers(true)
               .set_debug_callback(debugCallback);
        LOG_INFO("Vulkan validation layers requested");

        // WOWEE_VULKAN_GPU_VALIDATION=1 additionally instruments the shaders.
        //
        // The plain layer only checks API calls, so a fault that lives inside a
        // shader - an index past the end of a storage buffer, a descriptor read
        // that was never written - is invisible to it: the log stays clean right
        // up to the device being lost, which says nothing about where. This
        // reports the shader and the instruction instead. It is very slow, which
        // is why it is its own switch rather than part of the one above.
        if (const char* g = std::getenv("WOWEE_VULKAN_GPU_VALIDATION");
            g && g[0] && g[0] != '0') {
            builder.add_validation_feature_enable(VK_VALIDATION_FEATURE_ENABLE_GPU_ASSISTED_EXT);
            builder.add_validation_feature_enable(
                VK_VALIDATION_FEATURE_ENABLE_GPU_ASSISTED_RESERVE_BINDING_SLOT_EXT);
            LOG_INFO("Vulkan GPU-assisted validation requested (expect a large slowdown)");
        }
    }
    validationActive_ = enableValidationEffective;

    auto instRet = builder.build();
    if (!instRet) {
        LOG_ERROR("Failed to create Vulkan instance: ", instRet.error().message());
        return false;
    }

    vkbInstance_ = instRet.value();
    instance = vkbInstance_.instance;
    debugMessenger = vkbInstance_.debug_messenger;

    // Query the actual instance API version for gating core 1.2+ calls
    uint32_t instVer = VK_API_VERSION_1_1;
    if (vkEnumerateInstanceVersion(&instVer) != VK_SUCCESS)
        instVer = VK_API_VERSION_1_1;
    instanceApiVersion_ = instVer;
    LOG_INFO("Vulkan instance created (instance API version: ",
             VK_VERSION_MAJOR(instVer), ".", VK_VERSION_MINOR(instVer), ".",
             VK_VERSION_PATCH(instVer), ")");
    return true;
}

bool VkContext::createSurface(SDL_Window* window) {
    if (!SDL_Vulkan_CreateSurface(window, instance, &surface)) {
        LOG_ERROR("Failed to create Vulkan surface: ", SDL_GetError());
        return false;
    }
    return true;
}

bool VkContext::selectPhysicalDevice() {
    vkb::PhysicalDeviceSelector selector{vkbInstance_};
    VkPhysicalDeviceFeatures requiredFeatures{};
    requiredFeatures.samplerAnisotropy = VK_TRUE;
    requiredFeatures.fillModeNonSolid = VK_TRUE;  // wireframe debug pipelines
    requiredFeatures.shaderStorageImageWriteWithoutFormat = VK_TRUE;  // FSR2 compute shaders
    requiredFeatures.shaderInt16 = VK_TRUE;  // FSR2 compute shaders
    selector.set_surface(surface)
            .set_minimum_version(1, 1)
            .set_required_features(requiredFeatures)
            .prefer_gpu_device_type(vkb::PreferredDeviceType::discrete);

    auto physRet = selector.select();
    if (!physRet) {
        LOG_ERROR("Failed to select Vulkan physical device: ", physRet.error().message());
        return false;
    }

    vkbPhysicalDevice_ = physRet.value();
    physicalDevice = vkbPhysicalDevice_.physical_device;

    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(physicalDevice, &props);
    (void)props.apiVersion; // Available if needed for version checks
    gpuVendorId_ = props.vendorID;
    // snprintf rather than strncpy: deviceName is the same size as gpuName_,
    // so strncpy copies exactly the buffer length and GCC reports it may not
    // terminate, even with the explicit NUL that followed. snprintf always
    // terminates and truncates on its own.
    std::snprintf(gpuName_, sizeof(gpuName_), "%s", props.deviceName);
    LOG_INFO("GPU: ", gpuName_, " (vendor 0x", std::hex, gpuVendorId_, std::dec, ")");

    VkPhysicalDeviceDepthStencilResolveProperties dsResolveProps{};
    dsResolveProps.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DEPTH_STENCIL_RESOLVE_PROPERTIES;
    VkPhysicalDeviceProperties2 props2{};
    props2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    props2.pNext = &dsResolveProps;
    vkGetPhysicalDeviceProperties2(physicalDevice, &props2);

    // Gate on instance API version - vkCreateRenderPass2 is core 1.2 and only
    // available when the instance was created with apiVersion >= 1.2.
    // The device may report 1.2+ but a 1.1 instance won't have the function pointer.
    if (instanceApiVersion_ >= VK_API_VERSION_1_2) {
        VkResolveModeFlags modes = dsResolveProps.supportedDepthResolveModes;
        if (modes & VK_RESOLVE_MODE_SAMPLE_ZERO_BIT) {
            depthResolveMode_ = VK_RESOLVE_MODE_SAMPLE_ZERO_BIT;
            depthResolveSupported_ = true;
        } else if (modes & VK_RESOLVE_MODE_MIN_BIT) {
            depthResolveMode_ = VK_RESOLVE_MODE_MIN_BIT;
            depthResolveSupported_ = true;
        } else if (modes & VK_RESOLVE_MODE_MAX_BIT) {
            depthResolveMode_ = VK_RESOLVE_MODE_MAX_BIT;
            depthResolveSupported_ = true;
        } else if (modes & VK_RESOLVE_MODE_AVERAGE_BIT) {
            depthResolveMode_ = VK_RESOLVE_MODE_AVERAGE_BIT;
            depthResolveSupported_ = true;
        }
    } else {
        depthResolveSupported_ = false;
        depthResolveMode_ = VK_RESOLVE_MODE_NONE;
    }

    LOG_INFO("Vulkan device: ", props.deviceName);
    LOG_INFO("Vulkan API version: ", VK_VERSION_MAJOR(props.apiVersion), ".",
             VK_VERSION_MINOR(props.apiVersion), ".", VK_VERSION_PATCH(props.apiVersion));
    LOG_INFO("Depth resolve support: ", depthResolveSupported_ ? "YES" : "NO");

    // Probe queue families to see if the graphics family supports multiple queues
    // (used in createLogicalDevice to request a second queue for parallel uploads).
    auto queueFamilies = vkbPhysicalDevice_.get_queue_families();
    for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilies.size()); i++) {
        if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            graphicsQueueFamilyQueueCount_ = queueFamilies[i].queueCount;
            LOG_INFO("Graphics queue family ", i, " supports ", graphicsQueueFamilyQueueCount_, " queue(s)");
            break;
        }
    }

    return true;
}

bool VkContext::createLogicalDevice() {
    vkb::DeviceBuilder deviceBuilder{vkbPhysicalDevice_};

    // Enable optional Vulkan 1.1/1.2 features for FSR2/FSR3 compute shaders.
    // shaderFloat16 covers fp16 *arithmetic*; the AMD FSR3 SDK shaders also pack
    // fp16 into storage buffers, which needs the 16-bit *storage* features
    // (storageBuffer16BitAccess / uniformAndStorageBuffer16BitAccess). Without
    // them the FFX Vulkan backend fails to build its compute pipelines and
    // ffxCreateContext returns rc=3 (RUNTIME_ERROR) - "Path C upscale failed".
    VkPhysicalDeviceVulkan11Features enabled11{};
    enabled11.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES;
    VkPhysicalDeviceVulkan12Features enabled12{};
    enabled12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    if (instanceApiVersion_ >= VK_API_VERSION_1_2) {
        VkPhysicalDeviceVulkan11Features supported11{};
        supported11.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES;
        VkPhysicalDeviceVulkan12Features supported12{};
        supported12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
        supported11.pNext = &supported12;
        VkPhysicalDeviceFeatures2 features2{};
        features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
        features2.pNext = &supported11;
        vkGetPhysicalDeviceFeatures2(physicalDevice, &features2);
        if (supported12.shaderFloat16) {
            enabled12.shaderFloat16 = VK_TRUE;
            LOG_INFO("Enabling shaderFloat16 for FSR2/FSR3 compute shaders");
        }
        if (supported12.shaderInt8) {
            enabled12.shaderInt8 = VK_TRUE;
        }
        // Core in 1.2 and required of any 1.2 implementation, but the query
        // costs nothing and this block only runs when the instance reached
        // 1.2 at all -- on a 1.1 instance the frame ring stays on fences.
        if (supported12.timelineSemaphore) {
            enabled12.timelineSemaphore = VK_TRUE;
            timelineSemaphoreSupported_ = true;
            LOG_INFO("Enabling timelineSemaphore for frame synchronisation");
        }
        // The AMD FSR3 SDK backend hardcodes fp16Supported=true and always
        // selects the fp16 shader permutations, whose wave/subgroup reductions
        // operate on 16-bit types - that needs shaderSubgroupExtendedTypes.
        // Without it, ffxCreateContext fails building those pipelines (rc=3).
        if (supported12.shaderSubgroupExtendedTypes) {
            enabled12.shaderSubgroupExtendedTypes = VK_TRUE;
            LOG_INFO("Enabling shaderSubgroupExtendedTypes for FSR3 fp16 wave ops");
        }
        if (supported11.storageBuffer16BitAccess) {
            enabled11.storageBuffer16BitAccess = VK_TRUE;
            LOG_INFO("Enabling 16-bit storage for FSR3 SDK compute shaders");
        }
        if (supported11.uniformAndStorageBuffer16BitAccess) {
            enabled11.uniformAndStorageBuffer16BitAccess = VK_TRUE;
        }
        // Add each struct separately - vk-bootstrap owns the pNext chaining;
        // manually linking them would be overwritten when it appends the next.
        deviceBuilder.add_pNext(&enabled11);
        deviceBuilder.add_pNext(&enabled12);
    }

    // Enable AMD device coherent memory feature if the extension was enabled
    // (prevents validation errors when VMA selects memory types with DEVICE_COHERENT_BIT_AMD)
    VkPhysicalDeviceCoherentMemoryFeaturesAMD coherentFeatures{};
    coherentFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_COHERENT_MEMORY_FEATURES_AMD;
    if (vkbPhysicalDevice_.enable_extension_if_present(VK_AMD_DEVICE_COHERENT_MEMORY_EXTENSION_NAME)) {
        coherentFeatures.deviceCoherentMemory = VK_TRUE;
        deviceBuilder.add_pNext(&coherentFeatures);
        LOG_INFO("Enabling AMD device coherent memory");
    }

    // If the graphics queue family supports >= 2 queues, request a second one
    // for parallel texture/buffer uploads.  Both queues share the same family
    // so no queue-ownership-transfer barriers are needed.
    const bool requestTransferQueue = (graphicsQueueFamilyQueueCount_ >= 2);

    if (requestTransferQueue) {
        // Build a custom queue description list: 2 queues from the graphics
        // family, 1 queue from every other family (so present etc. still work).
        auto families = vkbPhysicalDevice_.get_queue_families();
        uint32_t gfxFamily = UINT32_MAX;
        for (uint32_t i = 0; i < static_cast<uint32_t>(families.size()); i++) {
            if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                gfxFamily = i;
                break;
            }
        }

        std::vector<vkb::CustomQueueDescription> queueDescs;
        for (uint32_t i = 0; i < static_cast<uint32_t>(families.size()); i++) {
            if (i == gfxFamily) {
                // Request 2 queues: [0] graphics, [1] transfer uploads
                queueDescs.emplace_back(i, std::vector<float>{1.0f, 1.0f});
            } else {
                queueDescs.emplace_back(i, std::vector<float>{1.0f});
            }
        }
        deviceBuilder.custom_queue_setup(queueDescs);
    }

    auto devRet = deviceBuilder.build();
    if (!devRet) {
        LOG_ERROR("Failed to create Vulkan logical device: ", devRet.error().message());
        return false;
    }

    auto vkbDevice = devRet.value();
    device = vkbDevice.device;

    if (requestTransferQueue) {
        // With custom_queue_setup, we must retrieve queues manually.
        auto families = vkbPhysicalDevice_.get_queue_families();
        uint32_t gfxFamily = UINT32_MAX;
        for (uint32_t i = 0; i < static_cast<uint32_t>(families.size()); i++) {
            if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                gfxFamily = i;
                break;
            }
        }
        graphicsQueueFamily = gfxFamily;
        vkGetDeviceQueue(device, gfxFamily, 0, &graphicsQueue);
        vkGetDeviceQueue(device, gfxFamily, 1, &transferQueue_);
        hasDedicatedTransfer_ = true;

        // Present queue: try the graphics family first (most common), otherwise
        // find a family that supports presentation.
        presentQueue = graphicsQueue;
        presentQueueFamily = gfxFamily;

        LOG_INFO("Dedicated transfer queue enabled (family ", gfxFamily, ", queue index 1)");
    } else {
        // Standard path - let vkb resolve queues.
        auto gqRet = vkbDevice.get_queue(vkb::QueueType::graphics);
        if (!gqRet) {
            LOG_ERROR("Failed to get graphics queue");
            return false;
        }
        graphicsQueue = gqRet.value();
        graphicsQueueFamily = vkbDevice.get_queue_index(vkb::QueueType::graphics).value();

        auto pqRet = vkbDevice.get_queue(vkb::QueueType::present);
        if (!pqRet) {
            presentQueue = graphicsQueue;
            presentQueueFamily = graphicsQueueFamily;
        } else {
            presentQueue = pqRet.value();
            presentQueueFamily = vkbDevice.get_queue_index(vkb::QueueType::present).value();
        }
    }

    LOG_INFO("Vulkan logical device created");
    return true;
}

bool VkContext::createAllocator() {
    VmaAllocatorCreateInfo allocInfo{};
    allocInfo.instance = instance;
    allocInfo.physicalDevice = physicalDevice;
    allocInfo.device = device;
    allocInfo.vulkanApiVersion = instanceApiVersion_;

    if (vmaCreateAllocator(&allocInfo, &allocator) != VK_SUCCESS) {
        LOG_ERROR("Failed to create VMA allocator");
        return false;
    }

    LOG_INFO("VMA allocator created");
    return true;
}

// ---------------------------------------------------------------------------
// Pipeline cache persistence
// ---------------------------------------------------------------------------

static std::string getPipelineCachePath() {
#ifdef _WIN32
    if (const char* appdata = std::getenv("APPDATA"))
        return std::string(appdata) + "\\wowee\\pipeline_cache.bin";
    return ".\\pipeline_cache.bin";
#elif defined(__APPLE__)
    if (const char* home = std::getenv("HOME"))
        return std::string(home) + "/Library/Caches/wowee/pipeline_cache.bin";
    return "./pipeline_cache.bin";
#else
    if (const char* home = std::getenv("HOME"))
        return std::string(home) + "/.local/share/wowee/pipeline_cache.bin";
    return "./pipeline_cache.bin";
#endif
}

bool VkContext::createPipelineCache() {
    // NVIDIA drivers have their own built-in pipeline/shader disk cache.
    // Using VkPipelineCache on NVIDIA 590.x causes vkCmdBeginRenderPass to
    // SIGSEGV inside libnvidia-glcore - skip entirely on NVIDIA GPUs.
    if (gpuVendorId_ == 0x10DE) {
        LOG_INFO("Pipeline cache: skipped (NVIDIA driver provides built-in caching)");
        return true;
    }

    std::string path = getPipelineCachePath();

    // Try to load existing cache data from disk.
    std::vector<char> cacheData;
    {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (file.is_open()) {
            auto size = file.tellg();
            if (size > 0) {
                cacheData.resize(static_cast<size_t>(size));
                file.seekg(0);
                file.read(cacheData.data(), size);
                if (!file) {
                    LOG_WARNING("Pipeline cache file read failed, starting with empty cache");
                    cacheData.clear();
                }
            }
        }
    }

    VkPipelineCacheCreateInfo cacheCI{};
    cacheCI.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    cacheCI.initialDataSize = cacheData.size();
    cacheCI.pInitialData = cacheData.empty() ? nullptr : cacheData.data();

    VkResult result = vkCreatePipelineCache(device, &cacheCI, nullptr, &pipelineCache_);
    if (result != VK_SUCCESS) {
        // If loading stale/corrupt data caused failure, retry with empty cache.
        if (!cacheData.empty()) {
            LOG_WARNING("Pipeline cache creation failed with saved data, retrying empty");
            cacheCI.initialDataSize = 0;
            cacheCI.pInitialData = nullptr;
            result = vkCreatePipelineCache(device, &cacheCI, nullptr, &pipelineCache_);
        }
        if (result != VK_SUCCESS) {
            LOG_WARNING("Pipeline cache creation failed - pipelines will not be cached");
            pipelineCache_ = VK_NULL_HANDLE;
            return false;
        }
    }

    if (!cacheData.empty()) {
        LOG_INFO("Pipeline cache loaded from disk (", cacheData.size(), " bytes)");
    } else {
        LOG_INFO("Pipeline cache created (empty)");
    }
    return true;
}

void VkContext::savePipelineCache() {
    if (!pipelineCache_ || !device) return;

    size_t dataSize = 0;
    if (vkGetPipelineCacheData(device, pipelineCache_, &dataSize, nullptr) != VK_SUCCESS || dataSize == 0) {
        LOG_WARNING("Failed to query pipeline cache size");
        return;
    }

    std::vector<char> data(dataSize);
    if (vkGetPipelineCacheData(device, pipelineCache_, &dataSize, data.data()) != VK_SUCCESS) {
        LOG_WARNING("Failed to retrieve pipeline cache data");
        return;
    }

    std::string path = getPipelineCachePath();
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());

    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
        LOG_WARNING("Failed to open pipeline cache file for writing: ", path);
        return;
    }

    file.write(data.data(), static_cast<std::streamsize>(dataSize));
    file.close();

    LOG_INFO("Pipeline cache saved to disk (", dataSize, " bytes)");
}

bool VkContext::createSwapchain(int width, int height) {
    vkb::SwapchainBuilder swapchainBuilder{physicalDevice, device, surface};

    auto& builder = swapchainBuilder
        .set_desired_format({.format = VK_FORMAT_B8G8R8A8_UNORM, .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
        .set_desired_extent(static_cast<uint32_t>(width), static_cast<uint32_t>(height))
        .set_image_usage_flags(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT)
        .set_desired_min_image_count(2)
        .set_old_swapchain(swapchain);

    if (vsync_) {
        builder.set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR);
    } else {
        builder.set_desired_present_mode(VK_PRESENT_MODE_IMMEDIATE_KHR);
        builder.add_fallback_present_mode(VK_PRESENT_MODE_MAILBOX_KHR);
        builder.add_fallback_present_mode(VK_PRESENT_MODE_FIFO_RELAXED_KHR);
    }

    auto swapRet = builder.build();

    if (!swapRet) {
        LOG_ERROR("Failed to create Vulkan swapchain: ", swapRet.error().message());
        return false;
    }

    // Destroy old swapchain if recreating
    if (swapchain != VK_NULL_HANDLE) {
        destroySwapchain();
    }

    auto vkbSwap = swapRet.value();
    swapchain = vkbSwap.swapchain;
    swapchainFormat = vkbSwap.image_format;
    swapchainExtent = vkbSwap.extent;
    swapchainImages = vkbSwap.get_images().value();
    swapchainImageViews = vkbSwap.get_image_views().value();

    // Create framebuffers for ImGui render pass (created after ImGui resources)
    // Will be created in createImGuiResources or recreateSwapchain

    LOG_INFO("Vulkan swapchain created: ", swapchainExtent.width, "x", swapchainExtent.height,
             " (", swapchainImages.size(), " images)");
    swapchainDirty = false;
    return true;
}

void VkContext::destroySwapchain() {
    for (auto fb : swapchainFramebuffers) {
        if (fb) vkDestroyFramebuffer(device, fb, nullptr);
    }
    swapchainFramebuffers.clear();

    for (auto iv : swapchainImageViews) {
        if (iv) vkDestroyImageView(device, iv, nullptr);
    }
    swapchainImageViews.clear();
    swapchainImages.clear();

    if (swapchain) {
        vkDestroySwapchainKHR(device, swapchain, nullptr);
        swapchain = VK_NULL_HANDLE;
    }
}

bool VkContext::createCommandPools() {
    // Per-frame command pools (resettable)
    for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = graphicsQueueFamily;

        if (vkCreateCommandPool(device, &poolInfo, nullptr, &frames[i].commandPool) != VK_SUCCESS) {
            LOG_ERROR("Failed to create command pool for frame ", i);
            return false;
        }

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = frames[i].commandPool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;

        if (vkAllocateCommandBuffers(device, &allocInfo, &frames[i].commandBuffer) != VK_SUCCESS) {
            LOG_ERROR("Failed to allocate command buffer for frame ", i);
            return false;
        }
    }

    // Immediate submit pool
    VkCommandPoolCreateInfo immPoolInfo{};
    immPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    immPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    immPoolInfo.queueFamilyIndex = graphicsQueueFamily;

    if (vkCreateCommandPool(device, &immPoolInfo, nullptr, &immCommandPool) != VK_SUCCESS) {
        LOG_ERROR("Failed to create immediate command pool");
        return false;
    }

    // Separate command pool for the transfer queue (same family, different queue)
    if (hasDedicatedTransfer_) {
        VkCommandPoolCreateInfo transferPoolInfo{};
        transferPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        transferPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        transferPoolInfo.queueFamilyIndex = graphicsQueueFamily;

        if (vkCreateCommandPool(device, &transferPoolInfo, nullptr, &transferCommandPool_) != VK_SUCCESS) {
            LOG_ERROR("Failed to create transfer command pool");
            return false;
        }
    }

    return true;
}

bool VkContext::createSyncObjects() {
    VkSemaphoreCreateInfo semInfo{};
    semInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT; // Start signaled so first frame doesn't block

    // The timeline starts at 0 and every slot's timelineValue starts at 0, so
    // the first wait on each slot is already satisfied -- the same starting
    // state VK_FENCE_CREATE_SIGNALED_BIT gives the fences below.
    if (timelineSemaphoreSupported_) {
        VkSemaphoreTypeCreateInfo typeInfo{};
        typeInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
        typeInfo.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
        typeInfo.initialValue = 0;
        VkSemaphoreCreateInfo timelineInfo{};
        timelineInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        timelineInfo.pNext = &typeInfo;
        if (vkCreateSemaphore(device, &timelineInfo, nullptr, &frameTimeline_) != VK_SUCCESS) {
            LOG_WARNING("Could not create the frame timeline semaphore; using fences");
            frameTimeline_ = VK_NULL_HANDLE;
        } else {
            frameTimelineValue_ = 0;
            for (auto& f : frames) f.timelineValue = 0;
        }
    }

    for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        if (vkCreateFence(device, &fenceInfo, nullptr, &frames[i].inFlightFence) != VK_SUCCESS) {
            LOG_ERROR("Failed to create sync objects for frame ", i);
            return false;
        }
        // The handle, because validation reports a fence by handle and there is
        // no way to tell a frame fence from the upload fence in that message.
        // Two rounds of this went on a fence nobody could identify.
        LOG_WARNING("frame fence ", i, " = 0x", std::hex,
                    reinterpret_cast<uint64_t>(frames[i].inFlightFence), std::dec);
    }

    // Per-swapchain-image semaphores: avoids reuse while the presentation engine
    // still holds a reference.  After acquiring image N we swap the acquire semaphore
    // into imageAcquiredSemaphores_[N], recycling the old one for the next acquire.
    const uint32_t imgCount = static_cast<uint32_t>(swapchainImages.size());
    imageAcquiredSemaphores_.resize(imgCount);
    renderFinishedSemaphores_.resize(imgCount);
    for (uint32_t i = 0; i < imgCount; i++) {
        if (vkCreateSemaphore(device, &semInfo, nullptr, &imageAcquiredSemaphores_[i]) != VK_SUCCESS ||
            vkCreateSemaphore(device, &semInfo, nullptr, &renderFinishedSemaphores_[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to create per-image semaphores for image ", i);
            return false;
        }
    }
    // One extra acquire semaphore - we need it for the next vkAcquireNextImageKHR
    // before we know which image we'll get.
    if (vkCreateSemaphore(device, &semInfo, nullptr, &nextAcquireSemaphore_) != VK_SUCCESS) {
        LOG_ERROR("Failed to create next-acquire semaphore");
        return false;
    }

    // Immediate submit fence (not signaled initially)
    VkFenceCreateInfo immFenceInfo{};
    immFenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    // Logged for the same reason as the frame fences: validation reports a
    // fence by handle, and immFence is shared by endSingleTimeCommands and the
    // upload batches - which submit on different queues.
    if (vkCreateFence(device, &immFenceInfo, nullptr, &immFence) == VK_SUCCESS) {
        LOG_WARNING("immediate fence = 0x", std::hex,
                    reinterpret_cast<uint64_t>(immFence), std::dec);
    }
    if (immFence == VK_NULL_HANDLE) {
        LOG_ERROR("Failed to create immediate submit fence");
        return false;
    }

    return true;
}

bool VkContext::createDepthBuffer() {
    VkImageCreateInfo imgInfo{};
    imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgInfo.imageType = VK_IMAGE_TYPE_2D;
    imgInfo.format = depthFormat;
    imgInfo.extent = {.width = swapchainExtent.width, .height = swapchainExtent.height, .depth = 1};
    imgInfo.mipLevels = 1;
    imgInfo.arrayLayers = 1;
    imgInfo.samples = msaaSamples_;
    imgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imgInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                  | VK_IMAGE_USAGE_SAMPLED_BIT;  // HiZ pyramid reads depth as texture

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;

    if (vmaCreateImage(allocator, &imgInfo, &allocInfo, &depthImage, &depthAllocation, nullptr) != VK_SUCCESS) {
        LOG_ERROR("Failed to create depth image");
        return false;
    }

    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = depthImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = depthFormat;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(device, &viewInfo, nullptr, &depthImageView) != VK_SUCCESS) {
        LOG_ERROR("Failed to create depth image view");
        return false;
    }

    return true;
}

void VkContext::destroyDepthBuffer() {
    if (depthImageView) { vkDestroyImageView(device, depthImageView, nullptr); depthImageView = VK_NULL_HANDLE; }
    if (depthImage) { vmaDestroyImage(allocator, depthImage, depthAllocation); depthImage = VK_NULL_HANDLE; depthAllocation = VK_NULL_HANDLE; }
}

bool VkContext::createMsaaColorImage() {
    if (msaaSamples_ == VK_SAMPLE_COUNT_1_BIT) return true; // No MSAA image needed

    // Check if lazily allocated memory is available - only use TRANSIENT when it is.
    // AMD GPUs (especially RDNA4) don't expose lazily allocated memory; using TRANSIENT
    // without it can cause the driver to optimize for tile-only storage, leading to
    // crashes during MSAA resolve when the backing memory was never populated.
    bool hasLazyMemory = false;
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT) {
            hasLazyMemory = true;
            break;
        }
    }

    VkImageCreateInfo imgInfo{};
    imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgInfo.imageType = VK_IMAGE_TYPE_2D;
    imgInfo.format = swapchainFormat;
    imgInfo.extent = {.width = swapchainExtent.width, .height = swapchainExtent.height, .depth = 1};
    imgInfo.mipLevels = 1;
    imgInfo.arrayLayers = 1;
    imgInfo.samples = msaaSamples_;
    imgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;

    if (hasLazyMemory) {
        imgInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
        allocInfo.preferredFlags = VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT;
    } else {
        imgInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    }

    if (vmaCreateImage(allocator, &imgInfo, &allocInfo, &msaaColorImage_, &msaaColorAllocation_, nullptr) != VK_SUCCESS) {
        // Retry without TRANSIENT (some drivers reject it at high sample counts)
        imgInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        allocInfo.preferredFlags = 0;
        if (vmaCreateImage(allocator, &imgInfo, &allocInfo, &msaaColorImage_, &msaaColorAllocation_, nullptr) != VK_SUCCESS) {
            LOG_ERROR("Failed to create MSAA color image");
            return false;
        }
    }

    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = msaaColorImage_;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = swapchainFormat;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(device, &viewInfo, nullptr, &msaaColorView_) != VK_SUCCESS) {
        LOG_ERROR("Failed to create MSAA color image view");
        return false;
    }

    return true;
}

void VkContext::destroyMsaaColorImage() {
    if (msaaColorView_) { vkDestroyImageView(device, msaaColorView_, nullptr); msaaColorView_ = VK_NULL_HANDLE; }
    if (msaaColorImage_) { vmaDestroyImage(allocator, msaaColorImage_, msaaColorAllocation_); msaaColorImage_ = VK_NULL_HANDLE; msaaColorAllocation_ = VK_NULL_HANDLE; }
}

bool VkContext::createDepthResolveImage() {
    if (msaaSamples_ == VK_SAMPLE_COUNT_1_BIT || !depthResolveSupported_) return true;

    VkImageCreateInfo imgInfo{};
    imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgInfo.imageType = VK_IMAGE_TYPE_2D;
    imgInfo.format = depthFormat;
    imgInfo.extent = {.width = swapchainExtent.width, .height = swapchainExtent.height, .depth = 1};
    imgInfo.mipLevels = 1;
    imgInfo.arrayLayers = 1;
    imgInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imgInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                  | VK_IMAGE_USAGE_SAMPLED_BIT;  // HiZ pyramid reads depth as texture

    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;

    if (vmaCreateImage(allocator, &imgInfo, &allocInfo, &depthResolveImage, &depthResolveAllocation, nullptr) != VK_SUCCESS) {
        LOG_ERROR("Failed to create depth resolve image");
        return false;
    }

    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = depthResolveImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = depthFormat;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;
    if (vkCreateImageView(device, &viewInfo, nullptr, &depthResolveImageView) != VK_SUCCESS) {
        LOG_ERROR("Failed to create depth resolve image view");
        return false;
    }

    return true;
}

void VkContext::destroyDepthResolveImage() {
    if (depthResolveImageView) {
        vkDestroyImageView(device, depthResolveImageView, nullptr);
        depthResolveImageView = VK_NULL_HANDLE;
    }
    if (depthResolveImage) {
        vmaDestroyImage(allocator, depthResolveImage, depthResolveAllocation);
        depthResolveImage = VK_NULL_HANDLE;
        depthResolveAllocation = VK_NULL_HANDLE;
    }
}

VkSampleCountFlagBits VkContext::getMaxUsableSampleCount() const {
    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(physicalDevice, &props);
    VkSampleCountFlags counts = props.limits.framebufferColorSampleCounts
                               & props.limits.framebufferDepthSampleCounts;
    if (counts & VK_SAMPLE_COUNT_8_BIT) return VK_SAMPLE_COUNT_8_BIT;
    if (counts & VK_SAMPLE_COUNT_4_BIT) return VK_SAMPLE_COUNT_4_BIT;
    if (counts & VK_SAMPLE_COUNT_2_BIT) return VK_SAMPLE_COUNT_2_BIT;
    return VK_SAMPLE_COUNT_1_BIT;
}

void VkContext::setMsaaSamples(VkSampleCountFlagBits samples) {
    // Clamp to max supported
    VkSampleCountFlagBits maxSamples = getMaxUsableSampleCount();
    if (samples > maxSamples) samples = maxSamples;
    msaaSamples_ = samples;
    swapchainDirty = true;
}

bool VkContext::createSwapchainRenderTargets(const char* verb) {
    if (!createDepthBuffer()) return false;

    // Create MSAA color image if needed
    if (!createMsaaColorImage()) return false;
    // Create single-sample depth resolve image for MSAA path (if supported)
    if (!createDepthResolveImage()) return false;

    bool useMsaa = (msaaSamples_ > VK_SAMPLE_COUNT_1_BIT);

    if (useMsaa) {
        const bool useDepthResolve = (depthResolveImageView != VK_NULL_HANDLE);
        // MSAA render pass: 3 or 4 attachments
        VkAttachmentDescription attachments[4] = {};

        // Attachment 0: MSAA color target
        attachments[0].format = swapchainFormat;
        attachments[0].samples = msaaSamples_;
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        // Attachment 1: Depth (multisampled)
        attachments[1].format = depthFormat;
        attachments[1].samples = msaaSamples_;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        // Attachment 2: Resolve target (swapchain image)
        attachments[2].format = swapchainFormat;
        attachments[2].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[2].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[2].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[2].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[2].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[2].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[2].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        if (useDepthResolve) {
            attachments[3].format = depthFormat;
            attachments[3].samples = VK_SAMPLE_COUNT_1_BIT;
            attachments[3].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            attachments[3].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            attachments[3].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            attachments[3].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            attachments[3].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            attachments[3].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        }

        if (useDepthResolve) {
            VkAttachmentDescription2 attachments2[4]{};
            for (int i = 0; i < 4; ++i) {
                attachments2[i].sType = VK_STRUCTURE_TYPE_ATTACHMENT_DESCRIPTION_2;
                attachments2[i].format = attachments[i].format;
                attachments2[i].samples = attachments[i].samples;
                attachments2[i].loadOp = attachments[i].loadOp;
                attachments2[i].storeOp = attachments[i].storeOp;
                attachments2[i].stencilLoadOp = attachments[i].stencilLoadOp;
                attachments2[i].stencilStoreOp = attachments[i].stencilStoreOp;
                attachments2[i].initialLayout = attachments[i].initialLayout;
                attachments2[i].finalLayout = attachments[i].finalLayout;
            }

            VkAttachmentReference2 colorRef2{};
            colorRef2.sType = VK_STRUCTURE_TYPE_ATTACHMENT_REFERENCE_2;
            colorRef2.attachment = 0;
            colorRef2.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            VkAttachmentReference2 depthRef2{};
            depthRef2.sType = VK_STRUCTURE_TYPE_ATTACHMENT_REFERENCE_2;
            depthRef2.attachment = 1;
            depthRef2.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
            VkAttachmentReference2 resolveRef2{};
            resolveRef2.sType = VK_STRUCTURE_TYPE_ATTACHMENT_REFERENCE_2;
            resolveRef2.attachment = 2;
            resolveRef2.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            VkAttachmentReference2 depthResolveRef2{};
            depthResolveRef2.sType = VK_STRUCTURE_TYPE_ATTACHMENT_REFERENCE_2;
            depthResolveRef2.attachment = 3;
            depthResolveRef2.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

            VkSubpassDescriptionDepthStencilResolve dsResolve{};
            dsResolve.sType = VK_STRUCTURE_TYPE_SUBPASS_DESCRIPTION_DEPTH_STENCIL_RESOLVE;
            dsResolve.depthResolveMode = depthResolveMode_;
            dsResolve.stencilResolveMode = VK_RESOLVE_MODE_NONE;
            dsResolve.pDepthStencilResolveAttachment = &depthResolveRef2;

            VkSubpassDescription2 subpass2{};
            subpass2.sType = VK_STRUCTURE_TYPE_SUBPASS_DESCRIPTION_2;
            subpass2.pNext = &dsResolve;
            subpass2.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
            subpass2.colorAttachmentCount = 1;
            subpass2.pColorAttachments = &colorRef2;
            subpass2.pDepthStencilAttachment = &depthRef2;
            subpass2.pResolveAttachments = &resolveRef2;

            VkSubpassDependency2 dep2{};
            dep2.sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2;
            dep2.srcSubpass = VK_SUBPASS_EXTERNAL;
            dep2.dstSubpass = 0;
            dep2.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dep2.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dep2.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

            VkRenderPassCreateInfo2 rpInfo2{};
            rpInfo2.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO_2;
            rpInfo2.attachmentCount = 4;
            rpInfo2.pAttachments = attachments2;
            rpInfo2.subpassCount = 1;
            rpInfo2.pSubpasses = &subpass2;
            rpInfo2.dependencyCount = 1;
            rpInfo2.pDependencies = &dep2;

            if (vkCreateRenderPass2(device, &rpInfo2, nullptr, &imguiRenderPass) != VK_SUCCESS) {
                LOG_ERROR("Failed to ", verb, " MSAA render pass (depth resolve)");
                return false;
            }
        } else {
            VkAttachmentReference colorRef{};
            colorRef.attachment = 0;
            colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

            VkAttachmentReference depthRef{};
            depthRef.attachment = 1;
            depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

            VkAttachmentReference resolveRef{};
            resolveRef.attachment = 2;
            resolveRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

            VkSubpassDescription subpass{};
            subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
            subpass.colorAttachmentCount = 1;
            subpass.pColorAttachments = &colorRef;
            subpass.pDepthStencilAttachment = &depthRef;
            subpass.pResolveAttachments = &resolveRef;

            VkSubpassDependency dependency{};
            dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
            dependency.dstSubpass = 0;
            dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dependency.srcAccessMask = 0;
            dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

            VkRenderPassCreateInfo rpInfo{};
            rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
            rpInfo.attachmentCount = 3;
            rpInfo.pAttachments = attachments;
            rpInfo.subpassCount = 1;
            rpInfo.pSubpasses = &subpass;
            rpInfo.dependencyCount = 1;
            rpInfo.pDependencies = &dependency;

            if (vkCreateRenderPass(device, &rpInfo, nullptr, &imguiRenderPass) != VK_SUCCESS) {
                LOG_ERROR("Failed to ", verb, " MSAA render pass");
                return false;
            }
        }

        // Framebuffers: [msaaColorView, depthView, swapchainView, depthResolveView?]
        swapchainFramebuffers.resize(swapchainImageViews.size());
        for (size_t i = 0; i < swapchainImageViews.size(); i++) {
            VkImageView fbAttachments[4] = {msaaColorView_, depthImageView, swapchainImageViews[i], depthResolveImageView};

            VkFramebufferCreateInfo fbInfo{};
            fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            fbInfo.renderPass = imguiRenderPass;
            fbInfo.attachmentCount = useDepthResolve ? 4 : 3;
            fbInfo.pAttachments = fbAttachments;
            fbInfo.width = swapchainExtent.width;
            fbInfo.height = swapchainExtent.height;
            fbInfo.layers = 1;

            if (vkCreateFramebuffer(device, &fbInfo, nullptr, &swapchainFramebuffers[i]) != VK_SUCCESS) {
                LOG_ERROR("Failed to ", verb, " MSAA swapchain framebuffer ", i);
                return false;
            }
        }
    } else {
        // Non-MSAA render pass: 2 attachments (color + depth) - original path
        VkAttachmentDescription attachments[2] = {};

        // Color attachment (swapchain image)
        attachments[0].format = swapchainFormat;
        attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        // Depth attachment
        attachments[1].format = depthFormat;
        attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorRef{};
        colorRef.attachment = 0;
        colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference depthRef{};
        depthRef.attachment = 1;
        depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorRef;
        subpass.pDepthStencilAttachment = &depthRef;

        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependency.srcAccessMask = 0;
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo rpInfo{};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rpInfo.attachmentCount = 2;
        rpInfo.pAttachments = attachments;
        rpInfo.subpassCount = 1;
        rpInfo.pSubpasses = &subpass;
        rpInfo.dependencyCount = 1;
        rpInfo.pDependencies = &dependency;

        if (vkCreateRenderPass(device, &rpInfo, nullptr, &imguiRenderPass) != VK_SUCCESS) {
            LOG_ERROR("Failed to ", verb, " render pass");
            return false;
        }

        // Framebuffers: [swapchainView, depthView]
        swapchainFramebuffers.resize(swapchainImageViews.size());
        for (size_t i = 0; i < swapchainImageViews.size(); i++) {
            VkImageView fbAttachments[2] = {swapchainImageViews[i], depthImageView};

            VkFramebufferCreateInfo fbInfo{};
            fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            fbInfo.renderPass = imguiRenderPass;
            fbInfo.attachmentCount = 2;
            fbInfo.pAttachments = fbAttachments;
            fbInfo.width = swapchainExtent.width;
            fbInfo.height = swapchainExtent.height;
            fbInfo.layers = 1;

            if (vkCreateFramebuffer(device, &fbInfo, nullptr, &swapchainFramebuffers[i]) != VK_SUCCESS) {
                LOG_ERROR("Failed to ", verb, " swapchain framebuffer ", i);
                return false;
            }
        }
    }

    return true;
}

bool VkContext::createImGuiResources() {
    if (!createSwapchainRenderTargets("create")) return false;

    // Create descriptor pool for ImGui.
    // Budget: ~10 internal ImGui sets + up to 2000 UI icon textures (spells,
    // items, talents, buffs, etc.) that are uploaded and cached for the session.
    static constexpr uint32_t IMGUI_POOL_SIZE = 2048;
    VkDescriptorPoolSize poolSizes[] = {
        {.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = IMGUI_POOL_SIZE},
    };

    VkDescriptorPoolCreateInfo dpInfo{};
    dpInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    dpInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    dpInfo.maxSets = IMGUI_POOL_SIZE;
    dpInfo.poolSizeCount = 1;
    dpInfo.pPoolSizes = poolSizes;

    if (vkCreateDescriptorPool(device, &dpInfo, nullptr, &imguiDescriptorPool) != VK_SUCCESS) {
        LOG_ERROR("Failed to create ImGui descriptor pool");
        return false;
    }

    // One creation site for the overlay pass, shared by both MSAA and non-MSAA
    // configurations. Recreated from recreateSwapchain the same way.
    if (!createOverlayRenderPass()) return false;
    if (!createSceneContinueRenderPass()) return false;

    return true;
}


// The UI draws in its own pass, after the scene has resolved and after water
// refraction has copied the scene. Keeping it separate means the UI is never
// part of the refraction capture, and deliberately single-sampled: ImGui draws
// axis-aligned rectangles and pre-antialiased glyphs, which MSAA does almost
// nothing for, so multisampling it only costs fill rate. Colour only, loading
// what is already on the swapchain - no depth, no resolve.
bool VkContext::createOverlayRenderPass() {
    destroyOverlayRenderPass();

    VkAttachmentDescription color{};
    color.format = swapchainFormat;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorRef{};
    colorRef.attachment = 0;
    colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;

    // Wait for the scene resolve and for the refraction copy that reads it.
    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_TRANSFER_READ_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;

    VkRenderPassCreateInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpInfo.attachmentCount = 1;
    rpInfo.pAttachments = &color;
    rpInfo.subpassCount = 1;
    rpInfo.pSubpasses = &subpass;
    rpInfo.dependencyCount = 1;
    rpInfo.pDependencies = &dependency;

    if (vkCreateRenderPass(device, &rpInfo, nullptr, &overlayRenderPass) != VK_SUCCESS) {
        LOG_ERROR("Failed to create overlay (UI) render pass");
        overlayRenderPass = VK_NULL_HANDLE;
        return false;
    }

    // Same attachments, so ImGui's pipelines work in either, but clearing rather
    // than loading. The loading screen draws ImGui with nothing underneath it,
    // and it used to do that inside the scene pass - which stopped being valid
    // once ImGui's pipelines were built single-sampled for the overlay pass.
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (vkCreateRenderPass(device, &rpInfo, nullptr, &overlayClearRenderPass) != VK_SUCCESS) {
        LOG_ERROR("Failed to create clearing overlay render pass");
        overlayClearRenderPass = VK_NULL_HANDLE;
    }

    overlayFramebuffers.resize(swapchainImageViews.size());
    for (size_t i = 0; i < swapchainImageViews.size(); i++) {
        VkFramebufferCreateInfo fbInfo{};
        fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbInfo.renderPass = overlayRenderPass;
        fbInfo.attachmentCount = 1;
        fbInfo.pAttachments = &swapchainImageViews[i];
        fbInfo.width = swapchainExtent.width;
        fbInfo.height = swapchainExtent.height;
        fbInfo.layers = 1;
        if (vkCreateFramebuffer(device, &fbInfo, nullptr, &overlayFramebuffers[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to create overlay framebuffer ", i);
            destroyOverlayRenderPass();
            return false;
        }
    }
    return true;
}

// Continuation of the scene pass: same attachments as the scene pass (so the
// pipelines built for it work unchanged) but loading what is already drawn
// instead of clearing. Water renders here, after the scene has been copied for
// refraction, which is what keeps the water out of its own refraction source.
// Only built without MSAA - a multisampled continuation would have to resolve a
// second time and could not preserve the first resolve.
bool VkContext::createSceneContinueRenderPass() {
    if (msaaSamples_ > VK_SAMPLE_COUNT_1_BIT) {
        sceneContinueRenderPass = VK_NULL_HANDLE;
        return true;
    }

    VkAttachmentDescription attachments[2]{};
    attachments[0].format = swapchainFormat;
    attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    attachments[1].format = depthFormat;
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorRef{};
    colorRef.attachment = 0;
    colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    VkAttachmentReference depthRef{};
    depthRef.attachment = 1;
    depthRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    // Must match the scene pass's dependency exactly. This pass is begun against
    // the scene's own framebuffer, and render pass compatibility is checked
    // against how that framebuffer was created - a dependency that differs makes
    // the begin invalid, which is undefined behaviour and rendered the frame into
    // a corner of the screen on the FXAA path.
    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpInfo.attachmentCount = 2;
    rpInfo.pAttachments = attachments;
    rpInfo.subpassCount = 1;
    rpInfo.pSubpasses = &subpass;
    rpInfo.dependencyCount = 1;
    rpInfo.pDependencies = &dependency;

    if (vkCreateRenderPass(device, &rpInfo, nullptr, &sceneContinueRenderPass) != VK_SUCCESS) {
        LOG_WARNING("Failed to create scene continuation pass - water stays in the scene pass");
        sceneContinueRenderPass = VK_NULL_HANDLE;
    }
    return true;
}

void VkContext::destroyOverlayRenderPass() {
    for (VkFramebuffer fb : overlayFramebuffers) {
        if (fb) vkDestroyFramebuffer(device, fb, nullptr);
    }
    overlayFramebuffers.clear();
    if (overlayRenderPass) {
        vkDestroyRenderPass(device, overlayRenderPass, nullptr);
        overlayRenderPass = VK_NULL_HANDLE;
    }
    if (overlayClearRenderPass) {
        vkDestroyRenderPass(device, overlayClearRenderPass, nullptr);
        overlayClearRenderPass = VK_NULL_HANDLE;
    }
    if (sceneContinueRenderPass) {
        vkDestroyRenderPass(device, sceneContinueRenderPass, nullptr);
        sceneContinueRenderPass = VK_NULL_HANDLE;
    }
}

void VkContext::destroyImGuiResources() {
    // Destroy uploaded UI textures
    for (auto& tex : uiTextures_) {
        if (tex.view) vkDestroyImageView(device, tex.view, nullptr);
        if (tex.image) vkDestroyImage(device, tex.image, nullptr);
        if (tex.memory) vkFreeMemory(device, tex.memory, nullptr);
    }
    uiTextures_.clear();
    uiTextureSampler_ = VK_NULL_HANDLE; // Owned by sampler cache

    // Said here rather than by whoever decided to destroy them, so that the
    // generation cannot disagree with what actually happened to the sets.
    ++uiTextureGeneration_;

    // This context's own UI texture pool, which the sets above were allocated
    // from. Freed with them rather than with ImGui's, which is the whole point
    // of it existing.
    destroy(device, uiTexturePool_);
    destroy(device, uiTextureLayout_);

    destroy(device, imguiDescriptorPool);
    destroyMsaaColorImage();
    destroyDepthResolveImage();
    destroyDepthBuffer();
    // Framebuffers are destroyed in destroySwapchain()
    destroyOverlayRenderPass();
    if (imguiRenderPass) {
        vkDestroyRenderPass(device, imguiRenderPass, nullptr);
        imguiRenderPass = VK_NULL_HANDLE;
    }
}

static uint32_t findMemType(VkPhysicalDevice physDev, uint32_t typeFilter, VkMemoryPropertyFlags props) {
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(physDev, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) && (memProps.memoryTypes[i].propertyFlags & props) == props)
            return i;
    }
    LOG_ERROR("VkContext: no suitable memory type found");
    return UINT32_MAX;
}

bool VkContext::ensureUiTextureDescriptorPool() {
    if (uiTexturePool_ != VK_NULL_HANDLE) return true;

    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &binding;
    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr,
                                    &uiTextureLayout_) != VK_SUCCESS) {
        LOG_ERROR("Could not create the UI texture descriptor layout");
        return false;
    }

    // Sized for the interface with FrameXML loaded, which asks for several
    // hundred distinct files; the old path shared ImGui's pool and inherited
    // whatever that was sized for.
    VkDescriptorPoolSize size{};
    size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    size.descriptorCount = 4096;
    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = 4096;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &size;
    LOG_WARNING("UI textures allocate from this context's own descriptor pool, "
                "so they outlive an ImGui backend restart");
    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &uiTexturePool_) != VK_SUCCESS) {
        LOG_ERROR("Could not create the UI texture descriptor pool");
        vkDestroyDescriptorSetLayout(device, uiTextureLayout_, nullptr);
        uiTextureLayout_ = VK_NULL_HANDLE;
        return false;
    }
    return true;
}

VkDescriptorSet VkContext::uploadImGuiTexture(const uint8_t* rgba, int width, int height) {
    if (!device || !physicalDevice || width <= 0 || height <= 0 || !rgba)
        return VK_NULL_HANDLE;

    VkDeviceSize imageSize = static_cast<VkDeviceSize>(width) * height * 4;

    // Create shared sampler on first call (via sampler cache)
    if (!uiTextureSampler_) {
        VkSamplerCreateInfo si{};
        si.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        si.magFilter = VK_FILTER_LINEAR;
        si.minFilter = VK_FILTER_LINEAR;
        si.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        uiTextureSampler_ = getOrCreateSampler(si);
        if (!uiTextureSampler_) {
            LOG_ERROR("Failed to create UI texture sampler");
            return VK_NULL_HANDLE;
        }
    }

    // Staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingMemory;
    {
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = imageSize;
        bufInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateBuffer(device, &bufInfo, nullptr, &stagingBuffer) != VK_SUCCESS)
            return VK_NULL_HANDLE;

        VkMemoryRequirements memReqs;
        vkGetBufferMemoryRequirements(device, stagingBuffer, &memReqs);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memReqs.size;
        allocInfo.memoryTypeIndex = findMemType(physicalDevice, memReqs.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        if (vkAllocateMemory(device, &allocInfo, nullptr, &stagingMemory) != VK_SUCCESS) {
            vkDestroyBuffer(device, stagingBuffer, nullptr);
            return VK_NULL_HANDLE;
        }
        vkBindBufferMemory(device, stagingBuffer, stagingMemory, 0);

        void* mapped;
        vkMapMemory(device, stagingMemory, 0, imageSize, 0, &mapped);
        memcpy(mapped, rgba, imageSize);
        vkUnmapMemory(device, stagingMemory);
    }

    // Create image
    VkImage image;
    VkDeviceMemory imageMemory;
    {
        VkImageCreateInfo imgInfo{};
        imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imgInfo.imageType = VK_IMAGE_TYPE_2D;
        imgInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        imgInfo.extent = {.width = static_cast<uint32_t>(width), .height = static_cast<uint32_t>(height), .depth = 1};
        imgInfo.mipLevels = 1;
        imgInfo.arrayLayers = 1;
        imgInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imgInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imgInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imgInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        if (vkCreateImage(device, &imgInfo, nullptr, &image) != VK_SUCCESS) {
            vkDestroyBuffer(device, stagingBuffer, nullptr);
            vkFreeMemory(device, stagingMemory, nullptr);
            return VK_NULL_HANDLE;
        }

        VkMemoryRequirements memReqs;
        vkGetImageMemoryRequirements(device, image, &memReqs);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memReqs.size;
        allocInfo.memoryTypeIndex = findMemType(physicalDevice, memReqs.memoryTypeBits,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (vkAllocateMemory(device, &allocInfo, nullptr, &imageMemory) != VK_SUCCESS) {
            vkDestroyImage(device, image, nullptr);
            vkDestroyBuffer(device, stagingBuffer, nullptr);
            vkFreeMemory(device, stagingMemory, nullptr);
            return VK_NULL_HANDLE;
        }
        vkBindImageMemory(device, image, imageMemory, 0);
    }

    // Upload via immediate submit
    immediateSubmit([&](VkCommandBuffer cmd) {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

        VkBufferImageCopy region{};
        region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageExtent = {static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1};
        vkCmdCopyBufferToImage(cmd, stagingBuffer, image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    });

    // Freed now only if the copy has already run. Inside a batch immediateSubmit
    // records and returns without submitting, so destroying the staging here
    // pulls the source out from under a copy that has not happened - the image
    // then contains whatever was left behind, which draws as nothing at all.
    if (inUploadBatch_) {
        deferRawStagingCleanup(stagingBuffer, stagingMemory);
    } else {
        vkDestroyBuffer(device, stagingBuffer, nullptr);
        vkFreeMemory(device, stagingMemory, nullptr);
    }

    // Create image view
    VkImageView imageView;
    {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        viewInfo.subresourceRange = {.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
        if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
            vkDestroyImage(device, image, nullptr);
            vkFreeMemory(device, imageMemory, nullptr);
            return VK_NULL_HANDLE;
        }
    }

    // From this context's own pool rather than ImGui's, so the set survives a
    // backend restart. ImGui only ever binds what ImTextureID points at, and a
    // set built to the same layout binds identically.
    VkDescriptorSet ds = VK_NULL_HANDLE;
    if (ensureUiTextureDescriptorPool()) {
        VkDescriptorSetAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        alloc.descriptorPool = uiTexturePool_;
        alloc.descriptorSetCount = 1;
        alloc.pSetLayouts = &uiTextureLayout_;
        if (vkAllocateDescriptorSets(device, &alloc, &ds) != VK_SUCCESS) {
            ds = VK_NULL_HANDLE;
        } else {
            VkDescriptorImageInfo info{};
            info.sampler = uiTextureSampler_;
            info.imageView = imageView;
            info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            VkWriteDescriptorSet write{};
            write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            write.dstSet = ds;
            write.dstBinding = 0;
            write.descriptorCount = 1;
            write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            write.pImageInfo = &info;
            vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        }
    }

    if (!ds) {
        LOG_ERROR("UI descriptor pool exhausted - cannot upload UI texture");
        vkDestroyImageView(device, imageView, nullptr);
        vkDestroyImage(device, image, nullptr);
        vkFreeMemory(device, imageMemory, nullptr);
        return VK_NULL_HANDLE;
    }

    // Track for cleanup
    uiTextures_.push_back({image, imageMemory, imageView});

    return ds;
}

bool VkContext::recreateSwapchain(int width, int height) {
    vkDeviceWaitIdle(device);

    // Destroy old framebuffers
    for (auto fb : swapchainFramebuffers) {
        if (fb) vkDestroyFramebuffer(device, fb, nullptr);
    }
    swapchainFramebuffers.clear();

    // Destroy old image views
    for (auto iv : swapchainImageViews) {
        if (iv) vkDestroyImageView(device, iv, nullptr);
    }
    swapchainImageViews.clear();

    VkSwapchainKHR oldSwapchain = swapchain;

    vkb::SwapchainBuilder swapchainBuilder{physicalDevice, device, surface};
    auto& builder = swapchainBuilder
        .set_desired_format({.format = VK_FORMAT_B8G8R8A8_UNORM, .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
        .set_desired_extent(static_cast<uint32_t>(width), static_cast<uint32_t>(height))
        .set_image_usage_flags(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT)
        .set_desired_min_image_count(2)
        .set_old_swapchain(oldSwapchain);

    if (vsync_) {
        builder.set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR);
    } else {
        builder.set_desired_present_mode(VK_PRESENT_MODE_IMMEDIATE_KHR);
        builder.add_fallback_present_mode(VK_PRESENT_MODE_MAILBOX_KHR);
        builder.add_fallback_present_mode(VK_PRESENT_MODE_FIFO_RELAXED_KHR);
    }

    auto swapRet = builder.build();

    if (!swapRet) {
        // Destroy old swapchain now that we failed (it can't be used either)
        if (oldSwapchain) {
            vkDestroySwapchainKHR(device, oldSwapchain, nullptr);
            swapchain = VK_NULL_HANDLE;
        }
        LOG_ERROR("Failed to recreate swapchain: ", swapRet.error().message());
        // Keep swapchainDirty=true so the next frame retries
        swapchainDirty = true;
        return false;
    }

    // Success - safe to retire the old swapchain
    if (oldSwapchain) {
        vkDestroySwapchainKHR(device, oldSwapchain, nullptr);
    }

    auto vkbSwap = swapRet.value();
    swapchain = vkbSwap.swapchain;
    swapchainFormat = vkbSwap.image_format;
    swapchainExtent = vkbSwap.extent;
    swapchainImages = vkbSwap.get_images().value();
    swapchainImageViews = vkbSwap.get_image_views().value();

    // Resize per-image semaphore arrays if the swapchain image count changed
    {
        const uint32_t newCount = static_cast<uint32_t>(swapchainImages.size());
        const uint32_t oldCount = static_cast<uint32_t>(imageAcquiredSemaphores_.size());
        VkSemaphoreCreateInfo semInfo{};
        semInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        // Destroy excess semaphores if shrinking
        for (uint32_t i = newCount; i < oldCount; i++) {
            if (imageAcquiredSemaphores_[i]) vkDestroySemaphore(device, imageAcquiredSemaphores_[i], nullptr);
            if (renderFinishedSemaphores_[i]) vkDestroySemaphore(device, renderFinishedSemaphores_[i], nullptr);
        }
        imageAcquiredSemaphores_.resize(newCount);
        renderFinishedSemaphores_.resize(newCount);
        // Create new semaphores if growing
        for (uint32_t i = oldCount; i < newCount; i++) {
            vkCreateSemaphore(device, &semInfo, nullptr, &imageAcquiredSemaphores_[i]);
            vkCreateSemaphore(device, &semInfo, nullptr, &renderFinishedSemaphores_[i]);
        }
    }

    // Recreate depth buffer + MSAA color image + depth resolve image
    destroyMsaaColorImage();
    destroyDepthResolveImage();
    destroyDepthBuffer();

    // Destroy old render pass (needs recreation if MSAA changed)
    destroyOverlayRenderPass();
    if (imguiRenderPass) {
        vkDestroyRenderPass(device, imguiRenderPass, nullptr);
        imguiRenderPass = VK_NULL_HANDLE;
    }

    if (!createSwapchainRenderTargets("recreate")) return false;

    if (!createOverlayRenderPass()) return false;
    if (!createSceneContinueRenderPass()) return false;

    swapchainDirty = false;
    LOG_INFO("Swapchain recreated: ", swapchainExtent.width, "x", swapchainExtent.height);
    return true;
}

void VkContext::resetFrameSyncState() {
    if (device == VK_NULL_HANDLE) return;
    // How many asynchronous upload batches are still outstanding when a
    // rebuild happens. These are submitted without being waited on, one fence
    // each, and FrameXML makes hundreds where this client alone makes almost
    // none - which is the one difference that scales the way the fault does.
    if (!inFlightBatches_.empty()) {
        LOG_WARNING("rebuild with ", inFlightBatches_.size(),
                    " upload batches still in flight (", batchesSubmitted_,
                    " submitted, ", batchesRetired_, " retired)");
    }
    // Checked: if the device is already gone, everything below is theatre and
    // the fence wait in the next frame takes the blame for it.
    if (const VkResult idle = vkDeviceWaitIdle(device); idle != VK_SUCCESS) {
        LOG_ERROR("wait-idle before a rebuild failed: ", static_cast<int>(idle),
                  " - the device was already lost before this rebuild, not by it");
    }

    // Retire the upload batches now. The wait above means every one of them has
    // finished, so this frees each fence, command buffer and staging buffer
    // while the pools they came from are still alive - the same condition
    // flushDeferredCleanup needs, and for the same reason. Left alone they
    // survived the rebuild holding all three, and the only thing that would
    // ever collect them is a later frame happening to poll.
    pollUploadBatches();

    // Everything queued to be freed later can be freed now, because the wait
    // above says the GPU holds nothing. Left queued, these frees sit against
    // frame slots whose fences are about to be remade signalled - so the next
    // visit to each slot releases them on a fence that reports completion by
    // construction rather than because work finished. The pools they free from
    // are still alive at this point, which is the condition flushDeferredCleanup
    // is documented as needing.
    flushDeferredCleanup();

    // The timeline needs none of the surgery below. vkDeviceWaitIdle above
    // means every submit has completed, so the counter has reached
    // frameTimelineValue_; pointing every slot at that value leaves each one
    // already satisfied, which is the state the first frame expects. Nothing
    // is destroyed and the counter keeps running, so no value is ever reused.
    if (frameTimeline_ != VK_NULL_HANDLE) {
        for (auto& f : frames) {
            f.timelineValue = frameTimelineValue_;
        }
    }

    // Recreated rather than reset: a fence has to end up signalled, and
    // vkResetFences only ever unsignals. Destroying and remaking with
    // VK_FENCE_CREATE_SIGNALED_BIT is the state the first frame expects.
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        if (frames[i].inFlightFence) {
            vkDestroyFence(device, frames[i].inFlightFence, nullptr);
            frames[i].inFlightFence = VK_NULL_HANDLE;
        }
        if (vkCreateFence(device, &fenceInfo, nullptr, &frames[i].inFlightFence) != VK_SUCCESS) {
            LOG_ERROR("Could not remake frame fence ", i, " after a rebuild");
        }
        if (frames[i].commandBuffer) {
            vkResetCommandBuffer(frames[i].commandBuffer, 0);
        }
    }
    // The semaphores go the same way, and for the same reason the fences do.
    //
    // A binary semaphore cannot be reset, only waited on. An acquire signals
    // one; if the swapchain is rebuilt before the submit that would have
    // waited on it, it stays signalled with nothing left to consume it. The
    // rebuild only ever created or destroyed these when the image *count*
    // changed, so on a rebuild that keeps the same count - which an MSAA or
    // FSR change does - every one of them carried its state across.
    //
    // Handing an already-signalled semaphore to vkAcquireNextImageKHR is
    // undefined, and the driver answers by losing the device. That is the
    // shape of it: a settings change, a rebuild, then the very next frame
    // failing its fence wait with VK_ERROR_DEVICE_LOST.
    //
    // vkDeviceWaitIdle above guarantees nothing is still using them, so
    // destroying and remaking here is safe and leaves every one unsignalled.
    {
        VkSemaphoreCreateInfo semInfo{};
        semInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        auto remake = [&](VkSemaphore& sem) {
            if (sem) vkDestroySemaphore(device, sem, nullptr);
            sem = VK_NULL_HANDLE;
            if (vkCreateSemaphore(device, &semInfo, nullptr, &sem) != VK_SUCCESS) {
                LOG_ERROR("Could not remake a swapchain semaphore after a rebuild");
            }
        };
        for (auto& sem : imageAcquiredSemaphores_)  remake(sem);
        for (auto& sem : renderFinishedSemaphores_) remake(sem);
        remake(nextAcquireSemaphore_);
        // Not remade: it is one of the per-image ones above, already replaced.
        // Left dangling it would name a semaphore that no longer exists.
        currentAcquireSemaphore_ = VK_NULL_HANDLE;
    }

    currentFrame = 0;
    LOG_WARNING("Frame synchronisation reset after a rebuild: fences signalled, "
                "command buffers reset, back to slot 0");
}

VkCommandBuffer VkContext::beginFrame(uint32_t& imageIndex) {
    if (deviceLost_) return VK_NULL_HANDLE;
    if (swapchain == VK_NULL_HANDLE) return VK_NULL_HANDLE;  // Swapchain lost; recreate pending

    auto& frame = frames[currentFrame];

    // Wait for this frame's fence (with timeout to detect GPU hangs)
    static int beginFrameCounter = 0;
    beginFrameCounter++;
    VkResult fenceResult;
    if (frameTimeline_ != VK_NULL_HANDLE) {
        VkSemaphoreWaitInfo waitInfo{};
        waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
        waitInfo.semaphoreCount = 1;
        waitInfo.pSemaphores = &frameTimeline_;
        waitInfo.pValues = &frame.timelineValue;
        fenceResult = vkWaitSemaphores(device, &waitInfo, 5000000000ULL); // 5 second timeout
    } else {
        fenceResult = vkWaitForFences(device, 1, &frame.inFlightFence, VK_TRUE, 5000000000ULL); // 5 second timeout
    }
    if (fenceResult == VK_TIMEOUT) {
        LOG_ERROR("beginFrame[", beginFrameCounter, "] FENCE TIMEOUT (5s) on frame slot ", currentFrame,
                  " (waiting for timeline ", frame.timelineValue, ") - GPU hang detected!");
        return VK_NULL_HANDLE;
    }
    if (fenceResult != VK_SUCCESS) {
        LOG_ERROR("beginFrame[", beginFrameCounter, "] fence wait failed: ", static_cast<int>(fenceResult));
        if (fenceResult == VK_ERROR_DEVICE_LOST) {
            deviceLost_ = true;
        }
        return VK_NULL_HANDLE;
    }

    // Any work queued for this frame slot is now guaranteed to be unused by the GPU.
    runDeferredCleanup(currentFrame);

    // Acquire next swapchain image using the free semaphore.
    // After acquiring we swap it into the per-image slot so the old per-image
    // semaphore (now released by the presentation engine) becomes the free one.
    VkResult result = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX,
        nextAcquireSemaphore_, VK_NULL_HANDLE, &imageIndex);

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        swapchainDirty = true;
        return VK_NULL_HANDLE;
    }
    if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        LOG_ERROR("Failed to acquire swapchain image");
        return VK_NULL_HANDLE;
    }

    // Swap semaphores: the image's old acquire semaphore is now free (the presentation
    // engine released it when this image was re-acquired).  The semaphore we just used
    // becomes the per-image one for submit/present.
    currentAcquireSemaphore_ = nextAcquireSemaphore_;
    nextAcquireSemaphore_ = imageAcquiredSemaphores_[imageIndex];
    imageAcquiredSemaphores_[imageIndex] = currentAcquireSemaphore_;

    // A timeline only ever moves forward, so there is nothing to unsignal.
    if (frameTimeline_ == VK_NULL_HANDLE) {
        vkResetFences(device, 1, &frame.inFlightFence);
    }
    vkResetCommandBuffer(frame.commandBuffer, 0);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkBeginCommandBuffer(frame.commandBuffer, &beginInfo);

    // If async upload batches are still in flight (submitted to the transfer queue),
    // wait for their fences and insert a memory barrier so the graphics queue sees
    // the completed layout transitions and transfer writes.
    if (!inFlightBatches_.empty()) {
        waitAllUploads();

        VkMemoryBarrier memBarrier{};
        memBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        memBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        memBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(frame.commandBuffer,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0, 1, &memBarrier, 0, nullptr, 0, nullptr);
    }

    return frame.commandBuffer;
}

void VkContext::endFrame(VkCommandBuffer cmd, uint32_t imageIndex) {
    static int endFrameCounter = 0;
    endFrameCounter++;

    VkResult endResult = vkEndCommandBuffer(cmd);
    if (endResult != VK_SUCCESS) {
        LOG_ERROR("endFrame[", endFrameCounter, "] vkEndCommandBuffer FAILED: ", static_cast<int>(endResult));
    }

    auto& frame = frames[currentFrame];

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;

    // Use per-image semaphores: acquire semaphore was swapped into the per-image
    // slot in beginFrame; renderFinished is also indexed by the acquired image.
    VkSemaphore& acquireSem = imageAcquiredSemaphores_[imageIndex];
    VkSemaphore& renderSem = renderFinishedSemaphores_[imageIndex];

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = &acquireSem;
    submitInfo.pWaitDstStageMask = &waitStage;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;

    // Present still needs the binary renderFinished semaphore -- WSI does not
    // take a timeline. So the submit signals both: the binary one for
    // vkQueuePresentKHR, and the timeline for the CPU wait in beginFrame that
    // used to be a fence. The value paired with a binary semaphore is ignored,
    // but the arrays still have to be the same length.
    VkSemaphore signalSemaphores[2] = { renderSem, frameTimeline_ };
    uint64_t signalValues[2] = { 0, 0 };
    const uint64_t waitValue = 0;
    VkTimelineSemaphoreSubmitInfo timelineSubmit{};

    if (frameTimeline_ != VK_NULL_HANDLE) {
        signalValues[1] = ++frameTimelineValue_;
        timelineSubmit.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
        timelineSubmit.waitSemaphoreValueCount = 1;
        timelineSubmit.pWaitSemaphoreValues = &waitValue;
        timelineSubmit.signalSemaphoreValueCount = 2;
        timelineSubmit.pSignalSemaphoreValues = signalValues;
        submitInfo.pNext = &timelineSubmit;
        submitInfo.signalSemaphoreCount = 2;
        submitInfo.pSignalSemaphores = signalSemaphores;
    } else {
        submitInfo.signalSemaphoreCount = 1;
        submitInfo.pSignalSemaphores = &renderSem;
    }

    VkResult submitResult = vkQueueSubmit(graphicsQueue, 1, &submitInfo,
                                          frameTimeline_ != VK_NULL_HANDLE ? VK_NULL_HANDLE
                                                                           : frame.inFlightFence);
    if (submitResult == VK_SUCCESS && frameTimeline_ != VK_NULL_HANDLE) {
        // Only once the submit is in: on failure the timeline is never
        // signalled, and a slot left waiting on an unreachable value would
        // hang the next beginFrame for its whole timeout instead of failing.
        frame.timelineValue = signalValues[1];
    }
    if (submitResult != VK_SUCCESS) {
        LOG_ERROR("endFrame[", endFrameCounter, "] vkQueueSubmit FAILED: ", static_cast<int>(submitResult));
        if (submitResult == VK_ERROR_DEVICE_LOST) {
            deviceLost_ = true;
        }
    }

    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = &renderSem;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain;
    presentInfo.pImageIndices = &imageIndex;

    VkResult result = vkQueuePresentKHR(presentQueue, &presentInfo);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        swapchainDirty = true;
    }

    currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

VkCommandBuffer VkContext::beginSingleTimeCommands() {
    // Lazily allocate once and reuse. The pool was created with
    // VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT so individual buffers
    // can be reset without freeing the underlying allocation.
    if (immCmdBuf_ == VK_NULL_HANDLE) {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = immCommandPool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;
        vkAllocateCommandBuffers(device, &allocInfo, &immCmdBuf_);
    } else {
        vkResetCommandBuffer(immCmdBuf_, 0);
    }

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(immCmdBuf_, &beginInfo);

    return immCmdBuf_;
}

void VkContext::noteImmediateSubmitThread(const char* who) {
    static std::mutex seenMutex;
    static std::set<std::thread::id> seen;
    const std::thread::id self = std::this_thread::get_id();
    std::lock_guard<std::mutex> lock(seenMutex);
    if (seen.insert(self).second && seen.size() > 1) {
        LOG_WARNING("immFence is now being used from ", seen.size(),
                    " threads (latest via ", who, ") - it is shared and "
                    "unguarded, so two of them can reset a fence the other "
                    "is waiting on");
    }
}

void VkContext::endSingleTimeCommands(VkCommandBuffer cmd) {
    vkEndCommandBuffer(cmd);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;

    // immFence and the immediate command pool are shared and unguarded. If two
    // threads reach here at once, one resets a fence the other is waiting on -
    // which is what validation reports as VUID-vkResetFences-pFences-01123,
    // and the driver answers by losing the device. Said once per thread so a
    // log shows whether that is happening.
    noteImmediateSubmitThread("endSingleTimeCommands");

    // Checked, because this is where the first sign of trouble goes missing.
    //
    // A log of a lost device begins with validation complaining that immFence
    // is being reset while still in use - which is what happens *after* a wait
    // that returned an error rather than waiting. The submit and the wait were
    // both unchecked, so whatever actually went wrong left no line at all and
    // the reset took the blame for it.
    //
    // Said once. If the device is gone, every subsequent call fails the same
    // way and a log full of it buries the first one.
    static bool reported = false;
    const VkResult submitted = vkQueueSubmit(graphicsQueue, 1, &submitInfo, immFence);
    if (submitted != VK_SUCCESS && !reported) {
        reported = true;
        LOG_ERROR("immediate submit failed: ", static_cast<int>(submitted),
                  " - this is the first failure, whatever follows is its wake");
    }
    const VkResult waited = vkWaitForFences(device, 1, &immFence, VK_TRUE, UINT64_MAX);
    if (waited != VK_SUCCESS && !reported) {
        reported = true;
        LOG_ERROR("immediate wait failed: ", static_cast<int>(waited),
                  " (VK_ERROR_DEVICE_LOST is -4) - the fence is not signalled,"
                  " so the reset below is the symptom rather than the cause");
    }
    vkResetFences(device, 1, &immFence);
    // Buffer stays allocated; it will be reset on the next beginSingleTimeCommands.
}

void VkContext::immediateSubmit(std::function<void(VkCommandBuffer cmd)>&& function) {
    if (inUploadBatch_) {
        // Record into the batch command buffer - no submit, no fence wait.
        // Opened on demand, so a batch nothing writes to costs nothing.
        ensureBatchCmd();
        if (batchCmd_ != VK_NULL_HANDLE) function(batchCmd_);
        return;
    }
    VkCommandBuffer cmd = beginSingleTimeCommands();
    function(cmd);
    endSingleTimeCommands(cmd);
}

void VkContext::beginUploadBatch() {
    // WOWEE_VK_NO_UPLOAD_BATCH=1 turns batching off entirely: every
    // immediateSubmit then submits and waits on its own, as it did before the
    // batch path existed.
    //
    // A diagnostic rather than a setting, and here because three device losses
    // in a row have landed within a second of the first batch of the session -
    // at frames 803, 1194 and 8916, so it is the batch and not the frame count
    // - and moving the submit to the graphics queue did not change it. This
    // separates "the batch path is implicated" from "something else at world
    // entry is", which is a question no amount of reading has settled.
    //
    // Slow, because it is the path the batching replaced. Expect a long load.
    static const bool noBatch = [] {
        const char* v = std::getenv("WOWEE_VK_NO_UPLOAD_BATCH");
        return v && *v && *v != '0';
    }();
    if (noBatch) return;

    uploadBatchDepth_++;
    if (inUploadBatch_) return; // already in a batch (nested call)
    inUploadBatch_ = true;
    // The command buffer is not allocated here.
    //
    // A batch now wraps the whole interface render, which happens every frame
    // and usually uploads nothing at all. Allocating and freeing a command
    // buffer to record nothing into, sixty times a second, is churn the pool
    // does not need. ensureBatchCmd() opens one the first time something
    // actually records, and the end calls treat a null buffer as an empty
    // batch.
}

/// Opens the batch's command buffer if nothing has recorded into one yet.
void VkContext::ensureBatchCmd() {
    if (batchCmd_ != VK_NULL_HANDLE) return;
    // From the transfer pool where there is one, otherwise the immediate pool.
    VkCommandPool pool = hasDedicatedTransfer_ ? transferCommandPool_ : immCommandPool;

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = pool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;
    if (vkAllocateCommandBuffers(device, &allocInfo, &batchCmd_) != VK_SUCCESS) {
        batchCmd_ = VK_NULL_HANDLE;
        return;
    }

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(batchCmd_, &beginInfo);
}

void VkContext::endUploadBatch() {
    if (uploadBatchDepth_ <= 0) return;
    uploadBatchDepth_--;
    if (uploadBatchDepth_ > 0) return; // still inside an outer batch

    inUploadBatch_ = false;

    // Nothing ever recorded, so there is nothing to end, submit or free.
    if (batchCmd_ == VK_NULL_HANDLE) {
        batchStagingBuffers_.clear();
        batchRawStaging_.clear();
        return;
    }

    VkCommandPool pool = hasDedicatedTransfer_ ? transferCommandPool_ : immCommandPool;

    // Raw staging counts too. Checking only the allocator's list meant a batch
    // holding nothing but ImGui texture uploads looked empty, so the command
    // buffer holding every one of those copies was thrown away unsubmitted and
    // the images stayed blank.
    if (batchStagingBuffers_.empty() && batchRawStaging_.empty()) {
        // No GPU copies were recorded - skip the submit entirely.
        vkEndCommandBuffer(batchCmd_);
        vkFreeCommandBuffers(device, pool, 1, &batchCmd_);
        batchCmd_ = VK_NULL_HANDLE;
        return;
    }

    // Submit commands with a NEW fence - don't wait, let GPU work in parallel.
    vkEndCommandBuffer(batchCmd_);

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    VkFence fence = VK_NULL_HANDLE;
    vkCreateFence(device, &fenceInfo, nullptr, &fence);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &batchCmd_;

    // On the graphics queue, not the transfer one, unless asked otherwise.
    //
    // Both queues are in the same family, so no ownership transfer is needed -
    // that much of the setup comment is right. But same family is not same
    // queue: two queues run independently, and nothing here ordered them. The
    // upload was submitted with a fence and no semaphore, the fence only ever
    // used to free staging afterwards, so the graphics queue could sample a
    // texture while the transfer queue was still writing it. Undefined, and on
    // this driver it hangs.
    //
    // Submitting to the same queue the rendering uses makes submission order
    // the execution order, and the layout transitions already recorded in this
    // command buffer do the rest. It costs the parallelism the split was for.
    //
    // Why it surfaced now: this client alone makes almost no async batches, and
    // FrameXML makes hundreds. Both device losses under investigation landed
    // within a second of the *first* batch of the session.
    //
    // WOWEE_VK_ASYNC_UPLOAD_QUEUE=1 restores the old behaviour, for confirming
    // that is what this was rather than taking it on argument.
    static const bool asyncUploadQueue = [] {
        const char* v = std::getenv("WOWEE_VK_ASYNC_UPLOAD_QUEUE");
        return v && *v && *v != '0';
    }();
    VkQueue targetQueue = (hasDedicatedTransfer_ && asyncUploadQueue) ? transferQueue_ : graphicsQueue;
    vkQueueSubmit(targetQueue, 1, &submitInfo, fence);

    // Said once, with the handle, because these are the only fences created
    // after startup - so a validation message naming a high handle is one of
    // these rather than a frame fence or the immediate one. FrameXML makes
    // hundreds of them; without it there are almost none, which is the shape
    // of the difference between the two branches.
    if (batchesSubmitted_ == 0) {
        LOG_WARNING("first async upload batch fence = 0x", std::hex,
                    reinterpret_cast<uint64_t>(fence), std::dec);
    }
    ++batchesSubmitted_;

    // Stash everything for later cleanup when fence signals
    InFlightBatch batch;
    batch.fence = fence;
    batch.cmd = batchCmd_;
    batch.stagingBuffers = std::move(batchStagingBuffers_);
    batch.rawStaging = std::move(batchRawStaging_);
    inFlightBatches_.push_back(std::move(batch));

    batchCmd_ = VK_NULL_HANDLE;
    batchStagingBuffers_.clear();
    batchRawStaging_.clear();
}

void VkContext::endUploadBatchSync() {
    if (uploadBatchDepth_ <= 0) return;
    uploadBatchDepth_--;
    if (uploadBatchDepth_ > 0) return;

    inUploadBatch_ = false;

    // Nothing ever recorded, so there is nothing to end, submit or free.
    if (batchCmd_ == VK_NULL_HANDLE) {
        batchStagingBuffers_.clear();
        batchRawStaging_.clear();
        return;
    }

    VkCommandPool pool = hasDedicatedTransfer_ ? transferCommandPool_ : immCommandPool;

    if (batchStagingBuffers_.empty() && batchRawStaging_.empty()) {
        vkEndCommandBuffer(batchCmd_);
        vkFreeCommandBuffers(device, pool, 1, &batchCmd_);
        batchCmd_ = VK_NULL_HANDLE;
        return;
    }

    // Synchronous path for load screens - submit and wait on the target queue.
    VkQueue targetQueue = hasDedicatedTransfer_ ? transferQueue_ : graphicsQueue;

    vkEndCommandBuffer(batchCmd_);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &batchCmd_;

    // Its own fence, not the shared immediate one.
    //
    // immFence is also used by endSingleTimeCommands, which submits on the
    // graphics queue while this submits on the transfer queue when there is
    // one. Validation reports that fence - 0x170000000017, named at creation
    // - being reset while still in use, repeatedly, just before the device is
    // lost. Two queues signalling one fence is the fragility whatever the
    // exact interleaving; a fence per submit has no such question about it.
    //
    // FrameXML is what makes this reachable: it uploads hundreds of textures
    // through here, where this client alone uploads a handful.
    VkFenceCreateInfo batchFenceInfo{};
    batchFenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    VkFence batchFence = VK_NULL_HANDLE;
    if (vkCreateFence(device, &batchFenceInfo, nullptr, &batchFence) != VK_SUCCESS) {
        LOG_ERROR("Could not create an upload fence; falling back to the shared one");
        batchFence = immFence;
    }

    vkQueueSubmit(targetQueue, 1, &submitInfo, batchFence);
    vkWaitForFences(device, 1, &batchFence, VK_TRUE, UINT64_MAX);
    if (batchFence != immFence) {
        vkDestroyFence(device, batchFence, nullptr);
    } else {
        vkResetFences(device, 1, &immFence);
    }

    vkFreeCommandBuffers(device, pool, 1, &batchCmd_);
    batchCmd_ = VK_NULL_HANDLE;

    for (auto& staging : batchStagingBuffers_) {
        destroyBuffer(allocator, staging);
    }
    batchStagingBuffers_.clear();
    // The copies have completed by here, so the sources can go.
    freeRawStaging();
}

void VkContext::pollUploadBatches() {
    if (inFlightBatches_.empty()) return;

    VkCommandPool pool = hasDedicatedTransfer_ ? transferCommandPool_ : immCommandPool;

    for (auto it = inFlightBatches_.begin(); it != inFlightBatches_.end(); ) {
        VkResult result = vkGetFenceStatus(device, it->fence);
        if (result == VK_SUCCESS) {
            // GPU finished - free resources
            for (auto& raw : it->rawStaging) {
                vkDestroyBuffer(device, raw.buffer, nullptr);
                vkFreeMemory(device, raw.memory, nullptr);
            }
            for (auto& staging : it->stagingBuffers) {
                destroyBuffer(allocator, staging);
            }
            vkFreeCommandBuffers(device, pool, 1, &it->cmd);
            vkDestroyFence(device, it->fence, nullptr);
            it = inFlightBatches_.erase(it);
            ++batchesRetired_;
        } else {
            ++it;
        }
    }
}

void VkContext::waitAllUploads() {
    VkCommandPool pool = hasDedicatedTransfer_ ? transferCommandPool_ : immCommandPool;

    for (auto& batch : inFlightBatches_) {
        vkWaitForFences(device, 1, &batch.fence, VK_TRUE, UINT64_MAX);
        for (auto& raw : batch.rawStaging) {
            vkDestroyBuffer(device, raw.buffer, nullptr);
            vkFreeMemory(device, raw.memory, nullptr);
        }
        for (auto& staging : batch.stagingBuffers) {
            destroyBuffer(allocator, staging);
        }
        vkFreeCommandBuffers(device, pool, 1, &batch.cmd);
        vkDestroyFence(device, batch.fence, nullptr);
        // Counted, because the rebuild warning reports submitted against
        // retired and this path used to clear the list without saying so. It
        // read as a hundred and thirty-seven batches outstanding while one was,
        // which is a number that invites exactly the wrong conclusion.
        ++batchesRetired_;
    }
    inFlightBatches_.clear();
}

void VkContext::deferStagingCleanup(AllocatedBuffer staging) {
    batchStagingBuffers_.push_back(staging);
}

void VkContext::deferRawStagingCleanup(VkBuffer buffer, VkDeviceMemory memory) {
    batchRawStaging_.push_back({buffer, memory});
}

void VkContext::freeRawStaging() {
    for (const RawStaging& s : batchRawStaging_) {
        vkDestroyBuffer(device, s.buffer, nullptr);
        vkFreeMemory(device, s.memory, nullptr);
    }
    batchRawStaging_.clear();
}

} // namespace rendering
} // namespace wowee
