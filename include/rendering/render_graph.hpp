#pragma once

#include <vulkan/vulkan.h>
#include <string>
#include <vector>
#include <functional>
#include <cstdint>

namespace wowee {
namespace rendering {

// Lightweight Render Graph / Frame Graph
// Converts hardcoded pass sequence (shadow → reflection → compute cull →
// main → post-process → ImGui → present) into declarative graph nodes.
// Graph auto-inserts VkImageMemoryBarrier2 between passes.

// Resource handle - identifies a virtual resource (image or buffer) within the graph.
struct RGResource {
    uint32_t id = UINT32_MAX;
    [[nodiscard]] bool valid() const { return id != UINT32_MAX; }
};

// Image barrier descriptor for automatic synchronization between passes.
struct RGImageBarrier {
    VkImage image;
    VkImageLayout oldLayout;
    VkImageLayout newLayout;
    VkAccessFlags srcAccess;
    VkAccessFlags dstAccess;
    VkPipelineStageFlags srcStage;
    VkPipelineStageFlags dstStage;
    VkImageAspectFlags aspectMask;
};

// Buffer barrier descriptor for automatic synchronization between passes.
struct RGBufferBarrier {
    VkBuffer buffer;
    VkDeviceSize offset;
    VkDeviceSize size;
    VkAccessFlags srcAccess;
    VkAccessFlags dstAccess;
    VkPipelineStageFlags srcStage;
    VkPipelineStageFlags dstStage;
};

// Render pass node - wraps an execution callback with declared inputs/outputs.
struct RGPass {
    std::string name;
    std::vector<RGResource> inputs;
    std::vector<RGResource> outputs;
    std::function<void(VkCommandBuffer cmd)> execute;
    bool enabled = true; // Can be dynamically disabled per-frame

    // Barriers to insert before this pass executes
    std::vector<RGImageBarrier> imageBarriers;
    std::vector<RGBufferBarrier> bufferBarriers;
};

class RenderGraph {
public:
    RenderGraph() = default;
    ~RenderGraph() = default;

    // Reset graph for a new frame (clears passes, keeps resource registry).
    void reset();

    // Register a virtual resource (returns handle for input/output declarations).
    RGResource registerResource(const std::string& name);

    // Look up a previously registered resource by name.
    [[nodiscard]] RGResource findResource(const std::string& name) const;

    // Add a render pass node.
    // inputs: resources this pass reads from
    // outputs: resources this pass writes to
    // execute: callback invoked with the frame's command buffer
    void addPass(const std::string& name,
                 const std::vector<RGResource>& inputs,
                 const std::vector<RGResource>& outputs,
                 std::function<void(VkCommandBuffer cmd)> execute);

    // Enable/disable a pass by name (for dynamic toggling, e.g. shadows off).
    void setPassEnabled(const std::string& name, bool enabled);

    // Compile: topological sort by dependency order, insert barriers.
    // Must be called after all addPass() calls and before execute().
    void compile();

    // Execute all enabled passes in compiled order on the given command buffer.
    void execute(VkCommandBuffer cmd);

    // Query: get the compiled execution order (pass names, for debug HUD).
    [[nodiscard]] const std::vector<uint32_t>& getExecutionOrder() const { return executionOrder_; }
    [[nodiscard]] const std::vector<RGPass>& getPasses() const { return passes_; }

private:
    // Topological sort helper (Kahn's algorithm).
    void topologicalSort();

    // Resource registry: name → id
    struct ResourceEntry {
        std::string name;
        uint32_t id;
    };
    std::vector<ResourceEntry> resources_;
    uint32_t nextResourceId_ = 0;

    // Pass storage
    std::vector<RGPass> passes_;

    // Compiled execution order (indices into passes_)
    std::vector<uint32_t> executionOrder_;
    bool compiled_ = false;
};

} // namespace rendering
} // namespace wowee
