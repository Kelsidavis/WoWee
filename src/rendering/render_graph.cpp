#include "rendering/render_graph.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <unordered_map>
#include <queue>

namespace wowee {
namespace rendering {

void RenderGraph::reset() {
    passes_.clear();
    executionOrder_.clear();
    compiled_ = false;
    // Keep resource registry - resources are stable across frames
}

RGResource RenderGraph::registerResource(const std::string& name) {
    // Check for duplicate
    for (const auto& r : resources_) {
        if (r.name == name) return {r.id};
    }
    uint32_t id = nextResourceId_++;
    resources_.push_back({.name = name, .id = id});
    return {id};
}

RGResource RenderGraph::findResource(const std::string& name) const {
    for (const auto& r : resources_) {
        if (r.name == name) return {r.id};
    }
    return {}; // invalid
}

void RenderGraph::addPass(const std::string& name,
                          const std::vector<RGResource>& inputs,
                          const std::vector<RGResource>& outputs,
                          std::function<void(VkCommandBuffer cmd)> execute) {
    RGPass pass;
    pass.name = name;
    pass.inputs = inputs;
    pass.outputs = outputs;
    pass.execute = std::move(execute);
    pass.enabled = true;
    passes_.push_back(std::move(pass));
}

void RenderGraph::setPassEnabled(const std::string& name, bool enabled) {
    for (auto& pass : passes_) {
        if (pass.name == name) {
            pass.enabled = enabled;
            return;
        }
    }
}

void RenderGraph::compile() {
    topologicalSort();
    compiled_ = true;
}

void RenderGraph::topologicalSort() {
    const uint32_t n = static_cast<uint32_t>(passes_.size());
    if (n == 0) { executionOrder_.clear(); return; }

    // Build adjacency: if pass A outputs resource R and pass B inputs resource R,
    // then A must execute before B (edge A → B).
    // Map: resource id → index of pass that produces it
    std::unordered_map<uint32_t, uint32_t> producer;
    for (uint32_t i = 0; i < n; ++i) {
        for (const auto& out : passes_[i].outputs) {
            producer[out.id] = i;
        }
    }

    // Build in-degree and adjacency list
    std::vector<uint32_t> inDegree(n, 0);
    std::vector<std::vector<uint32_t>> adj(n);

    for (uint32_t i = 0; i < n; ++i) {
        for (const auto& inp : passes_[i].inputs) {
            auto it = producer.find(inp.id);
            if (it != producer.end() && it->second != i) {
                adj[it->second].push_back(i);
                inDegree[i]++;
            }
        }
    }

    // Kahn's algorithm
    std::queue<uint32_t> queue;
    for (uint32_t i = 0; i < n; ++i) {
        if (inDegree[i] == 0) queue.push(i);
    }

    executionOrder_.clear();
    executionOrder_.reserve(n);

    while (!queue.empty()) {
        uint32_t u = queue.front();
        queue.pop();
        executionOrder_.push_back(u);
        for (uint32_t v : adj[u]) {
            if (--inDegree[v] == 0) queue.push(v);
        }
    }

    // If not all passes are in the order, there's a cycle - fall back to insertion order
    if (executionOrder_.size() != n) {
        LOG_WARNING("RenderGraph: dependency cycle detected, falling back to insertion order");
        executionOrder_.clear();
        for (uint32_t i = 0; i < n; ++i) executionOrder_.push_back(i);
    }
}

void RenderGraph::execute(VkCommandBuffer cmd) {
    if (!compiled_) {
        LOG_WARNING("RenderGraph::execute called without compile()");
        compile();
    }

    for (uint32_t idx : executionOrder_) {
        const auto& pass = passes_[idx];
        if (!pass.enabled) continue;

        // Insert image barriers declared for this pass
        if (!pass.imageBarriers.empty()) {
            std::vector<VkImageMemoryBarrier> barriers;
            barriers.reserve(pass.imageBarriers.size());

            VkPipelineStageFlags srcStages = 0;
            VkPipelineStageFlags dstStages = 0;

            for (const auto& b : pass.imageBarriers) {
                VkImageMemoryBarrier ib{};
                ib.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
                ib.oldLayout = b.oldLayout;
                ib.newLayout = b.newLayout;
                ib.srcAccessMask = b.srcAccess;
                ib.dstAccessMask = b.dstAccess;
                ib.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                ib.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                ib.image = b.image;
                ib.subresourceRange = {.aspectMask = b.aspectMask, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1};
                barriers.push_back(ib);
                srcStages |= b.srcStage;
                dstStages |= b.dstStage;
            }

            vkCmdPipelineBarrier(cmd,
                srcStages, dstStages,
                0,
                0, nullptr,
                0, nullptr,
                static_cast<uint32_t>(barriers.size()), barriers.data());
        }

        // Insert buffer barriers declared for this pass
        if (!pass.bufferBarriers.empty()) {
            std::vector<VkBufferMemoryBarrier> barriers;
            barriers.reserve(pass.bufferBarriers.size());

            VkPipelineStageFlags srcStages = 0;
            VkPipelineStageFlags dstStages = 0;

            for (const auto& b : pass.bufferBarriers) {
                VkBufferMemoryBarrier bb{};
                bb.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
                bb.srcAccessMask = b.srcAccess;
                bb.dstAccessMask = b.dstAccess;
                bb.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                bb.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                bb.buffer = b.buffer;
                bb.offset = b.offset;
                bb.size = b.size;
                barriers.push_back(bb);
                srcStages |= b.srcStage;
                dstStages |= b.dstStage;
            }

            vkCmdPipelineBarrier(cmd,
                srcStages, dstStages,
                0,
                0, nullptr,
                static_cast<uint32_t>(barriers.size()), barriers.data(),
                0, nullptr);
        }

        // Execute the pass
        pass.execute(cmd);
    }
}

} // namespace rendering
} // namespace wowee
