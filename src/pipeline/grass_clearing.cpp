#include "pipeline/grass_clearing.hpp"

#include <algorithm>
#include <cmath>

namespace wowee {
namespace pipeline {

void GrassClearingField::clear() {
    sources_.clear();
    cells_.clear();
    cols_ = 0;
    rows_ = 0;
}

void GrassClearingField::build(std::vector<GrassClearingSource> sources,
                               float minX, float minY, float maxX, float maxY) {
    clear();
    if (maxX <= minX || maxY <= minY) return;

    originX_ = minX;
    originY_ = minY;
    cols_ = std::max(1, static_cast<int>(std::ceil((maxX - minX) / cellSize_)));
    rows_ = std::max(1, static_cast<int>(std::ceil((maxY - minY) / cellSize_)));
    cells_.resize(static_cast<size_t>(cols_) * static_cast<size_t>(rows_));

    for (auto& s : sources) {
        const float reach = s.clearing + s.ease;
        // Sources whose influence cannot reach the window cost lookups for
        // nothing; a window is rebuilt often enough that dropping them here
        // is cheaper than testing them per blade.
        if (s.maxX + reach < minX || s.minX - reach > maxX ||
            s.maxY + reach < minY || s.minY - reach > maxY) {
            continue;
        }
        const auto idx = static_cast<uint32_t>(sources_.size());
        sources_.push_back(s);

        const int x0 = std::clamp(
            static_cast<int>((s.minX - reach - originX_) / cellSize_), 0, cols_ - 1);
        const int x1 = std::clamp(
            static_cast<int>((s.maxX + reach - originX_) / cellSize_), 0, cols_ - 1);
        const int y0 = std::clamp(
            static_cast<int>((s.minY - reach - originY_) / cellSize_), 0, rows_ - 1);
        const int y1 = std::clamp(
            static_cast<int>((s.maxY + reach - originY_) / cellSize_), 0, rows_ - 1);
        for (int cy = y0; cy <= y1; ++cy) {
            for (int cx = x0; cx <= x1; ++cx) {
                cells_[static_cast<size_t>(cy) * cols_ + cx].push_back(idx);
            }
        }
    }
}

float GrassClearingField::wildness(float x, float y) const {
    if (cells_.empty()) return 1.0f;
    const int cx = static_cast<int>((x - originX_) / cellSize_);
    const int cy = static_cast<int>((y - originY_) / cellSize_);
    if (cx < 0 || cx >= cols_ || cy < 0 || cy >= rows_) return 1.0f;

    float wild = 1.0f;
    for (const uint32_t idx : cells_[static_cast<size_t>(cy) * cols_ + cx]) {
        const GrassClearingSource& s = sources_[idx];
        const float dx = std::max({s.minX - x, x - s.maxX, 0.0f});
        const float dy = std::max({s.minY - y, y - s.maxY, 0.0f});
        const float d = std::sqrt(dx * dx + dy * dy);
        if (d <= s.clearing) return 0.0f;
        if (s.ease <= 0.0f) continue;
        const float t = std::clamp((d - s.clearing) / s.ease, 0.0f, 1.0f);
        wild = std::min(wild, t * t * (3.0f - 2.0f * t));
    }
    return wild;
}

} // namespace pipeline
} // namespace wowee
