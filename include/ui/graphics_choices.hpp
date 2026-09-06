#pragma once

/// The values behind the graphics choices the options panel offers.
///
/// A dropdown offers words and stores the index of the one chosen; something
/// else turns that index into the thing it means. Both halves are the same
/// fact and both were written out twice - the sample counts and the FSR scale
/// factors each appeared in the settings panel that applies a changed setting
/// and again in the screen that applies the pending ones at startup.
///
/// Two copies of a lookup table indexed by a stored number is the worst shape
/// this can take: the lists drifting apart does not fail, it applies the wrong
/// one. Picking 4x multisampling would quietly give 2x, and only on one of the
/// two paths.
///
/// The index is clamped here rather than at each caller. One of the two did
/// clamp and one did not, and the one that did not read past the end of its
/// array for any stored value above the last choice.

#include <algorithm>
#include <cstddef>
#include <iterator>

#include <vulkan/vulkan.h>

namespace wowee::ui {

/// The multisample counts the "antialiasing" choice offers, in its order.
///
/// Four of them, matching the schema's "Off|2x MSAA|4x MSAA|8x MSAA".
inline VkSampleCountFlagBits msaaSamplesForChoice(int choice) {
    static constexpr VkSampleCountFlagBits kSamples[] = {
        VK_SAMPLE_COUNT_1_BIT, VK_SAMPLE_COUNT_2_BIT,
        VK_SAMPLE_COUNT_4_BIT, VK_SAMPLE_COUNT_8_BIT};
    constexpr int kCount = static_cast<int>(std::size(kSamples));
    return kSamples[std::clamp(choice, 0, kCount - 1)];
}

/// How far below the display resolution the world is drawn, per "fsrquality".
///
/// Four, matching the schema's "Ultra Quality (77%)|Quality (67%)|Balanced
/// (59%)|Native (100%)" - and native is last rather than first, which is the
/// order the panel lists them in and not the order the numbers run.
/// The frame limit each choice index means, or 0 for unlimited.
///
/// Kept beside the other choice tables rather than written out at the call
/// site, so the schema's choice string and the number it stands for cannot
/// drift apart - which is the whole failure mode of an index-valued setting.
inline int frameCapFpsForChoice(int choice) {
    switch (choice) {
        case 1:  return 30;
        case 2:  return 60;
        case 3:  return 90;
        case 4:  return 120;
        case 5:  return 144;
        case 6:  return 240;
        default: return 0;   // Unlimited
    }
}

inline float fsrScaleForChoice(int choice) {
    static constexpr float kScaleFactors[] = {0.77f, 0.67f, 0.59f, 1.00f};
    constexpr int kCount = static_cast<int>(std::size(kScaleFactors));
    return kScaleFactors[std::clamp(choice, 0, kCount - 1)];
}

/// How many choices each of the two offers, so a test can hold them against
/// the schema's own list rather than against a number written here.
inline constexpr int kMsaaChoiceCount = 4;
inline constexpr int kFsrQualityChoiceCount = 4;

/// Which choice a sample count is, or the nearest one below it.
///
/// The inverse of msaaSamplesForChoice, for when a device grants less than
/// was asked: Apple silicon stops at 4x, so the 8x this offers has to come
/// back as the choice actually in force rather than leave the control naming
/// a mode the client is not using.
inline int msaaChoiceForSamples(VkSampleCountFlagBits samples) {
    int best = 0;
    for (int i = 0; i < kMsaaChoiceCount; ++i) {
        if (msaaSamplesForChoice(i) <= samples) best = i;
    }
    return best;
}

}  // namespace wowee::ui
