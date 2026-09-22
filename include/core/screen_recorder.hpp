#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace wowee {
namespace core {

/// The size a recording is encoded at, from the size of the window.
///
/// No taller than maxHeight, the width scaled to keep the window's shape: a
/// Retina window is 3200 by 1800 or more, which is four times the pixels of
/// 1080p to read back and encode every frame for a file nobody watches at that
/// size. Both sides even, because the 4:2:0 video every player expects stores
/// colour at half resolution in each direction and an odd side has no half.
struct RecordingSize {
    uint32_t width = 0;
    uint32_t height = 0;
};

inline RecordingSize recordingFrameSize(uint32_t windowWidth, uint32_t windowHeight,
                                        uint32_t maxHeight = 1080) {
    if (windowWidth < 2 || windowHeight < 2) return {};
    uint32_t width = windowWidth;
    uint32_t height = windowHeight;
    if (height > maxHeight) {
        width = static_cast<uint32_t>(static_cast<uint64_t>(windowWidth) * maxHeight / windowHeight);
        height = maxHeight;
    }
    return {.width = std::max(2u, width & ~1u), .height = std::max(2u, height & ~1u)};
}

/**
 * Writes what the client shows, and what it plays, to a video file.
 *
 * Frames arrive from the renderer as BGRA rows it has read back from the GPU;
 * the sound comes from AudioEngine's output tap. Both are encoded on a thread
 * of this recorder's own - H.264 where the platform has an encoder for it,
 * VideoToolbox on a Mac, and MPEG-4 part 2 anywhere FFmpeg is at all - with AAC
 * sound, into an MP4.
 *
 * Built on FFmpeg, which the client already links; a build without it has the
 * class and every call answers that recording is unavailable.
 */
class ScreenRecorder {
public:
    /// One frame, in memory the recorder reads and then gives back through
    /// release - from its own thread, so release must be safe there.
    struct Frame {
        const uint8_t* bgra = nullptr;
        uint32_t stride = 0;   ///< bytes from one row to the next
        int64_t pts = 0;       ///< in frames at frameRate(), from the start
        std::function<void()> release;
    };

    struct Stats {
        uint64_t framesWritten = 0;
        uint64_t framesDropped = 0;
        double seconds = 0.0;
        bool hasAudio = false;
        std::string encoder;
    };

    ScreenRecorder();
    ~ScreenRecorder();
    ScreenRecorder(const ScreenRecorder&) = delete;
    ScreenRecorder& operator=(const ScreenRecorder&) = delete;

    /// Whether this build can record at all.
    static bool compiledIn();

    /// Open the file and the encoders and start the encoding thread. False,
    /// with the reason in error, when any of it fails.
    bool start(const std::string& path, RecordingSize size, std::string& error);

    /// Finish the file - every queued frame encoded, the encoders flushed, the
    /// index written - and close it. Blocks until that is done.
    Stats stop();

    [[nodiscard]] bool isRecording() const;
    /// The encoding thread gave up: the disk filled, an encoder refused a
    /// frame. The file so far is still finished properly by stop().
    [[nodiscard]] bool failed() const;
    [[nodiscard]] std::string failure() const;
    [[nodiscard]] double elapsedSeconds() const;

    /// The frame due now, if one is: its pts goes in *pts. Frames are taken
    /// at the recording's rate however fast the client draws, and none at all
    /// while it draws slower - a still frame is held, not repeated.
    bool frameDue(int64_t* pts) const;
    /// That frame was captured and will be submitted when the GPU is done.
    void frameTaken(int64_t pts);
    /// That frame could not be captured - every readback buffer was still with
    /// the encoder.
    void frameDropped();

    /// Hand over a captured frame. Never blocks; a frame the recorder cannot
    /// take is released at once and counted as dropped.
    void submit(Frame frame);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace core
} // namespace wowee
