#include "core/screen_recorder.hpp"
#include "audio/audio_engine.hpp"
#include "core/logger.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#ifdef HAVE_FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavutil/version.h>
#include <libswscale/swscale.h>
}
// AVChannelLayout and the codec context's ch_layout arrived in FFmpeg 5.1. An
// older one still builds the client, just without recording.
#if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(57, 28, 100)
#define WOWEE_RECORDER_ENCODES 1
#endif
#endif

namespace wowee {
namespace core {

namespace {
using Clock = std::chrono::steady_clock;
/// Frames the encoder may fall behind by before new ones are dropped. The
/// renderer's readback pool is smaller than this, so it is a backstop.
constexpr size_t kMaxQueuedFrames = 8;
}  // namespace

struct ScreenRecorder::Impl {
    mutable std::mutex mutex;
    std::condition_variable wake;
    std::deque<Frame> queue;
    bool stopping = false;
    bool recording = false;
    std::thread worker;

    Clock::time_point startTime;
    int fps = 30;
    int64_t lastPts = -1;
    std::atomic<uint64_t> written{0};
    std::atomic<uint64_t> dropped{0};
    std::atomic<bool> failed{false};
    std::string failure;  // under mutex
    std::string encoderName;
    bool hasAudio = false;

    void fail(const std::string& why) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (failure.empty()) failure = why;
        }
        failed.store(true);
        LOG_WARNING("ScreenRecorder: ", why);
    }

#ifdef WOWEE_RECORDER_ENCODES
    AVFormatContext* format = nullptr;
    AVPacket* packet = nullptr;

    AVCodecContext* video = nullptr;
    AVStream* videoStream = nullptr;
    AVFrame* videoFrame = nullptr;
    SwsContext* sws = nullptr;
    RecordingSize size;

    AVCodecContext* audio = nullptr;
    AVStream* audioStream = nullptr;
    AVFrame* audioFrame = nullptr;
    int64_t audioPts = 0;
    uint32_t audioChannels = 0;
    std::vector<float> audioScratch;

    static std::string errorText(int code) {
        char buf[AV_ERROR_MAX_STRING_SIZE] = {};
        av_strerror(code, buf, sizeof(buf));
        return buf;
    }

    bool openVideo(std::string& error);
    bool openAudio();
    bool write(AVCodecContext* ctx, AVStream* stream, AVFrame* frame);
    bool encodeVideo(const Frame& frame);
    void drainAudio();
    void run();
    void finish();
    void release();
#endif
};

#ifdef WOWEE_RECORDER_ENCODES

bool ScreenRecorder::Impl::openVideo(std::string& error) {
    // Hardware first, where the platform has it: it costs the game almost
    // nothing and keeps 60 frames a second. Then x264 if FFmpeg was built
    // with it, and last FFmpeg's own MPEG-4, which every build has and which
    // is cheap enough to hold 30 frames a second in software.
    struct Candidate {
        const char* name;
        AVPixelFormat pixels;
        bool fast;
    };
    static const Candidate kCandidates[] = {
#if defined(__APPLE__)
        {.name = "h264_videotoolbox", .pixels = AV_PIX_FMT_NV12, .fast = true},
#elif defined(_WIN32)
        {.name = "h264_nvenc", .pixels = AV_PIX_FMT_NV12, .fast = true},
        {.name = "h264_amf", .pixels = AV_PIX_FMT_NV12, .fast = true},
        {.name = "h264_qsv", .pixels = AV_PIX_FMT_NV12, .fast = true},
        {.name = "h264_mf", .pixels = AV_PIX_FMT_NV12, .fast = true},
#else
        {.name = "h264_nvenc", .pixels = AV_PIX_FMT_NV12, .fast = true},
#endif
        {.name = "libx264", .pixels = AV_PIX_FMT_YUV420P, .fast = true},
        {.name = "mpeg4", .pixels = AV_PIX_FMT_YUV420P, .fast = false},
    };

    std::string tried;
    for (const auto& c : kCandidates) {
        const AVCodec* codec = avcodec_find_encoder_by_name(c.name);
        if (!codec) continue;
        AVCodecContext* ctx = avcodec_alloc_context3(codec);
        if (!ctx) continue;

        const int rate = c.fast ? 60 : 30;
        ctx->width = static_cast<int>(size.width);
        ctx->height = static_cast<int>(size.height);
        ctx->pix_fmt = c.pixels;
        ctx->time_base = AVRational{1, rate};
        ctx->framerate = AVRational{rate, 1};
        ctx->gop_size = rate * 2;
        // No reordering: every frame is its own time, which keeps a frame
        // dropped under load from confusing anything downstream.
        ctx->max_b_frames = 0;
        // The game is drawn in sRGB and every player assumes BT.709 for video
        // this size; the conversion below uses the same matrix.
        ctx->color_range = AVCOL_RANGE_MPEG;
        ctx->colorspace = AVCOL_SPC_BT709;
        ctx->color_primaries = AVCOL_PRI_BT709;
        ctx->color_trc = AVCOL_TRC_BT709;
        if (format->oformat->flags & AVFMT_GLOBALHEADER) ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

        AVDictionary* options = nullptr;
        const std::string name = c.name;
        if (name == "libx264") {
            av_dict_set(&options, "preset", "veryfast", 0);
            av_dict_set(&options, "crf", "20", 0);
        } else if (name == "mpeg4") {
            ctx->flags |= AV_CODEC_FLAG_QSCALE;
            ctx->global_quality = FF_QP2LAMBDA * 3;
        } else {
            // A tenth of a bit per pixel per frame: 12 Mbit/s at 1080p60,
            // which holds up for a game's hard edges and fast pans.
            const int64_t bits = static_cast<int64_t>(size.width) * size.height * rate / 10;
            ctx->bit_rate = std::clamp<int64_t>(bits, 4'000'000, 40'000'000);
            if (name == "h264_videotoolbox") {
                av_dict_set(&options, "realtime", "1", 0);
                av_dict_set(&options, "allow_sw", "1", 0);
            }
        }

        const int result = avcodec_open2(ctx, codec, &options);
        av_dict_free(&options);
        if (result < 0) {
            tried += std::string(tried.empty() ? "" : ", ") + c.name + " (" + errorText(result) + ")";
            avcodec_free_context(&ctx);
            continue;
        }
        video = ctx;
        fps = rate;
        encoderName = c.name;
        if (!tried.empty()) LOG_INFO("ScreenRecorder: passed over ", tried);
        return true;
    }
    error = tried.empty() ? "FFmpeg has no video encoder this client can use"
                          : "no video encoder would open: " + tried;
    return false;
}

bool ScreenRecorder::Impl::openAudio() {
    auto& engine = audio::AudioEngine::instance();
    if (!engine.beginOutputCapture()) return false;
    audioChannels = engine.getOutputChannels();
    const uint32_t rate = engine.getOutputSampleRate();

    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
    AVCodecContext* ctx = codec ? avcodec_alloc_context3(codec) : nullptr;
    if (!ctx || audioChannels == 0 || rate == 0) {
        if (ctx) avcodec_free_context(&ctx);
        engine.endOutputCapture();
        return false;
    }
    ctx->sample_fmt = AV_SAMPLE_FMT_FLTP;
    ctx->sample_rate = static_cast<int>(rate);
    av_channel_layout_default(&ctx->ch_layout, static_cast<int>(audioChannels));
    ctx->bit_rate = 160'000;
    ctx->time_base = AVRational{1, static_cast<int>(rate)};
    if (format->oformat->flags & AVFMT_GLOBALHEADER) ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    if (avcodec_open2(ctx, codec, nullptr) < 0 || ctx->frame_size <= 0) {
        avcodec_free_context(&ctx);
        engine.endOutputCapture();
        return false;
    }

    audioFrame = av_frame_alloc();
    audioFrame->nb_samples = ctx->frame_size;
    audioFrame->format = ctx->sample_fmt;
    audioFrame->sample_rate = ctx->sample_rate;
    av_channel_layout_copy(&audioFrame->ch_layout, &ctx->ch_layout);
    if (av_frame_get_buffer(audioFrame, 0) < 0) {
        av_frame_free(&audioFrame);
        avcodec_free_context(&ctx);
        engine.endOutputCapture();
        return false;
    }
    audioScratch.resize(static_cast<size_t>(ctx->frame_size) * audioChannels);
    audio = ctx;
    return true;
}

bool ScreenRecorder::Impl::write(AVCodecContext* ctx, AVStream* stream, AVFrame* frame) {
    int result = avcodec_send_frame(ctx, frame);
    if (result < 0 && result != AVERROR_EOF) {
        fail(std::string("the encoder refused a frame: ") + errorText(result));
        return false;
    }
    for (;;) {
        result = avcodec_receive_packet(ctx, packet);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) return true;
        if (result < 0) {
            fail(std::string("the encoder failed: ") + errorText(result));
            return false;
        }
        av_packet_rescale_ts(packet, ctx->time_base, stream->time_base);
        packet->stream_index = stream->index;
        result = av_interleaved_write_frame(format, packet);
        if (result < 0) {
            fail(std::string("could not write to the file: ") + errorText(result));
            return false;
        }
    }
}

bool ScreenRecorder::Impl::encodeVideo(const Frame& frame) {
    if (av_frame_make_writable(videoFrame) < 0) return false;
    const uint8_t* src[1] = {frame.bgra};
    const int srcStride[1] = {static_cast<int>(frame.stride)};
    sws_scale(sws, src, srcStride, 0, static_cast<int>(size.height), videoFrame->data, videoFrame->linesize);
    videoFrame->pts = frame.pts;
    if (!write(video, videoStream, videoFrame)) return false;
    written.fetch_add(1);
    return true;
}

void ScreenRecorder::Impl::drainAudio() {
    if (!audio) return;
    auto& engine = audio::AudioEngine::instance();
    const int n = audio->frame_size;
    const int64_t due = static_cast<int64_t>(
        std::chrono::duration<double>(Clock::now() - startTime).count() * audio->sample_rate);

    // What the device played, a codec frame at a time.
    while (engine.capturedOutputAvailable() >= static_cast<uint32_t>(n)) {
        engine.readCapturedOutput(audioScratch.data(), static_cast<uint32_t>(n));
        if (av_frame_make_writable(audioFrame) < 0) return;
        for (uint32_t ch = 0; ch < audioChannels; ++ch) {
            auto* plane = reinterpret_cast<float*>(audioFrame->data[ch]);
            for (int i = 0; i < n; ++i) plane[i] = audioScratch[static_cast<size_t>(i) * audioChannels + ch];
        }
        audioFrame->pts = audioPts;
        audioPts += n;
        if (!write(audio, audioStream, audioFrame)) return;
    }

    // And silence for any stretch it played nothing - a device that stopped,
    // or one that was never there to begin with. Without it the sound would
    // close up over the gap and run ahead of the picture for the rest of the
    // file. A quarter second of slack, so a callback that is merely late is
    // not taken for one that stopped.
    while (audioPts + n < due - audio->sample_rate / 4) {
        if (av_frame_make_writable(audioFrame) < 0) return;
        for (uint32_t ch = 0; ch < audioChannels; ++ch) {
            std::fill_n(reinterpret_cast<float*>(audioFrame->data[ch]), n, 0.0f);
        }
        audioFrame->pts = audioPts;
        audioPts += n;
        if (!write(audio, audioStream, audioFrame)) return;
    }
}

void ScreenRecorder::Impl::run() {
    for (;;) {
        Frame frame;
        bool have = false;
        {
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait_for(lock, std::chrono::milliseconds(20),
                          [&] { return stopping || !queue.empty(); });
            if (!queue.empty()) {
                frame = std::move(queue.front());
                queue.pop_front();
                have = true;
            } else if (stopping) {
                return;
            }
        }
        if (have) {
            if (!failed.load()) encodeVideo(frame);
            if (frame.release) frame.release();
        }
        if (!failed.load()) drainAudio();
    }
}

void ScreenRecorder::Impl::finish() {
    if (format && format->pb) {
        if (!failed.load()) {
            drainAudio();
            write(video, videoStream, nullptr);
            if (audio) write(audio, audioStream, nullptr);
        }
        av_write_trailer(format);
    }
    release();
}

void ScreenRecorder::Impl::release() {
    if (audio) audio::AudioEngine::instance().endOutputCapture();
    if (sws) { sws_freeContext(sws); sws = nullptr; }
    if (videoFrame) av_frame_free(&videoFrame);
    if (audioFrame) av_frame_free(&audioFrame);
    if (video) avcodec_free_context(&video);
    if (audio) avcodec_free_context(&audio);
    if (packet) av_packet_free(&packet);
    if (format) {
        if (!(format->oformat->flags & AVFMT_NOFILE) && format->pb) avio_closep(&format->pb);
        avformat_free_context(format);
        format = nullptr;
    }
    videoStream = nullptr;
    audioStream = nullptr;
}

#endif  // WOWEE_RECORDER_ENCODES

ScreenRecorder::ScreenRecorder() : impl_(std::make_unique<Impl>()) {}

ScreenRecorder::~ScreenRecorder() {
    if (isRecording()) stop();
}

bool ScreenRecorder::compiledIn() {
#ifdef WOWEE_RECORDER_ENCODES
    return true;
#else
    return false;
#endif
}

bool ScreenRecorder::start(const std::string& path, RecordingSize size, std::string& error) {
#ifdef WOWEE_RECORDER_ENCODES
    Impl& d = *impl_;
    if (d.recording) {
        error = "already recording";
        return false;
    }
    if (size.width < 2 || size.height < 2) {
        error = "the window has no size to record";
        return false;
    }
    av_log_set_level(AV_LOG_ERROR);
    d.size = size;
    d.failed.store(false);
    d.failure.clear();
    d.written.store(0);
    d.dropped.store(0);
    d.lastPts = -1;
    d.audioPts = 0;

    int result = avformat_alloc_output_context2(&d.format, nullptr, "mp4", path.c_str());
    if (result < 0 || !d.format) {
        error = "could not set up an MP4: " + Impl::errorText(result);
        return false;
    }
    if (!d.openVideo(error)) {
        d.release();
        return false;
    }
    d.videoStream = avformat_new_stream(d.format, nullptr);
    d.packet = av_packet_alloc();
    d.videoFrame = av_frame_alloc();
    if (!d.videoStream || !d.packet || !d.videoFrame) {
        error = "out of memory";
        d.release();
        return false;
    }
    avcodec_parameters_from_context(d.videoStream->codecpar, d.video);
    d.videoStream->time_base = d.video->time_base;
    d.videoFrame->format = d.video->pix_fmt;
    d.videoFrame->width = d.video->width;
    d.videoFrame->height = d.video->height;
    if (av_frame_get_buffer(d.videoFrame, 0) < 0) {
        error = "out of memory";
        d.release();
        return false;
    }
    d.sws = sws_getContext(static_cast<int>(size.width), static_cast<int>(size.height), AV_PIX_FMT_BGRA,
                           static_cast<int>(size.width), static_cast<int>(size.height), d.video->pix_fmt,
                           SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!d.sws) {
        error = "could not set up the colour conversion";
        d.release();
        return false;
    }
    // Full-range RGB in, BT.709 limited-range video out, as tagged above.
    sws_setColorspaceDetails(d.sws, sws_getCoefficients(SWS_CS_ITU709), 1,
                             sws_getCoefficients(SWS_CS_ITU709), 0, 0, 1 << 16, 1 << 16);

    d.hasAudio = d.openAudio();
    if (d.hasAudio) {
        d.audioStream = avformat_new_stream(d.format, nullptr);
        if (!d.audioStream) {
            error = "out of memory";
            d.release();
            return false;
        }
        avcodec_parameters_from_context(d.audioStream->codecpar, d.audio);
        d.audioStream->time_base = d.audio->time_base;
    } else {
        LOG_WARNING("ScreenRecorder: no sound to record - the recording will be silent");
    }

    if (!(d.format->oformat->flags & AVFMT_NOFILE)) {
        result = avio_open(&d.format->pb, path.c_str(), AVIO_FLAG_WRITE);
        if (result < 0) {
            error = "could not create " + path + ": " + Impl::errorText(result);
            d.release();
            return false;
        }
    }
    result = avformat_write_header(d.format, nullptr);
    if (result < 0) {
        error = "could not start the file: " + Impl::errorText(result);
        d.release();
        return false;
    }

    d.startTime = Clock::now();
    d.stopping = false;
    d.recording = true;
    d.worker = std::thread([&d] { d.run(); });
    LOG_WARNING("ScreenRecorder: recording ", size.width, "x", size.height, " at ", d.fps,
                " fps with ", d.encoderName, d.hasAudio ? " and AAC sound" : ", no sound",
                " to ", path);
    return true;
#else
    (void)path;
    (void)size;
    error = "this build was made without FFmpeg 5.1 or later, which recording needs";
    return false;
#endif
}

ScreenRecorder::Stats ScreenRecorder::stop() {
    Stats stats;
    Impl& d = *impl_;
    if (!d.recording) return stats;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        d.stopping = true;
    }
    d.wake.notify_all();
    if (d.worker.joinable()) d.worker.join();
    stats.seconds = elapsedSeconds();
#ifdef WOWEE_RECORDER_ENCODES
    d.finish();
#endif
    d.recording = false;
    stats.framesWritten = d.written.load();
    stats.framesDropped = d.dropped.load();
    stats.hasAudio = d.hasAudio;
    stats.encoder = d.encoderName;
    LOG_WARNING("ScreenRecorder: stopped after ", stats.seconds, " s, ", stats.framesWritten,
                " frames written and ", stats.framesDropped, " dropped");
    return stats;
}

bool ScreenRecorder::isRecording() const {
    return impl_->recording;
}

bool ScreenRecorder::failed() const {
    return impl_->failed.load();
}

std::string ScreenRecorder::failure() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->failure;
}

double ScreenRecorder::elapsedSeconds() const {
    if (!impl_->recording) return 0.0;
    return std::chrono::duration<double>(Clock::now() - impl_->startTime).count();
}

bool ScreenRecorder::frameDue(int64_t* pts) const {
    const Impl& d = *impl_;
    if (!d.recording || d.failed.load() || !pts) return false;
    const int64_t now = static_cast<int64_t>(elapsedSeconds() * d.fps);
    if (now <= d.lastPts) return false;
    *pts = now;
    return true;
}

void ScreenRecorder::frameTaken(int64_t pts) {
    impl_->lastPts = pts;
}

void ScreenRecorder::frameDropped() {
    impl_->dropped.fetch_add(1);
}

void ScreenRecorder::submit(Frame frame) {
    Impl& d = *impl_;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        if (d.recording && !d.stopping && !d.failed.load() && d.queue.size() < kMaxQueuedFrames) {
            d.queue.push_back(std::move(frame));
            frame = Frame{};
        }
    }
    if (frame.release) {
        d.dropped.fetch_add(1);
        frame.release();
        return;
    }
    d.wake.notify_one();
}

} // namespace core
} // namespace wowee
