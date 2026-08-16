#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <vector>

namespace wowee {
namespace core {

// Persistent worker pool for short-lived parallel jobs that run every frame
// (bone matrix computation, secondary command buffer recording, collision
// floor queries). Replaces per-frame std::async calls, which create and
// destroy an OS thread per invocation on libstdc++/libc++ - hundreds of
// thread spawns per second at typical frame rates.
//
// Tasks submitted to the pool must not block waiting on other pool tasks,
// with one sanctioned exception: a task may submit chunked sub-work and wait
// for it, provided it also processes a share of that work on its own thread
// (caller-runs-a-chunk pattern). All other tasks are non-blocking compute,
// so the queue always drains and such waits always complete.
class ThreadPool {
public:
    explicit ThreadPool(size_t threadCount) {
        workers_.reserve(threadCount);
        for (size_t i = 0; i < threadCount; ++i) {
            workers_.emplace_back([this]() { workerLoop(); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& w : workers_) {
            if (w.joinable()) w.join();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Shared pool for per-frame render/update work, sized for the machine.
    static ThreadPool& frameWorkers() {
        static ThreadPool pool(defaultThreadCount());
        return pool;
    }

    // Small, separate pool for blocking filesystem work. Keeping it distinct from
    // frameWorkers prevents a slow disk read from occupying render/update workers.
    static ThreadPool& ioWorkers() {
        static ThreadPool pool(2);
        return pool;
    }

    // Schedule fn on a worker thread. The returned future carries fn's result
    // or any exception it threw (same semantics as std::async).
    template <typename F>
    auto submit(F&& fn) -> std::future<std::invoke_result_t<std::decay_t<F>>> {
        using R = std::invoke_result_t<std::decay_t<F>>;
        auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(fn));
        std::future<R> fut = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace([task]() { (*task)(); });
        }
        cv_.notify_one();
        return fut;
    }

    [[nodiscard]] size_t threadCount() const { return workers_.size(); }

private:
    static size_t defaultThreadCount() {
        size_t hw = std::thread::hardware_concurrency();
        if (hw == 0) hw = 4;
        // Leave a core for the main thread; cap for very wide machines.
        size_t count = (hw > 1) ? hw - 1 : 1;
        if (count < 2) count = 2;
        if (count > 16) count = 16;
        return count;
    }

    void workerLoop() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::function<void()>> tasks_;
    std::vector<std::thread> workers_;
    bool stop_ = false;
};

} // namespace core
} // namespace wowee
