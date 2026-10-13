#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <cstdio>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace rmcs_core::hardware {

// Timestamps are taken before acquiring the mutex. The buffer is written only
// after the motion has stopped, so disk I/O cannot delay a measured callback.
class YawLatencyRecorder {
public:
    using Clock = std::chrono::steady_clock;

    enum class Event : std::uint8_t {
        request,
        can_enqueue,
        motor_feedback,
        gimbal_gyro,
        chassis_gyro,
        test_end,
    };

    struct Sample {
        std::int64_t host_ns;
        Event event;
        std::uint32_t board_tick_quarter_us;
        double value1;
        double value2;
        double value3;
    };

    ~YawLatencyRecorder() {
        if (writer_.joinable())
            writer_.join();
    }

    void start() {
        if (writer_.joinable())
            writer_.join();
        recording_.store(false, std::memory_order_release);
        const auto guard = std::scoped_lock{mutex_};
        samples_.clear();
        samples_.reserve(kMaxSamples);
        dropped_ = 0;
        recording_.store(true, std::memory_order_release);
    }

    [[nodiscard]] bool recording() const noexcept {
        return recording_.load(std::memory_order_acquire);
    }

    void record(
        Event event, double value1 = 0.0, double value2 = 0.0, double value3 = 0.0,
        std::uint32_t board_tick_quarter_us = 0) {
        record_at(event, Clock::now(), value1, value2, value3, board_tick_quarter_us);
    }

    void record_at(
        Event event, Clock::time_point received_at, double value1 = 0.0,
        double value2 = 0.0, double value3 = 0.0,
        std::uint32_t board_tick_quarter_us = 0) {
        if (!recording())
            return;
        const auto host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 received_at.time_since_epoch())
                                 .count();
        const auto guard = std::scoped_lock{mutex_};
        if (!recording_.load(std::memory_order_relaxed))
            return;
        if (samples_.size() == kMaxSamples) {
            ++dropped_;
            return;
        }
        samples_.push_back({host_ns, event, board_tick_quarter_us, value1, value2, value3});
    }

    // File I/O runs on a separate thread so stopping cannot stall the control loop.
    void finish(const std::string& path, std::size_t& recorded, std::size_t& dropped) {
        recording_.store(false, std::memory_order_release);
        std::vector<Sample> completed;
        {
            const auto guard = std::scoped_lock{mutex_};
            completed.swap(samples_);
            dropped = dropped_;
        }
        recorded = completed.size();
        writer_ = std::thread{[path, completed = std::move(completed)] {
            std::ofstream out{path};
            if (out) {
                out << "host_ns,event,board_tick_quarter_us,value1,value2,value3\n";
                out << std::setprecision(17);
                for (const auto& sample : completed) {
                    out << sample.host_ns << ',' << name(sample.event) << ','
                        << sample.board_tick_quarter_us << ',' << sample.value1 << ','
                        << sample.value2 << ',' << sample.value3 << '\n';
                }
            }
            if (!out.good())
                std::fprintf(stderr, "Yaw latency CSV write failed: %s\n", path.c_str());
        }};
    }

private:
    static constexpr std::size_t kMaxSamples = 100'000;

    static constexpr const char* name(Event event) {
        switch (event) {
        case Event::request: return "request";
        case Event::can_enqueue: return "can_enqueue";
        case Event::motor_feedback: return "motor_feedback";
        case Event::gimbal_gyro: return "gimbal_gyro";
        case Event::chassis_gyro: return "chassis_gyro";
        case Event::test_end: return "test_end";
        }
        return "unknown";
    }

    std::atomic<bool> recording_{false};
    std::mutex mutex_;
    std::vector<Sample> samples_;
    std::size_t dropped_ = 0;
    std::thread writer_;
};

} // namespace rmcs_core::hardware
