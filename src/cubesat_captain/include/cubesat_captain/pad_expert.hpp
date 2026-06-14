#pragma once
#include "cubesat_captain/expert.hpp"
#include "cubesat_msgs/msg/accel_sample.hpp"

template <typename ValueT, std::size_t Length> class CCircularBuffer {
  public:
    using value_type = ValueT;
    static constexpr std::size_t size_ = Length;
    constexpr CCircularBuffer(const value_type &initial_value) { Fill(initial_value); }

    constexpr void Fill(const value_type &value) { underlying.fill(value); }

    constexpr void AddSample(const value_type &value) {
        underlying[oldest_index] = value;
        oldest_index++;
        oldest_index %= size_;
    }

    constexpr std::size_t Size() const { return size_; }

    constexpr value_type &OldestSample() { return underlying[oldest_index]; }
    constexpr value_type &NewestSample() {
        std::size_t newset_index = (oldest_index + size_ - 1)%size_;
        if (newset_index < 0) {
            newset_index += size_;
        }
        return underlying[newset_index];
    }

    // index of 0 is the oldest sample.
    // index of size()-1 is the newest sample
    // values > size() will wrap around
    constexpr value_type &operator[](std::size_t index) {
        std::size_t real_index = oldest_index + index;
        real_index %= size_;
        return underlying[real_index];
    }

  private:
    std::size_t oldest_index = 0;
    std::array<value_type, size_> underlying;
};
template <typename T, std::size_t len> class CRollingSum {
  public:
    static_assert(len > 0, "What is the sum of 0 elements? You probably don't want this (also it will break)");

    using value_type = T;

    constexpr CRollingSum(T start) : buf(start) { Fill(start); }

    constexpr std::size_t Size() const { return size; }

    constexpr void Fill(const value_type &start) {
        buf.Fill(start);
        total = buf.OldestSample();
        for (std::size_t i = 1; i < buf.Size(); i++) {
            total = total + buf[i];
        }
    }

    constexpr void Feed(const value_type &new_value) {
        value_type oldest = buf.OldestSample();
        total = total - oldest;
        total = total + new_value;
        buf.AddSample(new_value);
    }
    constexpr value_type Sum() const { return total; }

  private:
    static constexpr std::size_t size = len;

    value_type total;
    CCircularBuffer<value_type, size> buf;
};

template <typename T, std::size_t len> class CMovingAverage {
  public:
    using value_type = T;
    static constexpr std::size_t length = len;

    constexpr CMovingAverage(value_type start) : rolling_sum(start) {}

    constexpr void Feed(value_type value) { rolling_sum.Feed(value); }
    constexpr value_type Avg() { return rolling_sum.Sum() / len; }
    constexpr void Fill(const T &start) { rolling_sum.Fill(start); }

  private:
    CRollingSum<value_type, len> rolling_sum;
};

double accel_magnitude(const cubesat_msgs::msg::AccelSample &sample);
cubesat_msgs::msg::AccelSample normalize_accel(const cubesat_msgs::msg::AccelSample &sample);

namespace cubesat_captain {
class PadExpert : public Expert {
  public:
    PadExpert(rclcpp::Logger logger, Levers &levers) : Expert(logger, levers) {}
    ~PadExpert() {}
    void handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) override;
    void enter_state() override;

  private:
    bool has_boosted_ = false;
    // at 50 hz, 12 samples = 0.24 seconds
    CMovingAverage<double, 12> avger{0.0};
};

class PreboostExpert : public Expert {
  public:
    /**
     * Feeds data to the pad expert but handles turning on cameras and leaving expect mode
     */
    PreboostExpert(rclcpp::Logger logger, Levers &levers, PadExpert *pad_expert)
        : Expert(logger, levers), pad_expert(pad_expert) {}
    ~PreboostExpert() {}
    void handle_base_accel(const cubesat_msgs::msg::AccelSample &sample) override;

    void enter_state() override;

  private:
    PadExpert *pad_expert;
};

} // namespace cubesat_captain
