#pragma once
#include "cubesat_msgs/msg/accel_sample.hpp"
namespace pad {
void feed_boost_detect(const cubesat_msgs::msg::AccelSample &sample, double threshold);
bool has_boosted();
void fake_boost_detect();
} // namespace pad
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
        std::size_t newset_index = oldest_index - 1;
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
