/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_FUNCNTIOAL_FRAME_RATE_HELPER_H_
#define DEPTRUM_STREAM_FUNCNTIOAL_FRAME_RATE_HELPER_H_

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>

using namespace std::chrono;

namespace deptrum {

#define INTERVAL_BUFFER_SIZE 32

class FrameRateHelper {
 public:
  FrameRateHelper() { Reset(); }
  ~FrameRateHelper() {}

 public:
  float GetFrameRate() {
    uint64_t interval_avg = 0;
    float frame_rate = 0.0f;

    if (interval_count_ >= INTERVAL_BUFFER_SIZE) {
      interval_avg = interval_sum_ / INTERVAL_BUFFER_SIZE;
    } else if (interval_count_ > 0) {
      interval_avg = interval_sum_ / interval_count_;
    }

    if (0 != interval_avg) {
      frame_rate = 1000000.0f / interval_avg;
    }

    return frame_rate;
  }

  void Reset() {
    last_timestamp_ = 0;
    frame_count_ = 0;
    interval_count_ = 0;
    interval_sum_ = 0;
    memset(interval_buff, 0, sizeof(interval_buff));
  }

  void RecordTimestamp() {
    uint64_t now = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    frame_count_++;

    if (frame_count_ >= 2) {
      interval_sum_ -= interval_buff[interval_count_ % INTERVAL_BUFFER_SIZE];
      interval_buff[interval_count_ % INTERVAL_BUFFER_SIZE] = now - last_timestamp_;
      interval_sum_ += interval_buff[interval_count_ % INTERVAL_BUFFER_SIZE];
      interval_count_++;
    }

    last_timestamp_ = now;
  }

 private:
  uint64_t last_timestamp_;
  uint64_t frame_count_;
  uint64_t interval_count_;
  uint64_t interval_sum_;
  uint64_t interval_buff[INTERVAL_BUFFER_SIZE];
};
};  // namespace deptrum

#endif  // DEPTRUM_STREAM_FUNCNTIOAL_FRAME_RATE_HELPER_H_
