/*********************************************************************
 *  Copyright (c) 2018-2024 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_RECORD_PLAYBACK_H
#define DEPTRUM_STREAM_INCLUDE_RECORD_PLAYBACK_H

#include <functional>
#include <string>
#include "stream_types.h"

namespace deptrum {
namespace stream {
class Device;
class PlaybackImpl;
class RecordImpl;

class STREAM_DLL Playback {
 public:
  Playback(const std::string& file_path, std::shared_ptr<Device> device);
  virtual ~Playback() = default;

  /**
   * Pauses the playback
   * Calling pause() in "Paused" status does nothing
   * If pause() is called while playback status is "Playing" or "Stopped", the playback will not
   * play until resume() is called
   *
   * @return Zero on success, error code otherwise.
   */
  int32_t Pause();

  /**
   * Un-pauses the playback
   * Calling resume() while playback status is "Playing" or "Stopped" does nothing
   *
   * @return Zero on success, error code otherwise.
   */
  int32_t Resume();

 private:
  std::shared_ptr<PlaybackImpl> impl_;
};

class STREAM_DLL Record {
 public:
  /**
   * Creates a recording device to record the given device and save it to the given file directory.
   * @param[in]  file_path The desired path to which the recorder should save the data
   * @param[in]  device The device to record
   */
  Record(const std::string& file_path, std::shared_ptr<Device> device);
  virtual ~Record() = default;

  /**
   * Pause the recording device without stopping the actual device from streaming.
   *
   * @return Zero on success, error code otherwise.
   */
  int32_t Pause();

  /**
   * Unpauses the recording device, making it resume recording
   *
   * @return Zero on success, error code otherwise.
   */
  int32_t Resume();

 private:
  std::shared_ptr<RecordImpl> impl_;
};
}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_DEPTRUM_STREAM_H
