/*********************************************************************
 *  Copyright (c) 2018-2024 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_DEPTRUM_STREAM_H
#define DEPTRUM_STREAM_INCLUDE_DEPTRUM_STREAM_H

#include <functional>
#include "stream_types.h"
namespace deptrum {
namespace stream {

class STREAM_DLL Stream {
 public:
  virtual ~Stream() = default;
  /**
   * Start the stream.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int Start() = 0;

  /**
   * Stop the stream.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int Stop() = 0;

  /**
   * Get frames data and information.
   *
   * @param[out] frames StreamFrames data and information.
   * @param[in] timeout Timeout in milliseconds.
   *
   * @return void
   */
  virtual int GetFrames(StreamFrames& frames, uint32_t timeout = -1) = 0;

  /**
   * Get frames data, depend user alloc frame.
   *
   * @param[in/out] frames StreamFrames data and information.
   * @param[in] timeout Timeout in milliseconds.
   *
   * @return void
   */
  virtual int GetFramesWithUserAlloc(StreamFrames& frames, uint32_t timeout = -1) = 0;

  /**
   * Register frames callback.
   *
   * @param[in] cb Pointer to the callback function.
   *
   * @return void
   */
  virtual void RegisterFrameCb(std::function<int(StreamFrames& frames)> cb) = 0;
};
}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_DEPTRUM_STREAM_H
