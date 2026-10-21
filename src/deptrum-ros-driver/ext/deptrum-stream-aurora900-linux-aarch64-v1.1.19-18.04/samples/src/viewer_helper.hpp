/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_FUNCNTIOAL_VIEWER_HELPER_H_
#define DEPTRUM_STREAM_INCLUDE_FUNCNTIOAL_VIEWER_HELPER_H_
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include "deptrum/common_types.h"
#include "deptrum/device.h"
#include "deptrum/stream_types.h"
#include "deptrum/stream_visualizer.h"
#include "functional/base.h"
#include "math.h"
#include "opencv2/opencv.hpp"

namespace deptrum {
namespace stream {

class ViewerHelper {
 public:
  ViewerHelper(const std::string& device_type) : device_type_(device_type) {}
  ~ViewerHelper() { cv::destroyAllWindows(); };

  int DealWithDepthFrame(StreamFrame& frame_ptr, CameraParam& camera_param, uint8_t* color_image) {
    if (device_type_ == "Stellar200" || device_type_ == "Stellar400" ||
        device_type_ == "Stellar420" || device_type_ == "Stellar420s") {
      return AddShaderOnDepthMap(color_image,
                                 frame_ptr.rows,
                                 frame_ptr.cols,
                                 0,
                                 3000,
                                 6000,
                                 camera_param,
                                 static_cast<uint16_t*>(frame_ptr.data.get()));
    }

    else if (device_type_ == "Aurora900" || device_type_ == "Aurora930" ||
             device_type_ == "Aurora931") {
      return ColorizeDepth(color_image,
                           frame_ptr.rows,
                           frame_ptr.cols,
                           0,
                           1500,
                           6000,
                           static_cast<uint16_t*>(frame_ptr.data.get()));
    } else {
      return ColorizeDepthByHist(color_image,
                                 frame_ptr.rows,
                                 frame_ptr.cols,
                                 (uint16_t*) frame_ptr.data.get());
    }
  }

  void ShowFrame(const StreamFrames& frames, CameraParam camera_param = {}) {
    for (int index = 0; index < frames.count; index++) {
      auto frame = frames.frame_ptr[index];
      FrameType frame_type = frame->frame_type;
      std::string title;
      if (frame->frame_attr != FrameAttr::kAttrInvalid) {
        title = frame_type_to_string_map[frame_type] + "_" +
                frame_attr_to_string_map[frame->frame_attr];
      } else {
        title = frame_type_to_string_map[frame_type];
      }

      if (kPointCloudFrame == frame->frame_type || kRgbdPointCloudFrame == frame->frame_type ||
          kRgbdIrPointCloudFrame == frame->frame_type) {
        title = "PointCloud";
        continue;
      }

      // find view
      auto iter = viewer_set_.find(title);
      if (iter == viewer_set_.end()) {
        InitViewer(frame.get(), title);
      }

      switch (frame_type) {
        case kRgbFrame: {
          if (frame->rows * frame->cols * 1.5f == frame->size) {
            int rgb_height = frame->rows;
            int rgb_width = frame->cols;
            cv::Mat rgb_raw_mat1;
            cv::Mat yuv_mat(rgb_height * 1.5f, rgb_width, CV_8UC1, frame->data.get());
            rgb_raw_mat1 = cv::Mat(rgb_height, rgb_width, CV_8UC3);
            cvtColor(yuv_mat, rgb_raw_mat1, cv::COLOR_YUV2BGR_NV12);
            cv::imshow(title, rgb_raw_mat1);
          } else {
            if (frame->data && frame->size > 0) {
              cv::Mat rgb_mat(frame->rows, frame->cols, CV_8UC3, frame->data.get());
              cv::imshow(title, rgb_mat);
            }
          }
          break;
        }
        case kSemanticFrame: {
          if (device_type_ == "Nebula200" || device_type_ == "Nebula220" ||
              device_type_ == "Nebula400") {
            if (frame->data != nullptr && frame->size != 0) {
              cv::Mat flag_mat(frame->rows, frame->cols, CV_8UC3, frame->data.get());
              cv::imshow(title, flag_mat);
            }
          }
          break;
        }
        case kIrFrame: {
          if (frame->data && frame->size != 0) {
            cv::Mat ir_mat(frame->rows, frame->cols, CV_8UC1, frame->data.get());
            cv::imshow(title, ir_mat);
          }
          break;
        }
        case kDepthFrame: {
          int size = frame->rows * frame->cols * 3;

          std::shared_ptr<uint8_t> colored_depth(new uint8_t[size], [](uint8_t* p) { delete[] p; });
          DealWithDepthFrame(*frame, camera_param, colored_depth.get());
          cv::Mat depth_mat(frame->rows, frame->cols, CV_8UC3, colored_depth.get());
          cv::imshow(title, depth_mat);
          break;
        }
      }
      cv::waitKey(1);
    }
  }

  void ShowRgbImage(uint8_t* arr,
                    int row,
                    int col,
                    bool save,
                    const std::string& file_path,
                    int palm_x,
                    int palm_y,
                    int palm_w,
                    int palm_h) {
    cv::Mat m(row, col, CV_8UC3, arr);
    if (save) {
      cv::imwrite(file_path, m);
    }
    cv::rectangle(m, cv::Rect(palm_x, palm_y, palm_w, palm_h), cv::Scalar(255), 3);
    cv::imshow("rgb", m);
    cv::waitKey(10);
  }

  void ShowU8Image(uint8_t* arr,
                   int row,
                   int col,
                   bool save,
                   const std::string& file_path,
                   int palm_x,
                   int palm_y,
                   int palm_w,
                   int palm_h) {
    cv::Mat m(row, col, CV_8UC1, arr);
    if (save) {
      cv::imwrite(file_path, m);
    }
    cv::rectangle(m, cv::Rect(palm_x, palm_y, palm_w, palm_h), cv::Scalar(255), 3);
    cv::imshow("ir", m);
    cv::waitKey(10);
  }

  void ShowDepthImage(uint8_t* arr,
                      int row,
                      int col,
                      bool save,
                      const std::string& file_path,
                      int face_x,
                      int face_y,
                      int face_w,
                      int face_h) {
    cv::Mat depth_image(row, col, CV_8UC1, arr);

    cv::Mat normalized_depth;
    cv::normalize(depth_image, normalized_depth, 0, 255, cv::NORM_MINMAX);

    cv::Mat colored_depth;
    cv::applyColorMap(normalized_depth, colored_depth, cv::COLORMAP_JET);

    cv::rectangle(colored_depth,
                  cv::Rect(face_x, face_y, face_w, face_h),
                  cv::Scalar(255, 255, 255),
                  3);

    if (save) {
      cv::imwrite(file_path, colored_depth);
    }

    cv::imshow("depth", colored_depth);
    cv::waitKey(10);
  }

 private:
  int InitViewer(const StreamFrame* frame, const std::string& title) {
    cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(title, frame->cols, frame->rows);
    viewer_set_.emplace(title);
    return 0;
  }

 public:
  std::string device_type_;

  std::unordered_set<std::string> viewer_set_;
};  // namespace stream

}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_FUNCNTIOAL_VIEWER_MANAGER_H_
