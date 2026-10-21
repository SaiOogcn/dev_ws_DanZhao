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

const uint8_t default_colorbar360[128 * 3] = {
    0,   0,   143, 0,   0,   151, 0,   0,   159, 0,   0,   167, 0,   0,   175, 0,   0,   183, 0,
    0,   191, 0,   0,   199, 0,   0,   207, 0,   0,   215, 0,   0,   223, 0,   0,   231, 0,   0,
    239, 0,   0,   247, 0,   0,   255, 0,   8,   255, 0,   16,  255, 0,   24,  255, 0,   32,  255,
    0,   40,  255, 0,   48,  255, 0,   56,  255, 0,   64,  255, 0,   72,  255, 0,   80,  255, 0,
    88,  255, 0,   96,  255, 0,   104, 255, 0,   112, 255, 0,   120, 255, 0,   128, 255, 0,   136,
    255, 0,   143, 255, 0,   151, 255, 0,   159, 255, 0,   167, 255, 0,   175, 255, 0,   183, 255,
    0,   191, 255, 0,   199, 255, 0,   207, 255, 0,   215, 255, 0,   223, 255, 0,   231, 255, 0,
    239, 255, 0,   247, 255, 0,   255, 255, 8,   255, 247, 16,  255, 239, 24,  255, 231, 32,  255,
    223, 40,  255, 215, 48,  255, 207, 56,  255, 199, 64,  255, 191, 72,  255, 183, 80,  255, 175,
    88,  255, 167, 96,  255, 159, 104, 255, 151, 112, 255, 143, 120, 255, 136, 128, 255, 128, 136,
    255, 120, 143, 255, 112, 151, 255, 104, 159, 255, 96,  167, 255, 88,  175, 255, 80,  183, 255,
    72,  191, 255, 64,  199, 255, 56,  207, 255, 48,  215, 255, 40,  223, 255, 32,  231, 255, 24,
    239, 255, 16,  247, 255, 8,   255, 255, 0,   255, 247, 0,   255, 239, 0,   255, 231, 0,   255,
    223, 0,   255, 215, 0,   255, 207, 0,   255, 199, 0,   255, 191, 0,   255, 183, 0,   255, 175,
    0,   255, 167, 0,   255, 159, 0,   255, 151, 0,   255, 143, 0,   255, 136, 0,   255, 128, 0,
    255, 120, 0,   255, 112, 0,   255, 104, 0,   255, 96,  0,   255, 88,  0,   255, 80,  0,   255,
    72,  0,   255, 64,  0,   255, 56,  0,   255, 48,  0,   255, 40,  0,   255, 32,  0,   255, 24,
    0,   255, 16,  0,   255, 8,   0,   255, 0,   0,   247, 0,   0,   239, 0,   0,   231, 0,   0,
    223, 0,   0,   215, 0,   0,   207, 0,   0,   199, 0,   0,   191, 0,   0,   183, 0,   0,   175,
    0,   0,   167, 0,   0,   159, 0,   0,   151, 0,   0,   143, 0,   0,   136, 0,   0,   128, 0,
    0,   120, 0,   0,
};

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

  void ShowFrame(const StreamFrames& frames,
                 int face_x = 0,
                 int face_y = 0,
                 int face_w = 0,
                 int face_h = 0,
                 CameraParam camera_param = {}) {
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
          if (device_type_ == "Aurora912" || device_type_ == "Aurora930" ||
              device_type_ == "Aurora932") {
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
              cv::rectangle(rgb_mat, cv::Rect(face_x, face_y, face_w, face_h), cv::Scalar(255), 3);
              cv::imshow(title, rgb_mat);
            }
          }
          break;
        }
        case kSemanticFrame: {
          if (device_type_ == "Nebula200" || device_type_ == "Nebula220") {
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
            cv::rectangle(ir_mat, cv::Rect(face_x, face_y, face_w, face_h), cv::Scalar(255), 3);
            cv::imshow(title, ir_mat);
          }
          break;
        }
        case kDepthFrame: {
          int size = frame->rows * frame->cols * 3;

          std::shared_ptr<uint8_t> colored_depth(new uint8_t[size], [](uint8_t* p) { delete[] p; });
          DealWithDepthFrame(*frame, camera_param, colored_depth.get());
          cv::Mat depth_mat(frame->rows, frame->cols, CV_8UC3, colored_depth.get());
          cv::rectangle(depth_mat, cv::Rect(face_x, face_y, face_w, face_h), cv::Scalar(255), 3);
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
                    int palm_h,
                    float* land_mark_arr) {
    cv::Mat m(row, col, CV_8UC3, arr);
    if (save) {
      cv::imwrite(file_path, m);
    }
    cv::rectangle(m, cv::Rect(palm_x, palm_y, palm_w, palm_h), cv::Scalar(255), 3);
    for (int i = 0; i < 10; i += 2) {
      int x = land_mark_arr[i];
      int y = land_mark_arr[i + 1];
      cv::circle(m, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), 2);
    }
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
                   int palm_h,
                   float* land_mark_arr) {
    cv::Mat m(row, col, CV_8UC1, arr);
    if (save) {
      cv::imwrite(file_path, m);
    }
    cv::rectangle(m, cv::Rect(palm_x, palm_y, palm_w, palm_h), cv::Scalar(255), 3);
    for (int i = 0; i < 10; i += 2) {
      int x = land_mark_arr[i];
      int y = land_mark_arr[i + 1];
      cv::circle(m, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), 2);
    }
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
                      int face_h,
                      float* land_mark_arr) {
    uint16_t* depth = (uint16_t*) arr;
    uint8_t* color_image = new uint8_t[3 * col * row];

    DrawDepthMapInColorByHist(color_image, row, col, 128, default_colorbar360, depth);
    cv::Mat colored_depth_map(row, col, CV_8UC3, color_image);

    cv::rectangle(colored_depth_map, cv::Rect(face_x, face_y, face_w, face_h), cv::Scalar(255), 3);

    for (int i = 0; i < 10; i += 2) {
      int x = land_mark_arr[i];
      int y = land_mark_arr[i + 1];
      cv::circle(colored_depth_map, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), 2);
    }

    if (save) {
      cv::imwrite(file_path, colored_depth_map);
    }

    cv::imshow("depth", colored_depth_map);
    cv::waitKey(10);
    delete[] color_image;
  }

  int DrawDepthMapInColorByHist(uint8_t* color_image,
                                int rows,
                                int cols,
                                int color_num,
                                const uint8_t* colormap_table,
                                const uint16_t* depth_map) {
    uint32_t* depth_histogram = new uint32_t[UINT16_MAX + 1];
    CalcDepthHistInSteps(depth_map, rows, cols, color_num, depth_histogram);

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        int value = depth_map[i * cols + j];
        if (value == 0) {
          color_image[(i * cols + j) * 3] = 0;
          color_image[(i * cols + j) * 3 + 1] = 0;
          color_image[(i * cols + j) * 3 + 2] = 0;
        } else {
          uint32_t val = depth_histogram[value];
          color_image[(i * cols + j) * 3] = colormap_table[val * 3 + 0];
          color_image[(i * cols + j) * 3 + 1] = colormap_table[val * 3 + 1];
          color_image[(i * cols + j) * 3 + 2] = colormap_table[val * 3 + 2];
        }
      }
    }

    if (depth_histogram != nullptr) {
      delete[] depth_histogram;
      depth_histogram = nullptr;
    }

    return 0;
  }

  void CalcDepthHistInSteps(const uint16_t* depth_map,
                            const int rows,
                            const int cols,
                            const int color_num,
                            uint32_t* depth_histogram) {
    unsigned int value = 0;
    unsigned int points_num = 0;
    int depth_size = UINT16_MAX + 1;

    if (depth_histogram == nullptr) {
      depth_histogram = new uint32_t[depth_size];
    }

    // Calculate the accumulative histogram
    memset(depth_histogram, 0, depth_size * sizeof(uint32_t));
    for (int i = 0; i < rows * cols; i++) {
      value = depth_map[i];
      if (value != 0) {
        depth_histogram[value]++;
        points_num++;
      }
    }

    for (int index = 1; index < depth_size; index++) {
      depth_histogram[index] += depth_histogram[index - 1];
    }

    if (points_num != 0) {
      int color_step = points_num / (color_num - 1);
      int color_index = 1;
      for (int index = 1; index < depth_size; index++) {
        if (depth_histogram[index] > color_index * color_step) {
          color_index++;
        }
        depth_histogram[index] = color_index - 1;
      }
    }
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
