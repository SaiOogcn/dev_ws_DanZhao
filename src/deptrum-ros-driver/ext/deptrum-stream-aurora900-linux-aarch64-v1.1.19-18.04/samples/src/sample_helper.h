#ifndef SAMPLES_SRC_SAMPLE_HELPER_H_
#define SAMPLES_SRC_SAMPLE_HELPER_H_
#include <iostream>
#include <vector>
#include "deptrum/stream_types.h"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// void ChooceStreamType(std::vector<StreamType>& stream_types_vector);
int ChooceFrameMode(std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec);
int SaveOneFrame(const std::string& file_path, const StreamFrames& frames);

void SaveImage(const std::string& file_name, const char* pBuffer, int frame_length);
void SavePointXyzCg(const std::string& file_name, char* pBuffer, int frame_length);
void SavePointXyzAc(const std::string& file_name, char* pBuffer, int frame_length);

#endif  // SAMPLES_SRC_SAMPLE_HELPER_H_
