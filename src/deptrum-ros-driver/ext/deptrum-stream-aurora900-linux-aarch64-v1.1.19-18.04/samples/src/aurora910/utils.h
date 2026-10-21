#ifndef TEST_FACULA_AURORA_H_
#define TEST_FACULA_AURORA_H_

#include <iostream>
#include "face_arithmetic.h"
#include "face_device.h"

namespace deptrum {
namespace stream {

enum class DeviceId {
  kUnspecified = 0,
  kAurora300 = 0x1300,
  kAurora500 = 0x1500,
  kAurora700 = 0x1100,
  kAurora710 = 0x1110,
  kAurora900 = 0x1900,
  kAurora990 = 0x1990,
  kAurora910 = 0x1910,
  kStellar200 = 0x2200,
  kStellar400 = 0x2400,
};

std::ostream& operator<<(std::ostream& out, const Version& v);
std::ostream& operator<<(std::ostream& out, const DeviceVersionInfo& info);
std::ostream& operator<<(std::ostream& out, const StreamDeviceVersionInfo& info);

std::ostream& operator<<(std::ostream& out, const CaptureFaceResult& result);
std::string FeatureCodeToString(int code);
uint64_t GetRandomNum();
void PrintCurrentTime();

}  // namespace stream
}  // namespace deptrum

#endif  // TEST_FACULA_AURORA_H_
