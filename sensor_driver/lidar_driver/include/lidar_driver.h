#ifndef __LIDAR_DRIVER_H
#define __LIDAR_DRIVER_H

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "readerwriterqueue.h"
#include "Types.h"
#include "LidarScan.h"
#include "DecoderBase.h"
#include "rs_decode_difop.h"

namespace LIDAR {

class LidarDriver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum modeType {
    offline,
    online,
  };
  enum lidarType {
    None,
    LS_C_16,
    VLP_16,
    Ouster_OS1_128,
    Ouster_OS2_128,
    Ouster_OS1_32,
    Ouster_OS1_64,
    RS_LiDAR_16,
    RS_LiDAR_32,
    RS_Ruby_Lite,
    RS_Helios_16P,
    RS_Helios,
    Custom,
  };

 public:
  static lidarType getLidarTypeByName(std::string);
  static std::string getLidarNameByType(lidarType);

 public:
  LidarDriver(enum modeType modeIn, enum lidarType lidarIn, int affinityCpu = -1);
  virtual ~LidarDriver();

  void startRun(int portIn);
  void stopRun();
  void startPackageTransfer(std::string dest);
  void stopPackageTransfer();

  bool pointsInROI(float &x, float &y, float &z);
  void setExternalParameter(Transform& externalPara);
  Transform getExternalParameter();
  void setRangeFilter(RangeFilter rangeFilter);
  RangeFilter getRangeFilter();
  void setExcludeFilter(RangeFilter excludeFilter);

 private:
  void veloRun();
  void resetPoints();
  void resetRuntimeVariables();

  void packagePrase_VLP_16(char buf[]);
  void packagePrase_LS_C_16(char buf[]);
  void packagePrase_Ouster_OS1_32(char buf[]);
  void packagePrase_Ouster_OS1_64(char buf[]);
  void packagePrase_Ouster_OS1_128(char buf[]);
  void packagePrase_Ouster_OS2_128(char buf[]);
  void packagePrase_RS_LiDAR_16(char buf[]);
  void packagePrase_RS_LiDAR_32(char buf[]);
  void packagePrase_RS_Ruby_Lite(char buf[]);
  void packagePrase_RS_Helios_16P(char buf[]);
  void packagePrase_RS_Helios(char buf[]);
  void packagePrase_Custom(char buf[]);

  void xmlCorrection(void);

 public:
  moodycamel::BlockingReaderWriterQueue<LidarScan*> scanQueue;

 private:
  int packageLenth;

  enum modeType veloMode;
  enum lidarType LidarType;
  int AffinityCpu;
  int port;

  std::shared_ptr<std::vector<float>> pointCloud;
  std::shared_ptr<std::vector<float>> pointCloudAttr;
  Transform staticTransform;
  RangeFilter filter;
  RangeFilter exclude;
  uint64_t scanStartTime;
  uint64_t scanMonotonicTime;

  std::unique_ptr<std::thread> veloRunThread;
  bool threadStopFlag;

  std::string destinationIp;
  bool startTransfer;

  std::function<void(char*)> parseFun;
  // parsing related data
  float rotAngle, rotAngleOld;

  float xmlData[16];

  // Ouster-OS1-128
  double beamAltitudeAngles[128];
  double beamAzimuthAngles[128];

  // Ouster-OS2-128
  double beamAltitudeAnglesForOS2[128];
  double beamAzimuthAnglesForOS2[128];

  // Ouster-OS1-64
  double beamAltitudeAngles64[64];
  double beamAzimuthAngles64[64];

  // Ouster-OS1-32
  double beamAltitudeAngles32[32];
  double beamAzimuthAngles32[32];

  // Custom
  uint32_t prevFrameId;

  // LS-C-16
  Firing firings[FIRINGS_PER_PACKET];
  float sinTableForLSC16[6300];
  float cosTableForLSC16[6300];
  double lastAzimuth;
  bool isFirstSweep;
  double packetStartTime;

  // RS-16
  int vertAangleList16[16];
  int horiAangleList16[16];
  double cosTableForRS16[72000];
  double sinTableForRS16[72000];
  int azimuthForRS16;

  // RS-32
  int vertAangleList32[32];
  int horiAangleList32[32];
  int azimuthForRS32;

  // RS-Ruby-Lite
  int vertAangleList80[80];
  int horiAangleList80[80];
  int azimuthForRS80;

  // RS-HELIOS
  int azimuthForRSHelios16P;
  int azimuthForRSHelios;
  std::shared_ptr<RsDecodeDifop> rs_decode_difop_;
  bool is_received_difop_ = false;
  Trigon trigon_;
  RSDecoderMechConstParam mech_const_param_;
  ChanAngles chan_angles_;

};

}  // namespace LIDAR

#endif  //__LIDAR_DRIVER_H
