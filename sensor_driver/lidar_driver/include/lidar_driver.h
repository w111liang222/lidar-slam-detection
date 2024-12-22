#ifndef __LIDAR_DRIVER_H
#define __LIDAR_DRIVER_H

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "readerwriterqueue.h"
#include "Types.h"
#include "LidarScan.h"
#include "DecoderBase.h"
#include "rs_decode_difop.h"

namespace LIDAR {

struct OusterCfg {
  int rings;
  std::vector<double> beamAzimuthAngles;
  std::vector<double> beamAltitudeAngles;
  double n;
  double z_offset;
};

typedef std::unordered_map<int, OusterCfg> OusterCfgMap;
typedef std::unordered_map<int, std::function<void(char*, const OusterCfg&)>> OusterParsefunMap;

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
    Ouster_OS1,
    Ouster_OS2,
    RS_LiDAR_16,
    RS_LiDAR_32,
    RS_Ruby_Lite,
    RS_Helios_16P,
    RS_Helios,
    RS_LiDAR_M1,
    Livox_Mid_360,
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
  void setParseFun();
  void resetPoints();
  void resetRuntimeVariables();
  void setOusterCfgMap();

  void packagePrase_VLP_16(char buf[]);
  void packagePrase_LS_C_16(char buf[]);
  void packagePrase_Ouster_OS(char buf[]);
  void packagePrase_Ouster(char buf[], const OusterCfg& cfg);
  void packagePrase_Ouster_V3(char buf[], const OusterCfg& cfg);
  void packagePrase_RS_LiDAR_16(char buf[]);
  void packagePrase_RS_LiDAR_32(char buf[]);
  void packagePrase_RS_Ruby_Lite(char buf[]);
  void packagePrase_RS_Helios_16P(char buf[]);
  void packagePrase_RS_Helios(char buf[]);
  void packagePrase_RS_LiDAR_M1(char buf[]);
  void packagePrase_Livox_Mid_360(char buf[]);
  void packagePrase_Custom(char buf[]);

  void xmlCorrection(void);
  void CopyCartesianHighRawPoint(char buf[], RawPacket& raw_packet,
                                 unsigned int& start_pos, unsigned int& offset);
  void CopyCartesianlowRawPoint(char buf[], RawPacket& raw_packet,
                                unsigned int& start_pos, unsigned int& offset);                                 

 public:
  moodycamel::BlockingReaderWriterQueue<LidarScan*> scanQueue;

 private:
  int packageLenth;
  int receveSize;

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

  // Ouster
  OusterParsefunMap os_parsefun_maps_;
  OusterCfgMap os_cfg_maps_;

  // Ouster-OS1-128
  double beamAltitudeAngles[128];
  double beamAzimuthAngles[128];

  // Ouster-OS1-128(v2.5.2-v3)
  double beamAltitudeAngles_v3[128];
  double beamAzimuthAngles_v3[128];

  // Ouster-OS2-128
  double beamAltitudeAnglesForOS2[128];
  double beamAzimuthAnglesForOS2[128];

  // Ouster-OS2-128(v2.5.2-v3)
  double beamAltitudeAnglesForOS2_v3[128];
  double beamAzimuthAnglesForOS2_v3[128];  

  // Ouster-OS1-64
  double beamAltitudeAngles64[64];
  double beamAzimuthAngles64[64];

  // Ouster-OS1-64(v2.5.2-v3)
  double beamAltitudeAngles64_v3[64];
  double beamAzimuthAngles64_v3[64];

  // Ouster-OS1-32
  double beamAltitudeAngles32[32];
  double beamAzimuthAngles32[32];

  // Ouster-OS1-32(v2.5.2-v3)
  double beamAltitudeAngles32_v3[32];
  double beamAzimuthAngles32_v3[32];

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

  // livox mid360
  bool livox_first = true, livox_frame_start = true;
  uint64_t last_frame_time_;
  std::unique_ptr<LivoxLidarPacketOffsetInfo> livox_offset_;
};

}  // namespace LIDAR

#endif  //__LIDAR_DRIVER_H
