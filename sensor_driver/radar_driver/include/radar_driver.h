#ifndef __RADAR_DRIVER_H
#define __RADAR_DRIVER_H

#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <linux/can.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <time.h>
#include <arpa/inet.h>

#include "readerwriterqueue.h"
#include "radar_types.h"
#include "radar_frame.h"
#include "Transform.h"

class RadarDriver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum modeType {
    offline,
    online,
  };
  enum radarType {
    None,
    ARS408,
  };

 public:
  static radarType getRadarTypeByName(std::string);
  static std::string getRadarNameByType(radarType);

 public:
  RadarDriver(enum modeType modeIn, enum radarType radarIn);
  virtual ~RadarDriver();

  void startRun(std::string portIn);
  void stopRun();
  void setExternalParameter(Transform& externalPara);

 private:
  void run();
  void resetRuntimeVariables();
  void canParse_ARS408(can_frame can_read_data);

 public:
  moodycamel::BlockingReaderWriterQueue<RadarFrame*> scanQueue;

 private:
  std::string mCanPort;
  Transform mStaticTransform;
  enum modeType mMode;
  enum radarType RadarType;
  std::unique_ptr<std::thread> mThread;
  std::function<void(can_frame)> mParseFun;

  std::mutex mMutex;
  bool mThreadStopFlag;
  uint64_t scanStartTime;
  std::map<int32_t, RadarDataType> mRadarData;
};

#endif //__RADAR_DRIVER_H