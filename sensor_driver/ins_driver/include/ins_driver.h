#ifndef __INS_DRIVER_H
#define __INS_DRIVER_H

#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include "UnixSocket.h"
#include "InterProcess.h"

#include "Transform.h"
#include "Types.h"

class InsDriver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum modeType {
    offline,
    online,
  };

 public:
  InsDriver(std::string ins_type, std::string mode);
  virtual ~InsDriver();

  void startRun(int port, std::string device);
  void stopRun();
  void startPackageTransfer(std::string dest);
  void stopPackageTransfer();
  void setExtrinsicParameter(Transform& extrinsic);
  bool trigger(uint64_t timestamp, bool &motion_valid, std::vector<double> &ins_pose, std::vector<double> &motion_t,
               double &motion_heading, InsDataType &ins, std::vector<InsDataType> &imu);
  bool getMotion(std::vector<double> &ins_pose, std::vector<double> &motion_t, double &motion_heading, uint64_t t0, uint64_t t1);
  void setData(InsDataType &data, uint64_t timestamp, bool is_imu);
 protected:
  void onPoseMessage(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const nav_msgs::Odometry *msg);
  void run_udp();
  void run_com();
  void run_gps();
  void resetRuntimeVariables();
  bool parseGPCHC(std::string &message, InsDataType &ins);
  bool parseBDDB0B(std::string &message, InsDataType &ins);
  std::string formatGPCHC(InsDataType &ins);
  bool parseLivoxImu(char buf[], const int& len, InsDataType &ins);

 private:
  Transform mStaticTransform;
  int mPort;
  std::string mDevice;
  std::string mInsType;

  bool mUseSeperateIMU;
  bool mUseSeperateGPS;
  uint64_t mLastTriggerTime;
  std::vector<nav_msgs::Odometry> mPoseData;
  std::vector<InsDataType> mTimedData;
  std::vector<InsDataType> mQueuedData;
  bool mFirstTrigger;

  InsDataType mGPSData;
  std::mutex mGPSMutex;

  std::shared_ptr<zcm::ZCM> mCore;
  std::unique_ptr<UnixSocketClient> mUnixClient;
  std::unique_ptr<std::thread> mUdpThread;
  std::unique_ptr<std::thread> mComThread;
  std::unique_ptr<std::thread> mGPSThread;
  std::mutex mMutex;
  bool mThreadStopFlag;

  std::string mDestination;
  bool mStartTransfer;

  enum modeType mMode;
};

#endif //__INS_DRIVER_H