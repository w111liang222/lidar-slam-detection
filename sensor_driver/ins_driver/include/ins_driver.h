#ifndef __INS_DRIVER_H
#define __INS_DRIVER_H

#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include "UnixSocket.h"

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
  InsDriver();
  virtual ~InsDriver();

  void startRun(int portIn, std::string device);
  void stopRun();
  void startPackageTransfer(std::string dest);
  void stopPackageTransfer();
  void setExternalParameter(Transform& trans);
  bool trigger(uint64_t timestamp, std::vector<double> &motionT,
               double &motionR, InsDataType &ins, std::vector<InsDataType> &imu);
  Transform getInterplatedPosition(uint64_t t);
  void getMotion(std::vector<double> &motionT, double &motionR, uint64_t t0, uint64_t t1);
  uint64_t getValidMessageCount();
  uint64_t getReceiveMessageCount();
  void setOfflineMode();
  void setData(InsDataType &data, uint64_t timestamp);
 protected:
  void run_udp();
  void run_com();
  void run_gps();
  void resetRuntimeVariables();
  bool parseGPCHC(std::string &message, InsDataType &ins);
  std::string formatGPCHC(InsDataType &ins);

 private:
  Transform mStaticTransform;
  int mPort;
  std::string mDevice;

  bool mUseSeperateIMU;
  bool mUseSeperateGPS;
  uint64_t mValidMessageCount;
  uint64_t mReceiveMessageCount;
  uint64_t mLastTriggerTime;
  std::vector<InsDataType> mTimedData;
  std::vector<InsDataType> mQueuedData;
  bool mFirstTrigger;

  InsDataType mGPSData;
  std::mutex mGPSMutex;

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