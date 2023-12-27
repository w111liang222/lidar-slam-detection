#ifndef __RADAR_FRAME_H
#define __RADAR_FRAME_H

#include <stdint.h>

#include <functional>
#include <memory>
#include <vector>
#include <map>
#include "radar_types.h"

class RadarFrame {
 public:
  RadarFrame(std::string name, uint64_t timeStamp,
            std::map<int32_t, RadarDataType> &radarFrame);
  virtual ~RadarFrame();
  const std::map<int32_t, RadarDataType> &getRadarFrame() const;
  std::string getName();
  uint64_t getTimeStamp();
  void setTimeStamp(uint64_t timestamp);

 protected:
  std::string mName;
  uint64_t mTimeStamp;
  std::map<int32_t, RadarDataType> mRadarFrame;
};

#endif