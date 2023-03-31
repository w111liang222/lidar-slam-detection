#include "radar_frame.h"

RadarFrame::RadarFrame(std::string name, uint64_t timeStamp,
                     std::map<int32_t, RadarDataType> &radarFrame) {
  mName = name;
  mTimeStamp = timeStamp;
  mRadarFrame = radarFrame;
}

RadarFrame::~RadarFrame() {}

const std::map<int32_t, RadarDataType> &RadarFrame::getRadarFrame() const {
  return mRadarFrame;
}

std::string RadarFrame::getName() { return mName; }

uint64_t RadarFrame::getTimeStamp() { return mTimeStamp; }

void RadarFrame::setTimeStamp(uint64_t timestamp) { mTimeStamp = timestamp; }