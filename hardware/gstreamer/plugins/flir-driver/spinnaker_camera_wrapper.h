// https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver
// abstract for spinnaker original SDK
#pragma once

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <string>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

class SpinnakerCameraWrapper
{
private:
    CameraPtr pcam;

public:
    SpinnakerCameraWrapper(CameraPtr pcam) : pcam(pcam){};
    ~SpinnakerCameraWrapper() = default;

    // init & deinit
    void init() { pcam->Init(); }
    void deinit() { pcam->DeInit(); }

    // acquisition
    void beginAcquisition() { pcam->BeginAcquisition(); }
    void endAcquisition() { pcam->EndAcquisition(); }

    ImagePtr getNextImage(int grabTimeout=100) { return pcam->GetNextImage(grabTimeout); }

    // status
    bool isStreaming() { return pcam->IsStreaming(); };

    // config
    void setEnumValue(string, string);
    string getEnumValue(string) const;

    void setIntValue(string, int);
    int getIntValue(string) const;

    void setFloatValue(string, float);
    float getFloatValue(string) const;

    void setBoolValue(string, bool);
    bool getBoolValue(string) const;

    void setBufferMode(string mode);                  // special node map
    void setResolutionPixels(int width, int height); // for convinient and sanity check
};
