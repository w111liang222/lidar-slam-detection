#include "radar_driver.h"
#include "SystemUtils.h"

#include <iostream>
#include <sys/prctl.h>
#include <stdlib.h>
#include <unistd.h>

const double PI = 3.1415926535897932384626433832795;

RadarDriver::RadarDriver(modeType modeIn, enum radarType radarIn) {
    mMode = modeIn;
    RadarType = radarIn;
    switch (radarIn) {
        case radarType::ARS408: {
        mParseFun = std::bind(&RadarDriver::canParse_ARS408, this,
                            std::placeholders::_1);
        break;
        }
        default: {
        LOG_ERROR("No support radar type {}", radarIn);
        abort();
        break;
        }
    }
}

RadarDriver::~RadarDriver() {
    stopRun();
}

RadarDriver::radarType RadarDriver::getRadarTypeByName(std::string name) {
  if (name.compare("ARS408") == 0) {
    return radarType::ARS408;
  } else {
    return radarType::None;
  }
}

std::string RadarDriver::getRadarNameByType(radarType type) {
  switch (type) {
    case radarType::ARS408: {
      return "ARS408";
    }
    default: {
      return "";
    }
  }
}

void RadarDriver::startRun(std::string portIn) {
    if (mThread != nullptr) {
        return;
    }
    mCanPort = portIn;
    mThreadStopFlag = false;
    resetRuntimeVariables();
    mRadarData.clear();
    if (online == mMode) {
        mThread.reset(new std::thread(&RadarDriver::run, this));
    }
}

void RadarDriver::stopRun() {
    if (mThread == nullptr) {
        return;
    }
    mThreadStopFlag = true;
    mThread->join();
    mThread.reset(nullptr);
}

void RadarDriver::resetRuntimeVariables() {
    while (scanQueue.pop()) {
        ;
    }
    scanStartTime = 0;
}

void RadarDriver::setExternalParameter(Transform &externalPara) {
  mStaticTransform = externalPara;
}

void RadarDriver::run() {
    prctl(PR_SET_NAME, "Radar Recev", 0, 0, 0);
    LOG_INFO("start radar {} receving, can port {}", getRadarNameByType(RadarType), mCanPort);
    int socket_fd;
	unsigned long nbytes;
	struct sockaddr_can addr;
	struct ifreq ifrr;

	socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	strcpy(ifrr.ifr_name, mCanPort.c_str());
	if (ioctl(socket_fd, SIOCGIFINDEX, &ifrr) < 0) {
        LOG_ERROR("radar {}, can port {} can't fit into the device", getRadarNameByType(RadarType), mCanPort);
        return;
    }
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifrr.ifr_ifindex;
	bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr));
    while (!mThreadStopFlag) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(socket_fd, &fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        int ret = select(socket_fd + 1, &fds, NULL, NULL, &tv);

        if(!FD_ISSET(socket_fd, &fds))
            continue;

        struct can_frame can_read_data;
		nbytes = read(socket_fd, &can_read_data, sizeof(can_read_data));
		if (nbytes <= 0) {
            LOG_ERROR("{} can't read can data!!", mCanPort);
            continue;
        }
        mParseFun(can_read_data);
    }
}

void RadarDriver::canParse_ARS408(can_frame can_read_data) {
    if (can_read_data.can_id == 0x60A) {
        if (!mRadarData.empty()) {
            // new radar frame
            RadarFrame *frame =
                new RadarFrame("ARS408", scanStartTime, mRadarData);
            scanQueue.enqueue(frame);
            scanStartTime = getCurrentTime();
            mRadarData.clear();
        }
        int num_obstacle = can_read_data.data[0];
    } else if (can_read_data.can_id == 0x60B) { //tracked objects
        int32_t id = can_read_data.data[0];
        mRadarData[id].id = id;

        double x = (can_read_data.data[1] * 32 + ((can_read_data.data[2] & 0xF8) >> 3)) * 0.2 - 500;
        double y = ((can_read_data.data[2] & 0x07) * 256 + can_read_data.data[3]) * 0.2 - 204.6;
        double z = 0.0;

        // tranform radar object to vehicle coordinate system
        Eigen::Vector3d t(x, y, z);
        t = mStaticTransform.translation() + mStaticTransform.rotation() * t;
        mRadarData[id].x = t(0);
        mRadarData[id].y = t(1);
        mRadarData[id].z = t(2);

        double vx = (can_read_data.data[4] * 4 + ((can_read_data.data[5] & 0xC0) >> 6)) * 0.25 - 128;
        double vy = ((can_read_data.data[5] & 0x3F) * 8 + ((can_read_data.data[6] & 0xE0) >> 5)) * 0.25 - 64;
        double vz = 0.0;

        Eigen::Vector3d v(vx, vy, vz);
        v = mStaticTransform.rotation() * v;
        mRadarData[id].vx = v(0);
        mRadarData[id].vy = v(1);
    } else if (can_read_data.can_id == 0x60D) {
        int32_t id = can_read_data.data[0];
        mRadarData[id].ax = (can_read_data.data[1] * 8 +
                        ((can_read_data.data[2] & 0xE0) >> 5))
                        * 0.01 - 10;
        mRadarData[id].ay = ((can_read_data.data[2] & 0x1F) * 16 +
                        ((can_read_data.data[3] & 0xF0) >> 4))
                        * 0.01 - 2.50;
        int8_t type = can_read_data.data[3] & 0x07;
        if (type == 1 || type == 2) {
            type = 1; // vehicle
        } else if (type == 4 || type == 5) {
            type = 3; // cyclist
        } else if (type == 3) {
            type = 2; // pedestrain    
        } 
        else {
            type = 0; // unknown
        }
        mRadarData[id].type = type;
        mRadarData[id].ang  = (can_read_data.data[4] * 4 +
                        ((can_read_data.data[5] & 0xC0) >> 6))
                        * 0.4 - 180;
        mRadarData[id].length = can_read_data.data[6] * 0.2;
        mRadarData[id].width  = can_read_data.data[7] * 0.2;
    }
}