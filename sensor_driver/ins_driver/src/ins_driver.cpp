#include "ins_driver.h"

#include <sys/prctl.h>
#include <stdlib.h>
#include <libgpsmm.h>

#include "SystemUtils.h"
#include "Transform.h"
#include "UDPServer.h"
#include "SerialDriver.h"
#include "UTMProjector.h"
#include "InterProcess.h"

const double PI = 3.1415926535897932384626433832795;

double getYawAngle(Transform &trans) {
    Translation unit_vector(0, 1.0, 0);
    unit_vector = trans.rotation_ * unit_vector;
    double yaw = atan2(unit_vector(1), unit_vector(0));
    return (yaw - PI / 2.0);
}

int hex2dec(char ch) {
  if ('0' <= ch && ch <= '9') return ch - '0';
  if ('A' <= ch && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

char dec2hex(int d) {
  if (0 <= d && d <=9) return d + '0';
  if (d >= 10) return d - 10 + 'A';
  return '\0';
}

Transform computeRTKTransform(InsDataType &data) {
    static UTMProjector projector;
    double dataX, dataY;
    projector.FromGlobalToLocal(data.latitude, data.longitude, dataX, dataY);
    return getTransformFromRPYT(dataX, dataY, data.altitude, -data.heading, data.pitch, data.roll);
}

InsDriver::InsDriver() {
    mMode = modeType::online;
    mUseSeperateIMU = false;
    mUseSeperateGPS = false;
    mUnixClient.reset(new UnixSocketClient("/tmp/imu_data.sock"));
    mStaticTransform = Transform();
    mValidMessageCount = 0;
    mReceiveMessageCount = 0;
    mStartTransfer = false;
    resetRuntimeVariables();
}

InsDriver::~InsDriver() {
    stopRun();
}

void InsDriver::startRun(int portIn, std::string device) {
    mPort = portIn;
    mDevice = device;
    mThreadStopFlag = false;

    // UDP
    if (mStartTransfer && mDestination.compare("127.0.0.1") == 0) {
        LOG_WARN("INS relay destination is {}, UDP receiver won't be started", mDestination);
    } else {
        mUdpThread.reset(new std::thread(&InsDriver::run_udp, this));
        setThreadPriority(mUdpThread.get(), 99);
    }

    // COM
    if (mDevice.find("IMU") != std::string::npos) {
        mUseSeperateIMU = true;
    }
    if (mDevice.find("GPS") != std::string::npos) {
        mUseSeperateGPS = true;
    }
    mComThread.reset(new std::thread(&InsDriver::run_com, this));
    setThreadPriority(mComThread.get(), 99);

    // GPSD
    mGPSThread.reset(new std::thread(&InsDriver::run_gps, this));
}

void InsDriver::stopRun() {
    mThreadStopFlag = true;
    if (mUdpThread != nullptr) {
        // for notify the udp receiving thread
        std::unique_ptr<UDPServer> tmpServer(new UDPServer(0));
        tmpServer->UDPSendto("127.0.0.1", mPort, "stop", 4);
        mUdpThread->join();
        mUdpThread.reset(nullptr);
    }

    if (mComThread != nullptr) {
        mComThread->join();
        mComThread.reset(nullptr);
    }

    if (mGPSThread != nullptr) {
        mGPSThread->join();
        mGPSThread.reset(nullptr);
    }
}

void InsDriver::startPackageTransfer(std::string dest) {
  mDestination = dest;
  mStartTransfer = true;
}

void InsDriver::stopPackageTransfer() {
  mStartTransfer = false;
}

void InsDriver::resetRuntimeVariables() {
    mFirstTrigger = true;
    mLastTriggerTime = 0;
    mTimedData.clear();
    mQueuedData.clear();
}

void InsDriver::setExternalParameter(Transform& trans) {
    mStaticTransform = trans;
}

uint64_t InsDriver::getValidMessageCount() {
    return mValidMessageCount;
}

uint64_t InsDriver::getReceiveMessageCount() {
    return mReceiveMessageCount;
}

void InsDriver::setOfflineMode() {
    mMode = modeType::offline;
}

void InsDriver::setData(InsDataType &data, uint64_t timestamp) {
    // send to unix socket
    std::string msg = formatGPCHC(data);
    mUnixClient->Sendto(msg, msg.length());

    if (get_core_enable()) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = data.gps_timestamp;
        imu_msg.angular_velocity.x = data.gyro_x;
        imu_msg.angular_velocity.y = data.gyro_y;
        imu_msg.angular_velocity.z = data.gyro_z;
        imu_msg.linear_acceleration.x = data.acc_x;
        imu_msg.linear_acceleration.y = data.acc_y;
        imu_msg.linear_acceleration.z = data.acc_z;
        PUBLISH_MSG("imu_raw", imu_msg);
    }

    // udp relay
    if (mStartTransfer) {
        static std::unique_ptr<UDPServer> UDPSender(new UDPServer(0));
        UDPSender->UDPSendto(mDestination, mPort, msg, msg.length());
    }

    std::lock_guard<std::mutex> lck(mMutex);
    // check the timestamp order
    if (mTimedData.size() > 0 && mTimedData.back().gps_timestamp > timestamp) {
        LOG_WARN("Ins data is unordered, {}, {}", mTimedData.back().gps_timestamp, timestamp);
        resetRuntimeVariables();
    }

    // limit the size within 100
    if (mTimedData.size() > 100) {
        mTimedData.erase(mTimedData.begin());
    }
    if (mQueuedData.size() > 100) {
        mQueuedData.erase(mQueuedData.begin());
    }
    mTimedData.emplace_back(data);
    mQueuedData.emplace_back(data);
}

Transform InsDriver::getInterplatedPosition(uint64_t t) {
    int idx0, idx1;
    if (t <= mTimedData.front().gps_timestamp) {
        idx0 = 0;
        idx1 = 1;
    } else if (t >= mTimedData.back().gps_timestamp) {
        idx0 = mTimedData.size() - 2;
        idx1 = mTimedData.size() - 1;
    } else {
        int begin = 0, end = mTimedData.size() - 1, mid = 0;
        while(begin < end) {
            mid = (begin + end) / 2;
            if (mTimedData[mid].gps_timestamp > t) {
                end = mid;
            } else if (mTimedData[mid + 1].gps_timestamp < t) {
                begin = mid + 1;
            } else {
                break;
            }
        }
        idx0 = mid;
        idx1 = mid + 1;
    }

    double t_diff_ratio =
      static_cast<double>(t - mTimedData[idx0].gps_timestamp) /
      static_cast<double>(mTimedData[idx1].gps_timestamp - mTimedData[idx0].gps_timestamp);
    Transform T0 = computeRTKTransform(mTimedData[idx0]);
    Transform T1 = computeRTKTransform(mTimedData[idx1]);
    Vector6 diff_vector = (T0.inverse() * T1).log();
    Transform out = T0 * Transform::exp(t_diff_ratio * diff_vector);
    return out;
}

void InsDriver::getMotion(std::vector<double> &motionT, double &motionR, uint64_t t0, uint64_t t1) {
    Transform lastTransform = getInterplatedPosition(t0);
    Transform currentTransform = getInterplatedPosition(t1);

    Transform motion_TR = mStaticTransform.inverse() * (currentTransform.inverse() * lastTransform) * mStaticTransform;
    Matrix m = motion_TR.matrix();
    motionT[0]  = m(0, 0); motionT[1]  = m(0, 1); motionT[2]  = m(0, 2);  motionT[3] = m(0, 3);
    motionT[4]  = m(1, 0); motionT[5]  = m(1, 1); motionT[6]  = m(1, 2);  motionT[7] = m(1, 3);
    motionT[8]  = m(2, 0); motionT[9]  = m(2, 1); motionT[10] = m(2, 2); motionT[11] = m(2, 3);
    motionT[12] = m(3, 0); motionT[13] = m(3, 1); motionT[14] = m(3, 2); motionT[15] = m(3, 3);
    motionR = getYawAngle(motion_TR);

    double dt = (t1 - t0) / 1000000.0;
    double velocity = sqrt(motionT[3] * motionT[3] + motionT[7] * motionT[7] + motionT[11] * motionT[11]) / dt;
    if (velocity > 100.0) {
        spdlog::warn("Large velocity of INS, {}, {}", velocity, motionR / PI * 180.0);
        motionT[0]  = 1; motionT[1]  = 0; motionT[2]  = 0;  motionT[3] = 0;
        motionT[4]  = 0; motionT[5]  = 1; motionT[6]  = 0;  motionT[7] = 0;
        motionT[8]  = 0; motionT[9]  = 0; motionT[10] = 1; motionT[11] = 0;
        motionR = 0;
    }
}

bool InsDriver::trigger(uint64_t timestamp, std::vector<double> &motionT,
                        double &motionR, InsDataType &ins, std::vector<InsDataType> &imu) {
    std::lock_guard<std::mutex> lck(mMutex);
    if (mTimedData.size() < 2) {
        return false;
    }

    double age1 = (double(timestamp) - double(mTimedData.back().gps_timestamp)) / 1000.0;
    double age2 = (double(mTimedData.front().gps_timestamp) - double(timestamp)) / 1000.0;
    // No valid data received
    if (age1 > 1000) {
        LOG_WARN("No new ins data was receved, {}, {}", mTimedData.back().gps_timestamp, timestamp);
        resetRuntimeVariables();
        return false;
    }
    if (age2 > 1000) {
        LOG_WARN("Trigger timestamp is out of date, {}, {}", mTimedData.front().gps_timestamp, timestamp);
        resetRuntimeVariables();
        return false;
    }

    // No new data received within 1 period
    if (mQueuedData.size() <= 0) {
        LOG_WARN("No new ins data within 1 period");
        return false;
    }

    if (timestamp < mLastTriggerTime) {
        LOG_WARN("Trigger timestamp is unordered, {}, {}", mLastTriggerTime, timestamp);
        resetRuntimeVariables();
        return false;
    }

    // Get valid data
    if (!mFirstTrigger) {
        getMotion(motionT, motionR, mLastTriggerTime, timestamp);
    }

    mFirstTrigger = false;
    mLastTriggerTime = timestamp;
    ins = mQueuedData.back();
    std::swap(imu, mQueuedData);
    return true;
}

void InsDriver::run_udp() {
    prctl(PR_SET_NAME, "Ins Udp Recev", 0, 0, 0);
    LOG_INFO("start ins udp receving, port {}", mPort);
    std::unique_ptr<UDPServer> InsUDPServer(new UDPServer(mPort));

    char buf[65536] = "";
    while (!mThreadStopFlag) {
        auto clock = std::chrono::steady_clock::now();
        int receveSize = InsUDPServer->UDPServerReceive(buf, 65536);
        std::string message(buf, receveSize);
        if (0 == message.compare("stop")) {
            continue;
        }
        auto elapseMs = since(clock).count();
        if (elapseMs >= 50) {
            LOG_WARN("Ins udp receive buffer wait for {} ms", elapseMs);
        }

        InsDataType ins;
        mReceiveMessageCount++;
        bool found = parseGPCHC(message, ins);
        if (found) {
            mValidMessageCount++;
            uint64_t hostTime = getCurrentTime();
            uint64_t gpsTime = gps2Utc(ins.gps_week, ins.gps_time);
            double diffMSec = fabs(gpsTime / 1000.0 - hostTime / 1000.0);
            if (diffMSec > 1000) {
                ins.gps_timestamp = hostTime;
                LOG_WARN("UDP: gps/host time diff {} ms, check the time sync!", diffMSec);
            } else {
                ins.gps_timestamp = gpsTime;
            }
            setData(ins, ins.gps_timestamp);
        } else {
            LOG_WARN("Ins parse udp source error, {}", message);
        }
    }
}

void InsDriver::run_com() {
    prctl(PR_SET_NAME, "Ins Com Recev", 0, 0, 0);
    LOG_INFO("start ins com receving, device {}", mDevice);
    std::unique_ptr<SerialDriver> InsSerial(new SerialDriver(mDevice, 230400));
    char buf[64] = "";
    int buf_len = 0;
    std::string message = "";
    while (!mThreadStopFlag) {
        auto clock = std::chrono::steady_clock::now();
        int receveSize = InsSerial->readBuf(buf + buf_len, 64 - buf_len, 100000);
        if (0 == receveSize) {
            continue;
        }
        buf_len += receveSize;
        if (buf_len < 64) {
            continue;
        }
        auto elapseMs = since(clock).count();
        if (elapseMs >= 50) {
            LOG_WARN("Ins com receive buffer wait for {} ms", elapseMs);
        }
        message = message + std::string(buf, buf_len);
        buf_len = 0;

        InsDataType ins;
        mReceiveMessageCount++;
        bool found = parseGPCHC(message, ins);
        if (found) {
            mValidMessageCount++;
            if (mUseSeperateIMU) {
                ins.gps_week = getGPSweek(getCurrentTime());
                ins.gps_time = getGPSsecond(getCurrentTime());
            }
            if (mUseSeperateGPS) {
                std::lock_guard<std::mutex> lck(mGPSMutex);
                ins.latitude  = mGPSData.latitude;
                ins.longitude = mGPSData.longitude;
                ins.altitude  = mGPSData.altitude;
            }

            uint64_t hostTime = getCurrentTime();
            uint64_t gpsTime = gps2Utc(ins.gps_week, ins.gps_time);
            double diffMSec = fabs(gpsTime / 1000.0 - hostTime / 1000.0);
            if (diffMSec > 60000) {
                ins.gps_timestamp = gpsTime;
                setSystemTime(gpsTime);
                hostTime = gpsTime;
                LOG_WARN("COM: gps/host time diff {} ms, adjust system clock", diffMSec);
            } else {
                ins.gps_timestamp = hostTime;
            }
            setData(ins, ins.gps_timestamp);
        }

        if (message.size() > 512) {
            LOG_WARN("Ins unparsed data is too much, {}", message);
            message = message.substr(256);
        }
    }
}

void InsDriver::run_gps() {
    prctl(PR_SET_NAME, "Ins GPS Recev", 0, 0, 0);
    LOG_INFO("start ins gps receving");
    gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);
    if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
        LOG_ERROR("No GPSD running");
        return;
    }

    InsDataType ins;
    while (!mThreadStopFlag) {
        if (!gps_rec.waiting(20000)) {
            ins.gps_timestamp = getCurrentTime();
            // setData(ins, ins.gps_timestamp);
            continue;
        }

        struct gps_data_t* gpsd_data;
        if ((gps_rec.read()) == nullptr) {
            LOG_ERROR("GPSD read error");
            return;
        }

        while (((gpsd_data = gps_rec.read()) == nullptr) || (gpsd_data->fix.mode < MODE_2D)) {
            usleep(20000);
            ins.gps_timestamp = getCurrentTime();
            // setData(ins, ins.gps_timestamp);
            if (mThreadStopFlag) {
                return;
            }
        }

        ins.latitude = gpsd_data->fix.latitude;
        ins.longitude = gpsd_data->fix.longitude;
        ins.altitude = gpsd_data->fix.altitude;
        ins.Ve = gpsd_data->fix.speed;
        ins.gps_timestamp = getCurrentTime();
        // setData(ins, ins.gps_timestamp);

        {
            std::lock_guard<std::mutex> lck(mGPSMutex);
            mGPSData = ins;
        }
    }
}

bool InsDriver::parseGPCHC(std::string &message, InsDataType &ins) {
    auto clock = std::chrono::steady_clock::now();
    std::size_t head = message.find("$GPCHC");
    std::size_t tail = message.find("*");
    if (head == std::string::npos || tail == std::string::npos) {
        return false;
    }

    // keep the last 2 bytes as the checksum
    if ((message.size() - tail) < 3) {
        return false;
    }
    std::string msg = message.substr(head, tail - head + 3);
    message.erase(0, tail + 3);

    char Cs[512] = "";
    int result = sscanf(msg.c_str(), "$GPCHC,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d%s",
            &ins.gps_week, &ins.gps_time, &ins.heading, &ins.pitch, &ins.roll, &ins.gyro_x, &ins.gyro_y, &ins.gyro_z,
            &ins.acc_x, &ins.acc_y, &ins.acc_z, &ins.latitude, &ins.longitude, &ins.altitude, &ins.Ve, &ins.Vn, &ins.Vu,
            &ins.baseline, &ins.NSV1, &ins.NSV2, &ins.Status, &ins.age, &ins.Warnning, Cs);
    bool found = (result == 24);

    // check the checksum
    if (found) {
        uint8_t checksum = 0;
        if (Cs[0] == ',') {
            // CGI-610
            checksum = hex2dec(Cs[2]) * 16 + hex2dec(Cs[3]);
        } else if (Cs[0] == '*') {
            // P2
            checksum = hex2dec(Cs[1]) * 16 + hex2dec(Cs[2]);
        } else {
            LOG_WARN("Unknown INS checksum {}", Cs);
            found = false;
        }

        uint8_t datasum = 0;
        for (int i = 1; i < msg.size() - 3; i++) {
            datasum ^= msg[i];
        }
        if (checksum != datasum) {
            LOG_WARN("INS checksum is not match {}, {}", checksum, datasum);
            found = false;
        }
    }

    if (found) {
        ins.header   = "$GPCHC";
        ins.Cs = std::string(Cs);
        // std::cout << msg << std::endl;
        // printf("gps_week: %d\n", ins.gps_week);
        // printf("gps_time: %lf\n", ins.gps_time);
        // printf("heading: %lf\n", ins.heading);
        // printf("latitude: %lf\n", ins.latitude);
        // printf("longitude: %lf\n", ins.longitude);
    }
    auto elapseMs = since(clock).count();
    if (elapseMs >= 20) {
        LOG_WARN("Ins parse costs {} ms", elapseMs);
    }
    return found;
}

std::string InsDriver::formatGPCHC(InsDataType &ins) {
    char str[1024] = "";
    int gps_week = getGPSweek(ins.gps_timestamp);
    double gps_time = getGPSsecond(ins.gps_timestamp);
    sprintf(str, "$GPCHC,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%.10lf,%.10lf,%.10lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,",
            gps_week, gps_time, ins.heading, ins.pitch, ins.roll, ins.gyro_x, ins.gyro_y, ins.gyro_z,
            ins.acc_x, ins.acc_y, ins.acc_z, ins.latitude, ins.longitude, ins.altitude, ins.Ve, ins.Vn, ins.Vu,
            ins.baseline, ins.NSV1, ins.NSV2, ins.Status, ins.age, ins.Warnning);

    std::string msg = std::string(str);
    uint8_t datasum = 0;
    for (int i = 1; i < msg.size(); i++) {
        datasum ^= msg[i];
    }
    char Cs[4] = "";
    Cs[0] = '*';
    Cs[1] = dec2hex(datasum / 16);
    Cs[2] = dec2hex(datasum % 16);
    Cs[3] = '\0';

    msg = msg + std::string(Cs);
    return msg;
}