#include "ins_driver.h"

#include <sys/prctl.h>
#include <stdlib.h>
#include <libgpsmm.h>

#include "SystemUtils.h"
#include "Transform.h"
#include "UDPServer.h"
#include "SerialDriver.h"
#include "UTMProjector.h"

double getYawAngle(Transform &trans) {
    Translation unit_vector(0, 1.0, 0);
    unit_vector = trans.rotation_ * unit_vector;
    double yaw = atan2(unit_vector(1), unit_vector(0));
    return (yaw - M_PI / 2.0);
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
    double heading = data.heading - get_grid_convergence(projector.GetLongitude0(), data.latitude, data.longitude);
    return getTransformFromRPYT(dataX, dataY, data.altitude, -heading, data.pitch, data.roll);
}

InsDriver::InsDriver(std::string ins_type, std::string mode) : mInsType(ins_type) {
    mMode = (mode.compare("online") == 0) ? modeType::online : modeType::offline;
    mUseSeperateIMU = false;
    mUseSeperateGPS = false;
    mStaticTransform = Transform();
    mStartTransfer = false;
    resetRuntimeVariables();
}

InsDriver::~InsDriver() {
    stopRun();
}

void InsDriver::startRun(int port, std::string device) {
    mPort = port;
    mDevice = device;
    mThreadStopFlag = false;

    // Unix socket relay
    mUnixClient.reset(new UnixSocketClient("/tmp/imu_data.sock"));

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

    // SLAM odometry
    mCore = create_core();
    mCore->subscribe("slam.odometry", &InsDriver::onPoseMessage, this);
    mCore->start();
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

void InsDriver::setExtrinsicParameter(Transform& extrinsic) {
    mStaticTransform = extrinsic;
}

void InsDriver::setData(InsDataType &data, uint64_t timestamp, bool is_imu) {
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

    // do not push imu-only data to queue
    if (is_imu) {
        return;
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
    mQueuedData.emplace_back(data);

    // the same data
    if (mTimedData.size() > 0 && mTimedData.back().gps_timestamp == timestamp) {
        return;
    }
    mTimedData.emplace_back(data);
}

uint64_t getStamp(InsDataType &data) {
    return uint64_t(data.gps_timestamp);
}
uint64_t getStamp(nav_msgs::Odometry &data) {
    return uint64_t(data.header.stamp);
}
Transform getTransform(InsDataType &data) {
    return computeRTKTransform(data);
}
Transform getTransform(nav_msgs::Odometry &data) {
    Eigen::Matrix4d odometry = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q;
    q.x() = data.pose.pose.orientation.x;
    q.y() = data.pose.pose.orientation.y;
    q.z() = data.pose.pose.orientation.z;
    q.w() = data.pose.pose.orientation.w;
    odometry.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    odometry(0, 3) = data.pose.pose.position.x;
    odometry(1, 3) = data.pose.pose.position.y;
    odometry(2, 3) = data.pose.pose.position.z;
    return Transform(odometry);
}

template <typename T>
Transform getInterplatedPosition(T &data, uint64_t t) {
    int idx0, idx1;
    if (t <= getStamp(data.front())) {
        idx0 = 0;
        idx1 = 1;
    } else if (t >= getStamp(data.back())) {
        idx0 = data.size() - 2;
        idx1 = data.size() - 1;
    } else {
        int begin = 0, end = data.size() - 1, mid = 0;
        while(begin < end) {
            mid = (begin + end) / 2;
            if (getStamp(data[mid]) > t) {
                end = mid;
            } else if (getStamp(data[mid + 1]) < t) {
                begin = mid + 1;
            } else {
                break;
            }
        }
        idx0 = mid;
        idx1 = mid + 1;
    }

    double t_diff_ratio = (static_cast<double>(t) - getStamp(data[idx0])) / static_cast<double>(getStamp(data[idx1]) - getStamp(data[idx0]));
    Transform T0 = getTransform(data[idx0]);
    Transform T1 = getTransform(data[idx1]);
    Vector6 diff_vector = (T0.inverse() * T1).log();
    Transform out = T0 * Transform::exp(t_diff_ratio * diff_vector);
    return out;
}

bool InsDriver::getMotion(std::vector<double> &ins_pose, std::vector<double> &motion_t, double &motion_heading, uint64_t t0, uint64_t t1) {
    Transform lastTransform, currentTransform;
    if (mPoseData.size() >= 2 && t0 > getStamp(mPoseData.front()) && t1 < getStamp(mPoseData.back())) {
        lastTransform    = getInterplatedPosition(mPoseData, t0);
        currentTransform = getInterplatedPosition(mPoseData, t1);
    } else if (mInsType.compare("6D") == 0) {
        lastTransform    = getInterplatedPosition(mTimedData, t0);
        currentTransform = getInterplatedPosition(mTimedData, t1);
    } else {
        return false;
    }

    Transform motion = mStaticTransform.inverse() * (currentTransform.inverse() * lastTransform) * mStaticTransform;
    Matrix p = currentTransform.matrix().transpose();
    Matrix m = motion.matrix().transpose();

    ins_pose       = std::vector<double>(p.data(), p.data() + p.rows() * p.cols());
    motion_t       = std::vector<double>(m.data(), m.data() + m.rows() * m.cols());
    motion_heading = getYawAngle(motion);
    return true;
}

bool InsDriver::trigger(uint64_t timestamp, bool &motion_valid, std::vector<double> &ins_pose, std::vector<double> &motion_t,
                        double &motion_heading, InsDataType &ins, std::vector<InsDataType> &imu) {
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
    ins = mQueuedData.back();
    std::swap(imu, mQueuedData);

    // normal GPS is 1Hz data rate, need to force align to the main sensor (LiDAR)
    if (mUseSeperateGPS && ins.Status == 0) {
        for (auto &data : imu) {
            if (data.Status != 0) {
                ins.gps_timestamp = data.gps_timestamp;
                ins.latitude      = data.latitude;
                ins.longitude     = data.longitude;
                ins.altitude      = data.altitude;
                ins.Status        = data.Status;
                ins.Ve            = data.Ve;
                break;
            }
        }
    }

    // estimate the realtive motion
    if (!mFirstTrigger) {
        motion_valid = getMotion(ins_pose, motion_t, motion_heading, mLastTriggerTime, timestamp);
    }

    mFirstTrigger = false;
    mLastTriggerTime = timestamp;
    return true;
}

void InsDriver::onPoseMessage(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const nav_msgs::Odometry *msg) {
    std::lock_guard<std::mutex> lck(mMutex);
    if (mPoseData.size() > 0) {
        int64_t stamp_diff = msg->header.stamp - mPoseData.back().header.stamp;
        if (stamp_diff <= 0 || stamp_diff > 1000000) { // non-continuous or unorder
            LOG_WARN("slam.odometry stamp is invalid, {}, {}", mPoseData.back().header.stamp, msg->header.stamp);
            mPoseData.clear();
            return;
        }
    }

    if (mPoseData.size() > 100) {
        mPoseData.erase(mPoseData.begin());
    }
    mPoseData.emplace_back(*msg);
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
        bool found = (parseLivoxImu(buf, receveSize, ins) || parseGPCHC(message, ins));
        if (found) {
            if (mUseSeperateGPS) {
                std::lock_guard<std::mutex> lck(mGPSMutex);
                ins.Status      = mGPSData.Status;
                ins.latitude    = mGPSData.latitude;
                ins.longitude   = mGPSData.longitude;
                mGPSData.Status = 0; //reset the GPS status
            }

            uint64_t hostTime = getCurrentTime();
            uint64_t gpsTime = gps2Utc(ins.gps_week, ins.gps_time);
            double diffMSec = fabs(gpsTime / 1000.0 - hostTime / 1000.0);
            if (diffMSec > 1000) {
                ins.gps_timestamp = hostTime;
                LOG_WARN("UDP: gps/host time diff {} ms, check the time sync!", diffMSec);
            } else {
                ins.gps_timestamp = gpsTime;
            }
            setData(ins, ins.gps_timestamp, false);
        } else {
            LOG_WARN("Ins parse udp source error, {}", message);
        }
    }
}

void InsDriver::run_com() {
    prctl(PR_SET_NAME, "Ins Com Recev", 0, 0, 0);
    LOG_INFO("start ins com receving, device {}", mDevice);
    serial::Serial serial(mDevice, 230400, serial::Timeout::simpleTimeout(100));
    serial.open();

    std::string message = "";
    while (!mThreadStopFlag) {
        if (!serial.isOpen()) {
            usleep(100000);
            serial.open();
            continue;
        }

        auto clock = std::chrono::steady_clock::now();
        if (serial.waitReadable()) {
            message += serial.read(serial.available());
            auto elapseMs = since(clock).count();
            if (elapseMs >= 50) {
                LOG_WARN("Ins com receive buffer wait for {} ms", elapseMs);
            }

            InsDataType ins;
            bool found = parseBDDB0B(message, ins) || parseGPCHC(message, ins);
            if (found) {
                if (mUseSeperateIMU) {
                    ins.gps_week = getGPSweek(getCurrentTime());
                    ins.gps_time = getGPSsecond(getCurrentTime());
                }
                if (mUseSeperateGPS) {
                    std::lock_guard<std::mutex> lck(mGPSMutex);
                    ins.Status      = mGPSData.Status;
                    ins.latitude    = mGPSData.latitude;
                    ins.longitude   = mGPSData.longitude;
                    mGPSData.Status = 0; //reset the GPS status
                }

                uint64_t hostTime = getCurrentTime();
                uint64_t gpsTime = gps2Utc(ins.gps_week, ins.gps_time);
                double diffMSec = gpsTime / 1000.0 - hostTime / 1000.0;
                if (diffMSec > 60000) {
                    setSystemTime(gpsTime);
                    hostTime = gpsTime;
                    LOG_WARN("COM: gps/host time diff {} ms, adjust system clock", diffMSec);
                }

                ins.gps_timestamp = hostTime;
                setData(ins, ins.gps_timestamp, false);
            }

            if (message.size() > 512) {
                LOG_WARN("Ins unparsed data is too much, size: {}", message.size());
                message = message.substr(256);
            }
        }
    }
    serial.close();
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
        if (!gps_rec.waiting(100000)) {
            continue;
        }

        struct gps_data_t* gpsd_data = nullptr;
        while (!mThreadStopFlag && (gpsd_data = gps_rec.read()) == nullptr) {
            usleep(10000);
        }

        if (gpsd_data == nullptr) {
            continue;
        }

        if (gpsd_data->fix.mode < MODE_2D) {
            LOG_WARN("GPSD is not fix, {}", gpsd_data->fix.mode);
            continue;
        }

        ins.gps_timestamp = getCurrentTime();
        ins.Status        = 1;
        ins.latitude      = gpsd_data->fix.latitude;
        ins.longitude     = gpsd_data->fix.longitude;
        ins.Ve            = gpsd_data->fix.speed;

        {
            std::lock_guard<std::mutex> lck(mGPSMutex);
            mGPSData = ins;
        }
    }
}

bool InsDriver::parseGPCHC(std::string &message, InsDataType &ins) {
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
    } else {
        LOG_WARN("$GPCHC data parse error {}", msg);
    }

    if (found) {
        ins.header   = "$GPCHC";
        ins.Cs = std::string(Cs);
    }
    return found;
}

bool InsDriver::parseBDDB0B(std::string &message, InsDataType &ins) {
    if (message.size() < 3) {
        return false;
    }

    int header_pos = -1;
    for (size_t i = 0; i < message.size() - 2; ++i) {
        if (((message[i] & 0xff) == 0xbd) && ((message[i + 1] & 0xff) == 0xdb) && ((message[i + 2] & 0xff) == 0x0b)) { // find header
            header_pos = i;
            break;
        }
    }
    if (header_pos < 0) {
        return false;
    }

    if (header_pos > 0) {
        message.erase(0, header_pos);
    }

    if (message.size() < 63) {
        return false;
    }

    bool found = false;
    char checksum = 0;
    for(int i = 0; i < 57; i++) {
        checksum = checksum ^ message[i];
    }
    if (message[57] == checksum) {
        found = true;
        static int position_type = 0;
        DY5711Pkt pkt_struct;
        std::memcpy(&pkt_struct, message.data(), sizeof(pkt_struct));
        ins.header   = "$GPCHC";
        ins.gps_week = getGPSweek(getCurrentTime());
        ins.gps_time = getGPSsecond(getCurrentTime());

        ins.latitude = pkt_struct.latitude * 1e-7L;
        ins.longitude = pkt_struct.longitude *1e-7L;
        ins.altitude = pkt_struct.altitude * 1e-3L;

        ins.heading = pkt_struct.yaw * 360.0 / 32768;
        ins.pitch = pkt_struct.pitch * 360.0 / 32768;
        ins.roll = pkt_struct.roll * 360.0 / 32768;

        ins.gyro_x = pkt_struct.gyro_x * 300.0 / 32768;
        ins.gyro_y = pkt_struct.gyro_y * 300.0 / 32768;
        ins.gyro_z = pkt_struct.gyro_z * 300.0 / 32768;

        ins.acc_x = pkt_struct.acc_x * 12.0 / 32768;
        ins.acc_y = pkt_struct.acc_y * 12.0 / 32768;
        ins.acc_z = pkt_struct.acc_z * 12.0 / 32768;

        ins.Ve = pkt_struct.e_vel * 100.0 / 32768;
        ins.Vn = pkt_struct.n_vel * 100.0 / 32768;
        ins.Vu = pkt_struct.d_vel * 100.0 / 32768;
        short int pdata_type = pkt_struct.polling_type;
        if (pdata_type == 32) {
            position_type = int(pkt_struct.polling_data[0]);
        }
        ins.Status = position_type;
        message.erase(0, 58);
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

bool InsDriver::parseLivoxImu(char buf[], const int& len, InsDataType &ins) {
  if (len == 60) {
    uint64_t hostTime = getCurrentTime();
    LivoxLidarEthernetImuPacket* packet = reinterpret_cast<LivoxLidarEthernetImuPacket*>(buf);
    if (packet->data_type == kLivoxLidarImuData) {
      ins.gyro_x = packet->gyro_x / M_PI * 180.0; // rad/s to deg/s
      ins.gyro_y = packet->gyro_y / M_PI * 180.0; // rad/s to deg/s
      ins.gyro_z = packet->gyro_z / M_PI * 180.0; // rad/s to deg/s
      ins.acc_x = packet->acc_x;
      ins.acc_y = packet->acc_y;
      ins.acc_z = packet->acc_z;

      ins.heading = 0;
      ins.pitch = 0;
      ins.roll = 0;
      ins.latitude = 0;
      ins.longitude = 0;
      ins.altitude = 0;
      ins.Status = 0;
      ins.gps_week = getGPSweek(hostTime);
      ins.gps_time = getGPSsecond(hostTime);
      return true;
    }
  }

  return false;
}