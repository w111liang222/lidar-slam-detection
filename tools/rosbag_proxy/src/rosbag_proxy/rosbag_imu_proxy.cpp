#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "UDPServer.h"

struct InsDataType {
    std::string header; // header, $GPCHC
    int gps_week;       // weeks from 1980-1-6
    double gps_time;    // seconds from 0:00:00 of this sunday
    double heading;     // [deg]
    double pitch;       // [deg]
    double roll;        // [deg]
    double gyro_x;      // optional, [deg/s]
    double gyro_y;      // optional, [deg/s]
    double gyro_z;      // optional, [deg/s]
    double acc_x;       // optional, [g]
    double acc_y;       // optional, [g]
    double acc_z;       // optional, [g]
    double latitude;    // [deg]
    double longitude;   // [deg]
    double altitude;    // [deg]
    double Ve;          // optional, [m/s]
    double Vn;          // optional, [m/s]
    double Vu;          // optional, [m/s]
    double baseline;    // optional, [m] baseline length
    int NSV1;           // optional, number of satellite of major antenna
    int NSV2;           // optional, number of satellite of secondary antenna
    int Status;         // optional, INS status
    int age;            // optional, latency of INS
    int Warnning;       // optional, INS Warnning
    std::string Cs;     // optional, verification
};

#define SECS_PER_WEEK (60L*60*24*7)
#define LEAP_SECOND 18

uint64_t getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return static_cast<uint64_t>(tv.tv_sec * 1000000 + tv.tv_usec);
}

int GPSweek() {
  double diff = (getCurrentTime() - 315964800000000ULL) / 1000000.0;
  return (int) (diff / SECS_PER_WEEK);
}

double GPSsecond() {
  double diff = (getCurrentTime() - 315964800000000ULL) / 1000000.0;
  return (diff - (int) (diff / SECS_PER_WEEK) * SECS_PER_WEEK + LEAP_SECOND);
}

char dec2hex(int d) {
  if (0 <= d && d <=9) return d + '0';
  if (d >= 10) return d - 10 + 'A';
  return '\0';
}

std::string formatGPCHC(InsDataType ins) {
    char str[1024] = "";
    sprintf(str, "%s,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%.10lf,%.10lf,%.10lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,",
            ins.header.c_str(), ins.gps_week, ins.gps_time, ins.heading, ins.pitch, ins.roll, ins.gyro_x, ins.gyro_y, ins.gyro_z,
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

class rosbag_imu_proxy {
  public:
    rosbag_imu_proxy()
    : nh("~"),
      imu_topic(nh.param<std::string>("imu_topic", "/imu")),
      gps_topic(nh.param<std::string>("gps_topic", "/gps")),
      device_ip(nh.param<std::string>("device_ip", "127.0.0.1")),
      port(nh.param<int>("port", 9888))
    {
        udp_server.reset(new UDPServer());
        last_frame = 0;
        imu_sub = nh.subscribe(imu_topic, 3, &rosbag_imu_proxy::imu_callback, this);
        gps_sub = nh.subscribe(gps_topic, 3, &rosbag_imu_proxy::gps_callback, this);
    }

    void imu_callback(const sensor_msgs::ImuPtr& data) {
        boost::mutex::scoped_lock lock(send_mutex_);
        std::cout << getCurrentTime() << ": Get topic: " << imu_topic <<  std::endl;
        last_msg = data;
        last_frame = getCurrentTime();
        send_imu(data, last_gps_msg);
        // last_gps_msg = nullptr;
    }

    void gps_callback(const sensor_msgs::NavSatFixPtr& data) {
        boost::mutex::scoped_lock lock(send_mutex_);
        std::cout << getCurrentTime() << ": Get topic: " << gps_topic <<  std::endl;
        last_gps_msg = data;
    }

    void send_imu(const sensor_msgs::ImuPtr& imu, const sensor_msgs::NavSatFixPtr& gps) {
        InsDataType data;
        data.header   = "$GPCHC";
        data.gps_week = GPSweek();
        data.gps_time = GPSsecond();
        data.heading = 0;
        data.pitch = 0;
        data.roll = 0;
        data.gyro_x = imu->angular_velocity.x / M_PI * 180.0;
        data.gyro_y = imu->angular_velocity.y / M_PI * 180.0;
        data.gyro_z = imu->angular_velocity.z / M_PI * 180.0;
        data.acc_x = imu->linear_acceleration.x / 9.81;
        data.acc_y = imu->linear_acceleration.y / 9.81;
        data.acc_z = imu->linear_acceleration.z / 9.81;
        if (gps != nullptr) {
          data.latitude = gps->latitude;
          data.longitude = gps->longitude;
          data.altitude = gps->altitude;
          data.Status = 1;
        } else {
          data.latitude = 0;
          data.longitude = 0;
          data.altitude = 0;
          data.Status = 0;
        }
        data.Ve = 0;
        data.Vn = 0;
        data.Vu = 0;
        data.baseline = 00;
        data.NSV1 = 0;
        data.NSV2 = 0;
        data.age = 0;
        data.Warnning = 0;
        std::string message = formatGPCHC(data);
        udp_server->UDPSendtoBuf(device_ip, port, (char *)(message.c_str()), message.size());
    }

    void restreamFrame(double max_age) {
        if (last_msg == 0) {
            return;
        }
        if ( last_frame + uint64_t(max_age * 1000000ull) < getCurrentTime() ) {
            boost::mutex::scoped_lock lock(send_mutex_);
            send_imu(last_msg, last_gps_msg);
        }
    }

    void spin() {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        while(ros::ok()) {
            restreamFrame(0.5);
            usleep(100000);
        }
    }

  private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;
    std::string imu_topic;
    std::string gps_topic;
    std::string device_ip;
    int port;
    uint64_t last_frame;
    sensor_msgs::ImuPtr last_msg;
    sensor_msgs::NavSatFixPtr last_gps_msg;
    boost::mutex send_mutex_;
    std::unique_ptr<UDPServer> udp_server;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_imu_proxy");
    rosbag_imu_proxy node;
    node.spin();
    return 0;
}