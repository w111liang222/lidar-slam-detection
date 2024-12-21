#ifndef __TYPES__H
#define __TYPES__H

struct Imu_t {
  Imu_t() {
    gyro_x = 0;
    gyro_y = 0;
    gyro_z = 0;
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
    timestamp = 0;
  }
  double gyro_x;           // rad / s
  double gyro_y;           // rad / s
  double gyro_z;           // rad / s
  double acc_x;            // m / s^2
  double acc_y;            // m / s^2
  double acc_z;            // m / s^2
  uint64_t timestamp;      // us
};

struct Ins_t {
  Ins_t() {
    latitude = 0;
    longitude = 0;
    altitude = 0;
    status = 0;
    timestamp = 0;
  }
  double latitude;         // degrees
  double longitude;        // degrees
  double altitude;         // m
  double heading;          // degrees
  double pitch;            // degrees
  double roll;             // degrees
  double Ve;               // m / s
  double Vn;               // m / s
  double Vu;               // m / s
  int status;
  uint64_t timestamp;      // us
};

#endif //__TYPES__H