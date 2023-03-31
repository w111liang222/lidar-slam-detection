#ifndef __TYPES__H
#define __TYPES__H

struct InsDataType {
    InsDataType() {
        header = "";
        gps_week = 0;
        gps_time = 0;
        gps_timestamp = 0;
        heading = 0;
        pitch = 0;
        roll = 0;
        gyro_x = 0;
        gyro_y = 0;
        gyro_z = 0;
        acc_x = 0;
        acc_y = 0;
        acc_z = 0;
        latitude = 0;
        longitude = 0;
        altitude = 0;
        Ve = 0;
        Vn = 0;
        Vu = 0;
        baseline = 0;
        NSV1 = 0;
        NSV2 = 0;
        Status = 0;
        age = 0;
        Warnning = 0;
        Cs = "";
    }
    std::string header;         // header, $GPCHC
    int gps_week;               // weeks from 1980-1-6
    double gps_time;            // seconds from 0:00:00 of this sunday
    uint64_t gps_timestamp;
    double heading;             // [deg]
    double pitch;               // [deg]
    double roll;                // [deg]
    double gyro_x;              // optional, [deg/s]
    double gyro_y;              // optional, [deg/s]
    double gyro_z;              // optional, [deg/s]
    double acc_x;               // optional, [g]
    double acc_y;               // optional, [g]
    double acc_z;               // optional, [g]
    double latitude;            // [deg]
    double longitude;           // [deg]
    double altitude;            // [deg]
    double Ve;                  // optional, [m/s]
    double Vn;                  // optional, [m/s]
    double Vu;                  // optional, [m/s]
    double baseline;            // optional, [m] baseline length
    int NSV1;                   // optional, number of satellite of major antenna
    int NSV2;                   // optional, number of satellite of secondary antenna
    int Status;                 // optional, INS status
    int age;                    // optional, latency of INS
    int Warnning;               // optional, INS Warnning
    std::string Cs;             // optional, verification
};

#endif //__TYPES__H