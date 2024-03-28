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
    uint64_t gps_timestamp;     // us
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

#pragma pack(push, 1)
/* Livox lidar packet*/
typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;      /**< unit: 0.1 us */
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;
  uint8_t timestamp[8];
  float gyro_x;   /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;   /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;   /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;    /**< Accelerometer X axis, Unit:g */
  float acc_y;    /**< Accelerometer Y axis, Unit:g */
  float acc_z;    /**< Accelerometer Z axis, Unit:g */
} LivoxLidarEthernetImuPacket;

typedef enum {
  kLivoxLidarImuData = 0,
  kLivoxLidarCartesianCoordinateHighData = 0x01,
  kLivoxLidarCartesianCoordinateLowData = 0x02,
  kLivoxLidarSphericalCoordinateData = 0x03
} LivoxLidarPointDataType;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
  uint8_t header[3];
  short int roll;
  short int pitch;
  short int yaw;
  short int gyro_x;
  short int gyro_y;
  short int gyro_z;
  short int acc_x;
  short int acc_y;
  short int acc_z;
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  short int n_vel;
  short int e_vel;
  short int d_vel;
  uint8_t status;
  uint8_t null_40_45[6];
  short int polling_data[3];
  uint32_t gps_time;
  uint8_t polling_type;
  uint8_t xor_57;
  uint32_t gps_week;
  uint8_t xor_62;
} DY5711Pkt;
#pragma pack(pop)

#endif //__TYPES__H