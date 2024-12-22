#ifndef __DECODER_BASE_H
#define __DECODER_BASE_H
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <arpa/inet.h>

#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)

static const float RS_ANGLE_RESOLUTION = 0.01;

/** LS-C-16 decoder variable **/
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96
static const int LSC16_FIRINGS_PER_BLOCK = 2;
static const int LSC16_SCANS_PER_FIRING = 16;

// block
typedef struct raw_block
{
  uint16_t header;  ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];  // 96
} raw_block_t;

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const double  DSR_TOFFSET       = 1;   
static const double  FIRING_TOFFSET    = 16; 
static const float LSC16_BLOCK_TDURATION = 100.0f;  // [µs]
static const float LSC16_DSR_TOFFSET = 3.125f;        // [µs]
static const float LSC16_FIRING_TOFFSET = 50.0f;    // [µs]
static const float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
static const int FIRINGS_PER_PACKET =
        LSC16_FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;
static const uint16_t UPPER_BANK = 0xeeff;
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint32_t time_stamp;
  uint8_t factory[2];
} raw_packet_t;

union two_bytes
{
  uint16_t distance;
  uint8_t bytes[2];
};

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_altitude[16] = {
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494
};

static const double cos_scan_altitude[16] = {
    std::cos(scan_altitude[ 0]), std::cos(scan_altitude[ 1]),
    std::cos(scan_altitude[ 2]), std::cos(scan_altitude[ 3]),
    std::cos(scan_altitude[ 4]), std::cos(scan_altitude[ 5]),
    std::cos(scan_altitude[ 6]), std::cos(scan_altitude[ 7]),
    std::cos(scan_altitude[ 8]), std::cos(scan_altitude[ 9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
};

static const double sin_scan_altitude[16] = {
    std::sin(scan_altitude[ 0]), std::sin(scan_altitude[ 1]),
    std::sin(scan_altitude[ 2]), std::sin(scan_altitude[ 3]),
    std::sin(scan_altitude[ 4]), std::sin(scan_altitude[ 5]),
    std::sin(scan_altitude[ 6]), std::sin(scan_altitude[ 7]),
    std::sin(scan_altitude[ 8]), std::sin(scan_altitude[ 9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
};

static const float ROTATION_RESOLUTION = 0.01f;   /**< degrees 旋转角分辨率*/
static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

struct Firing {
    // Azimuth associated with the first shot within this firing.
    double firing_azimuth;
    double azimuth[LSC16_SCANS_PER_FIRING];
    double distance[LSC16_SCANS_PER_FIRING];
    double intensity[LSC16_SCANS_PER_FIRING];
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
  uint8_t data[1];             /**< Point cloud data. */
} LivoxLidarEthernetPacket;

// SDK related
typedef enum {
  kIndustryLidarType = 1,
  kVehicleLidarType = 2,
  kDirectLidarType = 4,
  kLivoxLidarType = 8
} LidarProtoType;

typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarCartesianHighRawPoint;

typedef struct {
  int16_t x;            /**< X axis, Unit:cm */
  int16_t y;            /**< Y axis, Unit:cm */
  int16_t z;            /**< Z axis, Unit:cm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarCartesianLowRawPoint;

typedef struct {
  uint32_t depth;
  uint16_t theta;
  uint16_t phi;
  uint8_t reflectivity;
  uint8_t tag;
} LivoxLidarSpherPoint;

typedef struct {
  unsigned int version_offset = 0;            
  unsigned int length_offset = 1;            
  unsigned int time_interval_offset = 3;            
  unsigned int dot_num_offset = 5;            
  unsigned int udp_cnt_offset = 7;            
  unsigned int frame_cnt_offset = 9;            
  unsigned int data_type_offset = 10;            
  unsigned int time_type_offset = 11;            
  unsigned int reserved_offset = 12;         
  unsigned int crc32_offset = 24;         
  unsigned int timestamp_offset = 28;            
  unsigned int data_offset = 36;
  unsigned int cartesian_high_size = 14;
  unsigned int cartesian_low_size = 8;
  unsigned int spher_size = 10;
  uint64_t frame_interval_ = 100000000; // ns
  LivoxLidarCartesianHighRawPoint high_point;
  LivoxLidarCartesianLowRawPoint low_point;
  LivoxLidarSpherPoint spher_point;
} LivoxLidarPacketOffsetInfo;


typedef enum {
  kLivoxLidarImuData = 0,
  kLivoxLidarCartesianCoordinateHighData = 0x01,
  kLivoxLidarCartesianCoordinateLowData = 0x02,
  kLivoxLidarSphericalCoordinateData = 0x03
} LivoxLidarPointDataType;

/* Livox lidar rawpacket*/
typedef struct {
  LidarProtoType lidar_type;
  uint16_t length;
  uint32_t handle;
  bool extrinsic_enable;
  uint32_t point_num;
  uint8_t data_type;
  uint8_t line_num;
  uint64_t time_stamp;
  uint64_t point_interval;
  std::vector<uint8_t> raw_data;
} RawPacket;


/**RS-LiDAR decorder strcut**/
typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
} RSTimestampYMD;

typedef struct
{
  uint64_t id;
  uint8_t reserved_1[12];
  RSTimestampYMD timestamp;
  uint8_t lidar_type;
  uint8_t reserved_2[7];
  uint16_t temp_raw;
  uint8_t reserved_3[2];
} RSMsopHeader;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} RSChannel;

typedef struct
{
  uint16_t id;
  uint16_t azimuth;
  RSChannel channels[32];
} RS16MsopBlock;

typedef struct
{
  RSMsopHeader header;
  RS16MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS16MsopPkt;

typedef struct
{
  uint16_t id;
  uint16_t azimuth;
  RSChannel channels[32];
} RS32MsopBlock;

typedef struct
{
  RSMsopHeader header;
  RS32MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS32MsopPkt;

typedef struct
{
  uint8_t id;
  uint8_t ret_id;
  uint16_t azimuth;
  RSChannel channels[80];
} RS80MsopBlock;

typedef struct
{
  uint8_t sec[6];
  uint32_t us;
} RSTimestampUTC;

typedef struct
{
  uint32_t id;
  uint16_t protocol_version;
  uint8_t reserved_1;
  uint8_t wave_mode;
  uint8_t temp_low;
  uint8_t temp_high;
  RSTimestampUTC timestamp;
  uint8_t reserved_2[10];
  uint8_t lidar_type;
  uint8_t reserved_3[49];
} RSMsopHeaderNew;

typedef struct
{
  RSMsopHeaderNew header;
  RS80MsopBlock blocks[4];
  uint8_t reserved[188];
  unsigned int index;
} RS80MsopPkt;

// Echo mode
enum RSEchoMode
{
  ECHO_SINGLE = 0,
  ECHO_DUAL
};

// decoder const param
struct RSDecoderConstParam
{
  // packet len
  uint16_t MSOP_LEN;
  uint16_t DIFOP_LEN;

  // packet identity
  uint8_t MSOP_ID_LEN;
  uint8_t DIFOP_ID_LEN;
  uint8_t MSOP_ID[8];
  uint8_t DIFOP_ID[8];
  uint8_t BLOCK_ID[2];

  // packet structure
  uint16_t LASER_NUM;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;

  // distance & temperature
  float DISTANCE_MIN;
  float DISTANCE_MAX;
  float DISTANCE_RES;
  float TEMPERATURE_RES;
};

typedef struct
{
  uint64_t MSOP_ID;
  uint64_t DIFOP_ID;
  uint64_t BLOCK_ID;
  uint32_t PKT_RATE;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;
  uint16_t LASER_NUM;
  float DSR_TOFFSET;
  float FIRING_FREQUENCY;
  float DIS_RESOLUTION;
  float RX;
  float RY;
  float RZ;
} LidarConstantParameter;

typedef struct RSDecoderParam  ///< LiDAR decoder parameter
{
  bool config_from_file = false; ///< Internal use only for debugging
  std::string angle_path = "";   ///< Internal use only for debugging
  bool wait_for_difop = true;    ///< true: start sending point cloud until receive difop packet
  float min_distance = 0.2f;     ///< Minimum distance of point cloud range
  float max_distance = 200.0f;   ///< Max distance of point cloud range
  float start_angle = 0.0f;      ///< Start angle of point cloud
  float end_angle = 360.0f;      ///< End angle of point cloud
  // SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  
                                 ///< 1: Split frames by split_angle;
                                 ///< 2: Split frames by fixed number of blocks;
                                 ///< 3: Split frames by custom number of blocks (num_blks_split)
  float split_angle = 0.0f;      ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
  uint16_t num_blks_split = 1;   ///< Number of packets in one frame, only be used when split_frame_mode=3
  bool use_lidar_clock = false;  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points = false;     ///< true: discard NAN points; false: reserve NAN points
  // RSTransformParam transform_param; ///< Used to transform points

} RSDecoderParam;

struct RSDecoderMechConstParam
{
  RSDecoderConstParam base;

  // lens center
  float RX;
  float RY;
  float RZ;

  // firing_ts/chan_ts
  double BLOCK_DURATION;
  double CHAN_TSS[128];
  float CHAN_AZIS[128];
};

typedef struct
{
  uint16_t start_angle;
  uint16_t end_angle;
} RSFOV;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t dest_ip[4];
  uint8_t mac_addr[6];
  uint16_t msop_port;
  uint16_t reserve_1;
  uint16_t difop_port;
  uint16_t reserve_2;
} RSEthNetV2;

typedef struct
{
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
} RSVersionV2;

typedef struct
{
  uint8_t num[6];
} RSSN;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  RSTimestampUTC timestamp;
} RSTimeInfo;

typedef struct
{
  uint16_t device_current;
  uint16_t vol_fpga;
  uint16_t vol_12v;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_apd;
  uint8_t reserved[12];
} RSStatusV2;

typedef struct
{
  uint16_t bot_fpga_temperature;
  uint16_t recv_A_temperature;
  uint16_t recv_B_temperature;
  uint16_t main_fpga_temperature;
  uint16_t main_fpga_core_temperature;
  uint16_t real_rpm;
  uint8_t lane_up;
  uint16_t lane_up_cnt;
  uint16_t main_status;
  uint8_t gps_status;
  uint8_t reserved[22];
} RSDiagnoV2;

typedef struct
{
  uint8_t sign;
  uint16_t value;
} RSCalibrationAngle;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV2 eth;
  RSFOV fov;
  uint8_t reserved1[2];
  uint16_t phase_lock_angle;
  RSVersionV2 version;
  uint8_t reserved2[229];
  RSSN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  RSTimeInfo time_info;
  RSStatusV2 status;
  uint8_t reserved3[5];
  RSDiagnoV2 diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle vert_angle_cali[32];
  RSCalibrationAngle horiz_angle_cali[32];
  uint8_t reserved4[586];
  uint16_t tail;
} RSHELIOSDifopPkt;

typedef struct
{
  uint8_t tt[2];
} RSTemperature;

typedef struct
{
  uint8_t id[4];
  uint16_t protocol_version;
  uint8_t reserved1[14];
  RSTimestampUTC timestamp;
  uint8_t lidar_type;
  uint8_t reserved2[7];
  RSTemperature temp;
  uint8_t reserved3[2];
} RSHELIOSMsopHeader;

typedef struct
{
  uint8_t id[2];
  uint16_t azimuth;
  RSChannel channels[32];
} RSHELIOSMsopBlock;

typedef struct
{
  RSHELIOSMsopHeader header;
  RSHELIOSMsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RSHELIOSMsopPkt;

typedef struct
{
  uint8_t num[6];
} RSSn;

typedef struct
{
  uint16_t distance;
  uint16_t pitch;
  uint16_t yaw;
  uint8_t intensity;
  uint8_t point_attribute;
  uint8_t elongation;
} RSM1Channel;

typedef struct
{
  uint8_t time_offset;
  uint8_t return_seq;
  RSM1Channel channel[5];
} RSM1Block;

typedef struct
{
  uint32_t id;
  uint16_t pkt_cnt;
  uint16_t protocol_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp;
  uint8_t reserved[10];
  uint8_t lidar_type;
  int8_t temperature;
} RSM1MsopHeader;

typedef struct
{
  RSM1MsopHeader header;
  RSM1Block blocks[25];
  uint8_t reserved[3];
} RSM1MsopPkt;

typedef struct
{
  uint8_t ip_local[4];
  uint8_t ip_remote[4];
  uint8_t mac[6];
  uint8_t msop_port[2];
  uint8_t difop_port[2];
} RSM1DifopEther;

typedef struct
{
  uint8_t horizontal_fov_start[2];
  uint8_t horizontal_fov_end[2];
  uint8_t vertical_fov_start[2];
  uint8_t vertical_fov_end[2];
} RSM1DifopFov;

typedef struct
{
  uint8_t pl_ver[5];
  uint8_t ps_ver[5];
} RSM1DifopVerInfo;

typedef struct
{
  uint8_t current_1[3];
  uint8_t current_2[3];
  uint16_t voltage_1;
  uint16_t voltage_2;
  uint8_t reserved[10];
} RSM1DifopRunSts;

typedef struct
{
  uint8_t param_sign;
  uint16_t data;
} RSM1DifopCalibration;

typedef struct
{
  uint64_t id;
  uint8_t reserved_1;
  uint8_t frame_rate;
  RSM1DifopEther ether;
  RSM1DifopFov fov_setting;
  RSM1DifopVerInfo ver_info;
  RSSn sn;
  uint8_t return_mode;
  RSTimeInfo time_info;
  RSM1DifopRunSts status;
  uint8_t diag_reserved[40];
  RSM1DifopCalibration cali_param[20];
  uint8_t reserved_2[71];
} RSM1DifopPkt;

class ChanAngles
{
public:
  ChanAngles() {}
  ChanAngles(uint16_t chan_num)
    : chan_num_(chan_num)
  {
    vert_angles_.resize(chan_num_);
    horiz_angles_.resize(chan_num_);
    user_chans_.resize(chan_num_);
  }

  int loadFromDifop(const RSCalibrationAngle vert_angle_arr[], 
      const RSCalibrationAngle horiz_angle_arr[])
  {
    std::vector<int32_t> vert_angles;
    std::vector<int32_t> horiz_angles;
    int ret = 
      loadFromDifop (vert_angle_arr, horiz_angle_arr, chan_num_, vert_angles, horiz_angles);
    if (ret < 0)
      return ret;

    vert_angles_.swap(vert_angles);
    horiz_angles_.swap(horiz_angles);
    genUserChan(vert_angles_, user_chans_);
    return 0;
  }

  uint16_t toUserChan(uint16_t chan)
  {
    return user_chans_[chan];
  }

  int32_t horizAdjust(uint16_t chan, int32_t horiz)
  {
    return (horiz + horiz_angles_[chan]);
  }

  int32_t vertAdjust(uint16_t chan)
  {
    return vert_angles_[chan];
  }

  void print()
  {
    std::cout << "---------------------" << std::endl
              << "chan_num:" << chan_num_ << std::endl;

    std::cout << "vert_angle\thoriz_angle\tuser_chan" << std::endl;

    for (uint16_t chan = 0; chan < chan_num_; chan++)
    {
      std::cout << vert_angles_[chan] << "\t" 
                << horiz_angles_[chan] << "\t" 
                << user_chans_[chan] << std::endl;
    }
  }

#ifndef UNIT_TEST
private:
#endif

  static
  void genUserChan(const std::vector<int32_t>& vert_angles, std::vector<uint16_t>& user_chans)
  {
    user_chans.resize(vert_angles.size());

    for (size_t i = 0; i < vert_angles.size(); i++)
    {
      int32_t angle = vert_angles[i];
      uint16_t chan = 0;

      for (size_t j = 0; j < vert_angles.size(); j++)
      {
        if (vert_angles[j] < angle)
        {
          chan++;
        }
      }

      user_chans[i] = chan;
    }
  }

  static int loadFromDifop(const RSCalibrationAngle* vert_angle_arr, 
      const RSCalibrationAngle* horiz_angle_arr, size_t size, 
      std::vector<int32_t>& vert_angles, std::vector<int32_t>& horiz_angles)
  {
    vert_angles.clear();
    horiz_angles.clear();

    for (size_t i = 0; i < size; i++)
    {
      const RSCalibrationAngle& vert = vert_angle_arr[i];
      const RSCalibrationAngle& horiz = horiz_angle_arr[i];
      int32_t v;

      if (vert.sign == 0xFF)
        return -1;

      v = ntohs(vert.value);
      if (vert.sign != 0) v = -v;
      vert_angles.emplace_back(v);

      if (!angleCheck (v))
        return -1;

      v = ntohs(horiz.value);
      if (horiz.sign != 0) v = -v;
      horiz_angles.emplace_back(v);

      if (!angleCheck (v))
        return -1;

    }

    return ((vert_angles.size() > 0) ? 0 : -1);
  }

  static bool angleCheck(int32_t v)
  {
    return ((-9000 <= v) && (v < 9000));
  }

  uint16_t chan_num_;
  std::vector<int32_t> vert_angles_;
  std::vector<int32_t> horiz_angles_;
  std::vector<uint16_t> user_chans_;
};


class Trigon
{
public:

  constexpr static int32_t MIN = -9000;
  constexpr static int32_t MAX = 45000;

  Trigon()
  {
    int32_t range = MAX - MIN;
#ifdef DBG
    o_angles_ = (int32_t*)malloc(range * sizeof(int32_t));
#endif
    o_sins_ = (float*)malloc(range * sizeof(float));
    o_coss_ = (float*)malloc(range * sizeof(float));

    for (int32_t i = MIN, j = 0; i < MAX; i++, j++)
    {
      double rads = static_cast<double>(i) * 0.01;
      rads = rads * M_PI / 180;

#ifdef DBG
      o_angles_[j] = i;
#endif
      o_sins_[j] = (float)std::sin(rads);
      o_coss_[j] = (float)std::cos(rads);
    }

#ifdef DBG
    angles_ = o_angles_ - MIN;
#endif
    sins_ = o_sins_ - MIN;
    coss_ = o_coss_ - MIN;
  }

  ~Trigon()
  {
    free(o_coss_);
    free(o_sins_);
#ifdef DBG
    free(o_angles_);
#endif
  }

  float sin(int32_t angle)
  {
    if (angle < MIN || angle >= MAX)
    {
      angle = 0;
    }

    return sins_[angle];
  }

  float cos(int32_t angle)
  {
    if (angle < MIN || angle >= MAX)
    {
      angle = 0;
    }

    return coss_[angle];
  }

  void print()
  {
    for (int32_t i = -10; i < 10; i++)
    {
      std::cout << 
#ifdef DBG 
        angles_[i] << "\t" << 
#endif
        sins_[i] << "\t" << coss_[i] << std::endl;
    }
  }

private:
#ifdef DBG 
  int32_t* o_angles_;
  int32_t* angles_;
#endif
  float* o_sins_;
  float* o_coss_;
  float* sins_;
  float* coss_;
};

class AzimuthSection
{
public:
  AzimuthSection(int32_t start, int32_t end)
  {
    start_ = start % 36000;
    end_ = (end <= 36000) ? end : (end % 36000);
    cross_zero_ = (start_ > end_);

    full_round_ = ((start_ == 0) && (end_ == 36000));
  }

  bool in(int32_t angle)
  {
    if (full_round_)
      return true;

    if (cross_zero_)
    {
      return (angle >= start_) || (angle < end_);
    }
    else
    {
      return (angle >= start_) && (angle < end_);
    }
  }

#ifndef UNIT_TEST
private:
#endif
  int32_t start_;
  int32_t end_;
  bool cross_zero_;
  bool full_round_;
};

class DistanceSection
{
public:
  DistanceSection (float min, float max, float usr_min, float usr_max)
    : min_((usr_min > min) ? usr_min : min), max_((usr_max < max) ? usr_max : max)
  {
  }

  bool in(float distance)
  {
    return ((min_ <= distance) && (distance <= max_));
  }

#ifndef UNIT_TEST
private:
#endif

  float min_;
  float max_;
};

#pragma pack(pop)

#endif