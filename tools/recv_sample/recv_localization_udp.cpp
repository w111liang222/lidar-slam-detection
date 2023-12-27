#include "UDPServer.h"

struct RTKType {
  RTKType() {
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
    status = 0;
    state = "";
  }
  double heading;
  double pitch;
  double roll;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  double acc_x;
  double acc_y;
  double acc_z;
  double latitude;
  double longitude;
  double altitude;
  double Ve;
  double Vn;
  double Vu;
  int status;
  std::string state;
};

int hex2dec(char ch) {
  if ('0' <= ch && ch <= '9') return ch - '0';
  if ('A' <= ch && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

bool parseGPCHC(std::string &message, RTKType &ins) {
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

    char Cs[4] = "";
    int gps_week;
    double gps_time;
    double baseline;
    int NSV1, NSV2, age, Warnning;
    int result = sscanf(msg.c_str(), "$GPCHC,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d%s",
            &gps_week, &gps_time, &ins.heading, &ins.pitch, &ins.roll, &ins.gyro_x, &ins.gyro_y, &ins.gyro_z,
            &ins.acc_x, &ins.acc_y, &ins.acc_z, &ins.latitude, &ins.longitude, &ins.altitude, &ins.Ve, &ins.Vn, &ins.Vu,
            &baseline, &NSV1, &NSV2, &ins.status, &age, &Warnning, Cs);
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
            found = false;
        }

        uint8_t datasum = 0;
        for (int i = 1; i < msg.size() - 3; i++) {
            datasum ^= msg[i];
        }
        if (checksum != datasum) {
            found = false;
        }
    }

    if (found) {
        // printf("gps_week: %d\n", ins.gps_week);
        // printf("gps_time: %lf\n", ins.gps_time);
        printf("heading: %lf\n", ins.heading);
        printf("latitude: %lf\n", ins.latitude);
        printf("longitude: %lf\n", ins.longitude);
    }
    return found;
}

int main(int argc, char **argv) {
  std::unique_ptr<UDPServer> udp_server(new UDPServer(9001));
  char buf[65536] = "";
  while (true) {
    int receveSize = udp_server->UDPServerReceive(buf, 65536);
    std::string message(buf, receveSize);

    RTKType ins;
    bool found = parseGPCHC(message, ins);
  }
}