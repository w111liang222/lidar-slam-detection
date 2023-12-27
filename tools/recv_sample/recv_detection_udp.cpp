#include <iostream>
#include <memory>

#include "UDPServer.h"
#include "detection.pb.h"

int main(int argc, char **argv) {
    std::cout << "UDP receive detection results" << std::endl;
    std::unique_ptr<UDPServer> udp_server(new UDPServer(9000));
    while (true) {
        char recv_buf[65536] = "";
        int recveSize = udp_server->UDPServerReceive(recv_buf, 65536);
        std::cout << "received " << recveSize << " bytes" << std::endl;
        Detection det;
        bool isValid = det.ParseFromArray(recv_buf, recveSize);
        if (isValid) {
            Freespace freespace;
            freespace.ParseFromString(det.freespace());
            det.clear_freespace();
            // std::cout << freespace.DebugString() << std::endl;
            std::cout << det.DebugString() << std::endl;
        }
    }
}