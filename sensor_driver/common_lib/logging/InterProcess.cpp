#include "InterProcess.h"

#include <thread>
#include <mutex>

#include <zcm/zcm-cpp.hpp>
#include "std_msgs/Byte.hpp"
#include "std_msgs/Float.hpp"
#include "std_msgs/Int32.hpp"
#include "std_msgs/Int64.hpp"
#include "std_msgs/String.hpp"

#define ZCM_URL "ipc://zcm_core"

std::shared_ptr<zcm::ZCM> get_core() {
    static std::shared_ptr<zcm::ZCM> core(nullptr);
    static std::mutex mutex;

    if (core == nullptr) {
        std::lock_guard<std::mutex> lock(mutex);
        if (core == nullptr) {
            core = std::shared_ptr<zcm::ZCM>(new zcm::ZCM(ZCM_URL));
        }
    }
    return core;
}

std::shared_ptr<zcm::ZCM> create_core() {
    return std::shared_ptr<zcm::ZCM>(new zcm::ZCM(ZCM_URL));
}

static bool core_enable = false;

void set_core_enable(bool enable) {
    core_enable = enable;
}

bool get_core_enable() {
    return core_enable;
}

template <typename T, class Msg>
void publish_std_msgs(const std::string &topic, const T &data, Msg *msg) {
    msg->data = data;
    get_core()->publish(topic.c_str(), msg);
}

void publish_msgs(const std::string &topic, const char &data) {
    std_msgs::Byte msg;
    publish_std_msgs(topic, data, &msg);
}
void publish_msgs(const std::string &topic, const float &data) {
    std_msgs::Float msg;
    publish_std_msgs(topic, data, &msg);
}
void publish_msgs(const std::string &topic, const double &data) {
    std_msgs::Float msg;
    publish_std_msgs(topic, data, &msg);
}
void publish_msgs(const std::string &topic, const int32_t &data) {
    std_msgs::Int32 msg;
    publish_std_msgs(topic, data, &msg);
}
void publish_msgs(const std::string &topic, const int64_t &data) {
    std_msgs::Int64 msg;
    publish_std_msgs(topic, data, &msg);
}
void publish_msgs(const std::string &topic, const std::string &data) {
    std_msgs::String msg;
    publish_std_msgs(topic, data, &msg);
}