#ifndef __CPP_LOGGER_H
#define __CPP_LOGGER_H

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include "spdlog/spdlog.h"

std::shared_ptr<spdlog::logger> get_perception_logger();
void set_perception_log_level(std::string level);

#define LOG_DEBUG(fmt, ...) do {                                            \
    SPDLOG_LOGGER_DEBUG(get_perception_logger(), fmt, ##__VA_ARGS__);       \
} while (0)

#define LOG_INFO(fmt, ...)  do {                                            \
    SPDLOG_LOGGER_INFO (get_perception_logger(), fmt, ##__VA_ARGS__);       \
} while (0)

#define LOG_WARN(fmt, ...)  do {                                            \
    SPDLOG_LOGGER_WARN (get_perception_logger(), fmt, ##__VA_ARGS__);       \
} while (0)

#define LOG_ERROR(fmt, ...) do {                                            \
    SPDLOG_LOGGER_ERROR(get_perception_logger(), fmt, ##__VA_ARGS__);       \
} while (0)


#endif //__CPP_LOGGER_H