#include "Logger.h"
#include <map>
#include <thread>

#include "spdlog/sinks/stdout_color_sinks.h"

std::shared_ptr<spdlog::logger> get_perception_logger() {
    static std::shared_ptr<spdlog::logger> logger(nullptr);
    static std::mutex mutex;

    if (logger == nullptr) {
        std::lock_guard<std::mutex> lock(mutex);
        if (logger == nullptr) {
            logger = spdlog::stdout_color_mt("perception");
            logger->set_pattern("%^%Y-%m-%d %H:%M:%S.%e[%P:%t][%s:%#] %v%$");
            auto sink = static_cast<spdlog::sinks::stdout_color_sink_mt*>(logger->sinks()[0].get());
            sink->set_color(spdlog::level::debug,    "\033[37m");
            sink->set_color(spdlog::level::warn,     "\033[33m");
            sink->set_color(spdlog::level::err,      "\033[31m");
            sink->set_color(spdlog::level::critical, "\033[31m\033[1m");
        }
    }
    return logger;
}

void set_perception_log_level(std::string level) {
    std::map<std::string, spdlog::level::level_enum> level_map = {{"DEBUG",    spdlog::level::debug},
                                                                  {"INFO",     spdlog::level::info},
                                                                  {"WARNING",  spdlog::level::warn},
                                                                  {"ERROR",    spdlog::level::err},
                                                                  {"CRITICAL", spdlog::level::critical}};
    get_perception_logger()->set_level(level_map[level]);
}