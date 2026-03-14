#include "Logger.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace tp {

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::Logger() {
    std::filesystem::create_directories("logs");
    // 生成带时间戳的日志文件名：app_YYYYMMDD_HHMMSS.log
    using namespace std::chrono;
    auto now = system_clock::now();
    auto tt  = system_clock::to_time_t(now);
    std::tm buf{};
    localtime_s(&buf, &tt);
    std::ostringstream fname;
    fname << "logs/app_"
          << std::put_time(&buf, "%Y%m%d_%H%M%S")
          << ".log";
    file_.open(fname.str(), std::ios::app);
}

Logger::~Logger() noexcept {
    if (file_.is_open()) file_.close();
}

// log / helpers
void Logger::log(LogLevel level, std::string_view message) {
    if (level < minLevel_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    auto line = "[" + timestamp() + "][" + levelStr(level) + "] "
                + std::string(message) + "\n";
    if (file_.is_open()) file_ << line << std::flush;
    std::cerr << line;
}

void Logger::debug(std::string_view msg) { log(LogLevel::DEBUG, msg); }
void Logger::info(std::string_view msg)  { log(LogLevel::INFO,  msg); }
void Logger::warn(std::string_view msg)  { log(LogLevel::WARN,  msg); }
void Logger::error(std::string_view msg) { log(LogLevel::ERR, msg); }

void Logger::setMinLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    minLevel_ = level;
}

const char* Logger::levelStr(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERR: return "ERROR";
    }
    return "UNKNOWN";
}

std::string Logger::timestamp() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto tt  = system_clock::to_time_t(now);
    auto ms  = duration_cast<milliseconds>(
                   now.time_since_epoch()) % 1000;
    std::tm buf{};
    localtime_s(&buf, &tt);
    std::ostringstream oss;
    oss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

} // namespace tp
