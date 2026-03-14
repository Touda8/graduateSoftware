#pragma once

#include <string>
#include <string_view>
#include <mutex>
#include <fstream>
#include <filesystem>

namespace tp {

enum class LogLevel { DEBUG, INFO, WARN, ERR };

class Logger {
public:
    static Logger& instance();

    void log(LogLevel level, std::string_view message);
    void debug(std::string_view msg);
    void info(std::string_view msg);
    void warn(std::string_view msg);
    void error(std::string_view msg);

    void setMinLevel(LogLevel level);

private:
    Logger();
    ~Logger() noexcept;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::mutex mutex_;
    std::ofstream file_;
    LogLevel minLevel_ = LogLevel::DEBUG;

    static const char* levelStr(LogLevel level);
    std::string timestamp();
};

} // namespace tp
