#pragma once

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace bt {

enum class LogLevel {
    Debug,
    Info,
    Warn,
    Error
};

class Logger {
public:
    static Logger& Instance();

    void SetLogFile(std::filesystem::path path);
    void Write(LogLevel level, const std::string& message);

private:
    std::mutex mutex_;
    std::filesystem::path log_file_;
    std::ofstream log_stream_;
};

const char* ToString(LogLevel level);

} // namespace bt
