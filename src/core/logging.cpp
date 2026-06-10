#include "core/logging.h"

#include <fstream>
#include <iostream>
#include <utility>

namespace bt {

Logger& Logger::Instance() {
    static Logger logger;
    return logger;
}

void Logger::SetLogFile(std::filesystem::path path) {
    std::scoped_lock lock(mutex_);
    if (log_stream_.is_open()) {
        log_stream_.flush();
        log_stream_.close();
    }
    log_file_ = std::move(path);
    if (!log_file_.empty()) {
        log_stream_.open(log_file_, std::ios::app);
    }
}

void Logger::Write(LogLevel level, const std::string& message) {
    std::scoped_lock lock(mutex_);
    const std::string line = std::string("[") + ToString(level) + "] " + message;
    std::cout << line << '\n';
    if (log_stream_) {
        log_stream_ << line << '\n';
        log_stream_.flush();
    }
}

const char* ToString(LogLevel level) {
    switch (level) {
    case LogLevel::Debug: return "debug";
    case LogLevel::Info: return "info";
    case LogLevel::Warn: return "warn";
    case LogLevel::Error: return "error";
    default: return "unknown";
    }
}

} // namespace bt
