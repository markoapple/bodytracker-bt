#pragma once

#include <cassert>
#include <string>
#include <utility>
#include <variant>

namespace bt {

enum class StatusCode {
    Ok = 0,
    InvalidArgument,
    FailedPrecondition,
    ValidationError,
    DeviceUnavailable,
    Unsupported,
    InternalError
};

struct Status {
    StatusCode code = StatusCode::Ok;
    std::string message;

    [[nodiscard]] bool ok() const noexcept {
        return code == StatusCode::Ok;
    }

    static Status OK() {
        return {};
    }

    static Status Error(StatusCode c, std::string msg) {
        assert(c != StatusCode::Ok);
        if (c == StatusCode::Ok) {
            c = StatusCode::InternalError;
        }
        return Status{c, std::move(msg)};
    }
};

template <typename T>
class Result {
public:
    Result(const T& value) : storage_(value) {}
    Result(T&& value) : storage_(std::move(value)) {}
    Result(Status status) : storage_(NormalizeErrorStatus(std::move(status))) {}

    [[nodiscard]] bool ok() const {
        return std::holds_alternative<T>(storage_);
    }

    [[nodiscard]] const Status& status() const {
        if (const auto* s = std::get_if<Status>(&storage_)) {
            return *s;
        }
        static const Status ok_status = Status::OK();
        return ok_status;
    }

    [[nodiscard]] const T& value() const {
        return std::get<T>(storage_);
    }

    [[nodiscard]] T& value() {
        return std::get<T>(storage_);
    }

private:
    static Status NormalizeErrorStatus(Status status) {
        assert(!status.ok());
        if (status.ok()) {
            return Status::Error(StatusCode::InternalError, "Result<T> cannot store an OK status as an error");
        }
        return status;
    }

    std::variant<T, Status> storage_;
};

} // namespace bt
