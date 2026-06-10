#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace bt::test {

inline void Fail(const char* expression, const char* file, int line) {
    std::cerr << file << ':' << line << ": check failed: " << expression << '\n';
    std::exit(1);
}

inline void Check(bool condition, const char* expression, const char* file, int line) {
    if (!condition) {
        Fail(expression, file, line);
    }
}

inline void CheckNear(double actual, double expected, double epsilon, const char* expression, const char* file, int line) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > epsilon) {
        std::cerr << file << ':' << line << ": check failed: " << expression
                  << " actual=" << actual << " expected=" << expected
                  << " epsilon=" << epsilon << '\n';
        std::exit(1);
    }
}

} // namespace bt::test

#define BT_CHECK(expr) ::bt::test::Check(static_cast<bool>(expr), #expr, __FILE__, __LINE__)
#define BT_CHECK_NEAR(actual, expected, epsilon) ::bt::test::CheckNear((actual), (expected), (epsilon), #actual " ~= " #expected, __FILE__, __LINE__)
