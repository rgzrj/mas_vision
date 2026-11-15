#pragma once

#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace tests {
namespace fs = std::filesystem;

inline void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

template <class Work> int run(const char *name, Work work) {
    try {
        work();
        std::cout << name << " passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << name << ": " << error.what() << '\n';
        return 1;
    }
}

struct TemporaryDirectory {
    fs::path path =
        fs::temp_directory_path() /
        ("mas_buff_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    TemporaryDirectory() { require(fs::create_directory(path), "Cannot create test directory"); }
    ~TemporaryDirectory() {
        if (std::uncaught_exceptions() == 0) {
            std::error_code ignored;
            fs::remove_all(path, ignored);
        } else
            std::cerr << "Test files: " << path << '\n';
    }
};

inline fs::path argument_path(const char *text) {
    const std::string value(text);
    return fs::path(std::u8string(value.begin(), value.end()));
}

} // namespace tests

#define TEST_MAIN int main(int argc, char *argv[])
