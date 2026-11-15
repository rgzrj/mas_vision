#include "threads/frame_pool.hpp"
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
CameraFrame frame(int index) {
    const auto time = std::chrono::steady_clock::time_point(std::chrono::milliseconds(index));
    return {cv::Mat(1, 1, CV_32S, cv::Scalar(index)), time, time};
}
} // namespace

int main() {
    try {
        rigtorp::SPSCQueue<CameraFrame> queue(2);
        CameraFrame result;
        require(!threads::take_latest_frame(queue, result), "Empty queue produced a frame");
        require(queue.try_push(frame(1)) && queue.try_push(frame(2)), "Cannot enqueue frames");
        require(!queue.try_push(frame(3)), "Full SPSC queue unexpectedly overwrote a frame");
        require(threads::take_latest_frame(queue, result), "No buffered frame returned");
        require(result.frame.at<int>(0, 0) == 2 && result.timestamp == frame(2).timestamp,
                "Latest image and timestamp did not stay paired");
        require(!threads::take_latest_frame(queue, result), "Frame was returned twice");
        require(queue.try_push(frame(4)), "Queue did not resume after consumption");
        require(result.frame.at<int>(0, 0) == 2, "Producer changed the consumer's image");
        require(threads::take_latest_frame(queue, result) && result.frame.at<int>(0, 0) == 4,
                "Single frame was lost");
        std::cout << "Frame pool: SPSC backlog, ownership and paired timestamps passed.\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
