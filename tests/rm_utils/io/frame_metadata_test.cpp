#include "rm_utils/io/frame_metadata.hpp"
#include "tests/common/test.hpp"

#include <fstream>
#include <sstream>

int main()
{
    return tests::run(
        "frame_metadata_test",
        []
        {
            using tests::require;
            tests::TemporaryDirectory temporary;
            const auto csv = temporary.path / std::filesystem::path(u8"元数据.csv");
            std::vector<tool::RecordedFrame> expected;
            for (int mode = -1; mode <= 5; ++mode)
            {
                tool::RecordedFrame frame;
                frame.index        = expected.size();
                frame.timestamp_ns = 9007199254740993LL + frame.index * 1234567;
                frame.t_recv_ns    = frame.timestamp_ns + 321;
                frame.mode         = mode;
                if (mode >= 0)
                    frame.pose = Eigen::Quaterniond(2, .1 + mode, -.3, .7);
                expected.push_back(frame);
            }
            {
                std::ofstream out(csv);
                tool::write_metadata_header(out);
                for (const auto &frame : expected)
                    tool::write_frame_metadata(out, frame);
                require(bool(out), "Metadata write failed");
            }
            const auto actual = tool::read_metadata(csv);
            require(actual.size() == expected.size(), "Frame count changed");
            for (std::size_t i = 0; i < actual.size(); ++i)
            {
                const auto &a = actual[i], &e = expected[i];
                require(a.index == e.index && a.timestamp_ns == e.timestamp_ns && a.t_recv_ns == e.t_recv_ns &&
                            a.mode == e.mode && bool(a.pose) == bool(e.pose),
                        "Metadata fields changed");
                if (a.pose)
                    require(a.pose->angularDistance(e.pose->normalized()) < 1e-12 &&
                                std::abs(a.pose->norm() - 1) < 1e-12,
                            "Quaternion direction or normalization changed");
            }
            const std::string header = "frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode,pose_valid";
            std::ifstream saved(csv);
            std::string line;
            std::getline(saved, line);
            require(line == header, "CSV output protocol changed");
            saved.close();

            const auto read = [&](const std::string &text)
            {
                {
                    std::ofstream out(csv);
                    out << text;
                }
                return tool::read_metadata(csv);
            };
            const auto reordered =
                read("\xef\xbb\xbfq_z,mode,frame_index,q_x,timestamp_ns,q_w,t_recv_ns,q_y,diagnostic\r\n"
                     "4,5,0,2,10,1,12,3,\r\n");
            require(reordered.size() == 1 && reordered[0].mode == 5 && reordered[0].timestamp_ns == 10 &&
                        reordered[0].t_recv_ns == 12 && reordered[0].pose &&
                        reordered[0].pose->angularDistance(Eigen::Quaterniond(1, 2, 3, 4).normalized()) < 1e-12,
                    "Legacy/reordered/BOM/CRLF metadata or empty diagnostic column rejected");
            require(!read(header + "\n0,10,12,0,0,0,0,-1,0\n")[0].pose, "Missing pose became an invented orientation");

            const auto reject = [&](const std::string &text, int row, const char *reason)
            {
                try
                {
                    (void)read(text);
                }
                catch (const std::exception &error)
                {
                    const std::string message = error.what();
                    require(message.find(":" + std::to_string(row) + ": ") != std::string::npos &&
                                message.find(reason) != std::string::npos,
                            "CSV error lost line number or cause");
                    return;
                }
                throw std::runtime_error("Malformed CSV accepted");
            };
            reject(header + ",q_w\n", 1, "Duplicate CSV column");
            reject("frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z\n", 1, "Missing CSV column: mode");
            reject(header + "\n0,10,12,1,0,0,0,2\n", 2, "column count");
            reject(header + "\n0,10,12,1,0,0,0,2,\n", 2, "Expected integer");
            reject(header + "\n0,10,12,nan,0,0,0,2,1\n", 2, "Invalid number");
            reject(header + "\n0,10,12,0,0,0,0,2,1\n", 2, "Invalid quaternion");
            reject(header + "\n0,10,12,1,0,0,0,6,1\n", 2, "Invalid mode");
            reject(header + "\n0,10,12,1,0,0,0,2,2\n", 2, "Invalid mode or pose_valid");
            reject(header + "\n0,10,9,1,0,0,0,2,1\n", 2, "timestamp order");
            reject(header + "\n0,10,12,1,0,0,0,2,1\n1,10,13,1,0,0,0,2,1\n", 3, "timestamp order");
        });
}
