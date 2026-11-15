#include "auto_buff/buff_detect/detection.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "auto_buff/buff_shoot/rune_shooter.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include "tests/common/performance.hpp"
#include "tests/common/test.hpp"

TEST_MAIN
{
    return tests::run(
        "buff_profile",
        [&]
        {
            if (argc == 1 || tool::utf8(tests::argument_path(argv[1])) == "--help")
            {
                std::cout << "buff_profile config/auto_buff.yaml image.png [frames=600]\n"
                             "Repeats one image through production detect/track/shoot; no windows or serial.\n";
                return;
            }
            if (argc < 3 || argc > 4)
                throw std::runtime_error("Provide config, image and optional frame count");
            const auto config = std::filesystem::absolute(tests::argument_path(argv[1]));
            const int frames  = argc == 4 ? std::stoi(argv[3]) : 600;
            if (frames <= 0)
                throw std::runtime_error("Frame count must be positive");
            tool::MediaReader reader(tests::argument_path(argv[2]), "image");
            tool::Frame frame;
            if (!reader.next(frame))
                throw std::runtime_error("Cannot read profiling image");
            auto detector = auto_buff::make_detector(config);
            auto_buff::RmcsTracker tracker(config, config.parent_path() / "hikcamera.yaml");
            auto_buff::RuneShooter shooter(config);
            detector->disable_debug();
            tracker.disable_debug();
            shooter.disable_debug();
            const auto output = tool::create_session_directory("output/profiling");
            tests::PerformanceLog log(output / "performance.csv");
            const auto began = rm_utils::Clock::now();
            for (int i = 0; i < frames; ++i)
            {
                const auto start       = rm_utils::Clock::now();
                const double seconds   = std::chrono::duration<double>(start - began).count();
                const auto observation = detector->detect(frame.image, seconds);
                const auto detected    = rm_utils::Clock::now();
                const auto tracking    = tracker.update(observation, seconds, frame.image);
                const auto tracked     = rm_utils::Clock::now();
                if (tracking.initialized)
                    shooter.update({tracking.timestamp});
                shooter.shoot(tracking, frame.image);
                const auto finished = rm_utils::Clock::now();
                log.write(i, {rm_utils::elapsed_ms(start, detected), rm_utils::elapsed_ms(detected, tracked),
                              rm_utils::elapsed_ms(tracked, finished), rm_utils::elapsed_ms(start, finished)});
            }
            std::cout << "Profiling output: " << tool::utf8(output) << '\n';
        });
}
