#include "tests/common/test.hpp"
#include "rm_utils/io/media_reader.hpp"
#include <opencv2/core.hpp>

void check_traditional();
void check_yolo(bool run_model);

TEST_MAIN
{
    return tests::run("buff_detect_test",
                      [&]
                      {
                          cv::setNumThreads(1);
                          const auto argument = argc > 1 ? std::string(argv[1]) : "--self-test";
                          if (argument == "--help")
                          {
                              std::cout << "buff_detect_test [--self-test [traditional|yolo|geometry]]\n"
                                           "Use buff_preview config.yaml for interactive detection.\n";
                              return;
                          }
                          tests::require(argument == "--self-test", "Expected --self-test; use buff_preview for media");
                          const auto backend = argc > 2 ? std::string(argv[2]) : "all";
                          tests::require(backend == "all" || backend == "traditional" || backend == "yolo" ||
                                             backend == "geometry",
                                         "Unknown self-test backend");
                          if (backend != "yolo")
                              check_traditional();
                          if (backend != "traditional")
                              check_yolo(backend != "geometry");
                      });
}
