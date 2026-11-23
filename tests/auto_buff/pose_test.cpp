#include "auto_buff/buff_track/pose/rune_pose.hpp"
#include "auto_buff/buff_track/pose/rune_camera.hpp"
#include "auto_buff/buff_track/pose/rune_projection.hpp"
#include "auto_buff/buff_track/pose/big_activated.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "auto_buff/buff_detect/yolo/observation/pose_output.hpp"
#include "auto_buff/buff_detect/yolo/pixels/pixel_geometry.hpp"
#include "rm_utils/algorithm/math_tools.hpp"
#include "rm_utils/algorithm/orientation.hpp"
#include "tests/common/synthetic_rune.hpp"
#include "tests/common/test.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>

static void check_big_activated(const std::filesystem::path &camera_yaml)
{
    using namespace auto_buff;
    using tests::require;
    for (int scenario = 0; scenario < 3; ++scenario)
    {
        cv::Mat intrinsic(3, 3, CV_64F, const_cast<double *>(tests::intrinsics.data()));
        const std::array<double, 5> coefficients = scenario == 0 ? std::array<double, 5>{} :
            std::array<double, 5>{.08, -.02, .002, -.001, 0};
        cv::Mat distortion(1, 5, CV_64F, const_cast<double *>(coefficients.data()));
        Eigen::Isometry3d camera_transform = Eigen::Isometry3d::Identity();
        const Eigen::Quaterniond camera_rotation = scenario == 0 ? Eigen::Quaterniond::Identity() :
            Eigen::Quaterniond(Eigen::AngleAxisd(.04, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(-.03, Eigen::Vector3d::UnitY()));
        const Eigen::Vector3d camera_origin = scenario == 0 ? Eigen::Vector3d::Zero() : Eigen::Vector3d(.1, -.03, .04);
        camera_transform.linear() = camera_rotation.toRotationMatrix();
        camera_transform.translation() = camera_origin;
        const Eigen::Vector3d center(5, 0, 0);
        const auto face = Eigen::AngleAxisd(scenario == 0 ? 0. : .18, Eigen::Vector3d::UnitZ());
        const double phase = scenario == 2 ? -1.2 : .3;
        const auto project = [&](const Eigen::Vector3d &local, double angle) {
            const Eigen::Vector3d world = center + face * (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()) * local);
            const Eigen::Vector3d position = camera_rotation.conjugate() * (world - camera_origin);
            std::vector<cv::Point2d> pixels;
            cv::projectPoints(std::vector<cv::Point3d>{{-position.y(), -position.z(), position.x()}},
                              cv::Vec3d{}, cv::Vec3d{}, intrinsic, distortion, pixels);
            return pixels[0];
        };
        const auto icon = project({-.1, 0, 0}, 0);
        const std::vector<auto_buff::RuneIcon> icons{{cv::Point2d(icon), 1}};
        auto_buff::RuneBullseye pending{};
        pending.center = cv::Point2d(project({0, 0, .7}, phase));
        constexpr std::array<std::array<double, 3>, 4> tips{{{{0, 0, .85}}, {{0, .15, .7}},
                                                            {{0, 0, .55}}, {{0, -.15, .7}}}};
        for (int i = 0; i < 4; ++i)
            pending.corners[i] = cv::Point2d(project(Eigen::Vector3d(tips[i].data()), phase));
        const std::vector<auto_buff::RuneBullseye> boards{pending};
        // Render a filled physical silhouette independently of the production model point array.
        const auto render_arm = [&](double angle, cv::Mat &image) {
            LightArm arm;
            arm.color = auto_buff::EnemyColor::RED;
            arm.pose.state = 2;
            arm.pose.score = .95;
            arm.pose.valid.fill(true);
            arm.pose.point_scores.fill(.99);
            arm.pose.points = {{project({-.0111, .16, .32}, angle),
                                project({-.0111, .06, .164}, angle), icon,
                                project({-.0111, -.06, .164}, angle),
                                project({-.0111, -.16, .32}, angle)}};
            const std::array<cv::Point2d, 12> silhouette{{
                {-.068, .15394}, {.068, .15394}, {.182776, .311916}, {.158635, .312715},
                {.057809, .17394}, {.03, .17394}, {.03, .306107}, {-.03, .306107},
                {-.03, .17394}, {-.057809, .17394}, {-.158635, .312715}, {-.182776, .311916}}};
            std::vector<cv::Point> polygon;
            for (auto p : silhouette)
                polygon.emplace_back(project({-.0111, -p.x, p.y}, angle));
            cv::fillPoly(image, std::vector<std::vector<cv::Point>>{polygon}, {0, 0, 255});
            extract_yolo_light_arm_contour(image, arm);
            return arm;
        };
        // Deliberately not a 72-degree multiple in scenario 1: fitting must follow the measured rod.
        const double arm_phase = phase + (scenario == 1 ? 1.12 : 2 * std::numbers::pi / 5);
        cv::Mat image(540, 960, CV_8UC3, cv::Scalar());
        const auto arm = render_arm(arm_phase, image);
        require(arm.contour.size() >= 8, "Physical lamp silhouette failed contour extraction");
        const std::vector<LightArm> arms{arm};
        const auto derived = reconstruct_big_activated(arms, intrinsic, distortion, camera_transform, {}, icons, boards);
        if (derived.empty()) std::cerr << "Contour solver failed scenario " << scenario << '\n';
        require(derived.size() == 1 && cv::norm(derived[0].center - project({0, 0, .7}, arm_phase)) < 2.,
                "Lamp-contour fitting failed the independently projected board center");
        require(!derived[0].features_valid && derived[0].appearance == Appearance::big_activated &&
                    derived[0].geometry_support >= .8 && derived[0].geometry_error_px < 2 &&
                    std::ranges::all_of(derived[0].corners, [](auto p) { return p == cv::Point2d{}; }),
                "Derived big-activated center manufactured endpoints or became a ring");
        require(reconstruct_big_activated(arms, intrinsic, distortion, camera_transform, {}, icons, {}).empty() &&
                    reconstruct_big_activated(arms, intrinsic, distortion, camera_transform, {}, {}, boards).empty(),
                "Light-arm geometry invented a plane without same-frame pending/R evidence");
        require(reconstruct_big_activated(std::vector<LightArm>{arm, arm}, intrinsic, distortion, camera_transform, {}, icons, boards).size() == 1,
                "Duplicate light arms generated duplicate board measurements");
        auto direct = boards;
        auto_buff::RuneBullseye active{};
        active.center = cv::Point2d(derived[0].center);
        active.active = true;
        direct.push_back(active);
        require(reconstruct_big_activated(arms, intrinsic, distortion, camera_transform, {}, icons, direct).empty(),
                "Derived center duplicated a direct board measurement");
        for (int bad = 0; bad < 5; ++bad)
        {
            auto broken = arm;
            if (bad == 0) broken.contour.clear();
            if (bad == 1) broken.pose.valid[2] = false;
            if (bad == 2) broken.pose.points[2] += cv::Point2d(100, 100);
            if (bad == 3) broken.pose.points.fill(icon);
            if (bad == 4) broken.contour.assign(20, cv::Point(icon));
            require(reconstruct_big_activated(std::vector<LightArm>{broken}, intrinsic, distortion, camera_transform, {}, icons, boards).empty(),
                    "Missing, collapsed or unrelated lamp observations were accepted");
        }
        if (scenario == 0)
        {
            auto blank_arm = make_pose_detection({arm.pose}, cv::Mat::zeros(image.size(), image.type()), auto_buff::EnemyColor::RED, false);
            require(blank_arm.light_arms.size() == 1 && blank_arm.light_arms[0].contour.empty(),
                    "Blank ROI acquired a measured lamp contour");
            auto occluded_image = image.clone();
            const auto lamp_box = cv::boundingRect(arm.contour);
            cv::rectangle(occluded_image, {lamp_box.x, lamp_box.y, lamp_box.width / 2, lamp_box.height}, {}, -1);
            const auto occluded = make_pose_detection({arm.pose}, occluded_image, auto_buff::EnemyColor::RED, false);
            require(occluded.light_arms.empty() ||
                        reconstruct_big_activated(occluded.light_arms, intrinsic, distortion, camera_transform, {}, icons, boards).empty(),
                    "Heavily occluded silhouette produced a reliable-looking board center");
            cv::Mat multiple_image(image.size(), image.type(), cv::Scalar());
            const auto first_arm = render_arm(arm_phase, multiple_image);
            const auto second_arm = render_arm(arm_phase + 2 * std::numbers::pi / 5, multiple_image);
            require(reconstruct_big_activated(std::vector<LightArm>{first_arm, second_arm}, intrinsic, distortion, camera_transform, {}, icons, boards).size() == 2,
                    "Two measured rods were merged or a missing third rod was invented");
            auto jittered = arm;
            const auto radial = derived[0].center - icon;
            const auto tangent = cv::Point2d(-radial.y, radial.x) * (.4 / cv::norm(radial));
            for (int i : {0, 1, 3, 4}) jittered.pose.points[i] += tangent;
            const auto stable = reconstruct_big_activated(std::vector<LightArm>{jittered}, intrinsic, distortion, camera_transform, {}, icons, boards);
            require(stable.size() == 1 && cv::norm(stable[0].center - derived[0].center) < .2,
                    "NN point jitter displaced an unchanged measured contour");
            cv::Mat moved_image(image.size(), image.type(), cv::Scalar());
            auto moved = render_arm(arm_phase + .04, moved_image);
            const auto shifted = reconstruct_big_activated(std::vector<LightArm>{moved}, intrinsic, distortion, camera_transform, {}, icons, boards);
            require(shifted.size() == 1 && cv::norm(shifted[0].center - derived[0].center) > 3 &&
                        cv::norm(shifted[0].center - project({0, 0, .7}, arm_phase + .04)) < 2,
                    "Derived center ignored fresh contour motion or snapped to a slot");
            auto partial = arm;
            partial.pose.valid[0] = false;
            partial.pose.points[0].x = std::numeric_limits<double>::quiet_NaN();
            require(reconstruct_big_activated(std::vector<LightArm>{partial}, intrinsic, distortion, camera_transform, {}, icons, boards).size() == 1,
                    "One missing NN corner discarded a complete measured lamp contour");
            PoseDetection seed;
            seed.valid.fill(true);
            seed.score = .99;
            seed.points = {{pending.corners[0], pending.corners[1], icon,
                            pending.corners[3], pending.corners[2]}};
            for (int i : {0, 1, 3, 4}) cv::circle(image, seed.points[i], 3, {0, 0, 255}, -1);
            for (bool refine : {false, true})
            {
                auto detection = make_pose_detection({seed, arm.pose}, image, auto_buff::EnemyColor::RED, refine);
                detection.raw_poses = {seed, arm.pose};
                require(detection.light_arms[0].pose.points == arm.pose.points,
                        "Contour extraction overwrote raw NN keypoints");
                filter_rune_observations(detection, RuneMode::big, auto_buff::EnemyColor::RED);
                RmcsTracker tracker(camera_yaml, YAML::Node{});
                const auto initialized = tracker.update(detection, 0, image);
                require(initialized.initialized && initialized.geometry_boards.size() == 1,
                        "Three-class detect/pose/RMCS integration failed under the refinement switch");
                const auto updated = tracker.update(detection, .02, image);
                require(std::count(updated.state.inactive.begin(), updated.state.inactive.end(), true) == 1,
                        "Derived active center created an extra pending blade");
                require(updated.initialized && std::ranges::any_of(updated.matched, [&](const auto &match) {
                            return match.feature_id > 0 && cv::norm(cv::Point2d(match.point.x, match.point.y) -
                                                                   initialized.geometry_boards[0].center) < 1.;
                        }), "Original RMCS correct() did not consume the derived active center");
                const auto only_arm = make_pose_detection({arm.pose}, image, auto_buff::EnemyColor::RED, refine);
                RmcsTracker uninitialized(camera_yaml, YAML::Node{});
                require(!uninitialized.update(only_arm, 0, image).initialized,
                        "An activated arm initialized RMCS without a pending board");
            }
        }
        std::cout << "lamp scenario=" << scenario << " error_px="
                  << cv::norm(derived[0].center - project({0, 0, .7}, arm_phase))
                  << " contour_rms=" << derived[0].geometry_error_px << '\n';
    }
    std::cout << "Big activated: measured contours, moving camera, rejection and original RMCS integration passed.\n";
}

int main()
{
    return tests::run("buff_pose_test", [] {
        using tests::require;
        const double pi = std::numbers::pi;
        require(std::abs(rm_utils::limit_rad(-pi) - pi) < 1e-12 &&
                    std::abs(rm_utils::limit_rad(7 * pi / 2) + pi / 2) < 1e-12 &&
                    std::abs(rm_utils::rad2deg(rm_utils::deg2rad(123.4)) - 123.4) < 1e-12,
                "MAS angle normalization or degree conversion contract changed");
        const Eigen::Quaterniond rotation(Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitZ()));
        require((rm_utils::eulers(rotation, 2, 1, 0) - Eigen::Vector3d(pi / 2, 0, 0)).norm() < 1e-12,
                "MAS Euler angle order changed");
        const Eigen::Vector3d xyz(3, 4, 2);
        require((rm_utils::ypd2xyz(rm_utils::xyz2ypd(xyz)) - xyz).norm() < 1e-12,
                "MAS spherical coordinate round trip failed");
        const auto halfway = rm_utils::util::interpolate_orientation(Eigen::Quaterniond::Identity(), rotation, .5);
        require(std::abs(Eigen::AngleAxisd(halfway).angle() - pi / 4) < 1e-12,
                "Retained public orientation interpolation changed");
        // 直接使用 pose 的加载入口，再以已知几何检验 PnP，避免只测 YAML 能否解析。
        tests::TemporaryDirectory directory;
        const auto camera_yaml = directory.path / "hikcamera.yaml";
        std::ofstream(camera_yaml)
            << "calibration: {camera_matrix: [900,0,480,0,900,270,0,0,1], "
               "distort_coeffs: [0,0,0,0,0]}\n";
        auto_buff::RuneCamera pose_camera(camera_yaml);
        check_big_activated(camera_yaml);
        const auto &intrinsic = pose_camera.intrinsic();
        const auto &distortion = pose_camera.distortion();
        const auto &camera_transform = pose_camera.transform();
        auto observation = tests::synthetic(0, tests::intrinsics);
        // 按新模型的 top/left/R/right/bottom 编码，再走生产解码与公共观测转换。
        // 验证五点中的 R 不会被误当成板心，RMCS 的 PnP 无需分辨权重来源。
        const auto &pending = observation.boards.front();
        const std::array<cv::Point2d, 5> points{pending.corners[0], pending.corners[1],
                                                observation.markers.front().center, pending.corners[3],
                                                pending.corners[2]};
        std::vector<float> raw(18);
        raw[0] = .99f;
        cv::Mat image(540, 960, CV_8UC3, cv::Scalar());
        for (int i = 0; i < 5; ++i)
        {
            raw[3 + 3 * i] = float(points[i].x);
            raw[4 + 3 * i] = float(points[i].y);
            raw[5 + 3 * i] = .99f;
            if (i != 2)
                cv::circle(image, points[i], 4, {0, 0, 255}, -1);
        }
        const auto decoded = auto_buff::decode_pose_output(raw, 1, image.size(), 1, {}, .25, .8, 30);
        observation        = auto_buff::make_pose_detection(decoded, image, auto_buff::EnemyColor::RED, false);
        require(observation.markers.size() == 1 && observation.boards.size() == 1 &&
                    observation.boards.front().features_valid,
                "SZU five-point protocol failed before PnP");
        // 指定红色检测后，无色 R 应配合红色待击板建立模型。
        auto_buff::RmcsTracker tracker(camera_yaml, YAML::Node{});
        const auto tracked = tracker.update(observation, 0, image);
        require(tracked.initialized && tracked.color == auto_buff::EnemyColor::RED,
                "YOLO red selection prevented RMCS initialization with a real red board");
        std::vector<auto_buff::RuneIcon> icons;
        std::vector<auto_buff::RuneBullseye> boards;
        for (const auto &marker : observation.markers)
            icons.push_back({cv::Point2d(marker.center), marker.score});
        for (const auto &board : observation.boards)
        {
            auto_buff::RuneBullseye target{};
            target.center = cv::Point2d(board.center);
            for (int i = 0; i < 4; ++i)
                target.corners[i] = cv::Point2d(board.corners[i]);
            target.active = board.appearance == auto_buff::Appearance::single_ring;
            target.score  = 1;
            boards.push_back(target);
        }
        const auto pose = auto_buff::solve_rune_pose({}, intrinsic, distortion, camera_transform, icons, boards);
        require(pose && (pose->center_odom - Eigen::Vector3d(5, 0, 0)).norm() < .05, "PnP failed known 3D geometry");
        // 使用生产代码的投影检验解，而非只检查 PnP 返回了一个值。
        auto_buff::RuneObservable projection;
        projection.intrinsic = intrinsic;
        projection.distortion = distortion;
        projection.camera_to_world = camera_transform;
        auto_buff::RuneStateVector state;
        state << pose->center_odom, .8, pose->rotation_angle, pose->face_yaw_odom;
        projection.update(state);
        const auto r       = icons.front().center;
        const double error = (projection.projected[0] - Eigen::Vector2d(r.x, r.y)).norm();
        require(error < 1, "PnP R reprojection error exceeds one pixel");
        icons.insert(icons.begin(), auto_buff::RuneIcon{cv::Point2d(80, 60), 2});
        const auto multiple = auto_buff::solve_rune_pose({}, intrinsic, distortion, camera_transform, icons, boards);
        require(multiple && (multiple->center_odom - Eigen::Vector3d(5, 0, 0)).norm() < .05,
                "Higher-scored false R displaced the geometrically valid candidate");
        require(!auto_buff::solve_rune_pose({}, intrinsic, distortion, camera_transform, {}, boards), "Pose initialized without R");
        auto active = boards;
        for (auto &board : active)
            board.active = true;
        require(!auto_buff::solve_rune_pose({}, intrinsic, distortion, camera_transform, icons, active), "Hit rings initialized PnP");
        boards.front().corners.fill({});
        require(!auto_buff::solve_rune_pose({}, intrinsic, distortion, camera_transform, icons, boards), "Collapsed endpoints initialized PnP");
        std::cout << "center_m=" << pose->center_odom.transpose() << " angle_rad=" << pose->rotation_angle
                  << " R_reprojection_px=" << error << '\n';
    });
}
