#include "Obstacles.h"
#include "IMap.h"
#include "ITrajectory.h"
#include "Catch/catch.hpp"
#include "Eigen-3.3/Eigen/LU"
#include <cmath>
#include <cassert>
#include <vector>

namespace {

constexpr double kInnerRadius = 100.0;


class CircleMapDummy final : public IMap {
  double length() const override {
    return 2 * M_PI * kInnerRadius;
  }

  Eigen::Vector2d ToFrenetVel(const Eigen::Vector2d& cartesian,
                              const Eigen::Vector2d& vel) const override {
    const double norm = cartesian.norm();
    assert(norm > 1e-2);

    double arc_angle = std::atan2(cartesian.y(), cartesian.x());
    if (arc_angle < 0) {
      arc_angle += 2 * M_PI;
    }

    const double arc_length = norm * arc_angle;
    const double s_to_vel = norm / kInnerRadius;

    using Eigen::Vector2d;
    const Vector2d axis_d { cartesian / norm };
    const Vector2d axis_s { -s_to_vel * axis_d.y(), s_to_vel * axis_d.x() };

    Eigen::Matrix2d frenet_to_world;
    frenet_to_world <<
      axis_s.x(), axis_d.x(),
      axis_s.y(), axis_d.y();

    return frenet_to_world.inverse() * vel;
  }


  Eigen::Vector2d ToFrenet(const Eigen::Vector2d& cartesian) const override {
    const double norm = cartesian.norm();
    assert(norm > 1e-2);

    double arc_angle = std::atan2(cartesian.y(), cartesian.x());
    if (arc_angle < 0) {
      arc_angle += 2 * M_PI;
    }

    return {arc_angle * kInnerRadius, norm - kInnerRadius};
  }


  Eigen::Vector2d ToCartesian(const Eigen::Vector2d& frenet) const override {
    const double angle = frenet[0] / kInnerRadius;
    const double r = frenet[1] + kInnerRadius;
    return {std::cos(angle) * r, std::sin(angle) * r};
  }
};

}  // namespace


TEST_CASE("Obstacles unit tests", "[obstacles]") {
  CircleMapDummy circle_map_dummy;
  Obstacles obstacles(circle_map_dummy);

  obstacles.Clear();
  obstacles.Add({-kInnerRadius, 0.0}, {0.0, -10.0});
  obstacles.Add({-kInnerRadius, 0.0}, {0.0, -20.0});
  obstacles.Add({kInnerRadius + 0.5 * IMap::kLaneW, 0.0}, {0.0, 10.0});
  obstacles.Add({0.0, kInnerRadius + 0.5 * IMap::kLaneW}, {-10.0, 0.0});
  obstacles.Add({0.0, kInnerRadius + 1.5 * IMap::kLaneW}, {-12.0, 0.0});

  SECTION("Forward") {
    SECTION("Basic Cases") {
      REQUIRE(nullptr != obstacles.Forward(0.0, -1.0, 0, 10.0));
      REQUIRE(nullptr == obstacles.Forward(0.0, -1.0, 1, 10.0));
      REQUIRE(nullptr == obstacles.Forward(0.0,
                                           kInnerRadius * 3 * M_PI / 4,
                                           0,
                                           10.0));
    }

    SECTION("Wrap around s coordinate of frenet") {
      REQUIRE(nullptr != obstacles.Forward(0.0,
                                           kInnerRadius * 2 * M_PI - 1.0,
                                           0,
                                           10.0));
    }

    SECTION("Predict into the future") {
      REQUIRE(nullptr == obstacles.Forward(3.0, -1.0, 0, 10.0));
    }


    SECTION("Return nearest obstacle") {
      auto obstacle = obstacles.Forward(0.5, kInnerRadius * M_PI, 0, 10.0);
      REQUIRE(nullptr != obstacle);
      REQUIRE(Approx(10) == obstacle->velocity_mph()
              * ITrajectory::kMilesPerHour2MetersPerSecond);
    }
  }

  SECTION("min_distance") {
    SECTION("crossing the obstacle") {
      const double obstacle_x = -10.0 * ITrajectory::kDtInSeconds;
      const double obstacle_y = kInnerRadius + 0.5 * IMap::kLaneW;

      std::vector<double> path_x = {obstacle_x, obstacle_x + 0.1};
      std::vector<double> path_y = {obstacle_y, obstacle_y - 0.1};
      REQUIRE(Approx(0.0).epsilon(1e-3) ==
              obstacles.min_distance(path_x, path_y));
    }

    SECTION("distance comparison") {
      const double angle_step = 20 * ITrajectory::kDtInSeconds / kInnerRadius;
      const double angle_offset = -0.001;

      std::vector<double> far_x;
      std::vector<double> far_y;
      std::vector<double> near_x;
      std::vector<double> near_y;

      const double kFarR = kInnerRadius + 1.0 * IMap::kLaneW;
      const double kNearR = kInnerRadius + 0.4 * IMap::kLaneW;

      for (int i = 0; i < 100; ++i) {
        const double cos_a = std::cos(i * angle_step + angle_offset);
        const double sin_a = std::sin(i * angle_step + angle_offset);

        far_x.push_back(kFarR * cos_a);
        far_y.push_back(kFarR * sin_a);

        near_x.push_back(kNearR * cos_a);
        near_y.push_back(kNearR * sin_a);
      }

      REQUIRE(obstacles.min_distance(near_x, near_y) <
              obstacles.min_distance(far_x, far_y));
    }

    SECTION("angle wrap up for collisions") {
      std::vector<double> path_x;
      std::vector<double> path_y;

      const double kR = kInnerRadius + 0.5 * IMap::kLaneW;

      path_x.push_back(kR * std::cos(-0.001));
      path_y.push_back(kR * std::sin(-0.001));

      REQUIRE(1.0 > obstacles.min_distance(path_x, path_y));
    }
  }
}
