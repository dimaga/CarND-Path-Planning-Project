#include "Obstacles.h"
#include "IMap.h"
#include "ITrajectory.h"
#include "Catch/catch.hpp"
#include "Eigen-3.3/Eigen/LU"
#include <cmath>
#include <cassert>

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

  SECTION("Forward") {
    obstacles.Clear();
    obstacles.Add({-kInnerRadius, 0.0}, {0.0, -10.0});
    obstacles.Add({-kInnerRadius, 0.0}, {0.0, -20.0});
    obstacles.Add({kInnerRadius + 0.5 * IMap::kLaneW, 0.0}, {0.0, 10.0});
    obstacles.Add({0.0, kInnerRadius + 0.5 * IMap::kLaneW}, {-10.0, 0.0});
    obstacles.Add({0.0, kInnerRadius + 1.5 * IMap::kLaneW}, {-12.0, 0.0});

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
}
