#include "Trajectory.h"
#include "Catch/catch.hpp"
#include <cmath>

TEST_CASE("Trajectory unit tests", "[trajectory]") {
  Trajectory trajectory;

  SECTION("Straight From Steady Pos") {
    trajectory.set_car_pos({15.0, -20.0});
    trajectory.set_car_yaw_rad(M_PI / 2);
    trajectory.set_car_speed_mph(0.0);
    trajectory.set_previous_path({}, {});
    trajectory.Trace(5.0, {{15.0, 10.0}});

    {
      const auto& path_x = trajectory.path_x();
      const auto& path_y = trajectory.path_y();

      REQUIRE(path_x.size() == path_y.size());

      const std::size_t path_sz = path_x.size();
      REQUIRE(3 <= path_sz);

      REQUIRE(Approx(15.0) == path_x.back());

      const double dx = path_x.at(path_sz - 1) - path_x.at(path_sz - 2);
      const double dy = path_y.at(path_sz - 1) - path_y.at(path_sz - 2);

      const double vx = dx / ITrajectory::kDtInSeconds;
      const double vy = dy / ITrajectory::kDtInSeconds;
      REQUIRE(Approx(0.0) == vx);
      REQUIRE(Approx(5.0 * ITrajectory::kMilesPerHour2MetersPerSecond) == vy);
    }

    SECTION("Then Circular with Static Velocity") {
      const double r = 7.0;
      const double speed_mph = 2.0;

      trajectory.set_car_pos({0.0, -r});
      trajectory.set_car_yaw_rad(0.0);
      trajectory.set_car_speed_mph(speed_mph);

      using std::cos;
      using std::sin;

      const double v = speed_mph * ITrajectory::kMilesPerHour2MetersPerSecond;
      const double lin_step = v * ITrajectory::kDtInSeconds;
      const double ang_step = lin_step / r;

      trajectory.set_previous_path({r * cos(-2 * ang_step - M_PI / 2),
                                    r * cos(-ang_step - M_PI / 2)},
                                   {r * sin(-2 * ang_step - M_PI / 2),
                                    r * sin(-ang_step - M_PI / 2)});

      trajectory.Trace(speed_mph,
                      {{r * cos(-M_PI / 4), r * sin(-M_PI / 4)},
                       {r * cos(-M_PI / 8), r * sin(-M_PI / 8)},
                       {r * cos(0), r * sin(0)}});

      const auto& path_x = trajectory.path_x();
      const auto& path_y = trajectory.path_y();
      REQUIRE(path_x.size() == path_y.size());

      const std::size_t path_sz = path_x.size();
      REQUIRE(5 <= path_sz);

      for (std::size_t i = 1, sz = path_x.size(); i < sz; ++i) {
        const std::size_t prev_i = i - 1;

        const double x = path_x.at(i);
        const double y = path_y.at(i);

        const double dx = x - path_x.at(prev_i);
        const double dy = y - path_y.at(prev_i);

        const double vx = dx / ITrajectory::kDtInSeconds;
        const double vy = dy / ITrajectory::kDtInSeconds;

        CAPTURE(i);
        CAPTURE(x);
        CAPTURE(y);
        CAPTURE(vx);
        CAPTURE(vy);
        CAPTURE(dx);
        CAPTURE(dy);
        REQUIRE(Approx(v).epsilon(1e-2) == std::sqrt(vx * vx + vy * vy));

        CAPTURE(i);
        CAPTURE(x);
        CAPTURE(y);
        CAPTURE(vx);
        CAPTURE(vy);
        CAPTURE(dx);
        CAPTURE(dy);
        REQUIRE(Approx(r).epsilon(1e-2) == std::sqrt(x * x + y * y));
      }
    }
  }
}
