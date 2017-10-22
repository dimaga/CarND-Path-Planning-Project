#include "Trajectory.h"
#include "Catch/catch.hpp"
#include <cmath>

TEST_CASE("Trajectory unit tests", "[trajectory]") {
  Trajectory trajectory;

  SECTION("Straight From Steady Pos") {
    const double initial_pos_y = -20.0;
    const double final_pos_y = 10.0;
    const double pos_x = 15.0;
    const double initial_speed_mph = 0.0;
    const double final_speed_mph = 5.0;

    trajectory.set_car_pos({pos_x, initial_pos_y});
    trajectory.set_car_yaw_rad(M_PI / 2);
    trajectory.set_car_speed_mph(initial_speed_mph);
    trajectory.set_previous_path({}, {});
    trajectory.Trace(final_speed_mph, {{pos_x, final_pos_y}});

    {
      const auto& path_x = trajectory.path_x();
      const auto& path_y = trajectory.path_y();

      REQUIRE(path_x.size() == path_y.size());

      const std::size_t path_sz = path_x.size();
      REQUIRE(3 <= path_sz);

      const double total_time =
        (final_pos_y - initial_pos_y) /
        (0.5 * (initial_speed_mph + final_speed_mph) *
         ITrajectory::kMilesPerHour2MetersPerSecond);

      for (std::size_t i = 0; i < path_sz; ++i) {
        CAPTURE(i);
        REQUIRE(Approx(pos_x) == path_x.at(i));

        if (i > 0 && i < path_sz - 1) {
          const double y0 = path_y.at(i - 1);
          const double y1 = path_y.at(i);
          const double y2 = path_y.at(i + 1);

          const double v1 = (y2 - y0) / (2 * ITrajectory::kDtInSeconds);
          const double mph = v1 / ITrajectory::kMilesPerHour2MetersPerSecond;

          const double expected_mph =
              initial_speed_mph
              + (final_speed_mph - initial_speed_mph)
              * i * ITrajectory::kDtInSeconds / total_time;

          CAPTURE(i);
          CAPTURE(y0);
          CAPTURE(y1);
          CAPTURE(y2);
          CAPTURE(expected_mph * ITrajectory::kMilesPerHour2MetersPerSecond);
          REQUIRE(Approx(expected_mph).epsilon(1e-2) == mph);
        }
      }
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
