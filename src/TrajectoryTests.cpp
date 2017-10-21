#include "Trajectory.h"
#include "Catch/catch.hpp"
#include <cmath>

TEST_CASE("Trajectory unit tests", "[trajectory]") {
  Trajectory trajectory;

  SECTION("Straight From Steady Pos") {
    trajectory.set_car_pos({10.0, -20.0});
    trajectory.set_car_yaw_rad(M_PI / 2);
    trajectory.set_car_speed(0.0);
    trajectory.set_previous_path({}, {});

    trajectory.Trace(10.0, {{10.0, -10.0}, {10.0, 0.0}, {10.0, 10.0}});

    const auto& path_x = trajectory.path_x();
    const auto& path_y = trajectory.path_y();

    REQUIRE(path_x.size() == path_y.size());
    REQUIRE(5 <= path_x.size());
  }
}
