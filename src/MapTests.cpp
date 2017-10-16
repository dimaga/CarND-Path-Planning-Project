#include "Map.h"
#include "Catch/catch.hpp"
#include <sstream>
#include <cmath>

TEST_CASE("Map unit tests", "[map]") {
  std::stringstream s {
    // x y s d_x d_y
    "10.0 0.0 0.0 0.0 1.0\n"
    "10.0 10.0 10 -1.0 0.0\n"
    "0.0 10.0 20 0.0 -1.0\n"
    "0.0 0.0 30 1.0 0.0\n"
  };

  Map map(s);

  SECTION("length") {
    REQUIRE(40 == map.length());
  }

  SECTION("ClosestWaypoint") {
    REQUIRE(0 == map.ClosestWaypoint({9.0, 1.0}));
    REQUIRE(3 == map.ClosestWaypoint({-1.0, -0.5}));
  }

  SECTION("NextWaypoint") {
    REQUIRE(0 == map.NextWaypoint({1.0, 0.0}));
    REQUIRE(0 == map.NextWaypoint({7.0, 0.0}));
    REQUIRE(1 == map.NextWaypoint({10.0, 7.0}));
    REQUIRE(2 == map.NextWaypoint({9.0, 10.0}));
  }
}
