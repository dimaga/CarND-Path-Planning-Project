#include "Map.h"
#include "Catch/catch.hpp"
#include <sstream>

TEST_CASE("Map unit tests", "[map]") {
  std::stringstream s {
    // x y s d_x d_y
    "10.0 0.0 0.0 1.0 0.0\n"
    "10.0 10.0 10 0.0 1.0\n"
    "0.0 10.0 20 -1.0 0.0\n"
    "0.0 0.0 30 0.0 -1.0\n"
  };

  Map map(s);

  SECTION("length") {
    REQUIRE(40 == map.length());
  }

  SECTION("ClosestWaypoint") {
    REQUIRE(0 == map.ClosestWaypoint({9.0, 1.0}));
    REQUIRE(3 == map.ClosestWaypoint({-1.0, -0.5}));
  }
}
