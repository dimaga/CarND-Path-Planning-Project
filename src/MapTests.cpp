#include "Map.h"
#include "Catch/catch.hpp"
#include <sstream>
#include <cmath>

TEST_CASE("Map unit tests", "[map]") {
  std::stringstream s {
    // x y s nx ny
    "-20.0 0.0 0.0 -0.5547 -0.83205\n"
    "-5.0 -10.0 18.027756 0.0 -1.0\n"
    "5.0 -10.0 28.027756 0.5547 -0.83205\n"
    "20.0 0.0 46.055512 0.5547 0.83205\n"
    "5.0 10.0 64.083269 0.0 1.0\n"
    "-5.0 10.0 74.083269 -0.5547 0.83205"
  };

  Map map(s);

  SECTION("length") {
    REQUIRE(Approx(92.1110255) == map.length());
  }

  SECTION("ClosestWaypoint") {
    REQUIRE(1 == map.ClosestWaypoint({-4.0, -11.0}));
    REQUIRE(4 == map.ClosestWaypoint({6.0, 9.5}));
  }

  SECTION("NextWaypoint") {
    REQUIRE(1 == map.NextWaypoint({-6.0, -10.0}));
    REQUIRE(2 == map.NextWaypoint({-4.0, -10.0}));
    REQUIRE(5 == map.NextWaypoint({0.0, 10.0}));
    REQUIRE(5 == map.NextWaypoint({-4.0, 10.0}));
    REQUIRE(0 == map.NextWaypoint({-6.0, 9.0}));
  }

  using Eigen::Vector2d;

  SECTION("ToFrenet()") {
    SECTION("Point 1") {
      const Vector2d frenet = map.ToFrenet({-5.0, -10.0});
      REQUIRE(Approx(18.027756) == frenet[0]);
      REQUIRE(Approx(0.0) == frenet[1]);
    }

    SECTION("Point 4") {
      const Vector2d frenet = map.ToFrenet({5.0, 10.0});
      REQUIRE(Approx(64.083269) == frenet[0]);
      REQUIRE(Approx(0.0) == frenet[1]);
    }
  }

  SECTION("ToCartesian()") {
    SECTION("Point 1") {
      const Vector2d cartesian = map.ToCartesian({18.027756, 0.0});
      REQUIRE(Approx(-5.0) == cartesian[0]);
      REQUIRE(Approx(-10.0) == cartesian[1]);
    }

    SECTION("Point 4") {
      const Vector2d cartesian = map.ToCartesian({64.083269, 0.0});
      REQUIRE(Approx(5.0) == cartesian[0]);
      REQUIRE(Approx(10.0) == cartesian[1]);
    }
  }

  SECTION("ToFrenet() -> ToCartesian()") {
    SECTION("Normal Conversion") {
      const Vector2d cartesian = map.ToCartesian(map.ToFrenet({7.0, -12.0}));
      REQUIRE(Approx(7.0) == cartesian[0]);
      REQUIRE(Approx(-12.0) == cartesian[1]);
    }

    SECTION("s + length()") {
      Vector2d frenet = map.ToFrenet({-5.0, 3.0});
      frenet[0] += map.length();

      const Vector2d cartesian = map.ToCartesian(frenet);
      REQUIRE(Approx(-5.0) == cartesian[0]);
      REQUIRE(Approx(3.0) == cartesian[1]);
    }

    SECTION("s - length()") {
      Vector2d frenet = map.ToFrenet({3.0, 0.0});
      frenet[0] -= map.length();

      const Vector2d cartesian = map.ToCartesian(frenet);
      REQUIRE(Approx(3.0) == cartesian[0]);
      REQUIRE(Approx(0.0) == cartesian[1]);
    }

    SECTION("s == 0") {
      const Vector2d cartesian = map.ToCartesian({0.0, 0.0});
      REQUIRE(Approx(-20.0) == cartesian[0]);
      REQUIRE(Approx(0.0) == cartesian[1]);
    }

    SECTION("s == 74.083269") {
      const Vector2d cartesian = map.ToCartesian({74.083269, 0.0});
      REQUIRE(Approx(-5.0) == cartesian[0]);
      REQUIRE(Approx(10.0) == cartesian[1]);
    }
  }
}