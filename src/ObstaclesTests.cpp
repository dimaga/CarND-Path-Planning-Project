#include "Obstacles.h"
#include "IMap.h"
#include "Catch/catch.hpp"

namespace {

class MapDummy final : public IMap {
  Eigen::Vector2d ToFrenetVel(const Eigen::Vector2d& /*cartesian*/,
                              const Eigen::Vector2d& vel) const override {
    return vel;
  }


  Eigen::Vector2d ToFrenet(const Eigen::Vector2d& cartesian) const override {
    return cartesian;
  }


  Eigen::Vector2d ToCartesian(const Eigen::Vector2d& frenet) const override {
    return frenet;
  }
};

}  // namespace


TEST_CASE("Obstacles unit tests", "[planner]") {
  MapDummy map_dummy;
  Obstacles obstacles(map_dummy);


  SECTION("Forward") {
    obstacles.Clear();
    obstacles.Add({-10.0, IMap::kLaneW * 1.5}, {-10.0, 0.0});
    obstacles.Add({20.0, IMap::kLaneW * 1.5}, {-12.0, 0.0});
    obstacles.Add({-20.0, IMap::kLaneW * 2.4}, {-12.0, 0.0});

    REQUIRE(nullptr == obstacles.Forward(0.0, 5.0, 0, 10));
    REQUIRE(nullptr == obstacles.Forward(0.0, 20.0, 1, 20));
    REQUIRE(nullptr == obstacles.Forward(0.0, -50.0, 1, 20));
    REQUIRE(nullptr != obstacles.Forward(0.0, 0.0, 1, 30));
    REQUIRE(nullptr != obstacles.Forward(0.0, 10.0, 1, 40));
  }
}
