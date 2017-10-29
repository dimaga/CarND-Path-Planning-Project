#include "Planner.h"
#include <cassert>

Planner::Planner(const IMap& map)
  : map_(map) {}


void Planner::Plan(const IObstacles& obstacles, ITrajectory* pTrajectory) {
  assert(nullptr != pTrajectory);

  using Eigen::Vector2d;
  const Vector2d recent_cartesian = pTrajectory->recent_cartesian();
  const Vector2d recent_frenet = map_.ToFrenet(recent_cartesian);

  const double recent_time_sec = pTrajectory->recent_time_sec();

  auto obstacle = obstacles.Forward(recent_time_sec,
                                    recent_frenet[0],
                                    lane_,
                                    50);

  if (obstacle) {
    if (lane_ < 2) {
      auto right_obstacle = obstacles.Forward(recent_time_sec,
                                              recent_frenet[0],
                                              lane_ + 1,
                                              50);
      if (right_obstacle) {
        ref_vel_ = obstacle->velocity_mph();
      } else {
        ref_vel_ = 45.0;
        ++lane_;
      }
    } else if (lane_ > 0) {
      auto left_obstacle = obstacles.Forward(recent_time_sec,
                                             recent_frenet[0],
                                             lane_ - 1,
                                             50);
      if (left_obstacle) {
        ref_vel_ = obstacle->velocity_mph();
      } else {
        ref_vel_ = 45.0;
        --lane_;
      }
    } else {
      ref_vel_ = obstacle->velocity_mph();
    }
  }

  const double dest_d = IMap::kLaneW * (lane_ + 0.5);
  const auto xy0 = map_.ToCartesian({recent_frenet[0] + 30, dest_d});
  const auto xy1 = map_.ToCartesian({recent_frenet[0] + 60, dest_d});
  const auto xy2 = map_.ToCartesian({recent_frenet[0] + 90, dest_d});

  pTrajectory->Trace(ref_vel_, {xy0, xy1, xy2});
}
