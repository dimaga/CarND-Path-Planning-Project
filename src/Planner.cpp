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

  Config& config = last_config_;

  auto obstacle = obstacles.Forward(recent_time_sec,
                                    recent_frenet[0],
                                    config.lane_,
                                    50);

  if (obstacle) {
    if (config.lane_ < 2) {
      auto right_obstacle = obstacles.Forward(recent_time_sec,
                                              recent_frenet[0],
                                              config.lane_ + 1,
                                              50);
      if (right_obstacle) {
        config.ref_vel_ = obstacle->velocity_mph();
      } else {
        config.ref_vel_ = 45.0;
        ++config.lane_;
      }
    } else if (config.lane_ > 0) {
      auto left_obstacle = obstacles.Forward(recent_time_sec,
                                             recent_frenet[0],
                                             config.lane_ - 1,
                                             50);
      if (left_obstacle) {
        config.ref_vel_ = obstacle->velocity_mph();
      } else {
        config.ref_vel_ = 45.0;
        --config.lane_;
      }
    } else {
      config.ref_vel_ = obstacle->velocity_mph();
    }
  } else {
    config.ref_vel_ = 45.0;
  }

  Trace(recent_frenet[0], config, pTrajectory);
}


double Planner::cost(const IObstacles& obstacles,
                     const ITrajectory& trajectory) const {
  return 0.0;
}


void Planner::Trace(double recent_frenet_s,
                      const Planner::Config& config,
                      ITrajectory* pTrajectory) const {
  const double dest_d = IMap::kLaneW * (config.lane_ + 0.5);
  const auto xy0 = map_.ToCartesian({recent_frenet_s + 30, dest_d});
  const auto xy1 = map_.ToCartesian({recent_frenet_s + 60, dest_d});
  const auto xy2 = map_.ToCartesian({recent_frenet_s + 90, dest_d});

  pTrajectory->Trace(config.ref_vel_, {xy0, xy1, xy2});
}
