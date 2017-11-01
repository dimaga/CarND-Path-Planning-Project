#include "Planner.h"
#include <cassert>
#include <vector>
#include <limits>
#include <iostream>


Planner::Planner(const IMap& map)
  : map_(map) {}


void Planner::Plan(const IObstacles& obstacles, ITrajectory* pTrajectory) {
  assert(nullptr != pTrajectory);

  using Eigen::Vector2d;
  const Vector2d recent_cartesian = pTrajectory->recent_cartesian();
  const double recent_frenet_s = map_.ToFrenet(recent_cartesian).x();

  const double recent_time_sec = pTrajectory->recent_time_sec();

  std::vector<Config> configs;
  configs.push_back(last_config_);
  configs.emplace_back(last_config_.lane_,
                       std::min(45.0, last_config_.ref_vel_ * 2.0));
  configs.emplace_back(last_config_.lane_, last_config_.ref_vel_ * 0.5);
  configs.emplace_back(last_config_.lane_, 0.1);

  const double kMaxVelocityMph = 45.0;

  auto obstacle = obstacles.Forward(recent_time_sec,
                                    recent_frenet_s,
                                    last_config_.lane_,
                                    50);
  if (obstacle) {
    configs.emplace_back(last_config_.lane_, obstacle->velocity_mph());
  } else {
    configs.emplace_back(last_config_.lane_, kMaxVelocityMph);
  }

  if (last_config_.lane_ < 2) {
    auto right = obstacles.Forward(recent_time_sec,
                                   recent_frenet_s,
                                   last_config_.lane_ + 1,
                                   50);

    if (right) {
      configs.emplace_back(last_config_.lane_ + 1, right->velocity_mph());
    } else {
      configs.emplace_back(last_config_.lane_ + 1, kMaxVelocityMph);
    }
  }

  if (last_config_.lane_ > 0) {
    auto left = obstacles.Forward(recent_time_sec,
                                  recent_frenet_s,
                                  last_config_.lane_ - 1,
                                  50);

    if (left) {
      configs.emplace_back(last_config_.lane_ - 1, left->velocity_mph());
    } else {
      configs.emplace_back(last_config_.lane_ - 1, kMaxVelocityMph);
    }
  }

  double min_cost = std::numeric_limits<double>::max();
  const Config prev_config = last_config_;

  for (const auto& config : configs) {
    Trace(recent_frenet_s, config, pTrajectory, 70);
    const double cost = EstimateCost(obstacles,
                                     *pTrajectory,
                                     prev_config,
                                     config);

    if (cost < min_cost) {
      min_cost = cost;
      last_config_ = config;
    }
  }

  Trace(recent_frenet_s, last_config_, pTrajectory, 50);
}


double Planner::EstimateCost(const IObstacles& obstacles,
                             const ITrajectory& trajectory,
                             const Config& prev_config,
                             const Config& config) const {
  const auto& path_x = trajectory.path_x();
  const auto& path_y = trajectory.path_y();

  double cost = 1e3 * std::exp(-obstacles.min_distance(path_x, path_y));
  cost += 1e3 * std::exp(-trajectory.avg_speed());

  if (prev_config.lane_ != config.lane_) {
    cost += 1e-6;
  }

  return cost;
}


void Planner::Trace(double recent_frenet_s,
                    const Planner::Config& config,
                    ITrajectory* pTrajectory,
                    std::size_t samples) const {
  const double dest_d = IMap::kLaneW * (config.lane_ + 0.5);
  const auto xy0 = map_.ToCartesian({recent_frenet_s + 30, dest_d});
  const auto xy1 = map_.ToCartesian({recent_frenet_s + 60, dest_d});
  const auto xy2 = map_.ToCartesian({recent_frenet_s + 90, dest_d});

  pTrajectory->Trace(config.ref_vel_, {xy0, xy1, xy2}, samples);
}
