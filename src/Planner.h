#ifndef SRC_PLANNER_H_
#define SRC_PLANNER_H_

#include "IObstacles.h"
#include "IMap.h"
#include "ITrajectory.h"

class Planner final {
 public:
  explicit Planner(const IMap& map);

  void Plan(const IObstacles& obstacles, ITrajectory* pTrajectory);

 private:
  const IMap& map_;

  struct Config {
    Config(int lane, double ref_vel)
      : lane_{lane}
      , ref_vel_{ref_vel} {}

    int lane_;
    double ref_vel_;
  };

  Config last_config_{1, 45.0};

  double EstimateCost(const IObstacles& obstacles,
                      const ITrajectory& trajectory) const;

  void Trace(double recent_frenet_s,
             const Config& config,
             ITrajectory* pTrajectory) const;
};

#endif  // SRC_PLANNER_H_
