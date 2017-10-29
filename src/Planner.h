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
    int lane_{1};
    double ref_vel_{45.0};
  };

  Config last_config_;

  double cost(const IObstacles& obstacles, const ITrajectory& trajectory) const;

  void Trace(double recent_frenet_s,
             const Config& config,
             ITrajectory* pTrajectory) const;
};

#endif  // SRC_PLANNER_H_
