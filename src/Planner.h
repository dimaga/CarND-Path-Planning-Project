#ifndef SRC_PLANNER_H_
#define SRC_PLANNER_H_

#include "IObstacles.h"
#include "IMap.h"
#include "ITrajectory.h"
#include "ICosts.h"

class Planner final {
 public:
  Planner(const IMap& map, const ICosts& costs);

  void Plan(const IObstacles& obstacles, ITrajectory* pTrajectory);

 private:
  const IMap& map_;
  const ICosts& costs_;
  int lane_{1};
  double ref_vel_{45.0};
};

#endif  // SRC_PLANNER_H_
