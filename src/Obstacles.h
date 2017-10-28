#ifndef SRC_OBSTACLES_H_
#define SRC_OBSTACLES_H_

#include "IObstacles.h"
#include "IMap.h"
#include <vector>

class Obstacles final : public IObstacles {
 public:
  explicit Obstacles(const IMap& map);

  void Clear();
  void Add(const Eigen::Vector2d& cartesian, const Eigen::Vector2d& vel);

  // IObstacles
 public:
  const Obstacle* Forward(double future_time_sec,
                          double future_ego_s,
                          int lane,
                          double range) const override;

 private:
  const IMap& map_;
  std::vector<Obstacle> obstacles_;
};

#endif  // SRC_OBSTACLES_H_
