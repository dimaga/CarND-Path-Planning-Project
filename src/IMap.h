#ifndef SRC_IMAP_H_
#define SRC_IMAP_H_

#include "Eigen-3.3/Eigen/Core"

class IMap {
 public:
  static constexpr double kLaneW {4.0};

  virtual ~IMap() = default;

  virtual double length() const = 0;

  virtual Eigen::Vector2d ToFrenetVel(const Eigen::Vector2d& cartesian,
                                      const Eigen::Vector2d& vel) const = 0;

  virtual Eigen::Vector2d ToFrenet(const Eigen::Vector2d& cartesian) const = 0;
  virtual Eigen::Vector2d ToCartesian(const Eigen::Vector2d& frenet) const = 0;
};

#endif  // SRC_IMAP_H_
