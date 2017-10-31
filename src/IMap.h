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

  inline Eigen::Vector2d DiffFrenet(const Eigen::Vector2d& f1,
                                    const Eigen::Vector2d& f0) const {
    const double track_length = length();
    const double half_track_length = track_length * 0.5;

    double ds = std::fmod(f1[0] - f0[0], track_length);
    if (ds < -half_track_length) {
      ds += track_length;
    } else if (ds > half_track_length) {
      ds -= track_length;
    }

    const double dd = f1[1] - f0[1];

    return {ds, dd};
  }
};

#endif  // SRC_IMAP_H_
