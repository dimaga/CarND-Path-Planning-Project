#ifndef SRC_MAP_H_
#define SRC_MAP_H_

#include "Waypoint.h"
#include <istream>
#include <vector>

class Map {
 public:
  explicit Map(std::istream& is);

  double length() const;

 private:
  std::vector<Waypoint> track_;
  double length_{0.0};
};

#endif  // SRC_MAP_H_
