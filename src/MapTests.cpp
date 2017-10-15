#include "Map.h"
#include "Catch/catch.hpp"
#include <sstream>

TEST_CASE("Map unit tests", "[map]") {
  std::stringstream s{
    "784.6001 1135.571 0 -0.02359831 -0.9997216\n"
    "815.2679 1134.93 30.6744785308838 -0.01099479 -0.9999396\n"
    "844.6398 1134.911 60.0463714599609 -0.002048373 -0.9999979"
  };

  Map map(s);
}
