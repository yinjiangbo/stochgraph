// Unit tests for StochEdge.
#include "sumo_util.hpp"

#include <gtest/gtest.h>

namespace sumo_util {

TEST(SumoUtilUnitTests, Test) {

   std::cerr << CartesianDistanceWGS84(11, 10, 10, 11) << " in meters\n";

}

}  // namespace sumo_util

int main(int argc, char** argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
