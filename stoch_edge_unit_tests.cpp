// Unit tests for StochEdge.
#include "stoch_edge.hpp"

#include <gtest/gtest.h>

namespace StochEdgeUnitTestUtil {

void LoadConstantDensity(stoch_edge* edge) {
   vector<float> means, dev;

   for (int i = 0; i < 100; i++) {
      means.push_back(10 + 0.01);
      dev.push_back(5 + 0.01);
   }

   edge->initialize_rho(0.5, means, dev);
}

// void LoadConstantVelocity(stoch_edge* edge) {
//    vector<float> means, dev;

//    for (int i = 0; i < 100; i++) {
//       means.push_back(10 + 0.01);
//       dev.push_back(5 + 0.01);
//    }

//    edge->initialize_vel(0.5, means, dev);
// }

} // namespace StochEdgeUnitTestUtil

using StochEdgeUnitTestUtil::LoadConstantDensity;
//using StochEdgeUnitTestUtil::LoadConstantVelocity;

TEST(StochEdgeUnitTests, TestCreateStochEdge) {
   stoch_edge edge;

   stoch_edge* new_edge = new stoch_edge();

   EXPECT_TRUE(new_edge != NULL);
}

TEST(StochEdgeUnitTests, TestInitializeRho) {
   stoch_edge edge;

   vector<float> means, dev;

   for (int i = 0; i < 100; i++) {
      means.push_back(i + 0.01);
      dev.push_back(i + 0.01);
   }

   edge.initialize_rho(0.5, means, dev);

   // Check that there are density distributions
   EXPECT_TRUE(edge.rho_.size() > 0);

   // Check that the values are as assigned.
   EXPECT_TRUE(abs(edge.rho_[4].mean() - 4.01) < 0.0001);
   EXPECT_TRUE(abs(edge.rho_[4].standard_deviation() - 4.01) < 0.0001);

   stoch_edge another_edge;
   LoadConstantDensity(&another_edge);

   // Check that there are density distributions
   EXPECT_TRUE(another_edge.rho_.size() > 0);

   // Check that the values are as assigned.
   EXPECT_TRUE(abs(another_edge.rho_[4].mean() - 10.01) < 0.0001);
   EXPECT_TRUE(abs(another_edge.rho_[4].standard_deviation() - 5.01) < 0.0001);
}
/*
TEST(StochEdgeUnitTests, TestInitializeVel) {
   stoch_edge edge;

   vector<float> means, dev;

   for (int i = 0; i < 100; i++) {
      means.push_back(i + 0.01);
      dev.push_back(i + 0.01);
   }

   //   edge.initialize_vel(0.5, means, dev);

   // Check that there are density distributions
   EXPECT_TRUE(edge.vel.size() > 0);

   // Check that the values are as assigned.
   EXPECT_TRUE(abs(edge.vel[4].mean() - 4.01) < 0.0001);
   EXPECT_TRUE(abs(edge.vel[4].standard_deviation() - 4.01) < 0.0001);
   }*/

TEST(StochEdgeUnitTests, TestGetMeanRho) {
   stoch_edge edge;
   LoadConstantDensity(&edge);

   EXPECT_TRUE(abs(edge.get_mean_rho(1) - 10.01) < 0.0001);
}

TEST(StochEdgeUnitTests, TestGetVarRho) {
   stoch_edge edge;
   LoadConstantDensity(&edge);

   EXPECT_TRUE(abs(edge.get_var_rho(1) - 5.01) < 0.0001);
}

// TEST(StochEdgeUnitTests, TestGetMeanVel) {
//    stoch_edge edge;
//    LoadConstantVelocity(&edge);

//    EXPECT_TRUE(abs(edge.get_mean_vel(1) - 10.01) < 0.0001);
// }

// TEST(StochEdgeUnitTests, TestGetVarVel) {
//    stoch_edge edge;
//    LoadConstantVelocity(&edge);

//    EXPECT_TRUE(abs(edge.get_var_vel(1) - 5.01) < 0.0001);
// }

TEST(StochEdgeUnitTests, TestAccessorsForNeighbors) {
   stoch_edge edge;
   int neighbor_A, neighbor_B, neighbor_C;
   neighbor_A = 3;
   neighbor_B = 6;
   neighbor_C = 9;

   edge.add_neighbor(neighbor_A);
   edge.add_neighbor(neighbor_B);
   edge.add_neighbor(neighbor_C);

   EXPECT_TRUE(edge.neighbors_size() == 3);
   EXPECT_TRUE(edge.get_neighbor(1) == neighbor_B);
}

TEST(StochEdgeUnitTests, TestPerEdgeFundDiag) {
   stoch_edge edge_a, edge_b;

   den_to_vel fund_a, fund_b;
   fund_a.set_v_max(14);
   fund_a.set_rho_max(0.5);

   fund_b.set_v_max(7);
   fund_b.set_rho_max(0.9);

   edge_a.set_den_to_vel(fund_a);
   edge_b.set_den_to_vel(fund_b);

   EXPECT_TRUE(edge_a.get_den_to_vel() == fund_a);
   EXPECT_TRUE(edge_b.get_den_to_vel() == fund_b);
}

TEST(StochEdgeUnitTests, TestCopyConstructor) {
   stoch_edge edge_a;

   edge_a.add_neighbor(2);
   edge_a.set_len(10);

   den_to_vel fund_a;
   fund_a.set_v_max(14);
   fund_a.set_rho_max(0.5);
   edge_a.set_den_to_vel(fund_a);

   stoch_edge edge_b;
   edge_b = edge_a;

   EXPECT_EQ(edge_b.get_len(), edge_a.get_len());
   EXPECT_EQ(edge_b.get_neighbor(0), edge_a.get_neighbor(0));
   EXPECT_EQ(edge_b.get_den_to_vel(), edge_a.get_den_to_vel());
}

int main(int argc, char** argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
