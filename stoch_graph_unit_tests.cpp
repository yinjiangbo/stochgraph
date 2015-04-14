// Unit tests for StochGraph.
#include "stoch_graph.hpp"
#include "local_frame.hpp"

#include <gtest/gtest.h>

TEST(StochGraphUnitTests, TestCopyConstructor) {
   stoch_graph original;
   stoch_edge edge;

   original.edges.push_back(edge);

   stoch_graph* new_stoch_graph = new stoch_graph(original);

   // TODO Do a better test, and include den_to_vel.

   EXPECT_TRUE(new_stoch_graph->edges.size() == 1);
}

TEST(StochGraphUnitTests, TestAStarPlan) {
   // Initializes a simple graph with two choices to the goal.
   stoch_graph graph;
   graph.edges.resize(4);

   graph.initialize_den_to_vel(60, 0.96);

   stoch_edge& edge_a = graph.edges[0];
   stoch_edge& edge_b = graph.edges[1];
   stoch_edge& edge_goal = graph.edges[2];
   stoch_edge& edge_start = graph.edges[3];

   vector<float> constant_high_mean_density;
   vector<float> constant_high_std_deviation_density;

   vector<float> constant_low_mean_density;
   vector<float> constant_low_std_deviation_density;

   vector<float> constant_high_mean_velocity;
   vector<float> constant_high_std_deviation_velocity;

   vector<float> constant_low_mean_velocity;
   vector<float> constant_low_std_deviation_velocity;

   // Adding an arbitrary number of constant elements for the density and velocity.
   // TODO This should be easier.
   for (int i = 0; i < 1000; i ++) {
      constant_high_mean_density.push_back(0.8);
      constant_high_std_deviation_density.push_back(0.4);

      constant_low_mean_density.push_back(0.1);
      constant_low_std_deviation_density.push_back(0.05);

      constant_high_mean_velocity.push_back(40);
      constant_high_std_deviation_velocity.push_back(15);

      constant_low_mean_velocity.push_back(10);
      constant_low_std_deviation_velocity.push_back(5);
   }

   // Shorter, but slower route.
   edge_a.initialize_rho(1, constant_high_mean_density, constant_high_std_deviation_density);
   // edge_a.initialize_vel(1, constant_low_mean_velocity, constant_low_std_deviation_velocity);
   edge_a.set_len(1900);
   edge_a.add_neighbor(2);
   edge_a.start.x = 0;
   edge_a.start.y = 200;
   edge_a.end.x = 0;
   edge_a.end.y = 1900;

   // Longer, but faster route.
   edge_b.initialize_rho(1, constant_low_mean_density, constant_low_std_deviation_density);
   // edge_b.initialize_vel(1, constant_high_mean_velocity, constant_high_std_deviation_velocity);
   edge_b.set_len(2000);
   edge_b.add_neighbor(2);
   edge_b.start.x = 0;
   edge_b.start.y = 200;
   edge_b.end.x = 0;
   edge_b.end.y = 1900;

   // This is the goal, and its stats should not affect the routing.
   edge_goal.initialize_rho(1, constant_low_mean_density, constant_low_std_deviation_density);
   // edge_goal.initialize_vel(1, constant_high_mean_velocity, constant_high_std_deviation_velocity);
   edge_goal.set_len(200);
   edge_goal.start.x = 0;
   edge_goal.start.y = 1900;
   edge_goal.end.x = 0;
   edge_goal.end.y = 2100;

   // This is the start, and its stats should not affect routing.
   edge_start.initialize_rho(1, constant_low_mean_density, constant_low_std_deviation_density);
   // edge_start.initialize_vel(1, constant_high_mean_velocity, constant_high_std_deviation_velocity);
   edge_start.set_len(200);
   edge_start.add_neighbor(0);
   edge_start.add_neighbor(1);
   edge_start.start.x = 0;
   edge_start.start.y = 0;
   edge_start.end.x = 0;
   edge_start.end.y = 200;

   float travel_time = 0;
   vector<util_path_element> path = graph.a_star_plan(3,
                                                      2,
                                                      0,
                                                      travel_time,
                                                      time_lower_bound,
                                                      1);

   // EXPECT_TRUEs that there is a path.
   EXPECT_TRUE(path.size() > 0);

   std::cout << path.size() << " is current path size\n";

   // EXPECT_TRUEs that the middle element (not start or goal) is the faster choice.
   EXPECT_TRUE(path[1].id == 1);
}

TEST(StochGraphUnitTests, TestVelocityMattersForPlan) {
   // Initializes a simple graph with two choices to the goal, with one longer
   // but faster due to its higher max velocity.
   stoch_graph graph;
   graph.edges.resize(4);

   den_to_vel fast_fund;
   den_to_vel slow_fund;

   fast_fund.set_v_max(30);
   fast_fund.set_rho_max(1.0);

   slow_fund.set_v_max(10);
   slow_fund.set_rho_max(1.0);

   stoch_edge& edge_a = graph.edges[0];
   stoch_edge& edge_b = graph.edges[1];
   stoch_edge& edge_goal = graph.edges[2];
   stoch_edge& edge_start = graph.edges[3];

   edge_a.set_den_to_vel(fast_fund);
   edge_goal.set_den_to_vel(fast_fund);
   edge_start.set_den_to_vel(fast_fund);

   // edge B is the slow road.
   edge_b.set_den_to_vel(slow_fund);

   graph.initialize_to_empty_roads(0.5, 1000);

   edge_a.set_len(1000);
   edge_a.add_neighbor(2);
   edge_a.start.x = 0;
   edge_a.start.y = 200;
   edge_a.end.x = 0;
   edge_a.end.y = 1000;

   edge_b.set_len(500);
   edge_b.add_neighbor(2);
   edge_b.start.x = 0;
   edge_b.start.y = 200;
   edge_b.end.x = 0;
   edge_b.end.y = 500;

   // This is the goal, and its stats should not affect the routing.
   edge_goal.set_len(200);
   edge_goal.start.x = 0;
   edge_goal.start.y = 1000;
   edge_goal.end.x = 0;
   edge_goal.end.y = 2000;

   // This is the start, and its stats should not affect routing.
   edge_start.set_len(200);
   edge_start.add_neighbor(0);
   edge_start.add_neighbor(1);
   edge_start.start.x = 0;
   edge_start.start.y = 0;
   edge_start.end.x = 0;
   edge_start.end.y = 200;

   float travel_time = 0;
   vector<util_path_element> path = graph.a_star_plan(3,
                                                      2,
                                                      0,
                                                      travel_time,
                                                      time_lower_bound,
                                                      1);

   // EXPECT_TRUEs that there is a path.
   EXPECT_TRUE(path.size() > 0);

   std::cout << path.size() << " is current path size\n";

   // EXPECT_TRUEs that the middle element (not start or goal) is the
   // faster choice, edge a which is longer but has a better den_to_vel.
   EXPECT_TRUE(path[1].id == 0);
}

TEST(StochGraphUnitTests, TestCreateGraphFromSumo) {
   std::string data_location = "/home/wilkie/Nexus/ALL/Data/RoadNetworks/";
   std::string nodes_file = data_location + "manhattan.nod.xml";
   std::string edges_file = data_location + "manhattan.edg.xml";
   std::string edge_type_file = data_location + "road_types.typ.xml";
   std::string connections_file = data_location + "manhattan.con.xml";

   sumo::network sumo_network =
   sumo::load_xml_network(nodes_file.c_str(),
                          edge_type_file.c_str(),
                          edges_file.c_str(),
                          connections_file.c_str());

   // These values assume manhattan.osm was usod as input and the
   // --junctions.join argument was given to netconvert.
   // TODO: These should use some input that does not change over time.
   EXPECT_EQ(sumo_network.edges.size(), 4074);
   EXPECT_EQ(sumo_network.nodes.size(), 2131);
   EXPECT_EQ(sumo_network.types.size(), 14);
   EXPECT_EQ(sumo_network.connections.size(), 8691);

   sumo_util::id_maps id_maps;
   stoch_graph graph;
   graph.create_graph_from_sumo(sumo_network, id_maps);

   int sum = 0;
   for (int i = 0; i < graph.edges.size(); i++) {
      EXPECT_GT(graph.edges[i].get_len(), 0);

      sum += graph.edges[i].neighbors_size();

   }

   EXPECT_EQ(graph.edges[3972].get_den_to_vel().get_v_max(), 44);

   EXPECT_EQ(sum, sumo_network.connections.size());
}

TEST(StochGraphUnitTests, TestEdgeFromPoint) {
   sumo_util::id_maps id_maps;
   stoch_graph graph;
   graph.load_graph_from_sumo("/home/wilkie/Nexus/ALL/Data/RoadNetworks/campus_to_home",
                              &id_maps);


   int id = graph.edge_from_point(-78.912, 35.907);

   std::cout << "Id returned is " << id << std::endl;

   id = graph.edge_from_point(-78.912, 35.907);

   std::cout << "Id returned is " << id << std::endl;
}

TEST(StochGraphUnitTests, TestGraphFromOSM) {
   stoch_graph graph;
   graph.create_graph_from_osm("./Data/san_fran.osm");
}

TEST(StochGraphUnitTests, TestShortestPath) {
   sumo_util::id_maps id_maps;
   stoch_graph graph;
   graph.load_graph_from_sumo("/home/wilkie/Nexus/ALL/Data/RoadNetworks/manhattan",
                              &id_maps);


   vector<int> path = graph.get_shortest_path(2, 123);

   std::cout << "path is... \n";
   for (int i = 0; i < path.size(); i++) {
      std::cout << "road_network->ColorLane(" << "\""
                << id_maps.stoch_edge_to_sumo[path[i]] << "\""
                << ", 0, 1, 1);\n";
   }
   std::cout << "\n";
}


TEST(StochGraphUnitTests, TestLoadGraphFromSumo) {
   std::string base_filename = "./Data/san_fran";

   sumo_util::id_maps id_maps;

   stoch_graph graph;
   graph.load_graph_from_sumo(base_filename, &id_maps);

   double x, y;
   LocalFrame::GetInstance()->LongLatToLocal(-122.514432, 37.698404, &x, &y);

   std::cout << x << " " << y << std::endl;
}


TEST(StochGraphUnitTests, TestProjections) {
   std::string base_filename = "./Data/san_fran";

   sumo_util::id_maps id_maps;

   stoch_graph graph;
   graph.load_graph_from_sumo(base_filename, &id_maps);

   double x, y;
   LocalFrame::GetInstance()->LongLatToLocal(-122.514432, 37.698404, &x, &y);

   std::cout << x << " " << y << std::endl;

   double new_long, new_lat;
   LocalFrame::GetInstance()->LocalToLongLat(x, y, &new_long, &new_lat);

   std::cout << new_long << " " << new_lat << std::endl;
}




int main(int argc, char** argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
