// Interface for stochastic graph library. (C) David Wilkie, 2013

#ifndef STOCH_GRAPH_HPP
#define STOCH_GRAPH_HPP

#include "util_point.hpp"
#include "util_path_element.hpp"
#include "sumo_util.hpp"
#include "stoch_edge.hpp"
#include "density_to_velocity.hpp"
#include "velocity_map.hpp"

#include <vector>
#include <iostream>
#include <queue>

#include "roadie/hwm_network.hpp"

#include <libxml++/libxml++.h>
#include <gtest/gtest_prod.h>

using std::vector;
using stoch_graph_util::util_path_element;

// Utility structures used in parsing OSM data.
struct OsmNode {
   float longitude;
   float latitude;
   string id;

   vector<int> associated_edge_indices;
};

struct OsmWay {
   string id;
   vector<string> node_ids;
   string road_class;
   bool one_way;

   OsmWay() {
      one_way = false;
      road_class = "non-road";
   }
};

// Calculate distance between two edges.
float euclid_distance(const stoch_edge& start, const stoch_edge& goal);

// Heuristic function for A* routing.
float time_lower_bound(const stoch_edge& start, const stoch_edge& goal);


class stoch_graph
{
public:
   stoch_graph(){

      // Determines the _constant_ uncertainty for travel time.
      // Also change rho_std_ in stoch_edge to match.
      T_std_ = 0.2;
   }

   stoch_graph(const stoch_graph& other);

   void update_fixed(vector<util_path_element> path);

   // Creates the time-varying density and velocity fields for the network,
   // assuming that all the roads are empty and the velocity is the maximum,
   // max_velocity, and has a specified standard deviation, std_velocity.
   // delta_t is the time interval length used for the time discretization,
   // and size is the number of time intervals.
   // ASSUMES that each edge has a valid den_to_vel.
   void initialize_to_empty_roads(float delta_t,
                                  int size);

   // Creates uniform den_to_vel functions for all edges.
   void initialize_den_to_vel(float v_max, float rho_max);

   // Globally set the maximum density.
   void set_max_rho(float max_rho);

   float get_intersection_time(float u, float t, stoch_edge& next_edge);

   // An A* planner that finds the path with the most optimal travel time
   // through a stochastic graph. 's' is the id of the starting edge. 'g'
   // is the id of the ending edge. 't' is the starting time.  'travel time'
   // is an output value recording the amount of time the trip is expected
   // to take. The function 'h' is a heuristic function that
   // TODO A* Planner should be separate & take a graph as input.
   vector<util_path_element> a_star_plan(int s,
                                         int g,
                                         float t,
                                         float& travel_time,
                                         std::function<
                                            float(const stoch_edge&,
                                                  const stoch_edge&)> h,
                                         float variance_weight,
                                         bool use_new_models);

   vector<util_path_element> a_star_plan(int s,
                                         int g,
                                         float t,
                                         float& travel_time,
                                         std::function<
                                            float(const stoch_edge&,
                                                  const stoch_edge&)> h,
                                         float variance_weight);

   vector<util_path_element> aware_and_sensing(int s,
                                               int g,
                                               float t,
                                               float& travel_time,
                                               const VelocityMap& velocities,
                                               std::function<
                                               float(const stoch_edge&,
                                                     const stoch_edge&)> h,
                                               float variance_weight);

   vector<util_path_element> plan_for_tuning(int s,
                                             int g,
                                             float t,
                                             float& travel_time,
                                             std::function<
                                             float(const stoch_edge&,
                                                   const stoch_edge&)> h,
                                             float variance_weight);

   // Gets the shortest path from road segment id s to road segment id g.
   // The path is returned as integer indices into 'edges'.
   vector<int> get_shortest_path(int s, int g,
                                 std::function<
                                    float(const stoch_edge&,
                                          const stoch_edge&)> h = euclid_distance);


   bool exhaustive_search(int s, int g);

   // Gets the shortest path from road segment id s to road segment id g.
   // The path is returned as integer indices into 'edges'.
   vector<util_path_element> get_fastest_path(
      int s,
      int g,
      float start_time,
      const VelocityMap& velocities,
      std::function<
      float(const stoch_edge&,
            const stoch_edge&)> h = euclid_distance);

   // Initializes the geometry and topology of a graph from a libroad network.
   // TODO: The input should be const.
   void create_graph_from_osm(const string& filename);

   // Initializes the geometry and topology of a graph from a libroad network.
   // TODO: The input should be const.
   void create_graph_from_roadie(hwm::network& net);

   // Used to initialize a graph from sumo. This calls 'create_graph...'
   // AS WELL AS SETS THE PROJECTION OFFSET.
   void load_graph_from_sumo(const string& base_filename,
                             sumo_util::id_maps* id_maps);

   void load_graph_from_sumo(const string& base_filename,
                             sumo::network& network,
                             sumo_util::id_maps* id_maps);


   // Builds a map of all road names.
   bool build_map_of_road_names(const string& filename);

   // Identifies the edge that is the best match for a lat-long point.
   int edge_from_point(double longitude, double latitude);

   int edge_from_WGS84_point(double x, double y);

   void long_lat_to_local(double longitude,
                          double latitude,
                          double* local_x,
                          double* local_y);

   // Populates a data structure of lat-long values for a sumo network.
   // 'basefilename' should be the same as given to create_graph_from_sumo.
   bool create_latlong_for_sumo(std::string basefilename);

   /*
   // Old planners for testing.
   // Planner for stochastic estimate using graph and updated conditions.
   vector<util_path_element> plan_for(int s,
                                      int g,
                                      float t,
                                      float& travel_time);

   // Planner for graph using only shortest distance.
   vector<util_path_element> plain_plan_for(int s,
                                            int g,
                                            float t,
                                            float& travel_time);

   // Planner for stochastic graph using 'historical' database.
   vector<util_path_element> stoch_plan_for(int s,
                                            int g,
                                            float t,
                                            float& travel_time,
                                            stoch_graph history);
   */
   void export_lanestats(float time_step, string filename);


   // TODO: Should be private.
public:
   vector<stoch_edge> edges;

   // A map of all road names, indexed by OSM ids.
   map<int, string> road_names_;

   // The time discretization used for storing traffic conditions.
   float delta_t_;

   float T_std_;

private:
   void parse_names(const xmlpp::Node* node);

   void build_nodes(const xmlpp::Node* node,
                    std::map<string, OsmNode>& node_map);

   void build_ways(const xmlpp::Node* node,
                   std::map<string, OsmWay>& way_map);

   void store_way_nd_and_tag(const xmlpp::Node* node,
                             OsmWay* way);

   // Initializes the geometry and topology of a graph from SUMO.
   // TODO: The input should be const.
   // DEPRECATED: Should be called from load_graph_from_sumo to set local_frame.
   void create_graph_from_sumo(sumo::network& net, sumo_util::id_maps& id_maps);

   FRIEND_TEST(StochGraphUnitTests, TestCreateGraphFromSumo);
};

// Utility path length output.
void output_expected_lengths(string filename,
                             const vector<vector<util_path_element>>& routes,
                             const stoch_graph& graph);

void output_expected_times(string filename,
                         const vector<vector<util_path_element>>& routes);



#endif
