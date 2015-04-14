// Class declaration for an edge of the stochastic graph.
#ifndef SUMO_UTIL_HPP
#define SUMO_UTIL_HPP

#include "util_path_element.hpp"

#include <proj_api.h>

#include <iostream>
#include <fstream>
#include <tuple>
#include <string>
#include <cmath>
#include <vector>
#include <map>

using stoch_graph_util::util_path_element;

using std::tuple;
using std::tie;
using std::string;
using std::to_string;
using std::pow;
using std::sqrt;
using std::vector;
using std::map;

namespace sumo_util {

// Maps between sumo ids and stoch_graph indices.
struct id_maps {
   map<std::string, int> sumo_edge_to_stoch;
   map<int, std::string> stoch_edge_to_sumo;
};

double distance(double x1, double y1, double x2, double y2);

// Used to compute the distance between two lat lang points by converting them
// to mercantor.
// It turns out this is not needed for Sumo as it does its own projection.
double
CartesianDistanceWGS84(double lat1, double lng1, double lat2, double lng2);

void output_routes_to_sumo(string filename,
                          const vector<vector<util_path_element>>& routes,
                          id_maps& id_maps);

void collect_routes_for_sumo(string filename,
                             const vector<vector<util_path_element>>& routes,
                             id_maps& id_maps,
                             int start_id);

}  // namespace sumo_util

#endif
