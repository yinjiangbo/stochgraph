#ifndef UTIL_PATH_ELEMENT_HPP
#define UTIL_PATH_ELEMENT_HPP

#include <boost/math/distributions/normal.hpp>

using boost::math::normal; // typedef provides default type is double.

namespace stoch_graph_util {

// A utility path element structure for the stochastic graph class.
// TODO rename file to account for name change.
struct util_path_element {
   float depart_time;
   int id;
   normal A; // arrival time.
   normal D; // departure time.

   // The density on the route at that time.
   float P;

   // The velocity on the route at that time.
   float V;

   float cost;

   int neighbors;
};

}

#endif
