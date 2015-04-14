// Class declaration for an edge of the stochastic graph.
#ifndef STOCH_EDGE_HPP
#define STOCH_EDGE_HPP

#include "density_to_velocity.hpp"
#include "util_point.hpp"

#include <vector>
#include <map>
#include <boost/math/distributions/normal.hpp>
#include <gtest/gtest_prod.h>

using boost::math::normal; // typedef provides default type is float.
using std::vector;
using std::string;

// TODO Generalize
class sparse_vector {
public:
   normal& operator[](int i) {
      if (map_.find(i) != map_.end()) {
         return map_[i];
      }
      else {
         map_[i] = default_;
         return map_[i];
      }
   }

   normal operator[](int i) const {
      if (map_.find(i) != map_.end()) {
         return map_.find(i)->second;
      }
      else {
         return default_;
      }
   }

   void set_default(normal v) {
      default_ = v;
   };

   int size() {
      return map_.size();
   }

private:
   std::map<int, normal> map_;

   // Value to return if an element being requested is not in the map.
   normal default_;
};

// An edge of the stochastic graph.
class stoch_edge {
public:
   // Constructor sets den_to_vel to impossible -1 values so that it can be
   // determined if it has been initialized.
   stoch_edge()
         : has_intersection(false) {
      fund_.set_v_max(-1);
      fund_.set_rho_max(-1);

      // Also change T_std_ in stoch_graph to match.
      rho_std_ = 0.2;
   }

   // Updates the estimated traffic density.
   void update(normal A, normal D, const den_to_vel& fund);

   void initialize_rho(float _delta_t, const vector<float>& means, const vector<float>& std_deviations);

   bool in_bins(float t) const;
   int get_bin_id(float t) const;

   //   void initialize_vel(float _delta_t, vector<float>, vector<float>);
   //   float get_mean_vel(float t) const;
   //   float get_var_vel(float t) const;
   float get_mean_rho(float t) const;
   float get_var_rho(float t) const;
   void set_len(const float& _len);
   const float& get_len() const;
   float T(float t, den_to_vel fund);
   float T(float t, den_to_vel fund, bool use_delay_model);
   float Ttilda(float t, den_to_vel fund) const;

   // TODO index neighboring only works with a storage structure.
   void add_neighbor(int _neighbor);
   int get_neighbor(int index) const;
   int neighbors_size() const;

   void set_den_to_vel(const den_to_vel& fund);
   // TODO This is awkward.
   den_to_vel& retrieve_den_to_vel();
   den_to_vel get_den_to_vel() const;

   int get_lane_count();
   void set_lane_count(int count);

   void set_osm_id(int osm_id);
   int get_osm_id() const;

   void set_sumo_id(const string& osm_id);
   string get_sumo_id() const;

   float distance_to_shape(double x, double y);
   float distance_to_segment(point query, point a, point b);

   // Pushes back a point onto the shape_ vector.
   void add_point(point p);

public: //but shouldn't be
   point start;
   point end;

   // TODO Temp weak intersection model
   bool has_intersection;

   // The shape is not used specifically by stoch_graph or the router, but it is
   // stored here for visualization purposes.
   vector<point> shape_;

   // This is the 'type' id, not used here.
   std::string type;

   // This is exclusively used for the latitude-longitude of the road, which is
   // missing from the Sumo data.
   vector<point> lat_longs_;

   // Retrieves the speed limit from the den_to_vel class.
   float get_speedlimit() const;

   float get_delay_time();

   void set_delay_time(float delay_time);

private:
   //Distribution
   //vector<normal> rho;
   //   map<int, normal> rho;
   sparse_vector rho_;

   // TODO I need an accessor for time.
   float t_e;
   float delta_t;

   //Topology
   vector<int> neighbors;
   int lane_count;

   //Geometry
   float len;

   // The fundamental diagram.
   den_to_vel fund_;

   // Id from Osm.
   int osm_id_;

   // Id from sumo.
   string sumo_id_;

   // The time a car needs to wait to enter this lane before it can enter the
   // lane due to congestion.
   float delay_time_;

   float rho_std_;

   FRIEND_TEST(StochEdgeUnitTests, TestInitializeRho);
   FRIEND_TEST(StochEdgeUnitTests, TestInitializeVel);
};

#endif
