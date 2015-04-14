// Class definition for an edge of the stochastic graph.

#include <Eigen/Dense>

#include "stoch_edge.hpp"

using std::endl;
using std::cerr;

const bool ADDITIVE_VARIANCE = false;

// Initializes the density values for all timesteps for this edge.
// TODO delta_t should almost surely not be a parameter here.
// TODO this is an ugly interface overall.
// TODO input vectors not needed anymore.
void stoch_edge::initialize_rho(float _delta_t,
                                const vector<float>& means,
                                const vector<float>& dev) {
   t_e = _delta_t * means.size();
   delta_t = _delta_t;

   // Check that the time and number of bins matches.
   int bins = t_e / delta_t;
   if ((bins != means.size()) or (means.size() != dev.size())) {
      cerr << "Time intervals did not match given vector data size, or vector sizes did not match: " << bins << ", " << means.size() << ", " << dev.size() << endl;
   }

   size_t size = std::min(means.size(), dev.size());

   // Sets the default road density and variance (0, epsilon);
   rho_.set_default(normal(means[0], dev[0]));
}

bool stoch_edge::in_bins(float t) const {

   // With a map, the bin is always 'in'
   return true;
}

// Returns the number of the bin that coorresponds with time t.
int stoch_edge::get_bin_id(float t) const {
   int bin = t / delta_t;

   return bin;
}

// Retrieves the mean density at time t.
float stoch_edge::get_mean_rho(float t) const
{
  int bin = t / delta_t;

  // Round up the bin if the difference is half of the delta_t value.
  if ((t - (bin*delta_t)) > (delta_t * 0.5))
  {
    bin++;
  }

  //Interpolate
  float frac = std::abs((t - (bin*delta_t)))/delta_t;
  float p_intrp;

  if (t > (bin * delta_t))
     p_intrp = frac*(rho_[bin+1].mean()) + (1 - frac)*(rho_[bin].mean());
  else
     p_intrp = frac*(rho_[bin-1].mean()) + (1 - frac)*(rho_[bin].mean());

  return p_intrp;
}

// Gets the density variance at time t.
float stoch_edge::get_var_rho(float t) const
{
  int bin = t / delta_t;

  if (ADDITIVE_VARIANCE) {
     return rho_[bin].standard_deviation();
  }
  else {
     return std::max(rho_[bin].mean() * rho_std_, 0.01);
  }
}

// Acccessors for length.
const float& stoch_edge::get_len() const
{
  return len;
}

void stoch_edge::set_len(const float& _len)
{
  len = _len;
}

// Simulated default parameter call.
float stoch_edge::T(float t, den_to_vel fund) {
   return T(t, fund, true);
}

// Retrieves the mean travel time to traverse this edge.
// t is the time of the query.
// fund is the fundamental diagram to use.
float stoch_edge::T(float t, den_to_vel fund, bool use_delay_model)
{
   delay_time_ = 0;

  float rho = get_mean_rho(t);

  float waiting_time = 0;

  if (use_delay_model) {
     while ((rho >= fund.get_rho_max()) and (waiting_time < 20)) {
        waiting_time += delta_t;

        rho = get_mean_rho(t + waiting_time);
     }
  }

  float v = fund.v(get_mean_rho(t + waiting_time));

  delay_time_ = waiting_time;

  return (len / std::max(std::numeric_limits<float>::min(), v)) + waiting_time;
}

// Retrieves the travel time variance for traversing this edge.
// t is the time of the query.
// fund is the fundamental diagram to use.
// TODO what is going on here?
float stoch_edge::Ttilda(float t, den_to_vel fund) const
{
  float dTdp2 = pow((1.0 / pow((1 - get_mean_rho(t) / fund.rho_max), 2)), 2);
  return dTdp2 * get_var_rho(t);
}

// Accessors for neighbors.
void stoch_edge::add_neighbor(int _neighbor)
{
   if (find(neighbors.begin(), neighbors.end(), _neighbor) == neighbors.end())
      neighbors.push_back(_neighbor);
}

int stoch_edge::get_neighbor(int index) const
{
  if (index < neighbors.size())
    return neighbors[index];
  else
  {
     std::cerr << "Index is beyond neighbors range." << endl;
  }
}

int stoch_edge::neighbors_size() const
{
  return neighbors.size();
}

// Sets the fundamental diagram.
void stoch_edge::set_den_to_vel(const den_to_vel& fund) {
   fund_ = fund;
}

// Gets the fundamental diagram.
den_to_vel stoch_edge::get_den_to_vel() const {
   return fund_;
}

den_to_vel& stoch_edge::retrieve_den_to_vel() {
   return fund_;
}

int stoch_edge::get_lane_count() {
   return lane_count;
}

void stoch_edge::set_lane_count(int count) {
   lane_count = count;
}

int stoch_edge::get_osm_id() const {
   return osm_id_;
}

void stoch_edge::set_osm_id(int osm_id) {
   osm_id_ = osm_id;
}

string stoch_edge::get_sumo_id() const {
   return sumo_id_;
}

void stoch_edge::set_sumo_id(const string& sumo_id) {
   sumo_id_ = sumo_id;
}

// TODO I think this is adding a single routed car into the estimate.
void stoch_edge::update(normal A, normal D, const den_to_vel& fund)
{
   float density_threshhold = 0.05;

   // TODO HACK
   A = normal(A.mean(), rho_std_ * A.mean());
   D = normal(D.mean(), rho_std_ * D.mean());

   // Update backwards until a condition is reached.
   float start_t = A.mean();
   int start_bin = get_bin_id(start_t);
   for (int i = start_bin; i > 0; i--) {
      float t = i*delta_t;

      // Update mean
      float rho_i = rho_[i].mean();

      // These are clearly not independent.
      float car_prob = cdf(A,t)*(1 - cdf(D,t));
      float q_i = car_prob/len;

      if (car_prob < density_threshhold)
         break;

      // Scales additional density by lane_count and adds it to rho_i.
      rho_i = rho_i + (q_i / lane_count);
      // TODO Took out lane count as it was not used well in the simulation.
      // rho_i = rho_i + q_i;

      float del_rho_mean = rho_i - rho_[i].mean();

      //TODO Update variance
      // float rho_i_v = pow(rho_[i].standard_deviation(), 2);
      // float del_rho_var = q_i*(1 - q_i)/pow(len, 2);
      // rho_i_v += del_rho_var;

      //Store
      // rho_[i] = normal(rho_i, std::sqrt(rho_i_v));
      rho_[i] = normal(rho_i, std::max(rho_i * rho_std_, 0.01f));
   }

   // Now update forwards until a condition is reached.
   // WARNING: possibly infinite.
   int i = start_bin + 1;
   float car_prob = 0;
   do {
      float t = i*delta_t;

      // Update mean
      float rho_i = rho_[i].mean();

      // These are clearly not independent.
      car_prob = cdf(A,t)*(1 - cdf(D,t));
      float q_i = car_prob/len;

      if (car_prob < density_threshhold)
         break;

      // Scales additional density by lane_count and adds it to rho_i.
      //    rho_i = rho_i + (q_i / lane_count);
      // TODO Took out lane count as it was not used well in the simulation.
      rho_i = rho_i + q_i;

      float del_rho_mean = rho_i - rho_[i].mean();

      //TODO Update variance
      if (ADDITIVE_VARIANCE) {
         float rho_i_v = pow(rho_[i].standard_deviation(), 2);
         float del_rho_var = q_i*(1 - q_i)/pow(len, 2);
         rho_i_v += del_rho_var;
         rho_[i] = normal(rho_i, std::sqrt(rho_i_v));
      }
      else {
         rho_[i] = normal(rho_i, std::max(rho_i * rho_std_, 0.01f));
      }

      i++;
   } while (car_prob > density_threshhold);

};

float stoch_edge::distance_to_shape(double x, double y) {
   point query(x, y);

   // TODO Clean up.
   float min_distance = 9999999;

   for (int i = 0; i < shape_.size() - 1; i++) {
      float distance = distance_to_segment(query, shape_[i], shape_[i + 1]);

      if (distance < min_distance) {
         min_distance = distance;
      }
   }

   return min_distance;
}

float length_squared(point point_a, point point_b) {
   return pow((point_a.x - point_b.x), 2) + pow((point_a.y - point_b.y), 2);
}

float distance(point point_a, point point_b) {
   return sqrt(pow((point_a.x - point_b.x), 2)
               + pow((point_a.y - point_b.y), 2));
}

// TODO Rewrite using real vector library.
float stoch_edge::distance_to_segment(point _query,
                                      point _point_a,
                                      point _point_b) {

   // TODO Rewrite only using Eigen.
   Eigen::Vector2d query, point_a, point_b;
   query[0] = _query.x;
   query[1] = _query.y;

   point_a[0] = _point_a.x;
   point_a[1] = _point_a.y;

   point_b[0] = _point_b.x;
   point_b[1] = _point_b.y;

   // The segent legth.
   const float l2 = length_squared(_point_a, _point_b);

   // Handles the degenerate case of a zero-sized line segment.
   if (l2 == 0.0) return distance(_query, _point_a);

   // Consider the line extending the segment, parameterized as v + t (w - v).
   // We find projection of point p onto the line.
   // It falls where t = [(p-v) . (w-v)] / |w-v|^2
   const float t = (query - point_a).dot(point_b - point_a) / l2;

   // Beyond the 'point_a' end of the segment
   if (t < 0.0) return distance(_query, _point_a);
   // Beyond the 'w' end of the segment
   else if (t > 1.0) return distance(_query, _point_b);

   // Else the projection falls on the segment
   const Eigen::Vector2d projection = point_a + t * (point_b - point_a);
   return (query - projection).norm();
}

// Pushes back a point onto the shape_ vector.
void stoch_edge::add_point(point p) {
   shape_.push_back(p);
}

// A speed limit interface that returs the value from the den_to_vel class.
float stoch_edge::get_speedlimit() const {
   return get_den_to_vel().get_v_max();
}

// The time a car needs to wait to enter this lane before it can enter the
// lane due to congestion.
float stoch_edge::get_delay_time() {
   return delay_time_;
}

// The time a car needs to wait to enter this lane before it can enter the
// lane due to congestion.
void stoch_edge::set_delay_time(float delay_time) {
   delay_time_ = delay_time;
}
