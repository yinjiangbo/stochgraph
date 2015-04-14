#include <limits>
#include <map>
#include <functional>
#include <list>

#include "stoch_graph.hpp"
#include "sumo_util.hpp"
#include "local_frame.hpp"

using std::multimap;
using std::list;
using std::endl;
using std::pair;
using std::make_pair;
using std::iterator;
using std::map;

#define kFloatMax std::numeric_limits<float>::max();

const bool kStochGraphVerboseDebug = false;

stoch_graph::stoch_graph(const stoch_graph& other)
{
  for (int i = 0; i < other.edges.size(); i++)
  {
    edges.push_back(other.edges[i]);
  }
}

struct _normal
{
  float mean;
  float var;
};

// Updates the density for each edge along a path.
// TODO Does not seem to be used.
/*void stoch_graph::update_for_path(vector<int> path, normal Ti)
{
  //Starting time delay
  _normal A;
  A.mean = Ti.mean();
  A.var = pow(Ti.standard_deviation(), 2);

  for (int i = 0; i < path.size(); i++)
  {
    //Load stats of lane for arrival time calc
    normal A_i(A.mean, A.var);

    //Update arrival time for next edge
    float vel;
    edge& edge_i = edges[path[i]];
    vel = edge_i.get_den_to_vel()(edges[path[i]].get_mean_rho(A_i.mean()));
    A.mean += edges[path[i]].get_len() / vel;

    //TODO derive variance update;
    normal D(A.mean, A.var);

    //Update edge for all bins
    edges[path[i]].update(A_i, D, edges[path[i]].get_den_to_vel());
  }
  }*/

// Updates each edge's density using path segments with start and end times.
void stoch_graph::update_fixed(vector<util_path_element> path)
{
  for (int i = 0; i < path.size(); i++)
  {
    //Update edge for all bins
    edges[path[i].id].update(path[i].A, path[i].D, edges[path[i].id].get_den_to_vel());
  }
}

class edge_key
{
public:
   bool operator<(const edge_key& b) const
   {
      return cost < b.cost;
   }

   edge_key(){}

   edge_key(int _id, float _cost, multimap<float, map<int, edge_key>::iterator>::iterator _init_p)
   {
      id         = _id;
      cost       = _cost;
      cost_to_here = std::numeric_limits<float>::max();
      position = _init_p;
      len = std::numeric_limits<float>::max();
      distance = std::numeric_limits<float>::max();
   }

   // TODO should be private.
public:
   multimap<float, map<int, edge_key>::iterator>::iterator position;
   int   id;
   float cost;
   float len;
   float cost_to_here;
   float T_mean;
   float T_var;
   int   pred;
   float distance;
};

void stoch_graph::initialize_to_empty_roads(float delta_t, const int size) {
   this->delta_t_ = delta_t;

   for (int i = 0; i < edges.size(); i++) {
      stoch_edge& e = edges[i];
      if (e.retrieve_den_to_vel().get_v_max() < 0) {
         std::cerr << "The function 'initialize_to_empty_roads' was called with"
                   << " an edge that had an uninitialized den_to_vel.\n";
         assert(0);
      }

      vector<float> mean_rho(size, 0.0);
      vector<float> var_rho(size, 0.0001);
      e.initialize_rho(delta_t, mean_rho, var_rho);

      // vector<float> mean_v(size, e.retrieve_den_to_vel().get_v_max());
      // vector<float> var_v(size, 0.0001);
      // e.initialize_vel(delta_t, mean_v, var_v);
   }
}

void stoch_graph::initialize_den_to_vel(float v_max, float rho_max) {
   den_to_vel fund;
   fund.set_v_max(v_max);
   fund.set_rho_max(rho_max);

   for (int i = 0; i < edges.size(); i++) {
      edges[i].set_den_to_vel(fund);
   }
}

void stoch_graph::set_max_rho(float max_rho) {
   for (int i = 0; i < edges.size(); i++) {
      edges[i].retrieve_den_to_vel().set_rho_max(max_rho);
   }
}

class key_comp
{
public:
   bool operator()(edge_key*& a, edge_key*& b)
   {
      return a->cost < b->cost;
   }
};

// Cost estimator for A* path planner using Euclidean distance..
float time_lower_bound(const stoch_edge& start, const stoch_edge& goal)
{
   //TODO This value should not be constant. (?)
  float MAX_V   = 13;
  float h_dist  = std::sqrt(pow(start.end.x - goal.start.x, 2) + pow(start.end.y - goal.start.y, 2));
  float h_max_v = MAX_V;
  return h_dist / h_max_v;
}

float euclid_distance(const stoch_edge& start, const stoch_edge& goal)
{
  float h_dist  = std::sqrt(pow(start.end.x - goal.start.x, 2) + pow(start.end.y - goal.start.y, 2));
  return h_dist;
}

// A helper iterator definition.
typedef map<int, edge_key>::iterator key_it;

// A* planner for a start, goal, start tim, through a stochastic road network.
/*
vector<util_path_element> stoch_graph::plan_for(int s, int g, float t, float& travel_time)
{
   // Weight factor for variance.
   float omega = 1;

   // Used to keep track of open edges and their costs.
   multimap<float, map<int, edge_key>::iterator> open;

   std::map<int, edge_key> keys;

   const stoch_edge& goal    = edges[g];
   const stoch_edge& start   = edges[s];

   // Initial edge and data.
   edge_key init(s, 0 + euclid_distance(start, goal), open.end());
   init.cost_to_here = 0;
   init.T_mean       = t;
   init.T_var        = 0.01;
   init.pred         = s;

   // Add the initial key into the map and multimap.
   keys.insert(make_pair(s, init));
   keys.find(s)->second.position = open.insert(
      pair<float, map<int, edge_key>::iterator>(
         init.cost,
         keys.find(s)));

   // Searches for a path with minimal cost.
   bool found = false;
   while (open.size() > 0)
   {
      // Gets the edge key with the lowest cost.
      edge_key& this_key = open.begin()->second->second;

      // Retrieve the edge from the key.
      stoch_edge& this_edge = edges[this_key.id];

      // Remove the edge from the open set.
      open.erase(open.begin());
      this_key.position = open.end();

      // Check to see if the path reaches the goal.
      if (this_key.id == g)
      {
         found = true;
         break;
      }

      //Gets successors and adds them to open if appropriate.
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         // Gets the neighboring edge.
         int id = this_edge.get_neighbor(i);
         const stoch_edge& next_edge = edges[id];

         // Calculates the heuristic cost to the goal.
         float h_s = euclid_distance(next_edge, goal);

         // Adds the cost to here.
         // TODO average time?
         float u = next_edge.T(this_key.T_mean, fund);

         if (id == g)
         {
            h_s = 0;
            u = 0;
         }

         // TODO standard deviation?
         float sig = next_edge.Ttilda(this_key.T_mean, fund);

         // Load or create key for the new edge..
         // Check if next_key exists.
         if (keys.find(id) == keys.end()) {
            keys.insert(make_pair(id, edge_key(id, std::numeric_limits<float>::max(), open.end())));
         }
         edge_key& next_key = (keys.find(id)->second);

         // TODO This probably checks to see if the next edge is already in the set
         // with a lower cost.
         if (this_key.cost_to_here + u + omega*sig < next_key.cost_to_here) {

            // Update the next_key values.
            // TODO This should be done through a function.
            next_key.cost_to_here = this_key.cost_to_here + u + omega*sig;
            next_key.T_mean       = this_key.T_mean + u;
            next_key.T_var        = this_key.T_var + sig;
            next_key.pred         = this_key.id;
            next_key.cost = next_key.cost_to_here + euclid_distance(next_edge, goal);

            // Inserts the new edge into open.
            // If the edge was previously there, it deletes the former version.
            float cost = next_key.cost;
            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);

               // TODO Insertion should only happen in one place.
               keys.find(id)->second.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   // Found indicates a path has been found that reaches the goal.
   if (found)
   {
      // Used to hold the path that was found.
      vector<util_path_element> path;

      // Retrieve the ending segment of the path.
      util_path_element elem;
      elem.id = g;
      elem.A = normal(keys[keys[g].pred].T_mean, keys[keys[g].pred].T_var);
      elem.D = normal(keys[g].T_mean, keys[g].T_var);
      path.push_back(elem);

      // TODO What in the world is this doing?
      travel_time = keys.find(keys.find(g)->second.pred)->second.T_mean - t;

      // Walks back through the path until reaching the start, adding segments.
      int last_id  = g;
      int id = g;
      while (id != s)
      {
         // Retrieves the id of the segment leading to the current edge.
         id = keys.find(id)->second.pred;

         //         const stoch_edge& foo = edges[last_id];

         // Creates a path element for the edge leading to the current edge.
         util_path_element elem;
         elem.id = id;
         elem.A = normal(keys[keys[id].pred].T_mean, keys[keys[id].pred].T_var);
         elem.D = normal(keys[id].T_mean, keys[id].T_var);

         // Add the incoming segment to the path.
         path.push_back(elem);
      }
      // vector<util_path_element> foo(path.rbegin(), path.rend());
      // assert(path.size() == foo.size());
      return vector<util_path_element>(path.rbegin(), path.rend());
   }

   return vector<util_path_element>();
}*/

// Heuristic using only distance, not velocity.
float h_distance_only(const stoch_edge& start, const stoch_edge& goal)
{
   float h_dist  = std::sqrt(pow(start.end.x - goal.start.x, 2) + pow(start.end.y - goal.start.y, 2));
   return h_dist;
}

/*
// Plans a path through the stochastic graph using _only_ the shortest distance.
vector<util_path_element> stoch_graph::plain_plan_for(int s, int g, float t, float& travel_time)
{
   // Weight given to the variance cost.
   float omega = 1;

   // This holds the open edges under consideration for expansion, ordered by cost.
   multimap<float, map<int, edge_key>::iterator> open;

   // A map of all the edges.
   std::map<int, edge_key> keys;

   // Start and goal edges.
   const stoch_edge& goal    = edges[g];
   const stoch_edge& start   = edges[s];

   // Creates the inital edge.
   edge_key init(s, 0 + h_distance_only(start, goal), open.end());
   init.cost_to_here = 0;
   init.T_mean       = t;
   init.T_var        = 0.01;
   init.pred         = s;
   init.len          = 0;

   // Inserts the initial edge into the edge map and into the 'open' multimap.
   keys.insert(make_pair(s, init));
   keys.find(s)->second.position = open.insert(pair<float, map<int, edge_key>::iterator>(init.cost, keys.find(s)));

   // Traverses the edges until a goal is found using A* algorithm.
   bool found = false;
   while (open.size() > 0)
   {
      // Gets the current best edge from 'open'.
      edge_key& this_key = open.begin()->second->second;
      stoch_edge& this_edge = edges[this_key.id];

      // Removes the edge from the open set.
      open.erase(open.begin());
      this_key.position = open.end();

      // Checks to see if the edge is the goal.
      if (this_key.id == g)
      {
         found = true;
         break;
      }

      // Gets all the successor edges and inserts them into 'open' if appropriate.
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         // Get ith neighbor.
         int id = this_edge.get_neighbor(i);
         const stoch_edge& next_edge = edges[id];

         // Calculates the heuristic cost to the goal.
         // TODO shouldn't this use h_distance_only?
         float h_s = euclid_distance(next_edge, goal);

         // This is the cost to get to this point. (mean travel time)
         float u = next_edge.T(this_key.T_mean, fund);

         // If the edge is the goal, makes the costs 0.
         if (id == g)
         {
            h_s = 0;
            u = 0;
         }

         // This is the time variance to get to this edge.
         float sig = next_edge.Ttilda(this_key.T_mean, fund);

         //Load or create key.
         //Check if next_key exists.
         if (keys.find(id) == keys.end())
         {
            keys.insert(make_pair(id, edge_key(id, std::numeric_limits<float>::max(), open.end())));
         }
         edge_key& next_key = (keys.find(id)->second);

         // Check to see if the distance to the neighbor is smaller using this new edge.
         // TODO This is one of the aspects that changes here.  Should be refactored.
         if (this_key.len + next_edge.get_len() < next_key.len)
         {
            next_key.len = this_key.len + next_edge.get_len();
            next_key.cost_to_here = this_key.cost_to_here + u + omega*sig;
            next_key.T_mean       = this_key.T_mean + u;
            next_key.T_var        = this_key.T_var + sig;
            next_key.pred         = this_key.id;
            next_key.cost = next_key.len + h_distance_only(next_edge, goal);
            float cost = next_key.cost;
            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);
               keys.find(id)->second.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   if (found)
   {
      int id = g;
      vector<util_path_element> path;

      util_path_element elem;
      elem.id = g;
      elem.A = normal(keys[keys[g].pred].T_mean, keys[keys[g].pred].T_var);
      elem.D = normal(keys[g].T_mean, keys[g].T_var);
      path.push_back(elem);


      travel_time = keys.find(keys.find(g)->second.pred)->second.T_mean - t;
      int last_id  = g;
      while (id != s)
      {
         id = keys.find(id)->second.pred;
         const stoch_edge& foo = edges[last_id];

         util_path_element elem;
         elem.id = id;
         elem.A = normal(keys[keys[id].pred].T_mean, keys[keys[id].pred].T_var);
         elem.D = normal(keys[id].T_mean, keys[id].T_var);
         path.push_back(elem);

      }
      vector<util_path_element> foo(path.rbegin(), path.rend());
      assert(path.size() == foo.size());
      return vector<util_path_element>(path.rbegin(), path.rend());
   }

   return vector<util_path_element>();
}

// Plans a route through the stochastic network using only estimated 'historical' data.
vector<util_path_element> stoch_graph::stoch_plan_for(int s, int g, float t, float& travel_time, stoch_graph history)
{
   float omega = 1; //TODO

   //priority_queue<edge_key*, vector<edge_key*>, key_comp> open;
   multimap<float, map<int, edge_key>::iterator> open;

   std::map<int, edge_key> keys;

   const stoch_edge& goal    = edges[g];
   const stoch_edge& start   = edges[s];

   edge_key init(s, 0 + euclid_distance(start, goal), open.end());
   init.cost_to_here = 0;
   init.T_mean       = t;
   init.T_var        = 0.01;
   init.pred         = s;
   init.len          = 0;

   keys.insert(make_pair(s, init));
   keys.find(s)->second.position = open.insert(pair<float, map<int, edge_key>::iterator>(init.cost, keys.find(s)));

   bool found = false;

   while (open.size() > 0)
   {
      edge_key& this_key = open.begin()->second->second;

      stoch_edge& this_edge = edges[this_key.id];

      open.erase(open.begin());
      this_key.position = open.end();

      if (this_key.id == g)
      {
         found = true;
         break;
      }

      //Get successors
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         int id = this_edge.get_neighbor(i);

         const stoch_edge& next_edge = edges[id];

         //heuristic
         float h_s = euclid_distance(next_edge, goal);

         //cost of here
         float u = next_edge.T(this_key.T_mean, fund);

         // TODO one of the aspects that's different here.  Should be refactored.
         float est_u = history.edges[id].T(this_key.T_mean, fund);

         if (id == g)
         {
            h_s = 0;
            u = 0;
         }

         float sig = next_edge.Ttilda(this_key.T_mean, fund);

         // TODO one of the aspects that's different here.  Should be refactored.
         float est_sig = history.edges[id].Ttilda(this_key.T_mean, fund);

         //Load or create key.
         //Check if next_key exists.
         if (keys.find(id) == keys.end())
         {
            keys.insert(make_pair(id, edge_key(id, std::numeric_limits<float>::max(), open.end())));
         }

         edge_key& next_key = (keys.find(id)->second);

         if (this_key.cost_to_here + est_u + omega*est_sig < next_key.cost_to_here)
         {
            next_key.cost_to_here = this_key.cost_to_here + est_u + omega*est_sig;
            next_key.T_mean       = this_key.T_mean + u;
            next_key.T_var        = this_key.T_var + sig;
            next_key.pred         = this_key.id;
            next_key.cost = next_key.cost_to_here + euclid_distance(next_edge, goal);

            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);
               keys.find(id)->second.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   if (found)
   {
      int id = g;
      vector<util_path_element> path;

      util_path_element elem;
      elem.id = g;
      elem.A = normal(keys[keys[g].pred].T_mean, keys[keys[g].pred].T_var);
      elem.D = normal(keys[g].T_mean, keys[g].T_var);
      path.push_back(elem);


      travel_time = keys.find(keys.find(g)->second.pred)->second.T_mean - t;
      int last_id  = g;
      while (id != s)
      {
         id = keys.find(id)->second.pred;
         const stoch_edge& foo = edges[last_id];

         util_path_element elem;
         elem.id = id;
         elem.A = normal(keys[keys[id].pred].T_mean, keys[keys[id].pred].T_var);
         elem.D = normal(keys[id].T_mean, keys[id].T_var);
         path.push_back(elem);

      }
      vector<util_path_element> foo(path.rbegin(), path.rend());
      assert(path.size() == foo.size());
      return vector<util_path_element>(path.rbegin(), path.rend());
   }

   return vector<util_path_element>();
}*/

// This planner is only for tuning the travel time estimate.
vector<util_path_element> stoch_graph::plan_for_tuning(int start_id,
                                                       int goal_id,
                                                       float start_time,
                                                       float& travel_time,
                                                       std::function<
                                                       float(const stoch_edge&,
                                                             const stoch_edge&)>
                                                       heuristic,
                                                       float variance_weight = 1) {

   /*
   // Used to keep track of open edges and their costs.
   multimap<float, map<int, edge_key>::iterator> open;
   // Maps an edge id to the edge_key, which holds cost information.
   std::map<int, edge_key> keys;

   // TODO Used for testing.
   float max_density = std::numeric_limits<float>::min();

   // The start and goal edges.
   stoch_edge& goal    = edges[goal_id];
   stoch_edge& start   = edges[start_id];

   // Create the initial edge key.
   edge_key init(start_id, 0 + heuristic(start, goal), open.end());
   init.cost_to_here = 0;
   init.T_mean       = start.T(start_time, start.get_den_to_vel()) + start_time;
   init.T_var        = start.Ttilda(start_time, start.get_den_to_vel());
   init.pred         = start_id;
   init.distance     = start.get_len();

   // Add the initial key into the map and multimap.
   keys.insert(make_pair(start_id, init));
   auto multimap_pair = pair<float, map<int, edge_key>::iterator>(
      init.distance,
      keys.find(start_id));
   keys.find(start_id)->second.position = open.insert(multimap_pair);

   // Searches for a path with minimal cost.
   bool found = false;
   while (open.size() > 0)
   {
      // Gets the edge key with the lowest cost.
      edge_key& this_key = open.begin()->second->second;

      // Retrieve the edge from the key.
      stoch_edge& this_edge = edges[this_key.id];

      if (this_edge.get_mean_rho(this_key.T_mean) > max_density)
         max_density = this_edge.get_mean_rho(this_key.T_mean);

      // Remove the edge from the open set.
      open.erase(open.begin());
      this_key.position = open.end();

      // Check to see if the path reaches the goal.
      if (this_key.id == goal_id)
      {
         found = true;
         break;
      }

      // Gets successors and adds them to open if appropriate.
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         // Gets the neighboring edge.
         int id = this_edge.get_neighbor(i);
         stoch_edge& next_edge = edges[id];

         // Calculates the heuristic cost to the goal.
         float h_s = heuristic(next_edge, goal);

         // The time to traverse this edge.
         float u = next_edge.T(this_key.T_mean, next_edge.get_den_to_vel());

         // Accounts for estimated intersection delay.
         if (next_edge.neighbors_size() > 0) {

            // We calculate the out flow,
            // q = p * v(p),
            // which is the density dependent velocity times the density.

            // The outflow is defined assuming an 'average' choice of lane.

#define WORST

#ifdef WORST
            float neighbor_min_vel = std::numeric_limits<float>::max();
            for (int j = 0; j < next_edge.neighbors_size(); j++) {
               const stoch_edge& neighbor = edges[next_edge.get_neighbor(j)];
               float neighbor_vel = neighbor.get_den_to_vel()
                                    .v(neighbor.get_mean_rho(this_key.T_mean));
               if (neighbor_vel < neighbor_min_vel)
                  neighbor_min_vel = neighbor_vel;
            }

            float out_flow = next_edge.get_mean_rho(this_key.T_mean)
               * neighbor_min_vel;
#endif

#ifdef BEST
            float neighbor_max_vel = std::numeric_limits<float>::min();
            for (int j = 0; j < next_edge.neighbors_size(); j++) {
               const stoch_edge& neighbor = edges[next_edge.get_neighbor(j)];
               float neighbor_vel = neighbor.get_den_to_vel().v(neighbor.get_mean_rho(this_key.T_mean));
               if (neighbor_vel > neighbor_max_vel)
                  neighbor_max_vel = neighbor_vel;
            }

            float out_flow = next_edge.get_mean_rho(this_key.T_mean)
               * neighbor_max_vel;
#endif


#ifdef AVG
            float neighbor_vels = 0;
            for (int j = 0; j < next_edge.neighbors_size(); j++) {
               const stoch_edge& neighbor = edges[next_edge.get_neighbor(j)];
               float neighbor_vel = neighbor.get_den_to_vel().v(neighbor.get_mean_rho(this_key.T_mean));
               neighbor_vels += neighbor_vel;
            }
            neighbor_vels /= next_edge.neighbors_size();

            float out_flow = next_edge.get_mean_rho(this_key.T_mean)
               * neighbor_vels;
#endif

            // Now, we calculate the current queue length, the cars in the lane,
            // c = p * len(edge),
            // which is the density (c/m) times the length of the road (m)
            float cars = next_edge.get_len()
                         * next_edge.get_mean_rho(this_key.T_mean);

            // Now we can calculate the time required to empty the road
            // assuming smooth flow,
            // t = c / q,
            // the number of cars (c) divided by the rate they leave (s/c)
            float queue_time;
            if (out_flow > 0)
               queue_time = cars / out_flow;
            else if (next_edge.get_mean_rho(this_key.T_mean) > 0) {
               // Assigns som arbitrary, large value.
               queue_time = 180;
            }
            else {
               queue_time = 0;
            }

            if (next_edge.has_intersection) {
               // The number of delay cycles is then calculated
               // n = t / a,
               // where a is the effective green light time.
               // TODO figure out/tune effective green light.
               float cycle_length = 35;

               // TODO tuning start up time.
               //   12 -- Over estimated a considerable amount (-3k).
               //    5 -- mu is (-2k)
               float start_up_time = 10;
               float effective_green_light = cycle_length - start_up_time;
               float cycles = queue_time / effective_green_light;

               // Add uniform probability of delay, regardless of flow.
               cycles += 0.5;

               // Estimates how many independent pairings of roads there are
               // based solely on the number of neighbors.
               int lane_pairs = std::ceil(
                  (next_edge.neighbors_size() - 4) / 2.0) + 2;
               float full_cycle_length = cycle_length * lane_pairs;

               // Now we calculate the final delay,
               // n * d,
               // where n is the number of cycles and d is the delay per cycle.
               float delay = cycles * full_cycle_length;

               u += 2 * delay;
            } else {
               u += queue_time;
            }
         }

         if (id == goal_id)
         {
            h_s = 0;
         }

         // TODO standard deviation?
         float sig =
            next_edge.Ttilda(this_key.T_mean, next_edge.get_den_to_vel());

         // Load or create key for the new edge..
         // Check if next_key exists.
         if (keys.find(id) == keys.end()) {
            keys.insert(
               make_pair(
                  id,
                  edge_key(id, std::numeric_limits<float>::max(), open.end())));
         }
         edge_key& next_key = (keys.find(id)->second);

         // TODO This probably checks to see if the next edge already exists
         // with a lower cost.
         float new_cost = this_key.distance + next_edge.get_len();
         if (new_cost < next_key.distance) {

            // Update the next_key values.
            // TODO This should be done through a function.
            next_key.cost_to_here = new_cost;
            next_key.T_mean       = this_key.T_mean + u;
            next_key.T_var        = this_key.T_var + sig;
            next_key.pred         = this_key.id;
            next_key.distance = new_cost;
            next_key.cost = next_key.distance + heuristic(next_edge, goal);

            // Inserts the new edge into open.
            // If the edge was previously there, it deletes the former version.
            float cost = next_key.cost;
            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);

               // TODO Insertion should only happen in one place.
               keys.find(id)->second.position = open.insert(
                  pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   // Found indicates a path has been found that reaches the goal.
   if (found)
   {
      // Used to hold the path that was found.
      vector<util_path_element> path;

      // TODO What in the world is this doing?
      travel_time = keys.find(keys.find(goal_id)->second.pred)->second.T_mean - start_time;

      // Walks back through the path until reaching the start, adding segments.
      int last_id  = goal_id;
      int id = goal_id;

      do {
         // Creates a path element for the edge leading to the current edge.
         util_path_element elem;
         elem.id = id;

         // TODO Is the start case handled here or below?
         if (id == start_id)
            elem.A = normal(start_time, 0.001);
         else
            elem.A = normal(keys[keys[id].pred].T_mean,
                            keys[keys[id].pred].T_var);
         elem.D = normal(keys[id].T_mean, keys[id].T_var);

         // Add the incoming segment to the path.
         path.push_back(elem);

         // Retrieves the id of the segment leading to the current edge.
         id = keys.find(id)->second.pred;

      } while (id != start_id);

      util_path_element elem;
      elem.id = start_id;

      elem.A = normal(start_time, 0.001);
      elem.D = normal(keys[id].T_mean, keys[id].T_var);

      // Add the incoming segment to the path.
      path.push_back(elem);

      // Retrieves the id of the segment leading to the current edge.
      id = keys.find(id)->second.pred;

      //      std::cout << "Max density was " << max_density << std::endl;

      return vector<util_path_element>(path.rbegin(), path.rend());
   }
   */
   return vector<util_path_element>();
}

// Accounts for estimated intersection delay.
float stoch_graph::get_intersection_time(float u,
                                         float t_current,
                                         stoch_edge& next_edge) {
   // Accounts for estimated intersection delay.
   if (next_edge.neighbors_size() > 0) {

      // We calculate the out flow,
      // q = p * v(p),
      // which is the density dependent velocity times the density.

      // The outflow is defined assuming an 'average' choice of lane.

#define WORST

#ifdef WORST
      float neighbor_min_vel = std::numeric_limits<float>::max();
      for (int j = 0; j < next_edge.neighbors_size(); j++) {
         const stoch_edge& neighbor = edges[next_edge.get_neighbor(j)];
         float neighbor_vel = neighbor.get_den_to_vel()
            .v(neighbor.get_mean_rho(t_current));
         if (neighbor_vel < neighbor_min_vel)
            neighbor_min_vel = neighbor_vel;
      }

      float out_flow = next_edge.get_den_to_vel().get_rho_max()
         * neighbor_min_vel;
#endif

#ifdef BEST
      float neighbor_max_vel = std::numeric_limits<float>::min();
      for (int j = 0; j < next_edge.neighbors_size(); j++) {
         const stoch_edge& neighbor = edges[next_edge.get_neighbor(j)];
         float neighbor_vel = neighbor.get_den_to_vel()
            .v(neighbor.get_mean_rho(t_current));
         if (neighbor_vel > neighbor_max_vel)
            neighbor_max_vel = neighbor_vel;
      }

      float out_flow = next_edge.get_den_to_vel().get_rho_max()
         * neighbor_max_vel;
#endif


#ifdef AVG
      float neighbor_vels = 0;
      for (int j = 0; j < next_edge.neighbors_size(); j++) {
         const stoch_edge& neighbor = edges[next_edge.get_neighbor(j)];
         float neighbor_vel = neighbor.get_den_to_vel().v(
            neighbor.get_mean_rho(t_current));
         neighbor_vels += neighbor_vel;
      }
      neighbor_vels /= next_edge.neighbors_size();

      float out_flow = next_edge.get_den_to_vel().get_rho_max()
         * neighbor_vels;
#endif

      // Now, we calculate the current queue length, the cars in the lane,
      // c = p * len(edge),
      // which is the density (c/m) times the length of the road (m)
      float cars = next_edge.get_len()
         * next_edge.get_mean_rho(t_current);

      // Now we can calculate the time required to empty the road
      // assuming smooth flow,
      // t = c / q,
      // the number of cars (c) divided by the rate they leave (s/c)
      float queue_time;
      if (out_flow > 0)
         queue_time = cars / out_flow;
      else if (cars > 0) {
         // Assigns som arbitrary, large value.
         queue_time = 180;
      }
      else {
         queue_time = 0;
      }

      //            if (next_edge.has_intersection) {
      if (true) {
         // The number of delay cycles is then calculated
         // n = t / a,
         // where a is the effective green light time.
         // TODO figure out/tune effective green light.
         float cycle_length = 35;

         // TODO tuning start up time.
         //   12 -- Over estimated a considerable amount (-3k).
         //    5 -- mu is (-2k)
         float start_up_time = 5;
         float effective_green_light = cycle_length - start_up_time;
         float cycles = queue_time / effective_green_light;

         // Estimates how many independent pairings of roads there are
         // based solely on the number of neighbors.
         int lane_pairs = 2
            + std::ceil(std::max(0, (next_edge.neighbors_size() - 4)) / 2.0);
         float full_cycle_length = cycle_length * lane_pairs;

         // std::cout << "full cycle: " << full_cycle_length  << std::endl
         //           << "cycle: " << cycle_length  << std::endl
         //           << "lanes: " << lane_pairs  << std::endl
         //           << "neighbors: " << next_edge.neighbors_size()  << std::endl;

         // Now we calculate the final delay,
         // n * d,
         // where n is the number of cycles and d is the delay per cycle.
         float delay = cycles * full_cycle_length;

         // Add uniform probability of delay, regardless of flow.
         float light_constant = (1 - (1/lane_pairs))
            * (full_cycle_length - cycle_length) * (0.5);

         u = std::max(delay, u) + light_constant;
      } else {
         u = std::max(queue_time, u);
      }
   }

   return u;
}

// An interface to a_star_plan that sets the value for use_new_models to true.
vector<util_path_element> stoch_graph::a_star_plan(int start_id,
                                                   int goal_id,
                                                   float start_time,
                                                   float& travel_time,
                                                   std::function<
                                                      float(const stoch_edge&,
                                                            const stoch_edge&)>
                                                      heuristic,
                                                   float variance_weight = 1) {
   return a_star_plan(start_id,
                      goal_id,
                      start_time,
                      travel_time,
                      heuristic,
                      variance_weight,
                      true);
}


// The self-aware planner.
vector<util_path_element> stoch_graph::a_star_plan(int start_id,
                                                   int goal_id,
                                                   float start_time,
                                                   float& travel_time,
                                                   std::function<
                                                      float(const stoch_edge&,
                                                            const stoch_edge&)>
                                                      heuristic,
                                                   float variance_weight = 1,
                                                   bool use_new_models = true) {
   int edges_opened_count = 0;

   // Used to keep track of open edges and their costs.
   multimap<float, map<int, edge_key>::iterator> open;
   // Maps an edge id to the edge_key, which holds cost information.
   std::map<int, edge_key> keys;

   // TODO Used for testing.
   float max_density = std::numeric_limits<float>::min();

   // The start and goal edges.
   stoch_edge& goal    = edges[goal_id];
   stoch_edge& start   = edges[start_id];

   // Create the initial edge key.
   edge_key init(start_id, 0 + heuristic(start, goal), open.end());
   init.cost_to_here = 0;
   init.T_mean       = start.T(start_time,
                               start.get_den_to_vel(),
                               use_new_models) + start_time;
   init.T_var        = start.Ttilda(start_time, start.get_den_to_vel());
   init.pred         = start_id;

   if (use_new_models) {
      init.T_mean = get_intersection_time(init.T_mean, start_time, start);
   }

   // Add the initial key into the map and multimap.
   keys.insert(make_pair(start_id, init));
   auto multimap_pair = pair<float, map<int, edge_key>::iterator>(init.cost, keys.find(start_id));
   keys.find(start_id)->second.position = open.insert(multimap_pair);

   // Searches for a path with minimal cost.
   bool found = false;
   while (open.size() > 0)
   {
      edges_opened_count++;

      // Gets the edge key with the lowest cost.
      edge_key& this_key = open.begin()->second->second;

      // Retrieve the edge from the key.
      stoch_edge& this_edge = edges[this_key.id];

      // Used for testing to determine the maximum density a car encounters.
      if (this_edge.get_mean_rho(this_key.T_mean) > max_density)
         max_density = this_edge.get_mean_rho(this_key.T_mean);

      // Remove the edge from the open set.
      open.erase(open.begin());
      this_key.position = open.end();

      // Check to see if the path reaches the goal.
      if (this_key.id == goal_id)
      {
         found = true;
         break;
      }

      // If the time is out of the bins, finish with this edge.
      if (!this_edge.in_bins(this_key.T_mean))
         continue;

      // Gets successors and adds them to open if appropriate.
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         // Gets the neighboring edge.
         int id = this_edge.get_neighbor(i);
         stoch_edge& next_edge = edges[id];

         // Calculates the heuristic cost to the goal.
         float h_s = heuristic(next_edge, goal);

         // The time to traverse this edge.
         float u = next_edge.T(this_key.T_mean,
                               next_edge.get_den_to_vel(),
                               use_new_models);

         if (use_new_models) {
            u = get_intersection_time(u, this_key.T_mean, next_edge);
         }

         if (id == goal_id) {
            h_s = 0;
         }

         // TODO standard deviation?
         float sig =
            next_edge.Ttilda(this_key.T_mean, next_edge.get_den_to_vel());


         // Load or create key for the new edge..
         // Check if next_key exists.
         if (keys.find(id) == keys.end()) {
            keys.insert(make_pair(id, edge_key(id, std::numeric_limits<float>::max(), open.end())));
         }
         edge_key& next_key = (keys.find(id)->second);

         // TODO
         variance_weight = 0;

         // TODO This probably checks to see if the next edge is already in the set
         // with a lower cost.
         if (this_key.cost_to_here + u + variance_weight*sig < next_key.cost_to_here) {

            // Update the next_key values.
            // TODO This should be done through a function.
            next_key.cost_to_here = this_key.cost_to_here + u + variance_weight*sig;
            next_key.T_mean       = this_key.T_mean + u;
            //            next_key.T_var        = this_key.T_var + sig;
            next_key.T_var        = next_key.T_mean * T_std_;

            next_key.pred         = this_key.id;
            next_key.cost = next_key.cost_to_here + heuristic(next_edge, goal);

            // Inserts the new edge into open.
            // If the edge was previously there, it deletes the former version.
            float cost = next_key.cost;
            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);

               // TODO Insertion should only happen in one place.
               keys.find(id)->second.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   // Found indicates a path has been found that reaches the goal.
   if (found)
   {
      // Used to hold the path that was found.
      vector<util_path_element> path;

      // TODO What in the world is this doing?
      travel_time = keys.find(keys.find(goal_id)->second.pred)->second.T_mean - start_time;

      // Move the delay times back to their predeccesors.
      int id = goal_id;
      int last_id = goal_id;

      do {
         stoch_edge& this_edge = edges[id];
         if (use_new_models) {
            if (this_edge.get_delay_time() > 0) {
               // TODO Double check all this.
               keys[keys[id].pred].T_mean += this_edge.get_delay_time();

               stoch_edge& next_edge = edges[keys.find(id)->second.pred];
               if (next_edge.get_mean_rho(keys[keys[id].pred].T_mean)
                   >= next_edge.get_den_to_vel().get_rho_max())
                  next_edge.set_delay_time(
                     std::max(this_edge.get_delay_time(),
                              next_edge.get_delay_time()));
               // TODO Double counting

            }
         }

         last_id = id;
         id = keys.find(id)->second.pred;

      } while (last_id != start_id);


      // Walks back through the path until reaching the start, adding segments.
      last_id  = goal_id;
      id = goal_id;

      do {
         // Creates a path element for the edge leading to the current edge.
         util_path_element elem;
         elem.id = id;

         // TODO Is the start case handled here or below?
         if (id == start_id) {
            elem.A = normal(start_time, 0.001);
         }
         else {
            elem.A = normal(keys[keys[id].pred].T_mean,
                            keys[keys[id].pred].T_var);
         }

         if (id == goal_id)
            elem.D = normal(keys[id].T_mean, keys[id].T_var);

         elem.P = edges[id].get_mean_rho(elem.A.mean());
         elem.neighbors = edges[id].neighbors_size();

         // Add the incoming segment to the path.
         path.push_back(elem);

         last_id = id;

         // Retrieves the id of the segment leading to the current edge.
         id = keys.find(id)->second.pred;

      } while (last_id != start_id);

      path = vector<util_path_element>(path.rbegin(), path.rend());

      // Set the other D values.
      for (int i = 0; i < path.size() - 1; i++) {
         path[i].D = path[i+1].A;
      }

      return path;
   }
   return vector<util_path_element>();
}

vector<util_path_element> stoch_graph::aware_and_sensing(
   int start_id,
   int goal_id,
   float start_time,
   float& travel_time,
   const VelocityMap& velocities,
   std::function<
   float(const stoch_edge&,
         const stoch_edge&)> heuristic,
   float variance_weight = 1) {

   int edges_opened_count = 0;

   // Used to keep track of open edges and their costs.
   multimap<float, map<int, edge_key>::iterator> open;
   // Maps an edge id to the edge_key, which holds cost information.
   std::map<int, edge_key> keys;

   // TODO Used for testing.
   float max_density = std::numeric_limits<float>::min();

   // The start and goal edges.
   stoch_edge& goal    = edges[goal_id];
   stoch_edge& start   = edges[start_id];

   // Create the initial edge key.
   edge_key init(start_id, 0 + heuristic(start, goal), open.end());
   init.cost_to_here = 0;
   init.T_mean       = start.T(start_time, start.get_den_to_vel()) + start_time;
   init.T_var        = start.Ttilda(start_time, start.get_den_to_vel());
   init.pred         = start_id;


   init.T_mean = get_intersection_time(init.T_mean, start_time, start);

   // Add the initial key into the map and multimap.
   keys.insert(make_pair(start_id, init));
   auto multimap_pair = pair<float, map<int, edge_key>::iterator>(init.cost, keys.find(start_id));
   keys.find(start_id)->second.position = open.insert(multimap_pair);

   // Searches for a path with minimal cost.
   bool found = false;
   while (open.size() > 0)
   {
      edges_opened_count++;

      // Gets the edge key with the lowest cost.
      edge_key& this_key = open.begin()->second->second;

      // Retrieve the edge from the key.
      stoch_edge& this_edge = edges[this_key.id];

      // Used for testing to determine the maximum density a car encounters.
      if (this_edge.get_mean_rho(this_key.T_mean) > max_density)
         max_density = this_edge.get_mean_rho(this_key.T_mean);

      // Remove the edge from the open set.
      open.erase(open.begin());
      this_key.position = open.end();

      // Check to see if the path reaches the goal.
      if (this_key.id == goal_id)
      {
         found = true;
         break;
      }

      // If the time is out of the bins, finish with this edge.
      if (!this_edge.in_bins(this_key.T_mean))
         continue;

      // Gets successors and adds them to open if appropriate.
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         // Gets the neighboring edge.
         int id = this_edge.get_neighbor(i);
         stoch_edge& next_edge = edges[id];

         // Update the speedlimit with the value in velocities.
         next_edge.retrieve_den_to_vel().set_v_max(
            velocities.GetVelocity(&next_edge));

         // Calculates the heuristic cost to the goal.
         float h_s = heuristic(next_edge, goal);

         // The time to traverse this edge.
         float u = next_edge.T(this_key.T_mean, next_edge.get_den_to_vel());

         u = get_intersection_time(u, this_key.T_mean, next_edge);

         if (id == goal_id)
         {
            h_s = 0;
         }

         // TODO standard deviation?
         float sig =
            next_edge.Ttilda(this_key.T_mean, next_edge.get_den_to_vel());

         // Load or create key for the new edge..
         // Check if next_key exists.
         if (keys.find(id) == keys.end()) {
            keys.insert(make_pair(id, edge_key(id, std::numeric_limits<float>::max(), open.end())));
         }
         edge_key& next_key = (keys.find(id)->second);

         // TODO
         variance_weight = 0;

         // TODO This probably checks to see if the next edge is already in the set
         // with a lower cost.
         if (this_key.cost_to_here + u + variance_weight*sig < next_key.cost_to_here) {

            // Update the next_key values.
            // TODO This should be done through a function.
            next_key.cost_to_here = this_key.cost_to_here + u + variance_weight*sig;
            next_key.T_mean       = this_key.T_mean + u;
            next_key.T_var        = next_key.T_mean * T_std_;
            // next_key.T_var     = this_key.T_var + sig;

            next_key.pred         = this_key.id;
            next_key.cost = next_key.cost_to_here + heuristic(next_edge, goal);

            // Inserts the new edge into open.
            // If the edge was previously there, it deletes the former version.
            float cost = next_key.cost;
            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);

               // TODO Insertion should only happen in one place.
               keys.find(id)->second.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   // Found indicates a path has been found that reaches the goal.
   if (found)
   {
      // Used to hold the path that was found.
      vector<util_path_element> path;

      // TODO What in the world is this doing?
      travel_time = keys.find(keys.find(goal_id)->second.pred)->second.T_mean - start_time;

      // Move the delay times back to their predeccesors.
      int id = goal_id;
      int last_id = goal_id;

      do {
         stoch_edge& this_edge = edges[id];
         if (this_edge.get_delay_time() > 0) {
            //            keys[id].T_mean -= this_edge.get_delay_time();
            keys[keys[id].pred].T_mean += this_edge.get_delay_time();

            stoch_edge& next_edge = edges[keys.find(id)->second.pred];
            if (next_edge.get_mean_rho(keys[keys[id].pred].T_mean)
                >= next_edge.get_den_to_vel().get_rho_max())
               next_edge.set_delay_time(
                  std::max(this_edge.get_delay_time(),
                           next_edge.get_delay_time()));


            //            std::cout << "Delay added: " << this_edge.get_delay_time() << std::endl;
         }

         last_id = id;
         id = keys.find(id)->second.pred;

      } while (last_id != start_id);


      // Walks back through the path until reaching the start, adding segments.
      last_id  = goal_id;
      id = goal_id;

      do {
         // Creates a path element for the edge leading to the current edge.
         util_path_element elem;
         elem.id = id;

         // TODO Is the start case handled here or below?
         if (id == start_id)
            elem.A = normal(start_time, 0.001);
         else
            elem.A = normal(keys[keys[id].pred].T_mean,
                            keys[keys[id].pred].T_var);

         if (id == goal_id)
            elem.D = normal(keys[id].T_mean, keys[id].T_var);

         elem.P = edges[id].get_mean_rho(elem.A.mean());
         elem.neighbors = edges[id].neighbors_size();

         // Add the incoming segment to the path.
         path.push_back(elem);

         last_id = id;

         // Retrieves the id of the segment leading to the current edge.
         id = keys.find(id)->second.pred;

      } while (last_id != start_id);

      path = vector<util_path_element>(path.rbegin(), path.rend());

      // Set the other D values.
      for (int i = 0; i < path.size() - 1; i++) {
         path[i].D = path[i+1].A;
      }

      return path;
   }
   return vector<util_path_element>();
}


vector<int> stoch_graph::get_shortest_path(int start_id,
                                           int goal_id,
                                           std::function<
                                           float(const stoch_edge&,
                                                 const stoch_edge&)>
                                           heuristic) {

   // Used to keep track of open edges and their costs.
   multimap<float, map<int, edge_key>::iterator> open;

   // Maps an edge id to the edge_key, which holds cost information.
   std::map<int, edge_key> keys;

   // The start and goal edges.
   const stoch_edge& goal    = edges[goal_id];
   const stoch_edge& start   = edges[start_id];

   // Create the initial edge key.
   edge_key init(start_id, 0 + heuristic(start, goal), open.end());
   init.cost_to_here = 0;
   init.pred         = start_id;

   // Add the initial key into the map and multimap.
   keys.insert(make_pair(start_id, init));
   auto multimap_pair = pair<float,
                             map<int,
                                 edge_key>::iterator>(init.cost,
                                                      keys.find(start_id));
   keys.find(start_id)->second.position = open.insert(multimap_pair);

   // Searches for a path with minimal cost.
   bool found = false;
   while (open.size() > 0) {
      // Gets the edge key with the lowest cost.
      edge_key& this_key = open.begin()->second->second;

      // Retrieve the edge from the key.
      stoch_edge& this_edge = edges[this_key.id];

      // Remove the edge from the open set.
      open.erase(open.begin());
      this_key.position = open.end();

      // Check to see if the path reaches the goal.
      if (this_key.id == goal_id)
      {
         found = true;
         break;
      }

      // Gets successors and adds them to open if appropriate.
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         // Gets the neighboring edge.
         int id = this_edge.get_neighbor(i);
         stoch_edge& next_edge = edges[id];

         // Calculates the heuristic cost to the goal.
         float h_s = heuristic(next_edge, goal);

         // Length of this edge.
         float u = next_edge.get_len();

         if (id == goal_id)
         {
            h_s = 0;
         }

         // Load or create key for the new edge..
         // Check if next_key exists.
         if (keys.find(id) == keys.end()) {
            keys.insert(make_pair(id,
                                  edge_key(id,
                                           std::numeric_limits<float>::max(),
                                           open.end())));
         }
         edge_key& next_key = (keys.find(id)->second);

         // TODO This probably checks to see if the next edge is already in the
         // set with a lower cost.
         if (this_key.cost_to_here + u < next_key.cost_to_here) {

            // Update the next_key values.
            // TODO This should be done through a function.
            next_key.cost_to_here = this_key.cost_to_here + u;
            next_key.pred         = this_key.id;
            next_key.cost = next_key.cost_to_here + heuristic(next_edge, goal);

            // Inserts the new edge into open.
            // If the edge was previously there, it deletes the former version.
            float cost = next_key.cost;
            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);

               // TODO Insertion should only happen in one place.
               keys.find(id)->second.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   // Found indicates a path has been found that reaches the goal.
   if (found)
   {
      // Used to hold the path that was found.
      vector<int> path;

      // Walks back through the path until reaching the start, adding segments.
      int last_id  = goal_id;
      int id = goal_id;

      do {
         // Creates a path element for the edge leading to the current edge.

         // Add the incoming segment to the path.
         path.push_back(id);

         // Retrieves the id of the segment leading to the current edge.
         id = keys.find(id)->second.pred;

      } while (id != start_id);

      // Add the start segment.
      path.push_back(start_id);

      // Return the reverse of the created path.
      return vector<int>(path.rbegin(), path.rend());
   }
   else {
      std::cout << "WARNING: Path not found.\n";
      return vector<int>();
   }
}

// test 19345 2349
bool stoch_graph::exhaustive_search(int s, int g) {
   stoch_edge& start_edge = edges[s];
   stoch_edge& goal_edge = edges[g];

   list<int> open;
   map<int, bool> visited;

   int count = 0;

   visited[s] = true;

   if (start_edge.neighbors_size() == 0)
      std::cout << "Start edge has no neighbors\n";

   for (int i = 0; i < start_edge.neighbors_size(); i++) {
      open.push_back(start_edge.get_neighbor(i));
      visited[start_edge.get_neighbor(i)] = true;
   }

   while (open.size() > 0) {
      stoch_edge& this_edge = edges[open.front()];
      count++;

      if (open.front() == g) {
         std::cout << "FOUND IT.\n";
         return true;
      }

      open.pop_front();

      for (int i = 0; i < this_edge.neighbors_size(); i++) {
         if (visited.find(this_edge.get_neighbor(i)) == visited.end()) {
            open.push_back(this_edge.get_neighbor(i));
            visited[this_edge.get_neighbor(i)] = true;
         }
      }
   }

   std::cout << count << " edges searched.\n";

   return false;
}

// TODO REFACTOR
 vector<util_path_element> stoch_graph::get_fastest_path(
    int start_id,
    int goal_id,
    float start_time,
    const VelocityMap& velocities,
    std::function<
    float(const stoch_edge&,
          const stoch_edge&)>
    heuristic) {


   // Used to keep track of open edges and their costs.
   multimap<float, map<int, edge_key>::iterator> open;

   // Maps an edge id to the edge_key, which holds cost information.
   std::map<int, edge_key> keys;

   // The start and goal edges.
   const stoch_edge& goal    = edges[goal_id];
   const stoch_edge& start   = edges[start_id];

   // Create the initial edge key.
   edge_key init(start_id, 0 + heuristic(start, goal), open.end());
   init.cost_to_here = 0;
   init.pred         = start_id;

   // Add the initial key into the map and multimap.
   keys.insert(make_pair(start_id, init));
   auto multimap_pair = pair<float,
                             map<int,
                                 edge_key>::iterator>(init.cost,
                                                      keys.find(start_id));
   keys.find(start_id)->second.position = open.insert(multimap_pair);

   // Searches for a path with minimal cost.
   bool found = false;
   while (open.size() > 0) {
      // Gets the edge key with the lowest cost.
      edge_key& this_key = open.begin()->second->second;

      // Retrieve the edge from the key.
      stoch_edge& this_edge = edges[this_key.id];

      // Remove the edge from the open set.
      open.erase(open.begin());
      this_key.position = open.end();

      // Check to see if the path reaches the goal.
      if (this_key.id == goal_id)
      {
         found = true;
         break;
      }

      // Gets successors and adds them to open if appropriate.
      for (int i = 0; i < this_edge.neighbors_size(); i++)
      {
         // Gets the neighboring edge.
         int id = this_edge.get_neighbor(i);
         const stoch_edge& next_edge = edges[id];

         // Calculates the heuristic cost to the goal.
         float h_s = heuristic(next_edge, goal);

         // Estimated travel time of this edge.
         float u = next_edge.get_len() / velocities.GetVelocity(&next_edge);

         if (id == goal_id)
         {
            h_s = 0;
         }

         // Load or create key for the new edge..
         // Check if next_key exists.
         if (keys.find(id) == keys.end()) {
            keys.insert(make_pair(id,
                                  edge_key(id,
                                           std::numeric_limits<float>::max(),
                                           open.end())));
         }
         edge_key& next_key = (keys.find(id)->second);

         // TODO This probably checks to see if the next edge is already in the
         // set with a lower cost.
         if (this_key.cost_to_here + u < next_key.cost_to_here) {

            // Update the next_key values.
            // TODO This should be done through a function.
            next_key.cost_to_here = this_key.cost_to_here + u;
            next_key.pred         = this_key.id;
            next_key.cost = next_key.cost_to_here + heuristic(next_edge, goal);

            // Inserts the new edge into open.
            // If the edge was previously there, it deletes the former version.
            float cost = next_key.cost;
            if (next_key.position != open.end())
            {
               open.erase(keys.find(id)->second.position);

               // TODO Insertion should only happen in one place.
               keys.find(id)->second.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
            else
            {
               next_key.position = open.insert(pair<float, key_it>(next_key.cost, keys.find(id)));
            }
         }
      }
   }

   // Found indicates a path has been found that reaches the goal.
   if (found)
   {
      // Used to hold the path that was found.
      vector<util_path_element> path;

      // Walks back through the path until reaching the start, adding segments.
      int last_id  = goal_id;
      int id = goal_id;

      do {
         // Creates a path element for the edge leading to the current edge.

         util_path_element elem;
         elem.id = id;

         // Add the incoming segment to the path.
         path.push_back(elem);

         // Retrieves the id of the segment leading to the current edge.
         id = keys.find(id)->second.pred;

      } while (id != start_id);

      // Add the start segment.
      util_path_element start_elem;
      start_elem.id = start_id;
      path.push_back(start_elem);
      path.back().A = normal(start_time, 0.001);

      // Return the reverse of the created path.
      return vector<util_path_element>(path.rbegin(), path.rend());
   }
   else {
      std::cout << "WARNING: Path not found.\n";
      return vector<util_path_element>();
   }
}

void stoch_graph::create_graph_from_osm(const string& filename) {
   xmlpp::DomParser parser;
   parser.parse_file(filename);

   //Walk the tree:
   const xmlpp::Node* pNode = parser.get_document()->get_root_node();

   std::map<string, OsmNode> node_map;

   build_nodes(pNode, node_map);

   std::cout << "Stored " << node_map.size() << " nodes.\n";

   std::map<string, OsmWay> way_map;

   pNode = parser.get_document()->get_root_node();

   build_ways(pNode, way_map);

   std::cout << "Stored " << way_map.size() << " ways.\n";

   // Create an edge for each way.
   for (auto i = way_map.begin(); i != way_map.end(); i++) {
      auto element = i->second;

      // Filter out non-road or private roads.
      if ((element.road_class != "non-road") and
          (element.road_class != "footway") and
          (element.road_class != "service") and
          (element.road_class != "unclassified") and
          (element.road_class != "steps") and
          (element.road_class != "cycleway") and
          (element.road_class != "pedestrian") and
          (element.road_class != "track")) {

         stoch_edge new_edge;
         int id = edges.size();

         new_edge.set_osm_id(atoi(i->first.c_str()));

         for (int i = 0; i < element.node_ids.size(); i++) {
            point shape_point;

            // Gets a node.
            OsmNode* n = &node_map[element.node_ids[i]];

            // Records on the node that this edge uses it.
            n->associated_edge_indices.push_back(id);

            shape_point.x = n->longitude;
            shape_point.y = n->latitude;

            new_edge.add_point(shape_point);
         }

         edges.push_back(new_edge);
      }
   }

   // Traverses the nodes and creates intersection connections based on the
   // associated edge indices.
   for (auto i = node_map.begin(); i != node_map.end(); i++) {
      auto element = i->second;

      for (int i = 0; i < element.associated_edge_indices.size(); i++) {
         int first_edge_id = element.associated_edge_indices[i];

         for (int j = i; j < element.associated_edge_indices.size(); j++) {
            int second_edge_id = element.associated_edge_indices[j];

            // Add the two edges as neighbors.
            // TODO This ignores one-way roads.
            if (first_edge_id != second_edge_id) {
               edges[first_edge_id].add_neighbor(second_edge_id);
               edges[second_edge_id].add_neighbor(first_edge_id);
            }
         }
      }
   }

   if (kStochGraphVerboseDebug) {
      for (int i = 0; i < edges.size(); i++) {
         std::cout << "Edge " << edges[i].get_osm_id() << " ";
         std::cout << "  connects to ";
         for (int j = 0; j < edges[i].neighbors_size(); j++) {
            std::cout << edges[i].get_neighbor(j) << std::endl;
         }
         for (int j = 0; j < edges[i].shape_.size(); j++) {
            std::cout << edges[i].shape_[j].x << " " << edges[i].shape_[j].y
                      << std::endl;
         }
      }
   }
}

void stoch_graph::build_ways(const xmlpp::Node* node,
                             std::map<string, OsmWay>& way_map) {
    const Glib::ustring nodename = node->get_name();

    // Ignore node elements.
    if (nodename == "way") {
       const xmlpp::Element* nodeElement =
          dynamic_cast<const xmlpp::Element*>(node);

       if(nodeElement) {
          OsmWay way;

          const xmlpp::Element::AttributeList& attributes =
             nodeElement->get_attributes();

          for(auto iter = attributes.begin();
              iter != attributes.end();
              ++iter) {

             const xmlpp::Attribute* attribute = *iter;
             const Glib::ustring namespace_prefix =
                attribute->get_namespace_prefix();

             if (attribute->get_name() == "id") {
                way.id = attribute->get_value();
             }
          }

          // Recurse and read each child node.
          xmlpp::Node::NodeList list = node->get_children();
          for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
          {
             store_way_nd_and_tag(*iter, &way);
          }

          way_map[way.id] = way;
       }
    } else {
       //Recurse throumgh child nodes:
       xmlpp::Node::NodeList list = node->get_children();
       for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
       {
          build_ways(*iter, way_map);
       }
    }
 }

void stoch_graph::store_way_nd_and_tag(const xmlpp::Node* node,
                               OsmWay* way) {
   const Glib::ustring nodename = node->get_name();

   // Ignore node elements.
   if (nodename == "nd") {
      const xmlpp::Element* nodeElement =
      dynamic_cast<const xmlpp::Element*>(node);

      if(nodeElement) {
         const xmlpp::Element::AttributeList& attributes =
         nodeElement->get_attributes();

         for(auto iter = attributes.begin();
             iter != attributes.end();
             ++iter) {

            const xmlpp::Attribute* attribute = *iter;
            const Glib::ustring namespace_prefix =
            attribute->get_namespace_prefix();

            if (attribute->get_name() == "ref") {
               way->node_ids.push_back(attribute->get_value());
            }
         }
      }
   }
   else if (nodename == "tag") {
      const xmlpp::Element* nodeElement =
      dynamic_cast<const xmlpp::Element*>(node);

      if(nodeElement) {
         const xmlpp::Element::AttributeList& attributes =
         nodeElement->get_attributes();

         string current_key = "";

         for(auto iter = attributes.begin();
             iter != attributes.end();
             ++iter) {

            const xmlpp::Attribute* attribute = *iter;
            const Glib::ustring namespace_prefix =
            attribute->get_namespace_prefix();

            if (attribute->get_name() == "k") {
               current_key = attribute->get_value();
            }
            if (attribute->get_name() == "v") {
               if (current_key == "highway") {
                  way->road_class = attribute->get_value();
               }
               if (current_key == "oneway") {
                  if (attribute->get_value() == "yes")
                     way->one_way = true;
               }
            }
         }
      }
   }
}

void stoch_graph::build_nodes(const xmlpp::Node* node,
                              std::map<string, OsmNode>& node_map) {
    const Glib::ustring nodename = node->get_name();

    // Ignore node elements.
    if (nodename == "node") {
       const xmlpp::Element* nodeElement =
          dynamic_cast<const xmlpp::Element*>(node);

       if(nodeElement) {
          OsmNode node;

          const xmlpp::Element::AttributeList& attributes =
             nodeElement->get_attributes();

          for(auto iter = attributes.begin();
              iter != attributes.end();
              ++iter) {

             const xmlpp::Attribute* attribute = *iter;
             const Glib::ustring namespace_prefix =
                attribute->get_namespace_prefix();

             if (attribute->get_name() == "id") {
                node.id = attribute->get_value();
             }
             if (attribute->get_name() == "lat") {
                node.latitude = atof(attribute->get_value().c_str());
             }
             if (attribute->get_name() == "lon") {
                node.longitude = atof(attribute->get_value().c_str());
             }
          }

          node_map[node.id] = node;
       }
    }

    //Recurse throumgh child nodes:
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
    {
       build_nodes(*iter, node_map);
    }
 }


// Creates the geometry and topology of the stochastic graph from a Roadie network.
void stoch_graph::create_graph_from_roadie(hwm::network& net) {
   map<std::string, int> lane_id_map;
   map<int, std::string> r_lane_id_map;

   // Create an edge for each lane.
   for (hwm::lane_map::iterator lane_i = net.lanes.begin();
        lane_i != net.lanes.end();
        lane_i++)
   {
      stoch_edge e;
      lane_id_map[lane_i->second.id] = edges.size();
      r_lane_id_map[edges.size()] = lane_i->second.id;
      edges.push_back(e);
   }

   bool uniform = false;
   int in_order = 0;
   int newmax = -1;

   // Sets the edges' lengths and end points.
   for (hwm::lane_map::iterator lane_i = net.lanes.begin();
        lane_i != net.lanes.end();
        lane_i++, in_order++)
   {
      stoch_edge& e = edges[in_order];
      e.set_len(lane_i->second.length());
      e.start.x = lane_i->second.point(0)[0];
      e.start.y = lane_i->second.point(0)[1];
      e.end.x = lane_i->second.point(0.99)[0];
      e.end.y = lane_i->second.point(0.99)[1];

      // Adds the downstream lanes' edges to the neighbors set.
      std::vector<const hwm::lane*> downstreams =
         lane_i->second.end->downstream_lanes(lane_i->second.id);

      for (int i = 0; i < downstreams.size(); i++)
      {
         vector<const hwm::lane*> downstreams_2 =
            downstreams[i]->end->downstream_lanes(downstreams[i]->id);

         for (int j = 0; j < downstreams_2.size(); j++)
         {
            e.add_neighbor(lane_id_map[downstreams_2[j]->id]);
         }
      }
   }
}

void stoch_graph::load_graph_from_sumo(const string& base_filename,
                                       sumo_util::id_maps* id_maps) {

   std::string nodes_file = base_filename + ".nod.xml";
   std::string edges_file = base_filename + ".edg.xml";
   std::string edge_type_file = base_filename + ".typ.xml";
   std::string connections_file = base_filename + ".con.xml";

   // Create the sumo network.
   sumo::network sumo_network =
   sumo::load_xml_network(nodes_file.c_str(),
                          edge_type_file.c_str(),
                          edges_file.c_str(),
                          connections_file.c_str());

   load_graph_from_sumo(base_filename, sumo_network, id_maps);

}

void stoch_graph::load_graph_from_sumo(const string& base_filename,
                          sumo::network& sumo_network,
                          sumo_util::id_maps* id_maps) {

   std::string nodes_file = base_filename + ".nod.xml";
   std::string edges_file = base_filename + ".edg.xml";
   std::string edge_type_file = base_filename + ".typ.xml";
   std::string connections_file = base_filename + ".con.xml";

   // Read and set the projection data.
   xmlpp::DomParser parser;
   parser.parse_file(nodes_file);

   // Get first node.
   const xmlpp::Node* pNode = parser.get_document()->get_root_node();

   const Glib::ustring nodename = pNode->get_name();

   // Get to the location node.
   xmlpp::Node::NodeList list = pNode->get_children();
   for(xmlpp::Node::NodeList::iterator iter = list.begin();
       iter != list.end();
       ++iter) {
      pNode = *iter;

      if (pNode->get_name() == "location") {

         const xmlpp::Element* nodeElement =
            dynamic_cast<const xmlpp::Element*>(pNode);

         if(nodeElement) {
            const xmlpp::Element::AttributeList& attributes =
               nodeElement->get_attributes();

            for(auto iter = attributes.begin();
                iter != attributes.end();
                ++iter) {

               const xmlpp::Attribute* attribute = *iter;

               // Get the offsets.
               if (attribute->get_name() == "netOffset") {
                  string offsets = attribute->get_value();
                  float x_offset =
                     strtod(offsets.substr(0, offsets.find(',')).c_str(), NULL);
                  float y_offset =
                     strtod(offsets.substr(offsets.find(',') + 1,
                                           string::npos).c_str(), NULL);
                  LocalFrame::GetInstance()->SetOffset(x_offset, y_offset);
               }

               // Get the projection parameters.
               if (attribute->get_name() == "projParameter") {
                  string proj_parameters = attribute->get_value();
                  LocalFrame::GetInstance()->SetProjParameters(proj_parameters);
               }
            }
         }

         break;
      }
   }

   // Populate the graph
   create_graph_from_sumo(sumo_network, *id_maps);
}

void stoch_graph::create_graph_from_sumo(sumo::network& net,
                                         sumo_util::id_maps& id_maps) {

   // Create an edge for each lane.
   for (auto lane_i = net.edges.begin();
        lane_i != net.edges.end();
        lane_i++) {


      id_maps.sumo_edge_to_stoch[lane_i->second.id] = edges.size();
      id_maps.stoch_edge_to_sumo[edges.size()] = lane_i->second.id;
      edges.resize(edges.size() + 1);
      edges.back().set_sumo_id(lane_i->second.id);

      string modified_osm_id = lane_i->second.id;

      // Strip off the "#.." portion added by Sumo.
      modified_osm_id = modified_osm_id.substr(0, modified_osm_id.find('#'));
      int osm_id = abs(atoi(modified_osm_id.c_str()));

      edges.back().set_osm_id(osm_id);
   }

   // Sets the edges' lengths and end points.
   int in_order = 0;
   for (auto lane_i = net.edges.begin();
        lane_i != net.edges.end();
        lane_i++, in_order++) {
      stoch_edge& e = edges[in_order];
      sumo::edge edge_i = lane_i->second;

      // TODO Should have accessor.
      e.set_lane_count(edge_i.type->nolanes);

      // Assign the speed limit to the lane's max_v.
      // Max rho still needed.
      e.retrieve_den_to_vel().set_v_max(edge_i.type->speed);

      // Copy the type information.
      e.type = edge_i.type->id;

      // Calculates the length of the sumo lane.
      /// (Common) Case for when the edge is two nodes:
      if (edge_i.shape.size() == 0) {
         sumo::node from, to;

         from = *edge_i.from;
         to = *edge_i.to;

         float length = sumo_util::distance(from.xy[0],
                                             from.xy[1],
                                             to.xy[0],
                                             to.xy[1]);
         e.set_len(length);

         e.shape_.push_back(point(from.xy[0], from.xy[1]));
         e.shape_.push_back(point(to.xy[0], to.xy[1]));
      } else {
         float length = 0;
         e.shape_.push_back(point(edge_i.shape[0][0], edge_i.shape[0][1]));
         for (int j = 1; j < edge_i.shape.size(); j++) {
            length += sumo_util::distance(edge_i.shape[j - 1][0],
                                          edge_i.shape[j - 1][1],
                                          edge_i.shape[j][0],
                                          edge_i.shape[j][1]);

            e.shape_.push_back(point(edge_i.shape[j][0], edge_i.shape[j][1]));
         }

         e.set_len(length);
      }

      // Sets the endpoints.
      e.start.x = edge_i.from->xy[0];
      e.start.y = edge_i.from->xy[1];
      e.end.x = edge_i.to->xy[0];
      e.end.y = edge_i.to->xy[1];
   }

   for (int i = 0; i < net.connections.size(); i++) {
      // Get 'from' edge.
      sumo::edge* from = net.connections[i].from;
      string from_id = from->id;

      // Retrieve the stoch_graph index of the edge.
      if (id_maps.sumo_edge_to_stoch.find(from_id) ==
          id_maps.sumo_edge_to_stoch.end())
         assert(0);
      int from_index = id_maps.sumo_edge_to_stoch[from_id];

      // Get 'to' edge.
      sumo::edge* to = net.connections[i].to;
      string to_id = to->id;

      // Retrieve the stoch_graph index of the edge.
      if (id_maps.sumo_edge_to_stoch.find(to_id) ==
          id_maps.sumo_edge_to_stoch.end())
         assert(0);
      int to_index = id_maps.sumo_edge_to_stoch[to_id];

      // Add the 'to_index' to the neighbors of the 'from_index' edge.
      edges[from_index].add_neighbor(to_index);

      // TODO: I'm ignoring the lane number.
   }

   for (int i = 0; i < net.light_connections.size(); i++) {
      sumo::light_connection& light = net.light_connections[i];
      int edge_index = id_maps.sumo_edge_to_stoch[light.from];
      edges[edge_index].has_intersection = true;
   }
 }

 // Reads the street names from the osm file.
void stoch_graph::parse_names(const xmlpp::Node* node)
 {
    // These save the state of the parsing for on-the fly logic.
    static int current_road_id;
    static string last_attribute;

    const Glib::ustring nodename = node->get_name();

    // Ignore node elements.
    if (nodename == "node")
       return;

    if (nodename == "relation")
       return;

   const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(node);
   if(nodeElement) {
     const xmlpp::Element::AttributeList& attributes =
        nodeElement->get_attributes();
     for(auto iter = attributes.begin();
         iter != attributes.end();
         ++iter) {

       const xmlpp::Attribute* attribute = *iter;
       const Glib::ustring namespace_prefix = attribute->get_namespace_prefix();

       // Store the 'id' of the most recent way.
       if (nodename == "way")
          if (attribute->get_name() == "id") {
             current_road_id = atoi(attribute->get_value().c_str());
          }

       if (nodename == "tag")
          // Attribute 'names' are actually values, so we need to save the state
          // for one iteration.
          if (last_attribute == "name") {
             std::cout << "Name is " << attribute->get_value() << " for "
                       << current_road_id << std::endl;

             // Store the road name for the current id.
             road_names_[current_road_id] = attribute->get_value();
          }

       // Keep a 'state' of the attribute value as osm uses a weird system.
       last_attribute = attribute->get_value();
     }
   }

   //Recurse through child nodes:
   xmlpp::Node::NodeList list = node->get_children();
   for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
   {
      parse_names(*iter);
   }
 }

 bool stoch_graph::build_map_of_road_names(const string& filename) {
    xmlpp::DomParser parser;
    parser.parse_file(filename);

    //Walk the tree:
    const xmlpp::Node* pNode = parser.get_document()->get_root_node();
    parse_names(pNode);

    std::cout << "**All roadnames loaded.**\n";
 }

// TODO Should the input be lat-long or in a local cartesian frame?  Where is
// the frame stored? lat and long is the logical choice, but I need to see where
// the local frame is computed and stored.  It should likely have its own class.
// I believe the frame of the data is local-cartesian in the sumo export, and
// the parameters can be read from the data file. The Location class serves this
// purpose in Android/LetsGo.
int stoch_graph::edge_from_point(double longitude, double latitude) {
   // TODO There should be a geometric hierarchy of roads to traverse.

   // Convert to local cartesian coordinates.
   double local_x, local_y;
   LocalFrame* instance = LocalFrame::GetInstance();
   instance->LongLatToLocal(longitude, latitude, &local_x, &local_y);

   // Done as edge_from_WGS84_point assumes no offset.
   local_x -= instance->get_x_offset();
   local_y -= instance->get_y_offset();

   return edge_from_WGS84_point(local_x, local_y);
}

int stoch_graph::edge_from_WGS84_point(double x, double y) {
   LocalFrame* instance = LocalFrame::GetInstance();
   x += instance->get_x_offset();
   y += instance->get_y_offset();

   // Brute force against all candidates.
   float min_distance = kFloatMax;
   int edge_id = -1;

   for (int i = 0; i < edges.size(); i++) {
      float distance = edges[i].distance_to_shape(x, y);
      if (distance < min_distance) {
         edge_id = i;
         min_distance = distance;
      }
   }

   // std::cout << "Edge found is " << edge_id << ", " << road_names_[edge_id]
   //           << " at distance " << min_distance << std::endl;

   return edge_id;
}


void output_expected_lengths(string filename,
                             const vector<vector<util_path_element>>& routes,
                             const stoch_graph& graph) {
   std::ofstream fout;
   fout.open(filename);
   fout << "<estimated_lengths>\n";
   for (int i = 0; i < routes.size(); i++) {
      float length = 0;

      // Adds the first segment.
      length += graph.edges[routes[i][0].id].get_len();

      // Adds any additional segments. The last edge is skipped as the car is
      // removed in sumo before it tarverses the edge.
      for (int j = 1; j < routes[i].size(); j++) {
         length += graph.edges[routes[i][j].id].get_len();
      }
      fout << "<vehicle id=\"" << i << "\" type=\"type1\""
           << " travel_length=\""
           << length
           << "\" >\n";
      fout << "</vehicle>\n";
   }

   fout << "</estimated_lengths>\n";
}

void output_expected_times(string filename,
                         const vector<vector<util_path_element>>& routes) {
   std::ofstream fout;
   fout.open(filename);
   fout << "<estimated_times>\n";
   for (int i = 0; i < routes.size(); i++) {
      float travel_time;
      if (routes[i].size() > 1) {
         float end = routes[i].back().D.mean();
         float start = routes[i][0].A.mean();
         travel_time = end - start;

         // std::cout << "___________________________\n";
         // for (int j = 0; j < routes[i].size(); j++) {

         //    std::cout << routes[i][j].A.mean() << " -> "
         //              << routes[i][j].D.mean()
         //              << " (" << routes[i][j].D.mean() - routes[i][j].A.mean()
         //              << ") "
         //              << " density was " << routes[i][j].P
         //              << " the neighbors were " << routes[i][j].neighbors
         //              << " for id " << routes[i][j].id << std::endl;
         // }
         // std::cout << "___________________________\n";
      }
      else {
         travel_time = routes[i][0].D.mean() - routes[i][0].A.mean();
      }
      fout << "<vehicle id=\"" << i << "\" type=\"type1\""
           << " travel_time=\""
           << travel_time;
      fout << "\" >\n";
      fout << "</vehicle>\n";
   }

   fout << "</estimated_times>\n";
}

// Populates a data structure of lat-long values for a sumo network.
// 'basefilename' should be the same as given to create_graph_from_sumo.
bool stoch_graph::create_latlong_for_sumo(std::string basefilename) {

   if (edges.size() == 0)
      return false;


}

void stoch_graph::export_lanestats(float time_step, string filename) {
   // Start the time at 0.
   float time_window_start = 0;
   float time_window_end = time_step;
   bool stop_condition = false;

   std::cout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
   std::cout << "<meandata xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo-sim.org/xsd/meandata_file.xsd\">\n";

   // We'll stop when all the roads are empty.
   while(stop_condition != true) {
      std::cout << "\t<interval begin=\"" << time_window_start
                << "\" end=\"" << time_window_end << "\" id=\"lane_stats\">\n";


      for (int i = 0; i < edges.size(); i++) {
         float density_sum = 0;
         float velocity_sum = 0;
         int bins = 0;

         // Check each bin for the time window and average the results
         for (float t = time_window_start; t < time_window_end; t += delta_t_) {
            // TODO Check the whole interval.
            float density = edges[i].get_mean_rho(t);
            float velocity = edges[i].get_den_to_vel().v(density);

            density_sum += density;
            velocity_sum += velocity;
            bins++;

            if (i == 0)
               std::cout << edges[i].get_bin_id(t) << " is bin id\n";
         }

         float density_avg = density_sum / bins;
         float velocity_avg = velocity_sum / bins;

         // In sumo, density is #vehicles/km

         if (density_avg > 0) {
            std::cout << "\t\t<edge id=\"" << edges[i].get_sumo_id() << "\">\n";

            std::cout << "\t\t\t<lane id=\"" << edges[i].get_sumo_id() << "_0\" "
                      << "density=\"" << density_avg << "\" "
                      << "speed=\"" << velocity_avg << "\" />\n";


            std::cout << "\t\t</edge>";
         }
      }

      // Move the window forward.
      time_window_start += time_step;
      time_window_end += time_step;

      std::cout << "\t</interval>\n;";

      // TODO REMOVE
      if (time_window_end > 2300)
         stop_condition = true;
   }

   std::cout <<"<meandata>\n";
}
