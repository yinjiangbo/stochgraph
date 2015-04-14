#include <iostream>
#include "stoch_graph.hpp"

using namespace std;

int main()
{
  float end_time;
  float delta_t = 5;

  stoch_graph G;
  for (int i = 0; i < 3; i++)
  {
    G.edges.push_back(stoch_edge());
    G.edges.back().set_len(100);
    if (i > 0)
    {
      G.edges[i].add_neighbor(i - 1);
    }
  }
  G.fund.v_max   = 22.352;
  G.fund.rho_max = 0.09;

  for (int i = 0; i < G.edges.size(); i++)
  {
    vector<float> mean_rho;
    for (int j = 0; j < 40; j++)
    {
      mean_rho.push_back(0.05);
    }
    vector<float> var_rho;
    for (int j = 0; j < 40; j++)
    {
      var_rho.push_back(0.00027);
    }
    G.edges[i].initialize_rho(delta_t, mean_rho, var_rho);
    end_time = mean_rho.size() *delta_t ;

    vector<float> mean_v;
    for (int j = 0; j < 40; j++)
    {
      mean_v.push_back(11.176);
    }
    vector<float> var_v;
    for (int j = 0; j < 40; j++)
    {
      var_v.push_back(2.220497);
    }
    G.edges[i].initialize_vel(delta_t, mean_v, var_v);
  }

  int _path[] = {0, 1, 2};
  vector<int> path(_path, _path + sizeof(_path)/sizeof(int));

  for (int i = 0; i < path.size(); i++)
  {
    std::cout << "Edge " << i << std::endl;
    for (float t = 0; t < end_time; t += delta_t)
    {
      std::cout << "t: " << G.edges[path[i]].get_mean_rho(t);
    }
    std::cout << std::endl;
  }

  std::cout << "Updating..." << std::endl;
  G.update_for_path(path);

  for (int i = 0; i < path.size(); i++)
  {
    std::cout << "Edge " << i << std::endl;
    for (float t = 0; t < end_time; t += delta_t)
    {
      std::cout << "t: " << G.edges[path[i]].get_mean_rho(t);
    }
    std::cout << std::endl;
  }


  return 1;
}
