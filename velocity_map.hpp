#ifndef VELOCITY_MAP_HPP
#define VELOCITY_MAP_HPP

#include <map>
#include <string>

#include <libxml++/libxml++.h>

#include "stoch_edge.hpp"

using std::string;
using std::map;

class VelocityMap {
public:
   float GetVelocity(const stoch_edge*) const;

   void ReadVelocities(string filename);
private:
   map<string, float> velocities_;

   void ReadInterval(const xmlpp::Node* node, string attribute_name);
   void ReadEdge(const xmlpp::Node* node, string attribute_name);
   float ReadLane(const xmlpp::Node* node, string attribute_name);
};


#endif
