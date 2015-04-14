#include "sumo_util.hpp"

namespace sumo_util {

double distance(double x1,
                double y1,
                double x2,
                double y2) {
   return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// Used to compute the distance between two lat lang points by converting them
// to mercantor.
// It turns out this is not needed for Sumo as it does its own projection.
double CartesianDistanceWGS84(double lat1,
                              double lng1,
                              double lat2,
                              double lng2) {
   projPJ pj_merc, pj_latlong;
   string merc_argument = string("+proj=merc +ellps=WGS84 +lat_ts=")
                          + to_string(lat1);



   if (!(pj_merc = pj_init_plus(merc_argument.c_str())))
      exit(1);

   if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84")) )
      exit(1);

   double to_radians = M_PI / 180.0;
   lat1 *= to_radians;
   lng1 *= to_radians;
   lat2 *= to_radians;
   lng2 *= to_radians;

   pj_transform(pj_latlong, pj_merc, 1, 1, &lat1, &lng1, NULL );
   pj_transform(pj_latlong, pj_merc, 1, 1, &lat2, &lng2, NULL );

   std::cerr << lat1 << " " << lng1 << std::endl;

   return distance(lat1, lng1, lat2, lng2);
}

void output_routes_to_sumo(string filename,
                          const vector<vector<util_path_element>>& routes,
                             id_maps& id_maps) {
   std::ofstream fout;
   fout.open(filename);
   fout << "<routes>\n";
   // TODO: Vehicle type parameters should be set carefully.
   // TODO: Heterogeneous vehicles.
   fout << "  <vType id=\"type1\" accel=\"0.8\" decel=\"4.5\" sigma=\"0.5\""
        << "  length=\"5\" maxSpeed=\"70\"/>\n";

   for (int i = 0; i < routes.size(); i++) {
      fout << "<vehicle id=\"" << i << "\" type=\"type1\""
           << " depart=\"" << routes[i][0].A.mean() << "\" color=\"1,0,0\">\n";
      fout << "<route edges=\"";
      // Outputs the id of each edge.
      for (int j = 0; j < routes[i].size(); j++) {
         fout << id_maps.stoch_edge_to_sumo[routes[i][j].id] << " ";
      }
      fout << "\"/>\n";
      fout << "</vehicle>\n";
   }

   fout << "</routes>\n";
}

void collect_routes_for_sumo(string filename,
                             const vector<vector<util_path_element>>& routes,
                             id_maps& id_maps,
                             int start_id) {

   std::ofstream fout;
   // Open to append.
   fout.open(filename, std::fstream::app);

   for (int i = 0; i < routes.size(); i++) {
      fout << "<vehicle id=\"" << start_id + i << "\" type=\"type1\""
           << " depart=\"" << routes[i][0].A.mean() << "\" color=\"1,0,0\">\n";
      fout << "<route edges=\"";
      // Outputs the id of each edge.
      for (int j = 0; j < routes[i].size(); j++) {
         fout << id_maps.stoch_edge_to_sumo[routes[i][j].id] << " ";
      }
      fout << "\"/>\n";
      fout << "</vehicle>\n";
   }
}


}  // namespace sumo_util
