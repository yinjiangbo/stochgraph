#include <cmath>
#include <iomanip>

#include "proj_api.h"

#include "local_frame.hpp"

LocalFrame* LocalFrame::instance_ = 0;

LocalFrame* LocalFrame::GetInstance() {
   if (instance_ == 0) {
      instance_ = new LocalFrame();
   }

   return instance_;
}

bool LocalFrame::LongLatToLocal(double longitude,
                                double latitude,
                                double* local_x,
                                double* local_y) {
   // TODO Projection does not quite line up with stated Sumo bounds in nodes.
   if (proj_parameters_ == "") {
      std::cerr << "ERROR: Projection parameters were not set.\n";
      return false;
   }

   if ((x_offset_ == 0) or (y_offset_ == 0)) {
      std::cerr << "WARNING: local offset is zero for projected space.\n";
   }

   long double center_x, center_y;
   center_x = longitude;
   center_y = latitude;

   // This is the value of M_PI / 180.0.
   long double deg_to_rad = 0.0174532925;

   center_x *= deg_to_rad;
   center_y *= deg_to_rad;

   double center[2];
   center[0] = center_x;
   center[1] = center_y;

   // Project location using road network parameters.
   projPJ pj_merc, pj_latlong;
   pj_merc = pj_init_plus(proj_parameters_.c_str());
   pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84 +datum=WGS84");

   int p = pj_transform(pj_latlong,
                        pj_merc,
                        1,
                        1,
                        &center[0],
                        &center[1],
                        NULL);

   *local_x = center[0] + x_offset_;
   *local_y = center[1] + y_offset_;

   return true;
}

bool LocalFrame::LocalToLongLat(double local_x,
                                double local_y,
                                double* longitude,
                                double* latitude) {
   // TODO Projection does not quite line up with stated Sumo bounds in nodes.
   if (proj_parameters_ == "") {
      std::cerr << "ERROR: Projection parameters were not set.\n";
      return false;
   }

   if (x_offset_ == 0 or y_offset_ == 0) {
      std::cerr << "Error: projected lat lon without offsets.\n";
   }

   double center[2];
   center[0] = local_x;
   center[1] = local_y;

   center[0] -= x_offset_;
   center[1] -= y_offset_;

   double deg_to_rad = M_PI / 180.0;

   // Project location using road network parameters.
   // TODO These configurations should be read out of the sumo .nod.xml file.
   projPJ pj_merc, pj_latlong;
   pj_merc = pj_init_plus(proj_parameters_.c_str());
   pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84 +datum=WGS84");

   pj_transform(pj_merc,
                pj_latlong,
                1,
                1,
                &center[0],
                &center[1],
                NULL);

   center[0] /= deg_to_rad;
   center[1] /= deg_to_rad;

   (*longitude) = center[0];
   (*latitude) = center[1];

   return true;
}
