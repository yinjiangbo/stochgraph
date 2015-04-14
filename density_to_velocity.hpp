// A class to map density to velocity.
#ifndef DENSITY_TO_VELOCITY_HPP
#define DENSITY_TO_VELOCITY_HPP

#include <iostream>

class den_to_vel
{
public:
   // Converts from a change in density to change in velocity.
   double del_v(double del_rho) const {
      return (-v_max/rho_max)*del_rho;
   }

   // Converts from density to velocity.
   double v(double rho) const {
      double v_0, v_1, v_min;

      v_0 = v_max * 0.7;
      v_1 = v_max * 0.2;
      v_min = 1;

      double vel =
         ((v_max - v_min) * (1 - std::min(1.0, (rho / rho_max)))) + v_min;

      return std::max(v_min, vel);


      // if ((rho / rho_max) <= 0.33333) {
      //    return ((v_max - v_0) * (1 - (rho / rho_max) / (0.33333))) + v_0;
      // }
      // else if ((rho / rho_max) <= 0.666666) {
      //    return (v_0 - v_1) * (1 - (((rho / rho_max) - 0.3333) / (0.3333333)))
      //       + v_min;
      // }
      // else if ((rho / rho_max) <= 1) {
      //    return (v_1 - v_min) * (1 - (((rho / rho_max) - 0.6666) / 0.333333))
      //       + v_min;
      // }
      // else {
      //    return v_min;
      // }
   }

   void set_v_max(double _v_max) {
      v_max = _v_max;
   }

   double get_v_max() const{
      return v_max;
   }

   void set_rho_max(double _rho_max) {
      rho_max = _rho_max;
   }

   double get_rho_max() const{
      return rho_max;
   }

   bool operator==(const den_to_vel& other) const {
      return ((v_max == other.v_max)
              and
              (rho_max == other.rho_max));
   }

   // TODO Unit tests.
   // TODO Should be private.
public:
   double v_max;
   double rho_max;
};

#endif
