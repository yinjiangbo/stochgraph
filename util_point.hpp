// Structure for simple 2D cartesian point.

#ifndef UTIL_POINT_HPP
#define UTIL_POINT_HPP

// A simple point representation.
struct point {
  float x;
  float y;

   point() {}

   point(float x_, float y_) {
      x = x_;
      y = y_;
   }

   point operator-(const point& other) {
      point result(x, y);
      result.x -= other.x;
      result.y -= other.y;

      return result;
   }
};

#endif
