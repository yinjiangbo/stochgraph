// Holds the parameters for the local frame and can map longitude, latitudes to
// the local frame.

#ifndef LOCAL_FRAME_HPP
#define LOCAL_FRAME_HPP

#include <iostream>

class LocalFrame {
public:
   // Gets/instantiates the instance for this class.
   static LocalFrame* GetInstance();

   // Maps a longitude, latitude point to the local coordinate system.
   // NOTE that this is (Long, Lat) not (Lat, Long).
   bool LongLatToLocal(double longitude,
                       double latitude,
                       double* local_x,
                       double* local_y);

   bool LocalToLongLat(double local_x,
                       double local_y,
                       double* longitude,
                       double* latitude);

   void SetOffset(double x, double y) {
      ClearOffset();
      AddOffset(x, y);
   }

   void ClearOffset() {
      x_offset_ = 0;
      y_offset_ = 0;
   }

   void AddOffset(double x, double y) {
      x_offset_ += x;
      y_offset_ += y;
   }

   void SetProjParameters(const std::string& proj_parameters) {
      proj_parameters_ = proj_parameters;
   }

   std::string GetProjParameters() {
      return proj_parameters_;
   }

   static bool Exists() {
      if (instance_ == NULL)
         return false;
      else
         return true;
   }

   double get_x_offset() {
      return x_offset_;
   }

   double get_y_offset() {
      return y_offset_;
   }

private:
   static LocalFrame* instance_;

   double x_offset_;
   double y_offset_;

   std::string proj_parameters_;

   // Private constructor due to this being a singleton.
   LocalFrame() {
      x_offset_ = 0;
      y_offset_ = 0;
      proj_parameters_ = "";
   }
};

#endif
