#ifndef __UTM_PROJECTOR__H
#define __UTM_PROJECTOR__H


class UTMProjector {
 public:
  UTMProjector(int zoneWidth = 6);
  virtual ~UTMProjector();
  int FromGlobalToLocal(double latitude, double longitude, double &x, double &y);
  int FromLocalToGlobal(double x, double y, double &latitude, double &longitude);
  double GetLongitude0();

 protected:
  int mZoneWidth;  // the width of projection zone (unit: deg)
  int mProjectNo;
};

double get_grid_convergence(const double &longitude0, const double &lat, const double &lon);

#endif  // __UTM_PROJECTOR__H