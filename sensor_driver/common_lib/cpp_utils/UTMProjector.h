#ifndef __UTM_PROJECTOR__H
#define __UTM_PROJECTOR__H


class UTMProjector {
 public:
  UTMProjector(int zoneWidth = 6);
  virtual ~UTMProjector();
  int FromGlobalToLocal(double latitude, double longitude, double &x, double &y);
  int FromLocalToGlobal(double x, double y, double &latitude, double &longitude);

 protected:
  int mZoneWidth;  // the width of projection zone (unit: deg)
};

#endif  // __UTM_PROJECTOR__H