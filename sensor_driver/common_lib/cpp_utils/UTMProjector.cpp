#include "UTMProjector.h"
#include <iostream>
#include <cmath>

const double PI = 3.1415926535897932384626433832795;

UTMProjector::UTMProjector(int zoneWidth) {
  mZoneWidth = zoneWidth;
  mProjectNo = -1;
}
UTMProjector::~UTMProjector() {}

int UTMProjector::FromGlobalToLocal(double latitude, double longitude,
                                    double &x, double &y) {
  double iPI = PI / 180;
  double a = 6378137.0;
  double f = 1.0 / 298.257222101;
  if (mProjectNo < 0) {
    mProjectNo = floor(longitude / mZoneWidth);
  }
  double ProjNo = mProjectNo;
  double longitude0 = ProjNo * mZoneWidth + mZoneWidth / 2;
  longitude0 = longitude0 * iPI;
  double latitude0 = 0;
  double longitude1 = longitude * iPI;
  double latitude1 = latitude * iPI;
  double e2 = 2 * f - f * f;
  double ee = e2 * (1.0 - e2);
  double NN = a / sqrt(1.0 - e2 * sin(latitude1) * sin(latitude1));
  double T = tan(latitude1) * tan(latitude1);
  double C = ee * cos(latitude1) * cos(latitude1);
  double A = (longitude1 - longitude0) * cos(latitude1);
  double M =
      a *
      ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * latitude1 -
       (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) *
           sin(2 * latitude1) +
       (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * latitude1) -
       (35 * e2 * e2 * e2 / 3072) * sin(6 * latitude1));
  double xval =
      NN * (A + (1 - T + C) * A * A * A / 6 +
            (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
  double yval =
      M + NN * tan(latitude1) *
              (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
               (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A *
                   A / 720);
  double X0 = 1000000 * (ProjNo + 1) + 500000;
  double Y0 = 0;
  xval = xval + X0;
  yval = yval + Y0;
  x = (xval - 500000) * 0.9996 + 500000;
  y = yval * 0.9996;
  return 0;
}

int UTMProjector::FromLocalToGlobal(double x, double y, double &latitude,
                                    double &longitude) {
  double iPI = PI / 180.0f;
  double mZoneWidth = 6;
  double a = 6378137.0;
  double f = 1.0 / 298.257222101;
  if (mProjectNo < 0) {
    mProjectNo = floor(x / 1000000) - 1;
  }
  double ProjNo = mProjectNo + 1;
  double longitude0 = (ProjNo - 1) * mZoneWidth + mZoneWidth / 2;
  longitude0 = longitude0 * iPI;
  double X0 = ProjNo * 1000000 + 500000;
  double Y0 = 0;
  double xval = (x - 500000) / 0.9996 + 500000 - X0;
  double yval = y / 0.9996 - Y0;

  double e2 = 2 * f - f * f;
  double e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
  double ee = e2 * (1.0 - e2);

  double M = yval;
  double u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
  double fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) +
               (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u) +
               (151 * e1 * e1 * e1 / 96) * sin(6 * u) +
               (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
  double C = ee * cos(fai) * cos(fai);
  double T = tan(fai) * tan(fai);
  double NN = a / sqrt(1.0 - e2 * sin(fai) * sin(fai));
  double R =
      a * (1 - e2) /
      sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) *
           (1 - e2 * sin(fai) * sin(fai)));
  double D = xval / NN;

  double longitude1 =
      longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 +
                    (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D *
                        D * D * D * D / 120) /
                       cos(fai);
  double latitude1 =
      fai -
      (NN * tan(fai) / R) *
          (D * D / 2 -
           (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 +
           (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D *
               D * D * D * D / 720);

  longitude = longitude1 / iPI;
  latitude = latitude1 / iPI;
  return 0;
}

double UTMProjector::GetLongitude0() {
  if (mProjectNo < 0) {
    std::cerr << "WARN: UTM Projection is not initialized!" << std::endl;
  }

  return mProjectNo * mZoneWidth + mZoneWidth / 2;
}

double get_grid_convergence(const double &longitude0, const double &lat, const double &lon) {
  double converage = std::atan(std::tan((lon - longitude0) / 180.0 * M_PI) * std::sin(lat / 180.0 * M_PI)) * 180.0 / M_PI;
  return converage;
}