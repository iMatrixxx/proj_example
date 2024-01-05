#include <proj.h>
#include <stdio.h>

#include <iostream>
int main(void) {
  // lla to webmercator
  auto lla2webmerctor =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:3857", NULL);

  // lla to utm
  auto lla2utm =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:32649", NULL);

  // utm to webmercator
  auto utm2webmercator =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:32649", "EPSG:3857", NULL);

  PJ_COORD input;
  input.xyzt.x = 0;      // Lat in deg
  input.xyzt.y = 111.0;  // Long in deg
  input.xyzt.z = 0;
  input.xyzt.t = 0.0;
  //
  auto output = proj_trans(lla2utm, PJ_FWD, input);
  printf("lla 2 utm: lat[%.8f], lon[%.8f] ---> x[%.8f], y[%.8f]\n",
         input.xyzt.x, input.xyzt.y, output.xyzt.x, output.xyzt.y);
  auto inv_output = proj_trans(lla2utm, PJ_INV, output);
  printf("utm 2 lla: x[%.8f], y[%.8f] ---> lat[%.8f], lon[%.8f]\n",
         output.xyzt.x, output.xyzt.y, inv_output.xyzt.x, inv_output.xyzt.y);

  //
  output = proj_trans(lla2webmerctor, PJ_FWD, input);
  printf("lla 2 webmercator: lat[%.8f], lon[%.8f] ---> x[%.8f], y[%.8f]\n",
         input.xyzt.x, input.xyzt.y, output.xyzt.x, output.xyzt.y);
  inv_output = proj_trans(lla2webmerctor, PJ_INV, output);
  printf("webmercator 2 lla: x[%.8f], y[%.8f] ---> lat[%.8f], lon[%.8f]\n",
         output.xyzt.x, output.xyzt.y, inv_output.xyzt.x, inv_output.xyzt.y);

  //
  input.xyzt.x = 500000.0;  // utm x
  input.xyzt.y = 0.0;       // utm y
  output = proj_trans(utm2webmercator, PJ_FWD, input);
  printf("utm to webmercator: lat[%.8f], lon[%.8f] ---> x[%.8f], y[%.8f]\n",
         input.xyzt.x, input.xyzt.y, output.xyzt.x, output.xyzt.y);
  inv_output = proj_trans(utm2webmercator, PJ_INV, output);
  printf("webmercator to utm: x[%.8f], y[%.8f] ---> lat[%.8f],lon[%.8f]\n",
         output.xyzt.x, output.xyzt.y, inv_output.xyzt.x, inv_output.xyzt.y);

  return 0;
}