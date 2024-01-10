#include "include/coordinate_trans.h"

int main(void) {
  Eigen::Vector3d lla = {31.2304, 121.4737, 0.0};
  Eigen::Vector3d webm;
  Eigen::Vector3d utm;
  int zone_id;
  //
  lla_to_webmercator(lla, webm);
  printf("lat:%f, long:%f, heigth:%f; web_x:%f, web_y:%f, height:%f\n", lla(0),
         lla(1), lla(2), webm(0), webm(1), webm(2));
  //
  webmercator_to_lla(webm, lla);
  printf("lat:%f, long:%f, heigth:%f; web_x:%f, web_y:%f, height:%f\n", lla(0),
         lla(1), lla(2), webm(0), webm(1), webm(2));
  //
  lla_to_utm(lla, utm, zone_id);
  printf(
      "lat:%f, long:%f, heigth:%f; utm_x:%f, utm_y:%f, height:%f， zoneid:%d\n",
      lla(0), lla(1), lla(2), utm(0), utm(1), utm(2), zone_id);
  //
  utm_to_lla(utm, zone_id, lla);
  printf(
      "lat:%f, long:%f, heigth:%f; utm_x:%f, utm_y:%f, height:%f， zoneid:%d\n",
      lla(0), lla(1), lla(2), utm(0), utm(1), utm(2), zone_id);
  //
  utm_to_webmercator(utm, zone_id, webm);
  printf(
      "utm_x:%f, utm_y:%f, heigth:%f; web_x:%f, web_y:%f, height:%f, "
      "zoneid:%d\n",
      utm(0), utm(1), utm(2), webm(0), webm(1), webm(2), zone_id);

  webmercator_to_lla(webm, lla);
  webmercator_to_utm(webm, utm, zone_id);
  printf(
      "utm_y:%f, utm_y:%f, heigth:%f; web_x:%f, web_y:%f, height:%f, "
      "zoneid:%d\n",
      utm(0), utm(1), utm(2), webm(0), webm(1), webm(2), zone_id);
  // points
  std::vector<Eigen::Vector3d> test_samples;
  for (int i = -50; i < 50; i++) {
    for (int j = -50; j < 50; j++) {
      test_samples.push_back(Eigen::Vector3d{i * 1.0, j * 1.0, 0.0});
    }
  }

  MatrixXd webm_coords(3, test_samples.size());
  MatrixXd utm_coords(3, test_samples.size());

  std::vector<std::pair<std::vector<double>, PJ_COORD>> test_observations;
  int count = 0;
  for (auto& sample : test_samples) {
    Eigen::Vector3d sample_utm = sample + utm;
    Eigen::Vector3d sample_webm;
    utm_to_webmercator(sample_utm, zone_id, sample_webm);
    webm_coords.col(count) = sample_webm - webm;
    count++;
  }

  Eigen::Vector3d utm_origin_pos;
  webmercator_to_utm(webm, webm_coords, utm_origin_pos, zone_id, utm_coords);

  for (int i = 0; i < webm_coords.cols(); i++) {
    Eigen::Vector3d utm_pos;
    Eigen::Vector3d webm_pos(webm + webm_coords.col(i));
    webmercator_to_utm(webm_pos, utm_pos, zone_id);

    printf(
        "webm_x:%f, webm_y:%f, webm_z:%f, utm_x:%f, utm_y:%f, utm_z:%f, "
        "delta_x: %f, delta_y: %f, delta_z: %f\n",
        webm_pos[0], webm_pos[1], webm_pos[2],
        utm_origin_pos[0] + utm_coords.col(i)[0],
        utm_origin_pos[1] + utm_coords.col(i)[1],
        utm_origin_pos[2] + utm_coords.col(i)[2],
        (utm_origin_pos[0] + utm_coords.col(i)[0] - utm_pos[0]),
        (utm_origin_pos[1] + utm_coords.col(i)[1] - utm_pos[1]),
        (utm_origin_pos[2] + utm_coords.col(i)[2] - utm_pos[2]));
  }

  return 0;
}