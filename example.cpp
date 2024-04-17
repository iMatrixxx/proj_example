/*
 * @Author: aoi
 * @Date: 2024-04-16 19:27:51
 * @LastEditors: aoi
 * @LastEditTime: 2024-04-16 19:30:52
 * @Description: 
 * Copyright (c) Air by aoi, All Rights Reserved. 
 */
#include "include/coordinate_trans.h"
#include "unordered_map"


MatrixXd calib_lsq(
    std::unordered_map<
        int, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>&
        observations) {
  int obs_sum = 0;
  int marker_sum = 0;
  for (auto& it : observations) {
    obs_sum += it.second.size();
    marker_sum++;
  }

  MatrixXd H(obs_sum * 2, 2 + 2 * marker_sum);
  MatrixXd L(obs_sum * 2, 1);
  int obs_count = 0;
  int marker_count = 0;
  for (auto& it : observations) {
    for (auto& obs : it.second) {
      L(obs_count * 2, 0) = obs.second(0);
      L(obs_count * 2 + 1, 0) = obs.second(1);

      H(obs_count * 2, 0) = obs.first(0);
      H(obs_count * 2, 1) = -obs.first(1);
      H(obs_count * 2, marker_count * 2 + 2) = -1;
      H(obs_count * 2, marker_count * 2 + 3) = 0;

      H(obs_count * 2 + 1, 0) = obs.first(1);
      H(obs_count * 2 + 1, 1) = obs.first(0);
      H(obs_count * 2 + 1, marker_count * 2 + 2) = 0;
      H(obs_count * 2 + 1, marker_count * 2 + 3) = -1;
      //
      obs_count++;
    }
    marker_count++;
  }
  //
  MatrixXd trans_matrix(1, 10);
  trans_matrix =
      (H.transpose() * H).ldlt().solve(H.transpose() * L).transpose();
  return trans_matrix;
}


void out_calib_res(
    std::unordered_map<
        int, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>&
        observations,
    MatrixXd& trans_matrix) {
  // result
  std::cout << "calib compute: " << trans_matrix(0, 0) << ","
            << trans_matrix(0, 1) << "," << trans_matrix(0, 2) << ","
            << trans_matrix(0, 3) << "," << trans_matrix(0, 4) << ","
            << trans_matrix(0, 5) << "," << trans_matrix(0, 6) << ","
            << trans_matrix(0, 7) << "," << trans_matrix(0, 8) << ","
            << trans_matrix(0, 9) << std::endl;

  int obs_sum = 0;
  int marker_sum = 0;
  std::vector<Eigen::Vector2d> offsets;
  for (auto& it : observations) {
    obs_sum += it.second.size();

    offsets.push_back(Eigen::Vector2d(trans_matrix(0, 2 + marker_sum * 2),
                                      trans_matrix(0, 3 + marker_sum * 2)));
    marker_sum++;
  }

  MatrixXd R(2, 2);
  R(0, 0) = trans_matrix(0, 0);
  R(0, 1) = -trans_matrix(0, 1);
  R(1, 0) = trans_matrix(0, 1);
  R(1, 1) = trans_matrix(0, 0);

  int obs_count = 0;
  int marker_count = 0;
  for (auto& it : observations) {
    for (auto& obs : it.second) {
      Eigen::Vector2d tmp = R * obs.first - obs.second - offsets[marker_count];
      std::cout << "res:" << tmp(0) << "," << tmp(1) << std::endl;
    }
    marker_count++;
  }
}

int main(void) {
  // marker postions
  std::vector<Eigen::Vector2d> marker_offsets = {
      Eigen::Vector2d(10000.0, 10000.0), Eigen::Vector2d(10000.0, -10000.0),
      Eigen::Vector2d(-10000.0, 10000.0), Eigen::Vector2d(-10000.0, 10000.0)};
  // sample by marker
  std::vector<Eigen::Vector2d> cam_samples = {
      Eigen::Vector2d(100.0, 100.0), Eigen::Vector2d(100.0, -100.0),
      Eigen::Vector2d(-100.0, 100.0), Eigen::Vector2d(-100.0, 100.0),
      Eigen::Vector2d(0.0, 0.0)};
  // scale & rotation
  double scale = 0.8;
  double theta = 0.1;
  MatrixXd R(2, 2);
  R(0, 0) = scale * cos(theta);
  R(0, 1) = -scale * sin(theta);
  R(1, 0) = scale * sin(theta);
  R(1, 1) = scale * cos(theta);

  std::cout << "calib groudtruth: " << R(0, 0) << "," << R(1, 0) << ","
            << marker_offsets[0](0) << "," << marker_offsets[0](1)
            << marker_offsets[1](0) << "," << marker_offsets[1](1)
            << marker_offsets[2](0) << "," << marker_offsets[2](1)
            << marker_offsets[3](0) << "," << marker_offsets[3](1) << std::endl;

  // simulator observation
  std::unordered_map<int,
                     std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>
      observations;
  // std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> observations;
  for (unsigned int i = 0; i < marker_offsets.size(); i++) {
    for (auto& sample : cam_samples) {
      observations[i].push_back(
          std::make_pair(R.inverse() * (sample + marker_offsets[i]), sample));
    }
  }

  // interface
  MatrixXd trans_matrix = calib_lsq(observations);

  // res
  out_calib_res(observations, trans_matrix);
}


// int main(void) {
//   Eigen::Vector3d lla = {31.2304, 121.4737, 0.0};
//   Eigen::Vector3d webm;
//   Eigen::Vector3d utm;
//   int zone_id;
//   //
//   lla_to_webmercator(lla, webm);
//   printf("lat:%f, long:%f, heigth:%f; web_x:%f, web_y:%f, height:%f\n", lla(0),
//          lla(1), lla(2), webm(0), webm(1), webm(2));
//   //
//   webmercator_to_lla(webm, lla);
//   printf("lat:%f, long:%f, heigth:%f; web_x:%f, web_y:%f, height:%f\n", lla(0),
//          lla(1), lla(2), webm(0), webm(1), webm(2));
//   //
//   lla_to_utm(lla, utm, zone_id);
//   printf(
//       "lat:%f, long:%f, heigth:%f; utm_x:%f, utm_y:%f, height:%f， zoneid:%d\n",
//       lla(0), lla(1), lla(2), utm(0), utm(1), utm(2), zone_id);
//   //
//   utm_to_lla(utm, zone_id, lla);
//   printf(
//       "lat:%f, long:%f, heigth:%f; utm_x:%f, utm_y:%f, height:%f， zoneid:%d\n",
//       lla(0), lla(1), lla(2), utm(0), utm(1), utm(2), zone_id);
//   //
//   utm_to_webmercator(utm, zone_id, webm);
//   printf(
//       "utm_x:%f, utm_y:%f, heigth:%f; web_x:%f, web_y:%f, height:%f, "
//       "zoneid:%d\n",
//       utm(0), utm(1), utm(2), webm(0), webm(1), webm(2), zone_id);

//   webmercator_to_lla(webm, lla);
//   webmercator_to_utm(webm, utm, zone_id);
//   printf(
//       "utm_y:%f, utm_y:%f, heigth:%f; web_x:%f, web_y:%f, height:%f, "
//       "zoneid:%d\n",
//       utm(0), utm(1), utm(2), webm(0), webm(1), webm(2), zone_id);
//   // points
//   std::vector<Eigen::Vector3d> test_samples;
//   for (int i = -50; i < 50; i++) {
//     for (int j = -50; j < 50; j++) {
//       test_samples.push_back(Eigen::Vector3d{i * 1.0, j * 1.0, 0.0});
//     }
//   }

//   MatrixXd webm_coords(3, test_samples.size());
//   MatrixXd utm_coords(3, test_samples.size());

//   std::vector<std::pair<std::vector<double>, PJ_COORD>> test_observations;
//   int count = 0;
//   for (auto& sample : test_samples) {
//     Eigen::Vector3d sample_utm = sample + utm;
//     Eigen::Vector3d sample_webm;
//     utm_to_webmercator(sample_utm, zone_id, sample_webm);
//     webm_coords.col(count) = sample_webm - webm;
//     count++;
//   }

//   Eigen::Vector3d utm_origin_pos;
//   webmercator_to_utm(webm, webm_coords, utm_origin_pos, zone_id, utm_coords);

//   for (int i = 0; i < webm_coords.cols(); i++) {
//     Eigen::Vector3d utm_pos;
//     Eigen::Vector3d webm_pos(webm + webm_coords.col(i));
//     webmercator_to_utm(webm_pos, utm_pos, zone_id);

//     printf(
//         "webm_x:%f, webm_y:%f, webm_z:%f, utm_x:%f, utm_y:%f, utm_z:%f, "
//         "delta_x: %f, delta_y: %f, delta_z: %f\n",
//         webm_pos[0], webm_pos[1], webm_pos[2],
//         utm_origin_pos[0] + utm_coords.col(i)[0],
//         utm_origin_pos[1] + utm_coords.col(i)[1],
//         utm_origin_pos[2] + utm_coords.col(i)[2],
//         (utm_origin_pos[0] + utm_coords.col(i)[0] - utm_pos[0]),
//         (utm_origin_pos[1] + utm_coords.col(i)[1] - utm_pos[1]),
//         (utm_origin_pos[2] + utm_coords.col(i)[2] - utm_pos[2]));
//   }

//   return 0;
// }