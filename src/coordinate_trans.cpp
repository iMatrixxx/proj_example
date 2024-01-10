#include "../include/coordinate_trans.h"

// lat, long, height
void lla_to_webmercator(const Eigen::Vector3d lla_pos,
                        Eigen::Vector3d &webm_pos) {
  auto lla2webmerctor =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:3857", NULL);

  PJ_COORD input = {{lla_pos(0), lla_pos(1), lla_pos(2), 0}};

  PJ_COORD output = proj_trans(lla2webmerctor, PJ_FWD, input);

  webm_pos = Eigen::Vector3d(output.xyzt.x, output.xyzt.y, output.xyzt.z);
}

void webmercator_to_lla(const Eigen::Vector3d &webm_pos,
                        Eigen::Vector3d &lla_pos) {
  auto lla2webmerctor =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:3857", NULL);

  PJ_COORD input = {webm_pos(0), webm_pos(1), webm_pos(2), 0};

  PJ_COORD output = proj_trans(lla2webmerctor, PJ_INV, input);

  lla_pos = Eigen::Vector3d(output.xyzt.x, output.xyzt.y, output.xyzt.z);
}

// lat, long, height
void lla_to_utm(const Eigen::Vector3d lla_pos, Eigen::Vector3d &utm_pos,
                int &zone_id) {
  zone_id = (int)(31 + (lla_pos(1) / 6));
  std::string target_frame = "EPSG:326" + std::to_string(zone_id);

  auto lla2utm = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326",
                                        target_frame.c_str(), NULL);

  PJ_COORD input = {{lla_pos(0), lla_pos(1), lla_pos(2), 0}};

  PJ_COORD output = proj_trans(lla2utm, PJ_FWD, input);

  utm_pos = Eigen::Vector3d(output.xyzt.x, output.xyzt.y, output.xyzt.z);
}

// lat, long, height
void utm_to_lla(const Eigen::Vector3d &utm_pos, const int &zone_id,
                Eigen::Vector3d &lla_pos) {
  std::string target_frame = "EPSG:326" + std::to_string(zone_id);

  auto lla2utm = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326",
                                        target_frame.c_str(), NULL);

  PJ_COORD input = {{utm_pos(0), utm_pos(1), utm_pos(2), 0}};

  PJ_COORD output = proj_trans(lla2utm, PJ_INV, input);

  lla_pos = Eigen::Vector3d(output.xyzt.x, output.xyzt.y, output.xyzt.z);
}

// lat, long, height
void utm_to_webmercator(const Eigen::Vector3d utm_pos, const int zone_id,
                        Eigen::Vector3d &webm_pos) {
  std::string source_frame = "EPSG:326" + std::to_string(zone_id);

  auto utm2webmercator = proj_create_crs_to_crs(
      PJ_DEFAULT_CTX, source_frame.c_str(), "EPSG:3857", NULL);

  PJ_COORD input = {{utm_pos(0), utm_pos(1), utm_pos(2), 0}};

  PJ_COORD output = proj_trans(utm2webmercator, PJ_FWD, input);

  webm_pos = Eigen::Vector3d(output.xyzt.x, output.xyzt.y, output.xyzt.z);
}

// lat, long, height
void webmercator_to_utm(const Eigen::Vector3d &webm_pos,
                        Eigen::Vector3d &utm_pos, int &zone_id) {
  Eigen::Vector3d lla_pos;
  webmercator_to_lla(webm_pos, lla_pos);
  lla_to_utm(lla_pos, utm_pos, zone_id);
}

// points
void webmercator_to_utm(const Eigen::Vector3d &webm_origin_pos,
                        const MatrixXd &webm_coords,
                        Eigen::Vector3d &utm_origin_pos, int &zone_id,
                        MatrixXd &utm_coords) {
  webmercator_to_utm(webm_origin_pos, utm_origin_pos, zone_id);

  std::vector<Eigen::Vector3d> samples = {
      Eigen::Vector3d(-100.0, -100.0, 0.0), Eigen::Vector3d(-100.0, 0.0, 0.0),
      Eigen::Vector3d(-100.0, 100.0, 0.0),  Eigen::Vector3d(0.0, -100.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),       Eigen::Vector3d(0.0, 100.0, 0.0),
      Eigen::Vector3d(100.0, -100.0, 0.0),  Eigen::Vector3d(100.0, 0.0, 0.0),
      Eigen::Vector3d(100.0, 100.0, 0.0)};

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> observations;
  for (auto &sample : samples) {
    Eigen::Vector3d sample_utm = sample + utm_origin_pos;
    Eigen::Vector3d sample_webm;
    utm_to_webmercator(sample_utm, zone_id, sample_webm);
    observations.push_back(
        std::make_pair(sample, sample_webm - webm_origin_pos));
  }

  MatrixXd H(18, 6);
  MatrixXd L(18, 1);
  for (unsigned int i = 0; i < observations.size(); i++) {
    L(i * 2, 0) = observations[i].first[0];
    L(i * 2 + 1, 0) = observations[i].first[1];

    H(i * 2, 0) = observations[i].second[0];
    H(i * 2, 1) = observations[i].second[1];
    H(i * 2, 2) = 0;
    H(i * 2, 3) = 0;
    H(i * 2, 4) = 1;
    H(i * 2, 5) = 0;

    H(i * 2 + 1, 0) = 0;
    H(i * 2 + 1, 1) = 0;
    H(i * 2 + 1, 2) = observations[i].second[0];
    H(i * 2 + 1, 3) = observations[i].second[1];
    H(i * 2 + 1, 4) = 0;
    H(i * 2 + 1, 5) = 1;
  }

  //
  MatrixXd trans_matrix(1, 6);
  trans_matrix =
      (H.transpose() * H).ldlt().solve(H.transpose() * L).transpose();

  MatrixXd webm_to_utm_matrix(3, 3);
  Eigen::Vector3d offset(trans_matrix(0, 4), trans_matrix(0, 5), 0.0);
  //
  webm_to_utm_matrix(0, 0) = trans_matrix(0, 0);
  webm_to_utm_matrix(0, 1) = trans_matrix(0, 1);
  webm_to_utm_matrix(0, 2) = 0.0;

  //
  webm_to_utm_matrix(1, 0) = trans_matrix(0, 2);
  webm_to_utm_matrix(1, 1) = trans_matrix(0, 3);
  webm_to_utm_matrix(1, 2) = 0.0;

  //
  webm_to_utm_matrix(2, 0) = 0.0;
  webm_to_utm_matrix(2, 1) = 0.0;
  webm_to_utm_matrix(2, 2) = 1.0;
  //
  utm_coords = webm_to_utm_matrix * webm_coords;
  utm_origin_pos += offset;
}
