#include <proj.h>
#include <stdio.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
// lat, long, height
void lla_to_webmercator(const Eigen::Vector3d lla_pos,
                        Eigen::Vector3d &webm_pos);

void webmercator_to_lla(const Eigen::Vector3d &webm_pos,
                        Eigen::Vector3d &lla_pos);

// lat, long, height
void lla_to_utm(const Eigen::Vector3d lla_pos, Eigen::Vector3d &utm_pos,
                int &zone_id);

// lat, long, height
void utm_to_lla(const Eigen::Vector3d &utm_pos, const int &zone_id,
                Eigen::Vector3d &lla_pos);

// lat, long, height
void utm_to_webmercator(const Eigen::Vector3d utm_pos, const int zone_id,
                        Eigen::Vector3d &webm_pos);

// lat, long, height
void webmercator_to_utm(const Eigen::Vector3d &webm_pos,
                        Eigen::Vector3d &utm_pos, int &zone_id);

// points
void webmercator_to_utm(const Eigen::Vector3d &webm_origin_pos,
                        const MatrixXd &webm_coords,
                        Eigen::Vector3d &utm_origin_pos, int &zone_id,
                        MatrixXd &utm_coords);