#include <proj.h>
#include <stdio.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;

Matrix3d estimateTransformationMatrix(const MatrixXd& sourcePoints,
                                      const MatrixXd& targetPoints) {
  // 将源点和目标点转换为齐次坐标
  MatrixXd homogeneousSource =
      MatrixXd::Ones(sourcePoints.rows(), sourcePoints.cols() + 1);
  MatrixXd homogeneousTarget =
      MatrixXd::Ones(targetPoints.rows(), targetPoints.cols() + 1);

  homogeneousSource.block(0, 0, sourcePoints.rows(), sourcePoints.cols()) =
      sourcePoints;
  homogeneousTarget.block(0, 0, targetPoints.rows(), targetPoints.cols()) =
      targetPoints;

  // 使用最小二乘法求解转换矩阵
  Matrix3d transformationMatrix =
      (homogeneousTarget.transpose() * homogeneousTarget)
          .ldlt()
          .solve(homogeneousTarget.transpose() * homogeneousSource)
          .transpose();

  return transformationMatrix;
}

int main(void) {
  // lla to webmercator
  auto lla2webmerctor =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:3857", NULL);

  // lla to utm
  auto lla2utm =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:32651", NULL);

  // utm to webmercator
  auto utm2webmercator =
      proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:32651", "EPSG:3857", NULL);

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

  // 31.2304° N, 121.4737° E
  //   PJ_COORD input;
  input.xyzt.x = 31.2304;   // Lat in deg
  input.xyzt.y = 121.4737;  // Long in deg
  input.xyzt.z = 0;
  input.xyzt.t = 0.0;
  //
  output = proj_trans(lla2utm, PJ_FWD, input);
  std::vector<std::vector<double>> samples = {
      {-50.0, -50.0}, {-50.0, 0.0},  {-50.0, 50.0}, {0.0, -50.0}, {0.0, 0.0},
      {0.0, 50.0},    {50.0, -50.0}, {50.0, 0.0},   {50.0, 50.0}};
  auto webm_center_point = proj_trans(utm2webmercator, PJ_FWD, output);
  std::vector<std::pair<std::vector<double>, PJ_COORD>> observations;
  for (auto& sample : samples) {
    PJ_COORD utm_point, webm_point;
    utm_point.xyzt.x = output.xyzt.x + sample[0];
    utm_point.xyzt.y = output.xyzt.y + sample[1];
    webm_point = proj_trans(utm2webmercator, PJ_FWD, utm_point);
    webm_point.xyzt.x = webm_point.xyzt.x - webm_center_point.xyzt.x;
    webm_point.xyzt.y = webm_point.xyzt.y - webm_center_point.xyzt.y;
    observations.push_back(std::make_pair(sample, webm_point));
    printf("utm_x:%f utm_y:%f webm_x:%f webm_y:%f\n", sample[0], sample[1],
           webm_point.xyzt.x, webm_point.xyzt.y);
  }

  MatrixXd spoints(18, 4);
  MatrixXd tpoints(18, 1);
  for (unsigned int i = 0; i < observations.size(); i++) {
    tpoints(i * 2, 0) = observations[i].first[0];
    tpoints(i * 2 + 1, 0) = observations[i].first[1];

    spoints(i * 2, 0) = observations[i].second.xyzt.x;
    spoints(i * 2, 1) = -observations[i].second.xyzt.y;
    spoints(i * 2, 2) = 1;
    spoints(i * 2, 3) = 0;

    spoints(i * 2 + 1, 0) = observations[i].second.xyzt.y;
    spoints(i * 2 + 1, 1) = observations[i].second.xyzt.x;
    spoints(i * 2 + 1, 2) = 0;
    spoints(i * 2 + 1, 3) = 1;
  }

  //
  MatrixXd transformationMatrix(1, 4);
  transformationMatrix = (spoints.transpose() * spoints)
                             .ldlt()
                             .solve(spoints.transpose() * tpoints)
                             .transpose();

  printf("cols: %d\n", transformationMatrix.cols());
  printf("rows: %d\n", transformationMatrix.rows());
  //   transformationMatrix.rows();
  // 输出结果
  double a = transformationMatrix.coeff(0, 0);
  printf("res: %f, %f, %f, %f\n", transformationMatrix(0, 0),
         transformationMatrix(0, 1), transformationMatrix(0, 2),
         transformationMatrix(0, 3));

  for (unsigned int i = 0; i < observations.size(); i++) {
    double x = observations[i].second.xyzt.x * transformationMatrix(0, 0) +
               -observations[i].second.xyzt.y * transformationMatrix(0, 1) +
               transformationMatrix(0, 2);

    double y = observations[i].second.xyzt.x * transformationMatrix(0, 1) +
               observations[i].second.xyzt.y * transformationMatrix(0, 0) +
               transformationMatrix(0, 3);
    printf("true_x:%f ture_y:%f trans_x:%f trans_y:%f\n",
           observations[i].first[0], observations[i].first[1], x, y);
  }

  //   std::vector<std::vector<double>> samples = {
  //       {-100.0, -100.0}, {-100.0, 0.0}, {-100.0, 100.0},
  //       {0.0, -100.0},    {0.0, 0.0},    {0.0, 100.0},
  //       {100.0, -100.0},  {100.0, 0.0},  {100.0, 100.0}};
  std::vector<std::vector<double>> test_samples;
  for (int i = -50; i < 50; i++) {
    for (int j = -50; j < 50; j++) {
      test_samples.push_back({i * 1.0, j * 1.0});
    }
  }

  std::vector<std::pair<std::vector<double>, PJ_COORD>> test_observations;
  for (auto& sample : test_samples) {
    PJ_COORD utm_point, webm_point;
    utm_point.xyzt.x = output.xyzt.x + sample[0];
    utm_point.xyzt.y = output.xyzt.y + sample[1];
    webm_point = proj_trans(utm2webmercator, PJ_FWD, utm_point);
    webm_point.xyzt.x = webm_point.xyzt.x - webm_center_point.xyzt.x;
    webm_point.xyzt.y = webm_point.xyzt.y - webm_center_point.xyzt.y;
    test_observations.push_back(std::make_pair(sample, webm_point));
    printf("utm_x:%f utm_y:%f webm_x:%f webm_y:%f\n", sample[0], sample[1],
           webm_point.xyzt.x, webm_point.xyzt.y);
  }

  for (unsigned int i = 0; i < test_observations.size(); i++) {
    double x =
        test_observations[i].second.xyzt.x * transformationMatrix(0, 0) +
        -test_observations[i].second.xyzt.y * transformationMatrix(0, 1) +
        transformationMatrix(0, 2);

    double y = test_observations[i].second.xyzt.x * transformationMatrix(0, 1) +
               test_observations[i].second.xyzt.y * transformationMatrix(0, 0) +
               transformationMatrix(0, 3);
    printf(
        "true_x:%f ture_y:%f trans_x:%f trans_y:%f delta_x: %f, delta_y:%f\n",
        test_observations[i].first[0], test_observations[i].first[1], x, y,
        (test_observations[i].first[0] - x),
        (test_observations[i].first[1] - y));
  }

  return 0;
}