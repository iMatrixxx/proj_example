/*
 * @Author: aoi
 * @Date: 2024-04-24 17:22:19
 * @LastEditors: aoi
 * @LastEditTime: 2024-04-28 22:29:30
 * @Description:
 * Copyright (c) Air by aoi, All Rights Reserved.
 */
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

using namespace std;

std::string data_dir = "";

Eigen::Matrix2d read_trans_matrix() {
  std::ifstream fin(data_dir + "/aoi_calib.txt");
  if (!fin) {
    std::cerr << "aoi_calib file not exsit: aoi_calib.txt" << std::endl;
  }
  // trans matrix
  double t1, t2;
  fin >> t1 >> t2;
  fin.close();

  Eigen::Matrix2d trans;
  trans << t1, -t2, t2, t1;
  return trans;
}

void read_localization(
    std::vector<std::pair<std::string, Eigen::Vector2d>> &locs,
    std::vector<Eigen::Vector2d> &rect_points) {
  std::ifstream fin(data_dir + "/aoi_loc.txt");
  if (!fin) {
    std::cerr << "aoi_loc file not exsit: aoi_loc.txt" << std::endl;
    return;
  }
  Eigen::Vector2d lb_point, ru_point;
  lb_point.x() = std::numeric_limits<double>::max();
  lb_point.y() = std::numeric_limits<double>::max();

  ru_point.x() = -std::numeric_limits<double>::max();
  ru_point.y() = -std::numeric_limits<double>::max();

  Eigen::Vector2d loc_point;
  std::string key;
  int key1, key2;
  locs.clear();
  while (fin >> key1 >> key2 >> loc_point.x() >> loc_point.y()) {
    if (loc_point.x() < lb_point.x()) lb_point.x() = loc_point.x();
    if (loc_point.y() < lb_point.y()) lb_point.y() = loc_point.y();

    if (loc_point.x() > ru_point.x()) ru_point.x() = loc_point.x();
    if (loc_point.y() > ru_point.y()) ru_point.y() = loc_point.y();

    // locs[key] = loc_point;
    // locs.push_back(std::make_pair(key, loc_point));
    locs.push_back(
        std::make_pair(std::to_string((key1 - 1) * 5 + key2), loc_point));
  }
  rect_points.clear();
  rect_points.push_back(lb_point);
  rect_points.push_back(ru_point);
  fin.close();
  return;
}

// void read_localization(std::map<std::string, Eigen::Vector2d> &locs,
//                        std::vector<Eigen::Vector2d> &rect_points) {
//   std::ifstream fin(data_dir + "/aoi_loc.txt");
//   if (!fin) {
//     std::cerr << "aoi_loc file not exsit: aoi_loc.txt" << std::endl;
//     return;
//   }
//   Eigen::Vector2d lb_point, ru_point;
//   lb_point.x() = std::numeric_limits<double>::max();
//   lb_point.y() = std::numeric_limits<double>::max();

//   ru_point.x() = -std::numeric_limits<double>::max();
//   ru_point.y() = -std::numeric_limits<double>::max();

//   Eigen::Vector2d loc_point;
//   std::string line;
//   while (std::getline(fin, line)) {  // 逐行读取文件
//     std::stringstream ss(line);
//     std::string token;
//     std::vector<std::string> tokens;

//     while (std::getline(ss, token, ',')) {  // 逐个分割逗号
//       tokens.push_back(token);
//     }

//     if (tokens.size() == 5) {               // 确保有五个数据
//       int data1 = std::stoi(tokens[0]);     // 转换为int
//       int data2 = std::stoi(tokens[1]);     // 转换为int
//       int data3 = std::stoi(tokens[2]);     // 转换为int
//       double data4 = std::stod(tokens[3]);  // 转换为double
//       double data5 = std::stod(tokens[4]);  // 转换为double

//       loc_point.x() = data4;
//       loc_point.y() = data5;

//       if (loc_point.x() < lb_point.x()) lb_point.x() = loc_point.x();
//       if (loc_point.y() < lb_point.y()) lb_point.y() = loc_point.y();

//       if (loc_point.x() > ru_point.x()) ru_point.x() = loc_point.x();
//       if (loc_point.y() > ru_point.y()) ru_point.y() = loc_point.y();

//     //   locs[tokens[0] + "_" + std::to_string(data3)] = loc_point;
//         locs[tokens[0]] = loc_point;
//     } else {
//       std::cerr << "Error: Incorrect number of data elements in line"
//                 << std::endl;
//     }
//   }

//   rect_points.clear();
//   rect_points.push_back(lb_point);
//   rect_points.push_back(ru_point);
//   fin.close();
//   return;
// }

int main(int argc, char *argv[]) {
  data_dir = argv[1];
  // read calib
  Eigen::Matrix2d trans = read_trans_matrix();

  // read localization
  std::unordered_map<std::string, Eigen::Vector2d> locs;
  std::vector<std::pair<std::string, Eigen::Vector2d>> locs_bak;
  std::vector<Eigen::Vector2d> rect_points;
  read_localization(locs_bak, rect_points);

  //
  //   const double stitch_image_width =
  //       (rect_points[1].x() - rect_points[0].x()) / 0.0015 + 5120 * 4;
  //   const double stitch_image_height =
  //       (rect_points[1].y() - rect_points[0].y()) / 0.0015 + 4096 * 5;

  Eigen::Vector2d sub_image_offset(5120.0, 4096.0);

  const double stitch_image_width = 10.0 / 0.0015 + 5120;
  const double stitch_image_height = 10.0 / 0.0015 + 4096;

  //
  cv::Mat stitch_image =
      cv::Mat::zeros(stitch_image_height, stitch_image_width, CV_8UC1);

  //
  std::vector<std::pair<std::string, cv::Point>> text_positions;
  int count = 0;
  for (auto &loc : locs_bak) {
    std::string image_name = loc.first + ".tiff";
    cv::Mat sub_image_tmp =
        cv::imread(data_dir + "/image/" + image_name, cv::IMREAD_GRAYSCALE);
    // cv::Mat sub_image_tmp =
    //     cv::imread(data_dir + "/3.tiff", cv::IMREAD_GRAYSCALE);
    if (sub_image_tmp.empty()) {
      std::cerr << "sub_image is not exist: " + image_name << std::endl;
      continue;
    }

    // Create a rectangle indicating the ROI
    cv::Rect roi(0, 0, sub_image_tmp.cols, sub_image_tmp.rows);

    // Crop the image using the ROI
    cv::Mat sub_image = sub_image_tmp(roi);

    // std::vector<cv::Point2f> img_corner_points;
    // int BOARDSIZE[2]{61, 76};  // 棋盘格每行每列角点个数
    // bool found_success = cv::findChessboardCorners(
    //     sub_image, cv::Size(BOARDSIZE[0], BOARDSIZE[1]), img_corner_points,
    //     cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
    //         cv::CALIB_CB_NORMALIZE_IMAGE);

    // // cv::TermCriteria criteria(cv::CV_TERMCRIT_EPS | cv::CV_TERMCRIT_ITER,
    // 30,
    // //                           0.001);
    // cv::TermCriteria criteria = cv::TermCriteria(
    //     cv::TermCriteria::Type::EPS + cv::TermCriteria::Type::MAX_ITER,
    //     40,      // maxCount=40
    //     0.001);  // epsilon=0.001

    // // 进一步提取亚像素角点
    // cv::cornerSubPix(sub_image, img_corner_points, cv::Size(11, 11),
    //                  cv::Size(-1, -1), criteria);

    // std::ofstream fout("/home/imatrix/aoi_algorithm/data/distort_obs.txt");

    // for (int i = 0; i < img_corner_points.size(); i++) {
    //   fout << "0"
    //        << " " << std::to_string(i / 61) + "_" + std::to_string(i %
    //        61)+"_0" << " "
    //        << std::fixed << img_corner_points[i].x << ' ' <<
    //        img_corner_points[i].y << std::endl;
    // }
    // fout.close();

    // std::vector<cv::Point3f> gt_corner_points;
    // for (int i = 0; i < 76; i++) {
    //   for (int j = 0; j < 61; j++) {
    //     cv::Point3f point(i * 1.0, j * 1.0, 1.0);
    //     gt_corner_points.push_back(point);
    //   }
    // }

    // cv::drawChessboardCorners(sub_image, cv::Size(BOARDSIZE[0],
    // BOARDSIZE[1]),
    //                           img_corner_points, found_success);

    // cv::Mat cameraMatrix, distCoeffs, R,
    //     T;  // 内参矩阵，畸变系数，旋转量，偏移量
    // std::vector<std::vector<cv::Point3f>> gt_corner_pointss;
    // gt_corner_pointss.push_back(gt_corner_points);

    // std::vector<std::vector<cv::Point2f>> img_corner_pointss;
    // img_corner_pointss.push_back(img_corner_points);

    // cv::calibrateCamera(gt_corner_pointss, img_corner_pointss,
    // sub_image.size(),
    //                     cameraMatrix, distCoeffs, R, T);

    // cout << "cameraMatrix:" << std::endl;
    // cout << cameraMatrix << endl;

    // cout << "*****************************" << endl;
    // cout << "distCoeffs:" << endl;
    // cout << distCoeffs << endl;
    // cout << "*****************************" << endl;

    // cout << "Rotation vector:" << endl;
    // cout << R << endl;

    // cout << "*****************************" << endl;
    // cout << "Translation vector:" << endl;
    // cout << T << endl;

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1.0;
    cameraMatrix.at<double>(1, 1) = 1.0;
    cameraMatrix.at<double>(0, 2) = sub_image.cols / 2.0;
    cameraMatrix.at<double>(1, 2) = sub_image.rows / 2.0;

    // -9.21086397e-11  2.04669321e-18 -7.50789904e-09 -8.61016430e-08
    // 0.000000001380684314148263
    // -0.000000000000000017329211,-0.000000154741448171158366,-0.000000514658184135754031
    // 4.30053737e-11 -5.76327462e-19 -6.46893879e-25 -1.80032894e-24
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    // distCoeffs.at<double>(0, 0) = -9.21086397e-11;
    // distCoeffs.at<double>(1, 0) = 2.04669321e-18;
    // distCoeffs.at<double>(2, 0) = -7.50789904e-09;
    // distCoeffs.at<double>(3, 0) = -8.61016430e-08;
    // 9.52116984e-11 -4.37165602e-18  6.74596177e-09  7.34871189e-08
    // 1.05731242e-11 1.28357755e-18 2.11199706e-08 7.94682917e-08

    distCoeffs.at<double>(0, 0) = 9.52116984e-11;
    distCoeffs.at<double>(1, 0) = -4.37165602e-18;
    distCoeffs.at<double>(2, 0) = 6.74596177e-09;
    distCoeffs.at<double>(3, 0) = 7.34871189e-08;

    // distCoeffs.at<double>(0, 0) = 1.05731242e-11;
    // distCoeffs.at<double>(1, 0) = 1.28357755e-18;
    // distCoeffs.at<double>(2, 0) = 2.11199706e-08;
    // distCoeffs.at<double>(3, 0) = 7.94682917e-08;

    // distCoeffs.at<double>(0, 0) = 4.30053737e-11;
    // distCoeffs.at<double>(1, 0) = -5.76327462e-19;
    // distCoeffs.at<double>(2, 0) = -6.46893879e-25;
    // distCoeffs.at<double>(3, 0) = -1.80032894e-24;

    cv::Mat sub_undistorted_image;
    cv::undistort(sub_image, sub_undistorted_image, cameraMatrix, distCoeffs);

    // cv::Mat sub_undistorted_image = cv::Mat::zeros(sub_image.rows,
    // sub_image.cols+ 50, CV_8UC1);  // 我要转化为512*512大小的
    // cv::resize(sub_image, sub_undistorted_image,
    // sub_undistorted_image.size());

    cv::imwrite(data_dir + "./sub_undistorted" + loc.first + ".tiff",
                sub_undistorted_image);

    // Create a rectangle indicating the ROI
    cv::Rect roi_undistort(
        sub_undistorted_image.cols * 0.01, sub_undistorted_image.rows * 0.01,
        sub_undistorted_image.cols * 0.98, sub_undistorted_image.rows * 0.98);

    // Crop the image using the ROI
    sub_undistorted_image = sub_undistorted_image(roi_undistort);

    count++;
    std::cout << "process image: " + image_name + "," << count << std::endl;
    //
    cv::Rect2i stitch_img_roi;
    cv::Rect2i sub_img_roi(0, 0, sub_undistorted_image.cols,
                           sub_undistorted_image.rows);

    Eigen::Vector2d offset =
        -trans * (loc.second - rect_points[0]) + sub_image_offset;

    if (offset.x() <= -sub_undistorted_image.cols ||
        offset.y() <= -sub_undistorted_image.rows ||
        offset.x() >= stitch_image_width || offset.y() >= stitch_image_height) {
      std::cout << "sub_img is out of the stitch image" << std::endl;
      // return false;
    }

    if (offset.x() < 0) {
      std::cout << "the sub_img:x is lower than stitch image:x_start "
                << std::endl;
      sub_img_roi.x = -offset.x();
      sub_img_roi.width += offset.x();
      offset.x() = 0;
    }

    if (offset.y() < 0) {
      std::cout << "the sub_img:y is lower than stitch image:y_start "
                << std::endl;
      sub_img_roi.y = -offset.y();
      sub_img_roi.height += offset.y();
      offset.y() = 0;
    }

    if (offset.x() + sub_img_roi.width > stitch_image_width) {
      std::cout << "the sub_img:x_end is larger than stitch image:x_end "
                << std::endl;
      sub_img_roi.width = stitch_image_width - offset.x();
    }

    if (offset.y() + sub_img_roi.height > stitch_image_height) {
      std::cout << "the sub_img:y_end is larger than stitch image:y_end "
                << std::endl;
      sub_img_roi.height = stitch_image_height - offset.y();
    }

    stitch_img_roi.x = offset.x();
    stitch_img_roi.y = offset.y();
    stitch_img_roi.width = sub_img_roi.width;
    stitch_img_roi.height = sub_img_roi.height;

    //
    cv::Mat roi_tile(stitch_image, stitch_img_roi);
    sub_undistorted_image(sub_img_roi).copyTo(roi_tile);

    cv::Point position(
        stitch_img_roi.x + stitch_img_roi.width / 2,
        stitch_img_roi.y + stitch_img_roi.height / 2);  // 文字的起始位置

    text_positions.push_back(std::make_pair(loc.first, position));
  }

  for (auto &position : text_positions) {
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;  // 字体类型
    double fontScale = 8.0;                   // 字体大小
    cv::Scalar color(0, 0, 0);                // 文字颜色，BGR 格式
    int thickness = 24;                       // 文字粗细
    // // 在图像上写入文字
    cv::putText(stitch_image, position.first, position.second, fontFace,
                fontScale, color, thickness);
  }

  int width_setp = int(stitch_image_width / 10.0);
  int height_setp = int(stitch_image_height / 10.0);
  //   for (int i = 0; i < 10; i++) {
  //     for (int j = 0; j < 10; j++) {
  //       cv::Mat spit_image = cv::Mat::zeros(height_setp, width_setp,
  //       CV_8UC1); cv::Rect2i split_img_roi(i * width_setp, j * height_setp,
  //       width_setp,
  //                                height_setp);
  //       stitch_image(split_img_roi).copyTo(spit_image);
  //       cv::imwrite(data_dir +
  //       "./stitch_image_"+std::to_string(i)+"_"+std::to_string(j)+".tiff",
  //       spit_image);
  //     }
  //   }

  cv::imwrite(data_dir + "./stitch_image.tiff", stitch_image);
}