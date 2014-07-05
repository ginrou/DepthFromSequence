#ifndef __BUNDLE_ADJUTMENT_HPP__
#define __BUNDLE_ADJUTMENT_HPP__

#include <opencv2/opencv.hpp>
#include "Eigen/Core"

using namespace std;
using namespace cv;
using namespace Eigen;

// 適当な解を作る
std::vector<Point3f> mock_3d_points(int N, cv::Point3f min, cv::Point3f max, int reduced_by);
std::vector<Point3f> mock_3d_cam_t(int N);
std::vector<Point3f> mock_3d_cam_rot(int N);
std::vector<Point2f> project_3d_to_2d( Point3f cam_t, Point3f cam_rot, std::vector<Point3f> &points);

cv::Mat1b print_point_to_image( vector<Point2f> pt_list,  cv::Size img_size ); // 適当に正規化する

#endif // __BUNDLE_ADJUTMENT_HPP__


