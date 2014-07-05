#include <cstdlib>
#include <ctime>

#include "bundle_adjustment.hpp"

std::vector<Point3f> mock_3d_points(int N, cv::Point3f min, cv::Point3f max, int reduced_by) {

  std::srand(std::time(0));

  std::vector<Point3f> points;
  for(int i = 0; i <= N; ++i ) {
    for(int j = 0; j <= N; ++j ) {
      for(int k = 0; k <= N; ++k ) {

	Point3f pt;
	pt.z = 1.0  / (min.z + (max.z - min.z) * (double)k / (double)N);
	pt.x = pt.z * (min.x + (max.x - min.x) * (double)i / (double)N);
	pt.y = pt.z * (min.y + (max.y - min.y) * (double)j / (double)N);

	if( rand() % reduced_by == 0 ) points.push_back(pt);

      }
    }
  }
  return points;
}

std::vector<Point3f> mock_3d_cam_t(int N) {

  Point3f min(-0.5, -0.5, -0.005), max(0.5, 0.5, 0.005);
  std::vector<Point3f> points;

  for(int i = 0; i < N; ++i ) {
    for(int j = 0; j < N; ++j ) {
      for(int k = 0; k < N; ++k ) {

	Point3f pt;
	pt.z = (min.z + (max.z - min.z) * (double)k / (double)N);
	pt.x = (min.x + (max.x - min.x) * (double)i / (double)N);
	pt.y = (min.y + (max.y - min.y) * (double)j / (double)N);
	points.push_back(pt);

      }
    }
  }

  return points;
}

std::vector<Point3f> mock_3d_cam_rot(int N) {

  Point3f min(-0.01, -0.01, -0.01), max(0.01, 0.01, 0.01);
  std::vector<Point3f> points;

  for(int i = 0; i < N; ++i ) {
    for(int j = 0; j < N; ++j ) {
      for(int k = 0; k < N; ++k ) {

	Point3f pt;
	pt.z = (min.z + (max.z - min.z) * (double)k / (double)N);
	pt.x = (min.x + (max.x - min.x) * (double)i / (double)N);
	pt.y = (min.y + (max.y - min.y) * (double)j / (double)N);
	points.push_back(pt);

      }
    }
  }

  return points;
}

std::vector<Point2f> project_3d_to_2d( Point3f cam_t, Point3f cam_rot, std::vector<Point3f> &points){
  std::vector<Point2f> projected;

  for(int i = 0; i < points.size(); ++i) {
    Point3f q, pt = points[i];
    q.x = ( pt.x - cam_rot.z * pt.y + cam_rot.y ) / pt.z + cam_t.x;
    q.y = ( pt.x * cam_rot.z + pt.y - cam_rot.x ) / pt.z + cam_t.y;
    q.z = ( -pt.x *cam_rot.y + cam_rot.x * pt.y + 1.0 ) / pt.z + cam_t.z;

    projected.push_back(Point2f(q.x/q.z, q.y/q.z));
  }
  return projected;
}

cv::Mat1b print_point_to_image( vector<Point2f> pt_list,  cv::Size img_size ) {
  cv::Mat1b img(img_size.width, img_size.height);
  img.setTo(Scalar(0.0));

  for( int i = 0; i < pt_list.size(); ++i ) {
    Point2f orig = pt_list[i], pt;
    pt.x = img_size.width * orig.x /2.0 + img_size.width/2.0;
    pt.y = img_size.height * orig.y /2.0 + img_size.height/2.0;

    cv::Scalar color = cv::Scalar::all(255);
    cv::circle(img, pt, 2, color, 1, 8, 0);
  }

  for( int i = 0; i < pt_list.size()-1; i += 5 ) {
    Point2f orig = pt_list[i], pt, orig2 = pt_list[i+1], pt2;
    pt.x = img_size.width * orig.x /2.0 + img_size.width/2.0;
    pt.y = img_size.height * orig.y /2.0 + img_size.height/2.0;

    pt2.x = img_size.width * orig2.x /2.0 + img_size.width/2.0;
    pt2.y = img_size.height * orig2.y /2.0 + img_size.height/2.0;

    cv::Scalar color = cv::Scalar(255);
    cv::line(img, pt, pt2, color, 1, 8, 0);
  }

  return img;
}

