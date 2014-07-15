#include <cstdlib>
#include <ctime>

#include "bundle_adjustment.hpp"

inline double drand() { return (double)rand()/RAND_MAX; }

std::vector<Point3d> mock_3d_points(int N, cv::Point3d min, cv::Point3d max, int reduced_by) {

  std::srand(std::time(0));

  std::vector<Point3d> points;
  for(int i = 0; i <= N; i+=3 ) {
    for(int j = 0; j <= N; j+=3 ) {
      for(int k = 0; k <= N; k+=3 ) {

	Point3d pt;
	pt.z = 1.0  / (min.z + (max.z - min.z) * (double)k / (double)N);
	pt.x = pt.z * (min.x + (max.x - min.x) * (double)i / (double)N);
	pt.y = pt.z * (min.y + (max.y - min.y) * (double)j / (double)N);

	//if( (i*j*k) % reduced_by == 0 ) points.push_back(pt);
	points.push_back(pt);

      }
    }
  }
  return points;
}

std::vector<Point3d> mock_3d_cam_t(int N) {

  Point3d min(-0.5, -0.5, -0.005), max(0.5, 0.5, 0.005);
  std::vector<Point3d> points;

  for(int i = 0; i < N; ++i ) {
    for(int j = 0; j < N; ++j ) {
      for(int k = 0; k < N; ++k ) {

	Point3d pt;
	pt.z = (min.z + (max.z - min.z) * (double)k / (double)N);
	pt.x = (min.x + (max.x - min.x) * (double)i / (double)N);
	pt.y = (min.y + (max.y - min.y) * (double)j / (double)N);
	points.push_back(pt);

      }
    }
  }

  return points;
}

std::vector<Point3d> mock_3d_cam_rot(int N) {

  Point3d min(-0.01, -0.01, -0.01), max(0.01, 0.01, 0.01);
  std::vector<Point3d> points;

  for(int i = 0; i < N; ++i ) {
    for(int j = 0; j < N; ++j ) {
      for(int k = 0; k < N; ++k ) {

	Point3d pt;
	pt.z = (min.z + (max.z - min.z) * (double)k / (double)N);
	pt.x = (min.x + (max.x - min.x) * (double)i / (double)N);
	pt.y = (min.y + (max.y - min.y) * (double)j / (double)N);
	points.push_back(pt);

      }
    }
  }

  return points;
}

std::vector<Point3d> mock_sequential_cam_t(int N) {
  std::vector<Point3d> ret;
  for( int i = 0; i < N; ++i ) {
    Point3d pt;
    pt.x = 0.5 * (double)i + 0.05 * (drand() - 0.5);
    pt.y = 0.05 * (drand() - 0.5);
    pt.z = 0.05 * (drand() - 0.5);
    ret.push_back(pt);
  }

  return ret;
}

std::vector<Point3d> mock_sequential_cam_rot(int N) {
  std::vector<Point3d> ret;
  for( int i = 0; i < N; ++i ) {
    Point3d pt;
    pt.x = 0.001 * (drand() - 0.5);
    pt.y = 0.001 * (drand() - 0.5);
    pt.z = 0.001 * (drand() - 0.5);
    ret.push_back(pt);
  }

  return ret;
}


std::vector<Point2d> project_3d_to_2d( Point3d cam_t, Point3d cam_rot, std::vector<Point3d> &points){
  std::vector<Point2d> projected;

  for(int i = 0; i < points.size(); ++i) {
    Point3d q, pt = points[i];
    q.x = ( pt.x - cam_rot.z * pt.y + cam_rot.y ) / pt.z + cam_t.x;
    q.y = ( pt.x * cam_rot.z + pt.y - cam_rot.x ) / pt.z + cam_t.y;
    q.z = ( -pt.x *cam_rot.y + cam_rot.x * pt.y + 1.0 ) / pt.z + cam_t.z;

    projected.push_back(Point2d(q.x/q.z, q.y/q.z));
  }
  return projected;
}

std::vector<Point2d> project_3d_to_2d_normalize( Point3d cam_t, Point3d cam_rot, std::vector<Point3d> &points, int normalize) {
  std::vector<Point2d> projected = project_3d_to_2d(cam_t, cam_rot, points);
  for(int i = 0; i < projected.size(); ++i) {
    Point2d pt = projected[i];
    int x = normalize * pt.x, y = normalize * pt.y;
    projected[i] = Point2d( x/(double)normalize, y/(double)normalize);
  }
  return projected;
}


cv::Mat1b print_point_to_image( vector<Point2d> pt_list,  cv::Size img_size ) {
  cv::Mat1b img(img_size.width, img_size.height);
  img.setTo(Scalar(0.0));

  for( int i = 0; i < pt_list.size(); ++i ) {
    Point2d orig = pt_list[i], pt;
    pt.x = img_size.width * orig.x /2.0 + img_size.width/2.0;
    pt.y = img_size.height * orig.y /2.0 + img_size.height/2.0;

    cv::Scalar color = cv::Scalar::all(255);
    cv::circle(img, pt, 2, color, 1, 8, 0);
  }

  for( int i = 0; i < pt_list.size()-1; i += 5 ) {
    Point2d orig = pt_list[i], pt, orig2 = pt_list[i+1], pt2;
    pt.x = img_size.width * orig.x /2.0 + img_size.width/2.0;
    pt.y = img_size.height * orig.y /2.0 + img_size.height/2.0;

    pt2.x = img_size.width * orig2.x /2.0 + img_size.width/2.0;
    pt2.y = img_size.height * orig2.y /2.0 + img_size.height/2.0;

    cv::Scalar color = cv::Scalar(255);
    cv::line(img, pt, pt2, color, 1, 8, 0);
  }

  return img;
}

void print_3d_point_to_file( vector<Point3d> pt_list, char filename[] ) {
  FILE *fp = fopen(filename, "w");
  for(int i = 0; i < pt_list.size(); ++i ) {
    double s = 1.0;
    fprintf(fp, "%lf,%lf,%lf\n", s * pt_list[i].x/pt_list[i].z, s * pt_list[i].y/pt_list[i].z, s * 1.0/pt_list[i].z);
  }
  fclose(fp);
}

void add_noise_to_init_values( BundleAdjustment::Solver &s ) {
  for ( int i = 0; i < s.Nc; ++i ) {
    s.cam_t_vec[i].x += 0.005 * ((double)rand()/RAND_MAX -0.5);
    s.cam_t_vec[i].y += 0.005 * ((double)rand()/RAND_MAX -0.5);
    s.cam_t_vec[i].z += 0.00005 * ((double)rand()/RAND_MAX -0.5);

    s.cam_rot_vec[i].x += 0.001 * ((double)rand()/RAND_MAX -0.5);
    s.cam_rot_vec[i].y += 0.001 * ((double)rand()/RAND_MAX -0.5);
    s.cam_rot_vec[i].x += 0.00001 * ((double)rand()/RAND_MAX -0.5);
  }

  for ( int j = 0; j < s.Np; ++j ) {
    s.points[j].x += ((double)rand()/RAND_MAX -0.5);
    s.points[j].y += ((double)rand()/RAND_MAX -0.5);
    s.points[j].z += ((double)rand()/RAND_MAX -0.5);
  }
}
