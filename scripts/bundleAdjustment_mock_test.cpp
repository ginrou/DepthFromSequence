#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/stitcher.hpp>

#include "Eigen/Core"
#include "Eigen/LU"
using namespace Eigen;

#include "bundle_adjustment.hpp"
#include "feature_tracking.hpp"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {

  vector<Point3f> pt_vec = reduce(gen_3d_points(10, cv::Point3f(-10,-10,10), cv::Point3f(10,10,30)) , 20);
  vector<Point3f> cam_t_vec = gen_3d_cam_t(3);
  vector<Point3f> cam_rot_vec = gen_3d_cam_rot(3);

  vector< vector<Point2f> > proj_vec;
  for( int i = 0 ; i < cam_t_vec.size() ; ++i ) {
    proj_vec.push_back( project_3d_to_2d(cam_t_vec[i], cam_rot_vec[i], pt_vec));

    Mat img = print_point_to_image( proj_vec[i], cv::Size(512, 512) );
    char filename[256];
    sprintf(filename, "tmp/pointed-%02d.png", i);
    imwrite(filename, img);
  }

  bundleAdjustment::Solver solver(proj_vec, cv::Size(512, 512));
  solver.mock_init( pt_vec, cam_t_vec, cam_rot_vec );

  for( int i = 0; i < 5; ++i ) {
    solver.run_one_step();
    cout << i << "  c = " << solver.c << " error = " << solver.reprojection_error << endl;

    for( int j = 0 ; j < pt_vec.size() ; ++j ) {
      Point3f a = pt_vec[j], b = Point3f(solver.point_x[j], solver.point_y[j], solver.point_z[j]);
      printf("point %d: ", j);
      printf(" a = %lf, %lf, %lf  ", a.x, a.y, a.z);
      printf(" b = %lf, %lf, %lf  ", b.x, b.y, b.z);
      printf(" diff = %lf, %lf, %lf  \n", a.x - b.x, a.y - b.y, a.z - b.z);
    }

  }

  FILE *fp = fopen(argv[argc-1], "w");
  for( int i = 0; i < solver.Np; ++i ) {
    double s = 1.0;
    double z = 1.0 / fabs(solver.point_z[i]);
    double x = solver.point_x[i] / solver.point_z[i];
    double y = solver.point_y[i] / solver.point_z[i];
    fprintf(fp, "%lf,%lf,%lf\n", x*s, y*s, z*s);
    printf("%lf,\t%lf,\t%lf\n", x*s, y*s, z*s);
  }
  fclose(fp);

  for ( int i = 0; i < solver.Nc; ++i ) {
    printf("camera %02d : trans = %lf, %lf, %lf\n", i
	   , solver.cam_t_x[i]
	   , solver.cam_t_y[i]
	   , solver.cam_t_z[i]);
    printf("          : pose = %lf, %lf, %lf\n"
	   , solver.cam_pose_x[i]
	   , solver.cam_pose_y[i]
	   , solver.cam_pose_z[i]);

  }

  return 0;
  
}

