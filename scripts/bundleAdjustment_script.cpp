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

  vector<Mat> input_images;

  // read images
  for (int i = 1; i < argc-1; ++i )
    input_images.push_back( imread(argv[i], CV_8UC1) );
  
  bundleAdjustment::FeatureTracker tracker(input_images);
  tracker.track();

  vector< vector<cv::Point2f> > stable_track_points = tracker.stable_track_points;

  bundleAdjustment::Solver solver(stable_track_points, cv::Size(512, 512));
  solver.initialize();

  for( int i = 0; i < 50; ++i ) {
    solver.run_one_step();
    cout << i << "  c = " << solver.c << " error = " << solver.reprojection_error << endl;
  }

  FILE *fp = fopen(argv[argc-1], "w");
  for( int i = 0; i < solver.Np; ++i ) {
    double s = 0.004;
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

  Mat hoge = bundleAdjustment::Solver::depth_drawn(input_images[0],
						   stable_track_points[0],
						   solver.point_z);
  imwrite("tmp/depth.png", hoge);
  return 0;
  
}

