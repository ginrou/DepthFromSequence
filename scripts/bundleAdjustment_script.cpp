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

  static int MAX_CORNERS = 200;
  cv::Size sub_pix_win_size(10, 10);
  cv::TermCriteria term_crit( cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03 );
  cv::Size img_size;
  vector<Mat> input_images;
  vector< vector<Point2f> > track_points;

  // read images
  for (int i = 1; i < argc-1; ++i )
    input_images.push_back( imread(argv[i], CV_8UC1) );
  
  img_size = input_images[0].size();  

  // initialize trackers
  {
    vector<Point2f> points;
    cv::goodFeaturesToTrack( input_images[0], points, MAX_CORNERS, 0.075, 10, noArray(), 5, true, 0.04);
    cv::cornerSubPix(input_images[0], points, sub_pix_win_size, cv::Size(-1,-1), term_crit );
    track_points.push_back(points);
  }

  std::vector<uchar> total_status(track_points[0].size(), 1);

  // tracking sequence
  for( int i = 1; i < input_images.size(); ++i ) {
    std::vector<uchar> status;
    std::vector<float> error;
    
    std::vector<Point2f> curr_points, prev_points = track_points[i-1];
    Mat curr_image = input_images[i], prev_image = input_images[i-1];

    cv::calcOpticalFlowPyrLK( prev_image, curr_image, prev_points, curr_points, status, error);

    // masking current status
    for(int j = 0; j < status.size(); ++j )
      total_status[j] &= status[j];

    track_points.push_back(curr_points);
  }

  std::vector< vector<Point2f> > stable_track_points; // 全フレームで取得できた特徴点
  for( int i = 0; i < track_points.size(); ++i ) {
    std::vector<Point2f> points;
    
    for( int j = 0; j < track_points[i].size(); ++j ) {
      if(total_status[j]) {
	points.push_back(track_points[i][j]);
      }
    }
    stable_track_points.push_back(points);
  }

  bundleAdjustment::FeatureTracker tracker(input_images);
  tracker.track();
  tracker.draw_correspondences("tmp/match-");
  return 0;

  bundleAdjustment::Solver solver(stable_track_points, cv::Size(512, 512));
  solver.initialize();

  for( int i = 0; i < 50; ++i ) {
    solver.run_one_step();
    cout << i << "  c = " << solver.c << " error = " << solver.reprojection_error << endl;
  }

  FILE *fp = fopen(argv[argc-1], "w");
  for( int i = 0; i < solver.Np; ++i ) {
    double s = 0.0075;
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


  // draw to circle
  Mat img = input_images[0].clone();
  
  for( int i = 0; i < solver.Np; ++i ) {
    cv::Point pt = stable_track_points[0][i];
    double s = 0.0002;
    double z = 1.0 / fabs(solver.point_z[i]);

    cv::Scalar color = cv::Scalar::all(255);
    cv::circle(img, pt, 2, color, 1, 8, 0);

    char buf[256];
    sprintf(buf, "%2.3lf", z*s);
    std::string str = buf;
    cv::putText(img, str, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, color, 1, 1, false);

  }
  imwrite("tmp/depth.png", img);

  return 0;
}

