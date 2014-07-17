#include "bundle_adjustment.hpp"
#include "feature_tracking.hpp"

int main(int argc, char* argv[]) {

  // 画像をロード
  vector<Mat> input_images;
  for( int i = 1; i < argc-1; ++i ) {
    input_images.push_back( imread(argv[i], CV_8UC1) );
  }

  // 特徴点追跡
  BundleAdjustment::FeatureTracker feature_tracker(input_images);
  feature_tracker.track();
  vector< vector<Point2d> > track_points = feature_tracker.pickup_stable_points();

  // Solver を初期化
  BundleAdjustment::Solver solver( track_points );
  solver.init_with_first_image( track_points, cv::Size(480, 480), 7500.0, 55.0);

  for(int j = 0; j < solver.points.size(); ++j ) {
    cout << solver.points[j] << endl;
  }
  

  // bundle adjustment を実行
  while ( solver.should_continue ) {
    solver.run_one_step();
    printf("reprojection error = %e\n", solver.reprojection_error());
  }

  print_3d_point_to_file(solver.points, "after.txt");

  for( int i = 0; i < solver.Nc ; ++i ) {
    printf("cam %02d\n", i);
    cout << "\t" << solver.cam_t_vec[i] << endl;
    cout << "\t" << solver.cam_rot_vec[i] << endl;
  }

  return 0;

}
