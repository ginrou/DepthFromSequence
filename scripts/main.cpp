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

  feature_tracker.draw_correspondences("tmp/matches-");

  // Solver を初期化
  BundleAdjustment::Solver solver( track_points );
  for(int i = 0; i < track_points.size(); ++i ) {
    for(int j = 0; j < track_points[i].size(); ++j ) {
      cout << track_points[i][j] << ",";
    }
    cout << endl;
  }
  // vector<Point3d> points_in_world = random_3d_points(solver.Np, Point3d( 0, 0, 20), Point3d( 100, 100, 300));
  // vector<Point3d> cam_t_vec = random_3d_cam_t(solver.Nc);
  // vector<Point3d> cam_rot_vec = random_3d_cam_rot(solver.Nc);
  // solver.init( points_in_world, cam_t_vec, cam_rot_vec );
  solver.init_with_first_image(100);

  // bundle adjustment を実行
  while ( solver.should_continue ) {
    solver.run_one_step();
    printf("reprojection error = %e\n", solver.reprojection_error());
  }

  return 0;

}

