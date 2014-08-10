#include "depth_from_sequence.hpp"

int main(int argc, char* argv[]) {

  // 画像をロード
  vector<Mat> input_images;
  for( int i = 1; i < argc-1; ++i ) {
    input_images.push_back( imread(argv[i], CV_8UC1) );
  }

  // 特徴点追跡
  FeatureTracker feature_tracker(input_images);
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

  // plane sweep の準備
  vector<double> depths;
  for( int i = 1; i <= 20; ++i ) depths.push_back( 1000.0 - 40 * i );

  // plane sweep + dencecrf で奥行きを求める
  PlaneSweep *ps = new PlaneSweep(input_images, solver.camera_params, depths);

  // 読み込みがカラー画像になるようにする
  Mat3b color_image(input_images[0].size(), CV_8UC3);
  for( int h = 0; h < color_image.rows; ++h ) {
    for( int w = 0; w < color_image.cols; ++w ) {
      Vec3b intenisty(input_images[0].at<unsigned char>(h,w));
      color_image.at<Vec3b>(h,w) = intenisty;
    }
  }

  ps->sweep(color_image);

  // output
  imwrite(argv[argc-1], ps->_depth_smooth);

  delete ps;
  return 0;
}

