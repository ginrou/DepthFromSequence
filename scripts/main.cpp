#include "bundle_adjustment.hpp"
#include "feature_tracking.hpp"

cv::Mat drawDepth( cv::Mat base_image, vector<Point2d> points, vector<double> depth);
void write_cam_params_to_file(const string& filename, vector<Point3d> params);

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

  print_3d_point_to_file(solver.points, "after.txt", 0.2);

  for( int i = 0; i < solver.Nc ; ++i ) {
    printf("cam %02d\n", i);
    cout << "\t" << solver.cam_t_vec[i] << endl;
    cout << "\t" << solver.cam_rot_vec[i] << endl;
  }

  vector<double> depth;
  for( int j = 0; j < solver.Np ; ++j ) {
    depth.push_back(1.0/solver.points[j].z);
  }

  imwrite("tmp/depth.png", drawDepth( input_images[0], track_points[0], depth));
  write_cam_params_to_file("cam_trans.txt", solver.cam_t_vec);
  write_cam_params_to_file("cam_rot.txt", solver.cam_rot_vec);

  return 0;

}

cv::Mat drawDepth( cv::Mat base_image, vector<Point2d> points, vector<double> depth) {
  cv::Mat img = base_image.clone();
  for( int i = 0; i < points.size(); ++i ) {
    cv::Scalar color = cv::Scalar::all(255);
    cv::circle(img, points[i], 2, color, 1, 8, 0);

    char buf[256];
    sprintf(buf, "%.2lf", depth[i]);
    std::string str = buf;
    cv::putText(img, str, points[i], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, color, 1, 1, false);

  }
  return img;
}

void write_cam_params_to_file(const string& filename, vector<Point3d> params) {
  FILE *fp = fopen(filename.c_str(), "w");
  for(int i = 0; i < params.size(); ++i ) {
    fprintf(fp, "%lf,%lf,%lf\n", params[i].x, params[i].y, params[i].z);
  }
  fclose(fp);
}
