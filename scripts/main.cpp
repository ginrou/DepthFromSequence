#include "bundle_adjustment.hpp"
#include "feature_tracking.hpp"
#include "plane_sweep.hpp"
#include "densecrf.h"

typedef Vec<double, 9> Vec9d;

cv::Mat drawDepth( cv::Mat base_image, vector<Point2d> points, vector<double> depth);
void write_cam_params_to_file(const string& filename, vector<Point3d> params);
Mat dence_crf_image(vector<Mat> images, vector<Point3d> trans_vec, vector<Point3d> rot_vec, vector<double> depth_variation);

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

  // plane sweep の準備
  vector<double> depths;
  for( int i = 1; i <= 20; ++i ) depths.push_back( 1000.0 - 40 * i );

  vector<Camera> cameras(solver.cam_t_vec.size());
  for(int i = 0; i < cameras.size(); ++i ) {
    cameras[i].t = solver.cam_t_vec[i];
    cameras[i].rot = solver.cam_rot_vec[i];
  }

  Mat3b color_image(input_images[0].size());
  for( int h = 0; h < color_image.rows; ++h ) {
    for( int w = 0; w < color_image.cols; ++w ) {
      unsigned char intensity = input_images[0].at<uchar>(h,w);
      color_image.at<Vec3b>(h,w)[0] = intensity;
      color_image.at<Vec3b>(h,w)[1] = intensity;
      color_image.at<Vec3b>(h,w)[2] = intensity;
    }
  }

  PlaneSweep *ps = new PlaneSweep(input_images, cameras, depths);
  ps->sweep(color_image);
  imwrite("tmp/plane_sweep.png", ps->_depth_smooth);

  delete ps;
  exit(0);

  // plane sweep + dencecrf で奥行きを求める
  Mat depth_map = dence_crf_image(input_images, solver.cam_t_vec, solver.cam_rot_vec, depths);
  imwrite("tmp/dence_crf_image.png", depth_map);

  for( int i = 0; i < depths.size(); ++i ) cout << depths[i] << endl;

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


Mat dence_crf_image(vector<Mat> images, 
		    vector<Point3d> trans_vec, 
		    vector<Point3d> rot_vec, 
		    vector<double> depth_variation)
{

  int W = images[0].cols;
  int H = images[0].rows;
  int M = depth_variation.size();

  // 輝度一致度(輝度の分散)をunary energyとして代入 unary(x,y,i) = -log(var)
  float *unary = new float[W*H*M];
  Mat1b pci(W,H);  

  Mat1b m(images.size()+1, M);
  m.setTo(0);

  for(int r = 0; r < H; ++r ) {
    for(int c = 0; c < W; ++c ) {
      double ref = images[0].at<uchar>(r,c);

      double min_val = DBL_MAX;
      int min_idx;

      for( int d = 0; d < M; ++d ) {
	double sqr_err = 0.0;
	for( int i = 1; i < images.size(); ++i ) {
	  double v = ps_intensity_at_depth(images[i], trans_vec[0], rot_vec[0], trans_vec[i], rot_vec[i],
					   Point2d(c,r), depth_variation[d]);

	  if(v != PlaneSweep::OutOfRangeIntensity)
	    sqr_err += (ref-v)*(ref-v);
	  else
	    sqr_err += 255 * 255; // とりあえず最大の値を足しておく

	}

	unary[r*W*M + c*M + d] = 0.001 * sqr_err;

	if( sqr_err < min_val ) {
	  min_val = sqr_err;
	  min_idx = d;
	}

      }

      if (r%10 == 0 && c%10 == 0 ) {
	printf("%3d, %3d => ", r, c);
	for( int d = 0; d < M; ++d ) printf("%f, ", unary[r*W*M + c*M + d]);
	printf("\n");
      }

      pci.at<uchar>(r,c) = min_idx;

    }
  }

  imwrite("tmp/pci.png", pci);  
  unsigned char* img = new unsigned char[W*H*3]; // rgb
  for(int h = 0; h < H; ++h ) {
    for(int w = 0; w < W; ++w ) {
      img[h*W*3 + w*3 + 0 ] = images[0].at<uchar>(h,w);
      img[h*W*3 + w*3 + 1 ] = images[0].at<uchar>(h,w);
      img[h*W*3 + w*3 + 2 ] = images[0].at<uchar>(h,w);
    }
  }

  DenseCRF2D crf(W,H,M);
  crf.setUnaryEnergy(unary);
  crf.addPairwiseGaussian(3,3,3);
  crf.addPairwiseBilateral(8, 8, 20, 20, 20, img, 10);

  short *map = new short[W*H];
  crf.map(10, map);

  Mat1b ret(W,H);
  for(int r = 0; r < H; ++r ) {
    for(int c = 0; c < W; ++c ) {
      ret.at<uchar>(r,c) = map[r*W+c];
    }
  }

  delete [] unary;
  delete [] img;
  delete [] map;
  
  return ret;
}
