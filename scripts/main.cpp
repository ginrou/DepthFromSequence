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
  for( int i = 1; i <= 32; ++i ) depths.push_back( 5000.0  / (double)i );

  // plane sweep + dencecrf で奥行きを求める
  Mat depth_map = dence_crf_image(input_images, solver.cam_t_vec, solver.cam_rot_vec, depths);
  imwrite("tmp/dence_crf_image.png", depth_map);

  for( int i = 0; i < 32; ++i ) cout << depths[i] << endl;

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

  for( int r = 0; r < H; ++r ) {
    for( int c = 0; c < W; ++c ) {

      int min_idx;
      double min_var = DBL_MAX;

      for( int d = 0; d < M; ++d ) {

	vector<Vec9d> vecs;

	for( int i = 0; i < images.size(); ++i ) {
	  Point2d pt = ps_homogenious_point(trans_vec[0], rot_vec[0], trans_vec[i], rot_vec[i],
					    Point2d(c, r), images[i].size(), depth_variation[d]);

	  int x = pt.x, y = pt.y;
	  if( x < 1 || x >= W-2 || y < 1 || y >= H-2 ) continue;

	  Vec9d v;
	  v[0] = images[i].at<uchar>(y-1, x-1);
	  v[1] = images[i].at<uchar>(y-1, x  );
	  v[2] = images[i].at<uchar>(y-1, x+1);
	  v[3] = images[i].at<uchar>(y  , x-1);
	  v[4] = images[i].at<uchar>(y  , x  );
	  v[5] = images[i].at<uchar>(y  , x+1);
	  v[6] = images[i].at<uchar>(y+1, x-1);
	  v[7] = images[i].at<uchar>(y+1, x  );
	  v[8] = images[i].at<uchar>(y+1, x+1);
	  vecs.push_back(v);

	  if ( r == 240 && c == 240 ) {
	    printf("d = %2d  i = %2d ", d, i);
	    for( int j = 0; j < 9; ++j ) printf(", %3d", (int)v[j]);

	    printf("\n");
	  }

	}

	Vec9d mean(0,0,0,0,0,0,0,0,0);
	for( int i = 0; i < vecs.size(); ++i ) {
	  for( int j = 0; j < 9; ++j ) {
	    mean[j] += vecs[i][j] / (double)vecs.size();
	  }
	}

	double var = 0.0;
	for( int i = 0; i < vecs.size(); ++i ) {
	  for( int j = 0; j < 9; ++j ) {
	    var += (vecs[i][j]-mean[j])*(vecs[i][j]-mean[j]);
	  }
	}

	if ( r == 240 && c == 240 ) printf("   var = %lf\n", var);

	if ( var == 0.0 )
	  unary[r*W*M + c*M + d] = 0.0;
	else
	  unary[r*W*M + c*M + d] = 1.0 / var;


	if( var < min_var ) {
	  min_idx = d;
	  min_var = var;
	}

      }
      pci.at<uchar>(r,c) = min_idx;

      //if( r % 10 == 0 && c % 10 == 0 ) printf("%d, %d -> %f\n", r,c, unary[r*W*M + c*M + 3]);

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
