#include "plane_sweep.hpp"

using namespace std;
using namespace cv;

vector<Point3d> load_points(const string& filename);

// argv[] = [obj_name, img0, img1, ... imgN-1, cam_trans_file, cam_rot_file]
int main(int argc, char* argv[]) {

  // 画像をロード
  vector<Mat> input_images;
  for( int i = 1; i < argc-2; ++i ) {
    input_images.push_back( imread(argv[i], CV_8UC1) );
  }

  vector<Point3d> cam_trans_vec = load_points(String(argv[argc-2]));
  vector<Point3d> cam_rot_vec = load_points(String(argv[argc-1]));

  if( cam_trans_vec.size() != input_images.size() || cam_rot_vec.size() != input_images.size() ) exit(1);

  Mat ref_image = input_images[0];
  Mat1b disp_map(ref_image.rows, ref_image.cols);

  for( int r = 0; r < disp_map.rows; ++r ) {
    for( int c = 0; c < disp_map.cols; ++c ) {

      int max_disp = -64, min_disp = -128;
      int disp;
      double min_val = DBL_MAX, ref_val = ref_image.at<uchar>(r,c);
      for( int d = min_disp; d < max_disp; ++d ) {

	double diff = 0.0;
	for( int i = 1; i < input_images.size(); ++i ) {
	  double depth = ref_image.cols * cam_trans_vec[i].x / (double)d;
	  double val = ps_intensity_at_depth( input_images[i], cam_trans_vec[0], cam_rot_vec[0],
					      cam_trans_vec[1], cam_rot_vec[1], Point2d(c,r), depth);
	  //printf("(%d, %d), d = %d, i = %d, depth = %lf, val = %lf, diff = %lf\n", r,c,d,i,depth,val, fabs(ref_val-val));

	  if ( val == PlaneSweep::OutOfRangeIntensity ) diff += 128;
	  else val += fabs(val-ref_val);

	}

	if( diff < min_val ) {
	  min_val = diff;
	  disp = d;
	}

      }

      disp_map.at<uchar>(r,c) = abs(disp);
      if( r % 10 == 0 && c % 10 == 0 )
	printf("(%d, %d) -> %d\n", r, c, disp);

    }
  }

  imwrite("dispmap.png", disp_map);

  return 0;

}



vector<Point3d> load_points(const string& filename) {
  cout << filename << endl;
  vector<Point3d> vec;
  FILE *fp = fopen(filename.c_str(), "r");
  double x, y, z;
  while( fscanf(fp, "%lf,%lf,%lf", &x, &y, &z) != EOF ) {
    vec.push_back(Point3d(x,y,z));
  }
  fclose(fp);

  return vec;
}
