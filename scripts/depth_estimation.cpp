#include "plane_sweep.hpp"

using namespace std;
using namespace cv;

vector<Point3d> load_points(const string& filename);
Mat photo_consistency_image(vector<Mat> images, vector<Point3d> trans_vec, vector<Point3d> rot_vec, int row, vector<double> depth_variation);

void show_points(vector<Mat> images, vector<Point3d> trans_vec, vector<Point3d> rot_vec, int row, int col, vector<double> depth_variation);

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

  vector<double> depths;
  for( int i = 30; i > 0; i-- ) depths.push_back( 500.0 * 200.0 / (double)i );


  show_points(input_images, cam_trans_vec, cam_rot_vec, 240, 240, depths);
  return 0;

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

Mat photo_consistency_image(vector<Mat> images, vector<Point3d> trans_vec, vector<Point3d> rot_vec,
			    int row,
			    vector<double> depth_variation)
{
  int cols = images[0].cols;
  Mat1b pci(depth_variation.size(), cols);
  pci.setTo(0);
  
  for(int c = 0; c < cols; ++c ) {
    for(int d_idx = 0; d_idx < depth_variation.size(); ++d_idx ) {
      double z = depth_variation[d_idx];
      vector<double> vals(images.size());
      
      cout << "(" << c << ", " << d_idx << ") ";
      for(int i = 0; i < images.size(); ++i ) {
	Point2d pt = ps_homogenious_point(trans_vec[0], rot_vec[0],
					  trans_vec[i], rot_vec[i],
					  Point2d(c, row), images[0].size(), z);
	cout << pt;
      }

      cout << endl;
      // printf("(%d, %d)  = [", c, d_idx);
      // for(int i = 0; i < vals.size(); ++i ) printf("%lf, ", vals[i]);
      // printf("]\n");

    }

  }

  return pci;
}

void show_points(vector<Mat> images, vector<Point3d> trans_vec, vector<Point3d> rot_vec, 
		 int row, int col, vector<double> depth_variation)
{
  for(int d_idx = 0; d_idx < depth_variation.size(); ++d_idx ) {
    double z = depth_variation[d_idx];

    for(int i = 0; i < images.size(); ++i ) {
      Point2d pt = ps_homogenious_point(trans_vec[0], rot_vec[0],
					trans_vec[i], rot_vec[i],
					Point2d(col, row), images[0].size(), z);
      cout << i << ", " << z << " -> "<< pt << " : "<< (int)images[i].at<uchar>((int)pt.y, (int)pt.x) << endl;

      Mat1b img = images[i].clone();
      cv::Scalar color = cv::Scalar::all(255);
      cv::circle(img, pt, 2, color, 1, 8, 0);

      char filename[256];
      sprintf(filename, "tmp/plane_sweep_%02d_%02d.png", i, d_idx);
      imwrite(filename, img);

    }
  }
}
