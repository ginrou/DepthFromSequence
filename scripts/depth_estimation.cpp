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
  for( int i = 1; i < 64; ++i ) depths.push_back( 500.0 * 100.0 / (double)i );

  if(0){ //奥行きを仮定した時の各画像の位置を描画する
    show_points(input_images, cam_trans_vec, cam_rot_vec, 240, 240, depths);
  }

  if(0) { //ある行についての奥行きを描画する
    Mat pci = photo_consistency_image(input_images, cam_trans_vec, cam_rot_vec, 240, depths);
    imwrite("tmp/pci.png", pci);
  }

  Mat1b disp_map(input_images[0].size());

  for( int r = 0; r < disp_map.rows; ++r ) {
    for( int c = 0; c < disp_map.cols; ++c ) {
      int d = ps_depth_index_for_point(input_images, cam_trans_vec, cam_rot_vec, r, c, depths);
      if( r % 10 == 0 && c % 10 == 0 )
	printf("%d, %d -> %d\n", r, c, d);
      disp_map.at<uchar>(r,c) = d;
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
      double mean = 0;

      for(int i = 0; i < images.size(); ++i ) {
	Point2d pt = ps_homogenious_point(trans_vec[0], rot_vec[0],
					  trans_vec[i], rot_vec[i],
					  Point2d(c, row), images[0].size(), z);

	if ( pt.x < 0 || pt.x >= images[i].cols || pt.y < 0 || pt.y >= images[i].rows ) continue;

	double intensity = (double)images[i].at<uchar>( (int)pt.y, (int)pt.x );
	vals.push_back(intensity);
	mean += intensity;

      }

      mean /= (double)vals.size();
      double var = 0;
      for(int i = 0; i < vals.size(); ++i ) var += (vals[i]-mean)*(vals[i]-mean);

      var = sqrt(var/(double)vals.size());

      printf("%d, %d -> %lf\n", d_idx, c, var);
      pci.at<uchar>(d_idx, c) = var;

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
