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
