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

  vector<Matx44d> P;
  for(int i = 0; i < input_images.size(); ++i ) {
    P.push_back( PlaneSweep::make_projection_matrix(cam_trans_vec[i], cam_rot_vec[i], input_images[i].size()) );
  }

  Matx33d M = PlaneSweep::homography_matrix( P[0], P[1], cam_trans_vec[1], -0.015 );
  cout << M << endl;

  Mat dst = input_images[1].clone();
  //  warpPerspective(input_images[1], dst, M, dst.size(), WARP_INVERSE_MAP);
  warpPerspective(input_images[1], dst, M, dst.size());
  imwrite("homo.png", dst);

  
  cout << Matx22d( 1,2,3,4)(0,1) << endl;

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
