#include "bundle_adjustment.hpp"

int main(int argc, char* argv[]) {

  vector<Point3f> points_in_world = mock_3d_points(10, Point3f( -10, -10, 10), Point3f( 10, 10, 30), 20);
  vector<Point3f> cam_t_vec = mock_3d_cam_t(3);
  vector<Point3f> cam_rot_vec = mock_3d_cam_rot(3);

  // 初期解をファイルへ書き込む
  FILE *fp = fopen(argv[argc-1], "w");
  for( int i = 0; i < points_in_world.size(); ++i ) {
    Point3f pt = points_in_world[i];
    fprintf(fp, "%lf,%lf,%lf\n", pt.x/pt.z, pt.y/pt.z, 1.0/pt.z);
  }
  fclose(fp);

  return 0;
  
}

