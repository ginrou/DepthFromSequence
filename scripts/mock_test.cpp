#include "bundle_adjustment.hpp"

int main(int argc, char* argv[]) {

  vector<Point3f> points_in_world = mock_3d_points(10, Point3f( -10, -10, 10), Point3f( 10, 10, 30), 20);
  vector<Point3f> cam_t_vec = mock_3d_cam_t(3);
  vector<Point3f> cam_rot_vec = mock_3d_cam_rot(3);

  if( cam_t_vec.size() != cam_rot_vec.size() ) exit(1);

  // 初期解をファイルへ書き込む
  FILE *fp = fopen(argv[argc-1], "w");
  for( int i = 0; i < points_in_world.size(); ++i ) {
    Point3f pt = points_in_world[i];
    fprintf(fp, "%lf,%lf,%lf\n", pt.x/pt.z, pt.y/pt.z, 1.0/pt.z);
  }
  fclose(fp);

  // 投影点を計算。画像に書き込む
  vector< vector<Point2f> > project_points;
  for( int j = 0; j < cam_t_vec.size(); ++j ) {
    project_points.push_back( project_3d_to_2d( cam_t_vec[j], cam_rot_vec[j], points_in_world) );
    char filename[256];
    sprintf(filename, "tmp/projected-%02d.png", j);
    imwrite(filename, print_point_to_image( project_points[j], Size(512, 512)));
  }

  // Solver を初期化
  BundleAdjustment::Solver solver( project_points );
  solver.init( points_in_world, cam_t_vec, cam_rot_vec );

  // 再投影誤差が0になることを確認する
  printf("reprojection error = %e\n", solver.reprojection_error());

  return 0;
  
}
