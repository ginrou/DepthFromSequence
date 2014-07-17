#include "bundle_adjustment.hpp"

bool test_reproj_gradient( BundleAdjustment::Solver &s );

int main(int argc, char* argv[]) {

  vector<Point3d> points_in_world = mock_3d_points(10, Point3d( -1000, -1000, 1000), Point3d( 1000, 1000, 3000), 20);
  vector<Point3d> cam_t_vec = mock_sequential_cam_t(10);
  vector<Point3d> cam_rot_vec = mock_sequential_cam_rot(10);

  // 投影点を計算。画像に書き込む
  vector< vector<Point2d> > project_points;
  for( int j = 0; j < cam_t_vec.size(); ++j ) {
    project_points.push_back( project_3d_to_2d_normalize( cam_t_vec[j], cam_rot_vec[j], points_in_world, 512) );
  }

  for( int j = 0; j < points_in_world.size(); ++j ) {
    cout << points_in_world[j] << endl;
  }

  // Solver を初期化
  BundleAdjustment::Solver solver( project_points );
  solver.init_with_first_image( project_points, Size(512,512), 20, 45);

  while ( solver.should_continue ) {
    solver.run_one_step();
    printf("reprojection error = %e\n", solver.reprojection_error());
  }

  print_3d_point_to_file( solver.points, "test.txt");

  for( int j = 0; j < solver.points.size(); ++j ) {
    double dx = fabs(points_in_world[j].x - solver.points[j].x );
    double dy = fabs(points_in_world[j].y - solver.points[j].y );
    double dz = fabs(points_in_world[j].z - solver.points[j].z );
    printf("%02d: %e, %e, %e\n", j, dx, dy, dz);
  }

  for( int i = 0; i < solver.Nc ; ++i ) {
    printf("cam %02d\n", i);
    cout << "\t" << solver.cam_t_vec[i] << endl;
    cout << "\t" << solver.cam_rot_vec[i] << endl;
  }


  return 0;

}

