#include "bundle_adjustment.hpp"

bool test_reproj_gradient( BundleAdjustment::Solver &s );

int main(int argc, char* argv[]) {

  vector<Point3d> points_in_world = mock_3d_points(10, Point3d( -10, -10, 10), Point3d( 10, 10, 30), 20);
  vector<Point3d> cam_t_vec = mock_sequential_cam_t(10);
  vector<Point3d> cam_rot_vec = mock_sequential_cam_rot(10);

  // 投影点を計算。画像に書き込む
  vector< vector<Point2d> > project_points;
  for( int j = 0; j < cam_t_vec.size(); ++j ) {
    project_points.push_back( project_3d_to_2d( cam_t_vec[j], cam_rot_vec[j], points_in_world) );
  }

  // Solver を初期化
  BundleAdjustment::Solver solver( project_points );
  solver.init( points_in_world, cam_t_vec, cam_rot_vec );

  add_noise_to_init_values(solver);

  while ( solver.should_continue ) {
    solver.run_one_step();
    printf("reprojection error = %e\n", solver.reprojection_error());
  }

  print_3d_point_to_file( solver.points, "test.txt");

  return 0;

}

