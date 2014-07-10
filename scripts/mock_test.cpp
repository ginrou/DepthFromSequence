#include "bundle_adjustment.hpp"

bool test_reproj_gradient( BundleAdjustment::Solver &s );

int main(int argc, char* argv[]) {

  vector<Point3d> points_in_world = mock_3d_points(10, Point3d( -10, -10, 10), Point3d( 10, 10, 30), 20);
  vector<Point3d> cam_t_vec = mock_3d_cam_t(3);
  vector<Point3d> cam_rot_vec = mock_3d_cam_rot(3);

  if( cam_t_vec.size() != cam_rot_vec.size() ) exit(1);

  // 初期解をファイルへ書き込む
  FILE *fp = fopen(argv[argc-1], "w");
  for( int i = 0; i < points_in_world.size(); ++i ) {
    Point3d pt = points_in_world[i];
    fprintf(fp, "%lf,%lf,%lf\n", pt.x/pt.z, pt.y/pt.z, 1.0/pt.z);
  }
  fclose(fp);

  // 投影点を計算。画像に書き込む
  vector< vector<Point2d> > project_points;
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

  // ba_get_reproject_gradient のテスト
  if( test_reproj_gradient( solver ) == false ) {
    printf("!!! ba_get_reproject_gradient ng !!!\n");
    exit(0);
  }

  printf("ba_get_reproject_gradient ok\n");

  imwrite("tmp/ground_truth.png", print_point_to_image( project_points[0], Size(512, 512)));
  imwrite("tmp/before.png", print_point_to_image( project_3d_to_2d( solver.cam_t_vec[0], solver.cam_rot_vec[0], solver.points ), Size(512, 512) ));

  fp = fopen("before.txt", "w");
  for( int i = 0; i < points_in_world.size(); ++i ) {
    Point3d pt = solver.points[i];
    fprintf(fp, "%lf,%lf,%lf\n", pt.x/pt.z, pt.y/pt.z, 1.0/pt.z);
  }
  fclose(fp);


  for ( int ittr = 0; ittr < 5; ++ittr ) {
    solver.run_one_step();
    printf("reprojection error = %e\n", solver.reprojection_error());
  }

  fp = fopen("after.txt", "w");
  for( int i = 0; i < points_in_world.size(); ++i ) {
    Point3d pt = solver.points[i];
    fprintf(fp, "%lf,%lf,%lf\n", pt.x/pt.z, pt.y/pt.z, 1.0/pt.z);
  }
  fclose(fp);

  imwrite("tmp/after.png", print_point_to_image( project_3d_to_2d( solver.cam_t_vec[0], solver.cam_rot_vec[0], solver.points ), Size(512, 512) ));

  return 0;

}

bool test_reproj_gradient( BundleAdjustment::Solver &s ) {

  int Nc = s.Nc, Np = s.Np, K = s.K;

  // ba_get_reproject_gradient_x のテストここから
  {
    // Tx での微分 i == k のときのみ1, それ以外は 0
    if ( ba_get_reproject_gradient_x( s, 0, 0, 0 ) != 1.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 1, 0, 1 ) != 1.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 1, 0, 0 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 0, 0, 1 ) != 0.0 ) return false;

    // Ty での微分 常に0
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 1, 0, Nc ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 0, 1, Nc ) != 0.0 ) return false;

    // Tz での微分常に0
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc*2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 1, 0, Nc*2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 0, 1, Nc*2 ) != 0.0 ) return false;

    // pose_x での微分 常に0
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc*3 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 1, 0, Nc*3 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_x( s, 0, 1, Nc*3 ) != 0.0 ) return false;

    // pose_y での微分 i==k の時のみ 1.0/wj
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc*4 ) != 1.0 / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 0, Nc*4+2 ) != 1.0 / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 4, Nc*4+2 ) != 1.0 / s.points[4].z ) return false;

    // pose_z での微分 i==k の時のみ -yj/wj
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc*5 ) != -s.points[0].y / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 0, Nc*5+2 ) != -s.points[0].y / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 3, Nc*5+2 ) != -s.points[3].y / s.points[3].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 3, Nc*5+1 ) != 0.0 ) return false;

    // pt.x で微分 j == k の時のみ 1.0/pt_j.z
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc*6 ) != 1.0 / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 0, 1, Nc*6+1 ) != 1.0 / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 0, 1, Nc*6 ) != 0.0 ) return false;

    // pt.y で微分 j == k の時のみ -rot_i.z / pt_j.z
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc*6 + Np ) != -s.cam_rot_vec[0].z / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 0, 1, Nc*6 + Np+1 ) != -s.cam_rot_vec[0].z / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 1, Nc*6 + Np+1 ) != -s.cam_rot_vec[2].z / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 3, Nc*6 + Np+3 ) != -s.cam_rot_vec[2].z / s.points[3].z ) return false;
    if ( ba_get_reproject_gradient_x( s, 2, 3, Nc*6 + Np+4 ) != 0.0 ) return false;

    // pt.z で微分 j == k の時のみ ( pt_j.x - rot_i.z * pt_j.y + rot_i.y ) / ( pt_j.z * pt_j.z )
    Point3d pt_j = s.points[0], rot_i = s.cam_rot_vec[0];
    if ( ba_get_reproject_gradient_x( s, 0, 0, Nc*6 + Np*2 ) != -( pt_j.x - rot_i.z * pt_j.y + rot_i.y  )/( pt_j.z * pt_j.z  ) ) return false;

    pt_j = s.points[1]; rot_i = s.cam_rot_vec[0];
    if ( ba_get_reproject_gradient_x( s, 0, 1, Nc*6 + Np*2 + 1) != -( pt_j.x - rot_i.z * pt_j.y + rot_i.y  )/( pt_j.z * pt_j.z  ) ) return false;

    pt_j = s.points[0]; rot_i = s.cam_rot_vec[1];
    if ( ba_get_reproject_gradient_x( s, 1, 0, Nc*6 + Np*2 ) != -( pt_j.x - rot_i.z * pt_j.y + rot_i.y  )/( pt_j.z * pt_j.z  ) ) return false;

    if ( ba_get_reproject_gradient_x( s, 1, 0, Nc*6 + Np*2 +1 ) != 0.0 ) return false;

  }   // ba_get_reproject_gradient_x のテストここまで
  printf("ba_get_reproject_gradient_x ok\n");

  // ba_get_reproject_gradient_y のテストここから
  {
    // Tx での微分 常に 0
    if ( ba_get_reproject_gradient_y( s, 0, 0, 0 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 0, 1 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 0, 0 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 0, 0, 1 ) != 0.0 ) return false;

    // Ty での微分 i == k のときのみ1.0
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc ) != 1.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 0, Nc ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 0, 1, Nc ) != 1.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 1, Nc+1 ) != 1.0 ) return false;

    // Tz での微分常に0
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc*2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 0, Nc*2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 0, 1, Nc*2 ) != 0.0 ) return false;

    // pose_x での微分 i==k の時のみ -1.0/wj
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc*3 ) != -1.0/s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 0, Nc*3 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 1, Nc*3 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 2, Nc*3+1 ) != -1.0/s.points[2].z ) return false;

    // pose_y での微分 常に0
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc*4 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 2, 0, Nc*4+2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_y( s, 2, 4, Nc*4+2 ) != 0.0 ) return false;

    // pose_z での微分 i==k の時のみ xj/wj
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc*5 ) != s.points[0].x / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 2, 0, Nc*5+2 ) != s.points[0].x / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 2, 3, Nc*5+2 ) != s.points[3].x / s.points[3].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 2, 3, Nc*5+1 ) != 0.0 ) return false;

    // pt.x で微分 j == k の時のみ rot_i_z/pt_j.z
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc*6 ) != s.cam_rot_vec[0].z / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 0, 1, Nc*6+1 ) != s.cam_rot_vec[0].z / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 1, 1, Nc*6+1 ) != s.cam_rot_vec[1].z / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 0, 1, Nc*6 ) != 0.0 ) return false;

    // pt.y で微分 j == k の時のみ 1.0 / pt_j.z
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc*6 + Np ) != 1.0 / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 0, 1, Nc*6 + Np+1 ) != 1.0 / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_y( s, 2, 3, Nc*6 + Np+4 ) != 0.0 ) return false;

    // pt.z で微分 j == k の時のみ -( rot_i_z * pt_j.x + pt_j.y - rot_i_x ) / ( pt_j_z * pt_j_z )
    Point3d pt_j = s.points[0], rot_i = s.cam_rot_vec[0];
    if ( ba_get_reproject_gradient_y( s, 0, 0, Nc*6 + Np*2 ) != -( rot_i.z * pt_j.x + pt_j.y - rot_i.x ) / ( pt_j.z * pt_j.z ) ) return false;

    pt_j = s.points[1]; rot_i = s.cam_rot_vec[0];
    if ( ba_get_reproject_gradient_y( s, 0, 1, Nc*6 + Np*2 +1) != -( rot_i.z * pt_j.x + pt_j.y - rot_i.x ) / ( pt_j.z * pt_j.z ) ) return false;

    pt_j = s.points[0]; rot_i = s.cam_rot_vec[1];
    if ( ba_get_reproject_gradient_y( s, 1, 0, Nc*6 + Np*2 ) != -( rot_i.z * pt_j.x + pt_j.y - rot_i.x ) / ( pt_j.z * pt_j.z ) ) return false;

    if ( ba_get_reproject_gradient_y( s, 1, 0, Nc*6 + Np*2 +1 ) != 0.0 ) return false;

  }   // ba_get_reproject_gradient_y のテストここまで
  printf("ba_get_reproject_gradient_y ok\n");

  // ba_get_reproject_gradient_z のテストここから
  {
    // Tx での微分 常に 0
    if ( ba_get_reproject_gradient_z( s, 0, 0, 0 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 0, 1 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 0, 0 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 0, 0, 1 ) != 0.0 ) return false;

    // Ty での微分 常に 0
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 0, Nc ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 0, 1, Nc ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 1, Nc+1 ) != 0.0 ) return false;

    // Tz での微分 i==k のときのみ1
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc*2 ) != 1.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 0, Nc*2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 1, Nc*2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 1, Nc*2+1 ) != 1.0 ) return false;

    // pose_x での微分 i==k の時のみ yj/wj
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc*3 ) != s.points[0].y/s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 0, Nc*3 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 1, Nc*3 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 2, Nc*3+1 ) != s.points[2].y/s.points[2].z ) return false;

    // pose_y での微分 i==k の時のみ -xj/wj
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc*4 ) != -s.points[0].x/s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 2, 0, Nc*4+2 ) != -s.points[0].x/s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 2, 4, Nc*4+2 ) != -s.points[4].x/s.points[4].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 2, 0, Nc*4+3 ) != 0.0 ) return false;

    // pose_z での微分 常に 0
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc*5 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 2, 0, Nc*5+2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 2, 3, Nc*5+2 ) != 0.0 ) return false;
    if ( ba_get_reproject_gradient_z( s, 2, 3, Nc*5+1 ) != 0.0 ) return false;

    // pt.x で微分 j == k の時のみ -rot_i_y/pt_j.z
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc*6 ) != -s.cam_rot_vec[0].y / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 0, 1, Nc*6+1 ) != -s.cam_rot_vec[0].y / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 1, Nc*6+1 ) != -s.cam_rot_vec[1].y / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 0, 1, Nc*6 ) != 0.0 ) return false;

    // pt.y で微分 j == k の時のみ rot_i.x / pt_j.z
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc*6 + Np ) != s.cam_rot_vec[0].x / s.points[0].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 0, 1, Nc*6 + Np+1 ) != s.cam_rot_vec[0].x / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 1, 1, Nc*6 + Np+1 ) != s.cam_rot_vec[1].x / s.points[1].z ) return false;
    if ( ba_get_reproject_gradient_z( s, 2, 3, Nc*6 + Np+4 ) != 0.0 ) return false;

    // pt.z で微分 j == k の時のみ -( -rot_i_y * pt_j.x + rot_i_x * pt_j.y + 1.0 ) / ( pt_j_z * pt_j_z )
    Point3d pt_j = s.points[0], rot_i = s.cam_rot_vec[0];
    if ( ba_get_reproject_gradient_z( s, 0, 0, Nc*6 + Np*2 ) != -( -rot_i.y * pt_j.x + rot_i.x * pt_j.y +1.0 ) / ( pt_j.z * pt_j.z ) ) return false;

    pt_j = s.points[1]; rot_i = s.cam_rot_vec[0];
    if ( ba_get_reproject_gradient_z( s, 0, 1, Nc*6 + Np*2 +1 ) != -( -rot_i.y * pt_j.x + rot_i.x * pt_j.y +1.0 ) / ( pt_j.z * pt_j.z ) ) return false;

    pt_j = s.points[0]; rot_i = s.cam_rot_vec[1];
    if ( ba_get_reproject_gradient_z( s, 1, 0, Nc*6 + Np*2 ) != -( -rot_i.y * pt_j.x + rot_i.x * pt_j.y +1.0 ) / ( pt_j.z * pt_j.z ) ) return false;

    if ( ba_get_reproject_gradient_z( s, 1, 0, Nc*6 + Np*2 +1 ) != 0.0 ) return false;

  }   // ba_get_reproject_gradient_z のテストここまで
  printf("ba_get_reproject_gradient_z ok\n");


  return true;
}
