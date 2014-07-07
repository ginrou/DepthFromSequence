#include "bundle_adjustment.hpp"

void BundleAdjustment::Solver::init( vector<Point3f> points_in, vector<Point3f> cam_t_in, vector<Point3f> cam_rot_in) {

  for(int i = 0; i < Nc; ++i ) {
    cam_t_vec[i] = cam_t_in[i];
    cam_rot_vec[i] = cam_rot_in[i];
  }

  for(int j = 0; j < Np; ++j ) {
    points[j] = points_in[j];
  }

}

double BundleAdjustment::Solver::reprojection_error() {

  double error = 0.0;

  for( int i = 0; i < Nc; ++i ) {
    for( int j = 0; j < Np; ++j ) {
      Point2f reproj = ba_reproject( points[j], cam_t_vec[i], cam_rot_vec[i]);
      double ex = captured[i][j].x - reproj.x, ey = captured[i][j].y - reproj.y;
      error += ex*ex + ey*ey;
    }
  }

  return error;

}

// 単体の関数ここから
Point2f ba_reproject( Point3f pt, Point3f cam_t, Point3f cam_rot) {
  Point3f ret;
  ret.x = ( pt.x - cam_rot.z * pt.y + cam_rot.y ) / pt.z + cam_t.x;
  ret.y = ( pt.x * cam_rot.z + pt.y - cam_rot.x ) / pt.z + cam_t.y;
  ret.z = (-pt.x * cam_rot.y + pt.y * cam_rot.x +1.0 ) / pt.z + cam_t.z;

  return Point2f( ret.x / ret.z, ret.y / ret.z );
}
