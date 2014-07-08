#include "bundle_adjustment.hpp"

void BundleAdjustment::Solver::init( vector<Point3d> points_in, vector<Point3d> cam_t_in, vector<Point3d> cam_rot_in) {

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
      Point2d reproj = ba_reproject( points[j], cam_t_vec[i], cam_rot_vec[i]);
      double ex = captured[i][j].x - reproj.x, ey = captured[i][j].y - reproj.y;
      error += ex*ex + ey*ey;
    }
  }

  return error;

}

// 単体の関数ここから
Point2d ba_reproject( Point3d pt, Point3d cam_t, Point3d cam_rot) {
  Point3d ret;
  ret.x = ( pt.x - cam_rot.z * pt.y + cam_rot.y ) / pt.z + cam_t.x;
  ret.y = ( pt.x * cam_rot.z + pt.y - cam_rot.x ) / pt.z + cam_t.y;
  ret.z = (-pt.x * cam_rot.y + pt.y * cam_rot.x +1.0 ) / pt.z + cam_t.z;

  return Point2d( ret.x / ret.z, ret.y / ret.z );
}

inline double delta(int i, int j) { return i == j ? 1.0 : 0.0; }

double ba_get_reproject_gradient_x( BundleAdjustment::Solver &s, int i, int j, int k) {
  Point3d pt = s.points[j], cam_rot = s.cam_rot_vec[i];

  if ( k < s.Nc ) // Txでの微分
    return delta(i,k); 

  else if ( k < 2*s.Nc ) // Tyでの微分
    return 0; 

  else if ( k < 3*s.Nc ) // Tzでの微分
    return 0;

  else if ( k < 4*s.Nc ) // pose_xでの微分
    return 0;

  else if ( k < 5*s.Nc ) // pose_yでの微分
    return delta(i,k -4*s.Nc) / pt.z; 

  else if ( k < 6*s.Nc ) // pose_zでの微分
    return -delta(i,k-5*s.Nc) * pt.y / pt.z;

  else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
    return delta(j,k-6*s.Nc) / pt.z; 

  else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
    return - delta(j, k - 6*s.Nc - s.Np) * cam_rot.z / pt.z; 
  
  else // point_zでの微分
    return -delta(j, k - 6*s.Nc - 2*s.Np) *  ( pt.x - cam_rot.z * pt.y + cam_rot.y ) / ( pt.z*pt.z );

}

double ba_get_reproject_gradient_y( BundleAdjustment::Solver &s, int i, int j, int k) {
  Point3d pt = s.points[j], cam_rot = s.cam_rot_vec[i];

  if ( k < s.Nc ) // Txでの微分
    return 0;

  else if ( k < 2*s.Nc ) // Tyでの微分
    return delta(i, k - s.Nc );
  
  else if ( k < 3*s.Nc ) // Tzでの微分
    return 0;

  else if ( k < 4*s.Nc ) // pose_xでの微分
    return -delta(i, k - 3*s.Nc) / pt.z;

  else if ( k < 5*s.Nc ) // pose_yでの微分
    return 0.0;

  else if ( k < 6*s.Nc ) // pose_zでの微分
    return delta(i, k - 5*s.Nc) * pt.x / pt.z;

  else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
    return delta(j, k - 6*s.Nc) * cam_rot.z / pt.z;

  else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
    return delta(j, k - 6*s.Nc - s.Np) / pt.z;

  else // point_zでの微分
    return -delta(j, k - 6*s.Nc - 2*s.Np) * ( cam_rot.z*pt.x + pt.y - cam_rot.x) / (pt.z*pt.z);

}


double ba_get_reproject_gradient_z( BundleAdjustment::Solver &s, int i, int j, int k) {
  Point3d pt = s.points[j], cam_rot = s.cam_rot_vec[i];

  if ( k < s.Nc ) // Txでの微分
    return 0;

  else if ( k < 2*s.Nc ) // Tyでの微分
    return 0;

  else if ( k < 3*s.Nc ) // Tzでの微分
    return delta(i, k - 2*s.Nc);

  else if ( k < 4*s.Nc ) // pose_xでの微分
    return delta(i, k - 3*s.Nc) * pt.y / pt.z;

  else if ( k < 5*s.Nc ) // pose_yでの微分
    return -delta(i, k - 4*s.Nc) * pt.x / pt.z;

  else if ( k < 6*s.Nc ) // pose_zでの微分
    return 0;

  else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
    return - delta(j, k - 6*s.Nc) * cam_rot.y / pt.z;

  else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
    return delta(j, k - 6*s.Nc - s.Np) * cam_rot.x / pt.z;

  else // point_zでの微分
    return -delta(j, k - 6*s.Nc - 2*s.Np) * ( -cam_rot.y*pt.x + cam_rot.x*pt.y + 1.0 ) / (pt.z*pt.z);

}


