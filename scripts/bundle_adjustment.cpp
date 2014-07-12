#include "bundle_adjustment.hpp"
#include "Eigen/LU"

#include <cstdlib>
#include <ctime>

void BundleAdjustment::Solver::init( vector<Point3d> points_in, vector<Point3d> cam_t_in, vector<Point3d> cam_rot_in) {

  for(int i = 0; i < Nc; ++i ) {
    Point3d pt = cam_t_in[i];
    pt.x += 0.005 * ((double)rand()/RAND_MAX -0.5);
    pt.y += 0.005 * ((double)rand()/RAND_MAX -0.5);
    pt.z += 0.00005 * ((double)rand()/RAND_MAX -0.5);
    cam_t_vec[i] = pt;
  }

  for(int i = 0; i < Nc; ++i ) {
    Point3d pt = cam_rot_in[i];
    pt.x += 0.001 * ((double)rand()/RAND_MAX -0.5);
    pt.y += 0.001 * ((double)rand()/RAND_MAX -0.5);
    pt.z += 0.00001 * ((double)rand()/RAND_MAX -0.5);
    cam_rot_vec[i] = pt;
  }

  for(int j = 0; j < Np; ++j ) {
    Point3d pt = points_in[j];
    pt.x += ((double)rand()/RAND_MAX -0.5);
    pt.y += ((double)rand()/RAND_MAX -0.5);
    pt.z += ((double)rand()/RAND_MAX -0.5);
    points[j] = pt;
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

void BundleAdjustment::Solver::run_one_step() {

  // 領域確保
  int K = this->K -7;
  Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(2*Nc*Np, K);
  Eigen::VectorXd target_error = Eigen::VectorXd::Zero(2*Nc*Np);

  printf("K = %d -> %d\n", (int)this->K, K);

  // 更新前の再投影エラー
  double error_before = this->reprojection_error();

  // Jacobianと誤差を計算
  for ( int i = 0; i < Nc; ++i ) {
    for ( int j = 0; j < Np; ++j ) {
      int nx = i*Np + j, ny = i*Np + j + Np*Nc;

      Point2d p = captured[i][j];
      Point3d q = ba_reproject3d(points[j],  cam_t_vec[i], cam_rot_vec[i]);
      target_error[nx] = p.x - q.x/q.z;
      target_error[ny] = p.y - q.y/q.z;

      for ( int k = 0, l = 0; k < this->K; ++k ) {
	if ( k == 0 ) continue;
	if ( k == 1 ) continue;
	if ( k == Nc ) continue;
	if ( k == 2*Nc ) continue;
	if ( k == 3*Nc ) continue;
	if ( k == 4*Nc ) continue;
	if ( k == 5*Nc ) continue;

	Point3d grad;
	grad.x = ba_get_reproject_gradient_x( *this, i, j, k );
	grad.y = ba_get_reproject_gradient_y( *this, i, j, k );
	grad.z = ba_get_reproject_gradient_z( *this, i, j, k );

	Jacobian(nx, l) = (q.x * grad.z - q.z * grad.x ) / (q.z*q.z);
	Jacobian(ny, l) = (q.y * grad.z - q.z * grad.y ) / (q.z*q.z);
	l++;
      }
    }
  }

  // 更新方向の計算
  cout << "get update direction" << endl;
  Eigen::VectorXd gradient = -Jacobian.transpose() * target_error;
  Eigen::MatrixXd Hessian = Jacobian.transpose() * Jacobian + this->c * Eigen::MatrixXd::Identity(K, K);

  cout << "|H| = " << Hessian.determinant() << endl;

  Eigen::VectorXd sol = Hessian.fullPivLu().solve(gradient);
  Eigen::VectorXd update = Eigen::VectorXd::Zero(this->K);

  // 更新
  for ( int k = 0, l = 0; k < this->K; ++k ) {
    if ( k == 0 ) continue;
    if ( k == 1 ) continue;
    if ( k == Nc ) continue;
    if ( k == 2*Nc ) continue;
    if ( k == 3*Nc ) continue;
    if ( k == 4*Nc ) continue;
    if ( k == 5*Nc ) continue;

    update[k] = sol[l];
    l++;
  }

  for ( int i = 0; i < Nc; ++i ) {
    cam_t_vec[i].x += update[i];
    cam_t_vec[i].y += update[ i+Nc ];
    cam_t_vec[i].z += update[ i+2*Nc ];
    cam_rot_vec[i].x += update[ i+3*Nc ];
    cam_rot_vec[i].y += update[ i+4*Nc ];
    cam_rot_vec[i].z += update[ i+5*Nc ];
  }

  for ( int j = 0; j < Np; ++j ) {
    points[j].x += update[ j + 0*Np + 6*Nc ];
    points[j].y += update[ j + 1*Np + 6*Nc ];
    points[j].z += update[ j + 2*Np + 6*Nc ];
  }

  double error_after = this->reprojection_error();
  error_before < error_after ? this->c *= 10.0 : this->c *= 0.1;
  this->should_continue = ba_should_continue( error_before, error_after, update.norm() );
}

// 単体の関数ここから
Point3d ba_reproject3d( Point3d pt, Point3d cam_t, Point3d cam_rot) {
  Point3d ret;
  ret.x = ( pt.x - cam_rot.z * pt.y + cam_rot.y ) / pt.z + cam_t.x;
  ret.y = ( pt.x * cam_rot.z + pt.y - cam_rot.x ) / pt.z + cam_t.y;
  ret.z = (-pt.x * cam_rot.y + pt.y * cam_rot.x +1.0 ) / pt.z + cam_t.z;
  return ret;
}

Point2d ba_reproject( Point3d pt, Point3d cam_t, Point3d cam_rot) {
  Point3d ret = ba_reproject3d(pt, cam_t, cam_rot);
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


bool ba_should_continue( double error_before, double error_after, double update_norm ) {
  if ( update_norm < 0.1 ) return false;
  if ( error_after / error_before > 10.0 ) return false;
  return error_before > error_after;
}
