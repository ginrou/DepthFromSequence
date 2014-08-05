#include "bundle_adjustment.hpp"
#include "Eigen/LU"

#include <cstdlib>
#include <ctime>

inline double drand() { return (double)rand()/RAND_MAX; }

std::vector<Point3d> random_3d_points(int N, cv::Point3d min, cv::Point3d max) {

  std::srand(std::time(0));
  std::vector<Point3d> points;
  for( int i = 0; i < N; ++i ) {
    Point3d pt;
    pt.z = 1.0  / (min.z + (max.z - min.z) * drand() );
    pt.x = pt.z * (min.x + (max.x - min.x) * drand() );
    pt.y = pt.z * (min.y + (max.y - min.y) * drand() );
    points.push_back(pt);
  }
  return points;
}

std::vector<Point3d> random_3d_cam_t(int N) {
  std::srand(std::time(0));
  std::vector<Point3d> points;
  Point3d  min(0.0, -0.5, -0.005), max(20.0, 0.5, 0.005);

  for( int i = 0; i < N; ++i ) {
    Point3d pt;
    pt.x = min.x + (max.x - min.x) * drand();
    pt.y = min.y + (max.y - min.y) * drand();
    pt.z = min.z + (max.z - min.z) * drand();

    if( i == 0 ) pt = Point3d(0, 0, 0);
    if( i == 1 ) pt.x = 5.0;

    points.push_back(pt);
  }
  return points;

}

std::vector<Point3d> random_3d_cam_rot(int N) {
  std::vector<Point3d> points;
  Point3d min(-0.001, -0.001, -0.001), max(0.001, 0.001, 0.001);

  for( int i = 0; i < N; ++i ) {
    Point3d pt;
    pt.x = min.x + (max.x - min.x) * drand();
    pt.y = min.y + (max.y - min.y) * drand();
    pt.z = min.z + (max.z - min.z) * drand();

    if( i == 0 ) pt = Point3d(0, 0, 0);

    pt = Point3d(0, 0, 0);
    points.push_back(pt);
  }
  return points;
}


void BundleAdjustment::Solver::init( vector<Point3d> points_in, vector<Point3d> cam_t_in, vector<Point3d> cam_rot_in) {

  for(int i = 0; i < Nc; ++i ) {
    Point3d pt = cam_t_in[i];
    cam_t_vec[i] = pt;
    cout <<"cam_t "<< pt << endl;
  }

  for(int i = 0; i < Nc; ++i ) {
    Point3d pt = cam_rot_in[i];
    cam_rot_vec[i] = pt;
    cout <<"cam_rot "<< pt << endl;
  }

  for(int j = 0; j < Np; ++j ) {
    Point3d pt = points_in[j];
    points[j] = pt;
    cout <<"points "<< pt << endl;
  }

}

void BundleAdjustment::Solver::init_with_first_image( vector< vector<Point2d> > captured_in,
						      Size img_size,
						      double mean_depth,
						      double fov
						      )
{
  double W = img_size.width, H = img_size.height;
  double tan_fov = tan(fov/(2.0*M_PI));

  // 1. initialize points
  for( int j = 0; j < Np; ++j ) {
    points[j].x = (2.0 * captured_in[0][j].x - W ) * tan_fov / W;
    points[j].y = (2.0 * captured_in[0][j].y - H ) * tan_fov / H;
    points[j].z = (1.0 + 0.01 * drand() ) / mean_depth;
  }

  // 2. initialize captured
  for( int i = 0; i < Nc; ++i ) {
    for( int j = 0; j < Np; ++j ) {
      captured[i][j].x = (2.0 * captured_in[i][j].x - W ) * tan_fov / W;
      captured[i][j].y = (2.0 * captured_in[i][j].y - H ) * tan_fov / H;
    }
  }

  // 3. initialize camera params
  cam_t_vec = random_3d_cam_t(Nc);
  cam_rot_vec = random_3d_cam_rot(Nc);

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
  ittr++;
  printf("ittr : %d\n", ittr);

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

  // 正則化パラメータは更新しないほうが収束が早い
  error_before > error_after ? this->c *= 0.1 : this->c *= 10.0;

  this->should_continue = get_should_continue( error_before, error_after, update.norm());

  printf("next c = %e\n\n\n", this->c);
}

bool BundleAdjustment::Solver::get_should_continue( double error_before, double error_after, double update_norm ) {
  printf("error_before = %e\n", error_before);
  printf("error_after = %e\n", error_after);
  printf("update_norm = %e\n", update_norm);
  printf("error_per_pixel = %e\n", error_after / (double)(Np*Nc));
  printf("error diff = %e\n", fabsf(error_after - error_before));

  if ( update_norm < 0.0001 ) return false;
  if ( fabsf(error_after - error_before) < 1.0e-3 ) return false;
  if ( ittr >= MAX_ITTR ) return false;

  return true;
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
