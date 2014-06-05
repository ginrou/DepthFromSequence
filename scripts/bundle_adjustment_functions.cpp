#include "bundle_adjustment.hpp"
#include "Eigen/LU"

inline double square(double a) { return a*a; }
inline double delta(int i, int j) { return i == j ? 1.0 : 0.0; }

void test_hessian(Eigen::MatrixXd &hessian);

double ba_reproject_x( bundleAdjustment::Solver &s, int i, int j) {
  return ( s.point_x[j] - s.cam_pose_z[i] * s.point_y[j] + s.cam_pose_y[i] ) / s.point_z[j] + s.cam_t_x[i];
}

double ba_reproject_y( bundleAdjustment::Solver &s, int i, int j) {
  return ( s.point_x[j] * s.cam_pose_z[i] + s.point_y[j] - s.cam_pose_x[i] ) / s.point_z[j] + s.cam_t_y[i];
}

double ba_reproject_z( bundleAdjustment::Solver &s, int i, int j) {
  return (-s.point_x[j] * s.cam_pose_y[i] + s.cam_pose_z[i] * s.point_y[j] + 1) / s.point_z[j] + s.cam_t_z[i];
}

double ba_get_reproject_gradient_x( bundleAdjustment::Solver &s, int i, int j, int k) {

  double xj = s.point_x[j], yj = s.point_y[j], zj = s.point_z[j];
  double pose_x_i = s.cam_pose_x[i], pose_y_i = s.cam_pose_y[i], pose_z_i = s.cam_pose_z[i];

  if ( k < s.Nc ) // Txでの微分
    return delta(i,k); 

  else if ( k < 2*s.Nc ) // Tyでの微分
    return 0; 

  else if ( k < 3*s.Nc ) // Tzでの微分
    return 0;

  else if ( k < 4*s.Nc ) // pose_xでの微分
    return 0;

  else if ( k < 5*s.Nc ) // pose_yでの微分
    return delta(i,k -4*s.Nc) / zj; 

  else if ( k < 6*s.Nc ) // pose_zでの微分
    return -delta(i,k-5*s.Nc) * yj / zj;

  else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
    return delta(j,k-6*s.Nc) / zj; 

  else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
    return - delta(j, k - 6*s.Nc - s.Np) * pose_z_i / zj; 
  
  else // point_zでの微分
    return -delta(j, k - 6*s.Nc - 2*s.Np) *  ( xj - pose_z_i*yj + pose_y_i ) / ( zj*zj );

}

double ba_get_reproject_gradient_y( bundleAdjustment::Solver &s, int i, int j, int k) {

  double xj = s.point_x[j], yj = s.point_y[j], zj = s.point_z[j];
  double pose_x_i = s.cam_pose_x[i], pose_y_i = s.cam_pose_y[i], pose_z_i = s.cam_pose_z[i];

  if ( k < s.Nc ) // Txでの微分
    return 0;

  else if ( k < 2*s.Nc ) // Tyでの微分
    return delta(i, k - s.Nc );
  
  else if ( k < 3*s.Nc ) // Tzでの微分
    return 0;

  else if ( k < 4*s.Nc ) // pose_xでの微分
    return -delta(i, k - 3*s.Nc) / zj;

  else if ( k < 5*s.Nc ) // pose_yでの微分
    return 0.0;

  else if ( k < 6*s.Nc ) // pose_zでの微分
    return delta(i, k - 5*s.Nc) * xj / zj;

  else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
    return delta(j, k - 6*s.Nc) * pose_z_i / zj;

  else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
    return delta(j, k - 6*s.Nc - s.Np) / zj;

  else // point_zでの微分
    return -delta(j, k - 6*s.Nc - 2*s.Np) * ( pose_z_i*xj + yj - pose_x_i) / (zj*zj);

}

double ba_get_reproject_gradient_z( bundleAdjustment::Solver &s, int i, int j, int k) {

  double xj = s.point_x[j], yj = s.point_y[j], zj = s.point_z[j];
  double pose_x_i = s.cam_pose_x[i], pose_y_i = s.cam_pose_y[i], pose_z_i = s.cam_pose_z[i];

  if ( k < s.Nc ) // Txでの微分
    return 0;

  else if ( k < 2*s.Nc ) // Tyでの微分
    return 0;

  else if ( k < 3*s.Nc ) // Tzでの微分
    return delta(i, k - 2*s.Nc);

  else if ( k < 4*s.Nc ) // pose_xでの微分
    return delta(i, k - 3*s.Nc) * yj / zj;

  else if ( k < 5*s.Nc ) // pose_yでの微分
    return -delta(i, k - 4*s.Nc) * xj / zj;

  else if ( k < 6*s.Nc ) // pose_zでの微分
    return 0;

  else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
    return - delta(j, k - 6*s.Nc) * pose_y_i / zj;

  else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
    return delta(j, k - 6*s.Nc - s.Np) * pose_x_i / zj;

  else // point_zでの微分
    return -delta(j, k - 6*s.Nc - 2*s.Np) * ( -pose_y_i*xj + pose_x_i*yj + 1.0 ) / (zj*zj);

}

double ba_get_gradient( bundleAdjustment::Solver &s, int k) {

  double ret = 0.0;
  for( int i = 0; i < s.Nc; ++i ) {
    for( int j = 0; j < s.Np; ++j ) {

      double px = s.captured_x[i][j];
      double py = s.captured_y[i][j];
      double qx = s.reproject_x[i][j];
      double qy = s.reproject_y[i][j];
      double qz = s.reproject_z[i][j];
      double grad_x = s.grad_reproject_x[i][j][k];
      double grad_y = s.grad_reproject_y[i][j][k];
      double grad_z = s.grad_reproject_z[i][j][k];
      
      ret += ( square(px - qx/qz) * ( qx * grad_z - qz * grad_x ) 
	       + square(py - qy/qz) * ( qy * grad_z - qz * grad_y ) 
	       ) / square(qz);
    }
  }
  return 2.0 * ret;
}

double ba_get_hessian_matrix( bundleAdjustment::Solver &s, int k, int l) {

  double ret = 0.0;
  for( int i = 0; i < s.Nc; ++i ) {
    for( int j = 0; j < s.Np; ++j ) {

      double qx = s.reproject_x[i][j];
      double qy = s.reproject_y[i][j];
      double qz = s.reproject_z[i][j];
      double grad_x_k = s.grad_reproject_x[i][j][k];
      double grad_y_k = s.grad_reproject_y[i][j][k];
      double grad_z_k = s.grad_reproject_z[i][j][k];
      double grad_x_l = s.grad_reproject_x[i][j][l];
      double grad_y_l = s.grad_reproject_y[i][j][l];
      double grad_z_l = s.grad_reproject_z[i][j][l];

      ret += -( ( qx*grad_z_k - qz*grad_x_k ) * ( qx*grad_z_l - qz*grad_x_l )
		+ ( qy*grad_z_k - qz*grad_y_k ) * ( qy*grad_z_l - qz*grad_y_l )
		) / square(square(qz));

    }
  }
  return 2.0 * ret;
}

Eigen::VectorXd ba_get_update_for_step( bundleAdjustment::Solver &s, vector< vector<double> > hessian_matrix, vector<double> gradient_vector)
{
  Eigen::MatrixXd H(s.K, s.K);
  for(int k1 = 0; k1 < s.K; ++k1 ) {
    for(int k2 = 0; k2 < s.K; ++k2 ) {
      H(k1, k2) = hessian_matrix[k1][k2];
    }
  }

  Eigen::VectorXd grad(s.K);
  for(int k = 0; k < s.K; ++k ) {
    grad(k) = -gradient_vector[k];
  }

  cout << "|H| = " << H.determinant() << endl;
  //  cout << H << endl;

  printf("\nall 0 rows\n");
  for( int k = 0; k < s.K; ++k ) {
    bool flag = true;
    for( int l = 0; l < s.K; ++l ) {
      if ( H(k,l) != 0 ) flag = false;
    }

    if(flag) cout << k << endl;

  }
  printf("Nc = %d, Ns = %d\n", s.Nc, s.Np);

  return H.fullPivLu().solve(grad);

}

Eigen::VectorXd ba_get_update_for_step2( bundleAdjustment::Solver &s) {

  // 最初のカメラの位置を原点、回転をゼロとして、次のカメラとのx方向の平行移動を1で固定
  // するので更新するのは変数が7に減る
  int K = s.K - 7;

  Eigen::MatrixXd Jacobian( s.Nc * s.Np, K );

  for( int i = 0; i < s.Nc; ++i ) {
    for( int j = 0; j < s.Np; ++j ) {
      for( int k = 0,  l = 0; k < s.K; ++k ) { 

	if ( k == 0 ) continue; // １つ目のカメラのx方向の平行移動
	if ( k == 1 ) continue; // １つ目のと２つ目のカメラの平行移動
	if ( k == s.Nc ) continue; // １つ目のカメラのy方向の平行移動
	if ( k == 2*s.Nc ) continue; // １つ目のカメラのz方向の平行移動
	if ( k == 3*s.Nc ) continue; // １つ目のカメラのx方向の回転
	if ( k == 4*s.Nc ) continue; // １つ目のカメラのy方向の回転
	if ( k == 5*s.Nc ) continue; // １つ目のカメラのz方向の回転

	double px = s.captured_x[i][j];
	double py = s.captured_y[i][j];
	double qx = s.reproject_x[i][j];
	double qy = s.reproject_y[i][j];
	double qz = s.reproject_z[i][j];
	double grad_x = s.grad_reproject_x[i][j][k];
	double grad_y = s.grad_reproject_y[i][j][k];
	double grad_z = s.grad_reproject_z[i][j][k];

	int n = i*s.Np + j;
	Jacobian(n, l) = 2.0 *( ( px - qx/qz) * ( qz * grad_x - qx * grad_z )
				+ ( py - qy/qz) * ( qz * grad_y - qy * grad_z )
				) / (qz*qz);
	l++;
      }
    }
  }

  Eigen::VectorXd target_error(s.Nc*s.Np);

  for( int i = 0; i < s.Nc; ++i ) {
    for( int j = 0; j < s.Np; ++j ) {

      double px = s.captured_x[i][j];
      double py = s.captured_y[i][j];
      double qx = s.reproject_x[i][j];
      double qy = s.reproject_y[i][j];
      double qz = s.reproject_z[i][j];

      int n = i*s.Np + j;
      target_error(n) = square(px - qx/qz) + square(py - qy/qz);
    }
  }

  Eigen::VectorXd gradient = Jacobian.transpose() * target_error;
  Eigen::MatrixXd Hessian = Jacobian.transpose() * Jacobian + s.c * Eigen::MatrixXd::Identity(K,K);

  cout << "|H| = " << Hessian.determinant() << endl;
  Eigen::FullPivLU<Eigen::MatrixXd> lu(Hessian);
  cout << "rank = " << lu.rank() << endl;
  
  Eigen::VectorXd update = Hessian.fullPivLu().solve(gradient);
  Eigen::VectorXd ret = Eigen::VectorXd::Zero(s.K);
  for ( int k = 0, l = 0; k < K; ++ k ) {

    if ( k == 0 ) continue; // １つ目のカメラのx方向の平行移動
    if ( k == 1 ) continue; // １つ目のと２つ目のカメラの平行移動
    if ( k == s.Nc ) continue; // １つ目のカメラのy方向の平行移動
    if ( k == 2*s.Nc ) continue; // １つ目のカメラのz方向の平行移動
    if ( k == 3*s.Nc ) continue; // １つ目のカメラのx方向の回転
    if ( k == 4*s.Nc ) continue; // １つ目のカメラのy方向の回転
    if ( k == 5*s.Nc ) continue; // １つ目のカメラのz方向の回転

    ret(k) = update(l);
    l++;
  }
  return ret;

}

void test_hessian(Eigen::MatrixXd &hessian) {

  int rows = hessian.rows();

  for( int r0 = 0; r0 < rows; ++r0 ) {
    for(int r1 = r0+1; r1 < rows; ++r1 ) {
      double norm = (hessian.row(r0) - hessian.row(r1)).norm();
      if ( norm < 0.0000001 ) {
	printf("row %d <--> %d : norm = %lf\n", r0, r1, norm);
      }
    }
  }

}

double ba_reprojection_error( bundleAdjustment::Solver &s ) {

  double err = 0.0;

  for( int i = 0; i < s.Nc; ++i ) {
    for( int j = 0; j < s.Np; ++j ) {

      double px = s.captured_x[i][j];
      double py = s.captured_y[i][j];
      double qx = s.reproject_x[i][j];
      double qy = s.reproject_y[i][j];
      double qz = s.reproject_z[i][j];

      err += square(px - qx/qz) + square(py - qy/qz);

    }
  }

  return err;
}