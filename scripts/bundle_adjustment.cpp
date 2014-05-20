#include "bundle_adjustment.hpp"

#include "Eigen/Core"

using namespace std;
using namespace cv;
using namespace Eigen;


/* Solverの初期化
   - カメラの平行移動なし
   - カメラの回転移動なし
   - 各特徴点は (0, 0, 1000) ( 画素が2mm四方として1m先くらい )
 */
void bundleAdjustment::Solver::initialize() {
  cout << "initialzie" << endl;

  for( int i = 0; i < Nc; ++i ) {
    cam_t_x[i] = 0; cam_t_y[i] = 0; cam_t_z[i] = 0;
    cam_pose_x[i] = 0; cam_pose_y[i] = 0; cam_pose_z[i] = 0;
  }
  
  for( int j = 0; j < Np; ++j ) {
    point_x[j] = 0;
    point_y[j] = 0;
    point_z[j] = 1.0/1000;
  }

}

/* Solverの計算を1ステップ進める。１ステップは以下の手順
   - 再投影点の計算
   - 再投影点の微分の計算
 */
void bundleAdjustment::Solver::run_one_step() {

  // 再投影点の計算
  for( int i = 0; i < Nc; ++i ) {
    for( int j = 0; j < Np; ++j ) {
      reproject_x[i][j] = ba_reproject_x( *this, i, j);
      reproject_y[i][j] = ba_reproject_y( *this, i, j);
      reproject_z[i][j] = ba_reproject_z( *this, i, j);
      //printf("reproject[%02d][%03d] : %lf, %lf, %lf\n", i, j, reproject_x[i][j], reproject_y[i][j], reproject_z[i][j]);
    }
  }

  // 再投影点の微分の計算
  for( int i = 0; i < Nc; ++i ) {
    for( int j = 0; j < Np; ++j ) {
      for( int k = 0; k < K; ++k ) {
	grad_reproject_x[i][j][k] = ba_get_reproject_gradient_x( *this, i, j, k);
	grad_reproject_y[i][j][k] = ba_get_reproject_gradient_y( *this, i, j, k);
	grad_reproject_z[i][j][k] = ba_get_reproject_gradient_z( *this, i, j, k);
	//printf("grad_reproject[%02d][%02d][%03d] : %lf, %lf, %lf\n", i, j, k, grad_reproject_x[i][j][k], grad_reproject_y[i][j][k], grad_reproject_z[i][j][k]);
      }
    }
  }  
  
  // コスト関数の勾配を計算
  for( int k = 0; k < K; ++k ) {
    gradient_vector[k] = ba_get_gradient( *this, k);
    //printf("gradient_vector[%03d] = %lf\n", k, gradient_vector[k]);
  }

  // コスト関数のヘッセ行列を計算
  for( int k1 = 0; k1 < K; ++k1 ) {
    for( int k2 = 0; k2 < K; ++k2 ) {
      double c = ( k1 == k2 ) ? 1.00001 : 1.0;
      hessian_matrix[k1][k2] = c * ba_get_hessian_matrix( *this, k1, k2);
      //printf("hessian_matrix[%03d][%03d] = %lf\n", k1, k2, hessian_matrix[k1][k2]);
    }
  }

  // 更新幅を求める
  Eigen::VectorXd v = ba_get_update_for_step( *this, hessian_matrix, gradient_vector);

  // 各変数を更新
  for( int i = 0; i < Nc; ++i ) { 
    cam_t_x[i] += v[i];
    cam_t_y[i] += v[ i+Nc ];
    cam_t_y[i] += v[ i+2*Nc ];
    cam_pose_x[i] += v[ i+3*Nc ];
    cam_pose_y[i] += v[ i+4*Nc ];
    cam_pose_z[i] += v[ i+5*Nc ];

    printf("camera %d : t = (%lf, %lf, %lf), pose = (%lf, %lf, %lf)\n", i, 
	   cam_t_x[i], cam_t_y[i], cam_t_z[i], 
	   cam_pose_x[i], cam_pose_y[i], cam_pose_z[i]);

  }

  for( int j = 0; j < Np; ++j ) {
    int offset = 6*Nc;
    point_x[j] += v[ j + 6*Nc ];
    point_y[j] += v[ j + Np + 6*Nc ];
    point_z[j] += v[ j + 2*Np + 6*Nc ];

    printf("point %d : (%lf, %lf, %lf)\n", j, point_x[j],point_y[j],point_z[j]);

  }

}
