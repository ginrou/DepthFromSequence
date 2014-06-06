#ifndef __BUNDLE_ADJUTMENT_HPP__
#define __BUNDLE_ADJUTMENT_HPP__

#include <opencv2/opencv.hpp>

#include "Eigen/Core"

using namespace std;
using namespace cv;

namespace bundleAdjustment {
  class Solver{
  public:
    int Nc, Np, K; // カメラの数、特徴点の数、変数の数(K = 6*Nc + 3*Np)

    // 各カメラで取得した特徴点
    // i番目のカメラで観測したj番目の特徴点は captured_x[i][j]
    vector< vector<double> > captured_x, captured_y;

    // 各カメラの位置と姿勢
    vector<double> cam_t_x, cam_t_y, cam_t_z;
    vector<double> cam_pose_x, cam_pose_y, cam_pose_z;

    // 特徴点の三次元位置
    vector <double> point_x, point_y, point_z; // ( 論文では point_z は w とも)
    
    // 推定したカメラ位置と姿勢から再投影した特徴点の位置
    // i番目のカメラでj番目の特徴点を再投影した点は reproject_x[i][j] で参照する
    vector< vector<double> > reproject_x, reproject_y, reproject_z;

    // 再投影した点について各変数で偏微分した値
    // reproject_x[i][j]をk番目の変数で偏微分したのが grad_reproject_x[i][j][k]
    vector< vector< vector<double> > > grad_reproject_x, grad_reproject_y, grad_reproject_z;
    
    // 更新幅を求める計算のための変数
    vector<double> gradient_vector;
    vector< vector<double> > hessian_matrix;
    vector<double> update_for_step; // 更新幅

    // LM法の正則化項の強さのパラメータ
    double c;
    double reprojection_error;

    Solver(std::vector< vector< cv::Point2f > > captured, cv::Size img_size) {
      c = 0.00001;

      Nc = captured.size();
      Np = captured[0].size();
      K  =  6*Nc + 3*Np;

      for( int i = 0; i < Nc; ++i ) {
	vector<double> pt_x, pt_y;
	for( int j = 0; j < Np; ++j ) {
	  cv::Point2f pt = captured[i][j];
	  pt_x.push_back(2.0 * pt.x / img_size.width  - 1.0); // [-1,1] におさまるようにする
	  pt_y.push_back(2.0 * pt.y / img_size.height - 1.0);
	}
	captured_x.push_back(pt_x);
	captured_y.push_back(pt_y);
      }

      // 領域の確保
      // カメラの位置
      cam_t_x = vector<double>(Nc);
      cam_t_y = vector<double>(Nc);
      cam_t_z = vector<double>(Nc);

      // カメラの姿勢
      cam_pose_x = vector<double>(Nc);
      cam_pose_y = vector<double>(Nc);
      cam_pose_z = vector<double>(Nc);

      // 特徴点の三次元位置
      point_x = vector<double>(Np);
      point_y = vector<double>(Np);
      point_z = vector<double>(Np);

      // 推定したカメラ位置と姿勢から再投影した特徴点の位置
      reproject_x = vector< vector<double> >(Nc);
      reproject_y = vector< vector<double> >(Nc);
      reproject_z = vector< vector<double> >(Nc);

      for( int i = 0; i < Nc; ++i ) {
	reproject_x[i] = vector<double>(Np);
	reproject_y[i] = vector<double>(Np);
	reproject_z[i] = vector<double>(Np);
      }

      // 再投影した点について各変数で偏微分した値
      grad_reproject_x = vector< vector< vector<double> > >(Nc);
      grad_reproject_y = vector< vector< vector<double> > >(Nc);
      grad_reproject_z = vector< vector< vector<double> > >(Nc);
      for( int i = 0; i < Nc; ++i ) {
	grad_reproject_x[i] = vector< vector<double> >(Np);
	grad_reproject_y[i] = vector< vector<double> >(Np);
	grad_reproject_z[i] = vector< vector<double> >(Np);
	for( int j = 0; j < Np; ++j ) {
	  grad_reproject_x[i][j] = vector<double>(K);
	  grad_reproject_y[i][j] = vector<double>(K);
	  grad_reproject_z[i][j] = vector<double>(K);
	}
      }

      // 更新幅を求める計算のための変数
      gradient_vector = vector<double>(K);
      hessian_matrix = vector< vector<double> >(K);
      for( int k = 0; k < K; ++k ) {
	hessian_matrix[k] = vector<double>(K);
      }
      update_for_step = vector<double>(K);

    }

    void initialize();
    void run_one_step();

    static cv::Mat depth_drawn( cv::Mat img, vector<Point2f> points, vector<double> depth);

  }; // class Solver

}; // namespace bundleAdjustment

/* 
   bundle adjustment の計算をする上で必要な関数の集合
   状態を持たない。ba_ プレフィックス
 */

// 再投影点の計算
double ba_reproject_x( bundleAdjustment::Solver &s, int i, int j);
double ba_reproject_y( bundleAdjustment::Solver &s, int i, int j);
double ba_reproject_z( bundleAdjustment::Solver &s, int i, int j);

// 再投影誤差
double ba_reprojection_error( bundleAdjustment::Solver &s );

// 再投影点の勾配
double ba_get_reproject_gradient_x( bundleAdjustment::Solver &s, int i, int j, int k);
double ba_get_reproject_gradient_y( bundleAdjustment::Solver &s, int i, int j, int k);
double ba_get_reproject_gradient_z( bundleAdjustment::Solver &s, int i, int j, int k);

// 連立方程式を解いて反復幅を求める
Eigen::VectorXd ba_get_update_for_step( bundleAdjustment::Solver &s);

#endif // __BUNDLE_ADJUTMENT_HPP__

