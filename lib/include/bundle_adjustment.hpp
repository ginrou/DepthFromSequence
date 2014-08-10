#pragma once

#include <opencv2/opencv.hpp>
#include "depth_from_sequence.hpp"
#include "Eigen/Core"

using namespace cv;
using namespace Eigen;

namespace BundleAdjustment {
  class Solver {

  public:
    
    size_t Nc, Np; // Nc = カメラの数, Np = 特徴点の数
    size_t K; // 変数の数
    
    // inputs
    // 観測した特徴点  i番目のカメラで撮影したj番目の点については captured[i][j] でアクセスする
    vector< vector<Point2d> > captured;

    // outputs
    // 特徴点の３次元位置
    vector<Point3d> points;

    // カメラの外部パラメータ
    vector<Camera> camera_params;

    // カメラの平行移動と回転
    vector<Point3d> cam_t_vec, cam_rot_vec;

    // 更新幅の正則化パラメータ
    double c;

    // run_one_stepのたびに更新される
    bool should_continue;
    int ittr;
    int MAX_ITTR;

    Solver( vector< vector<Point2d> > captured_in )
      :captured( captured_in )
    {
      Nc = captured_in.size(); Np = captured_in[0].size();
      points = vector<Point3d>(Np);
      camera_params = vector<Camera>(Nc);
      cam_t_vec = vector<Point3d>(Nc);
      cam_rot_vec = vector<Point3d>(Nc);

      K = Nc*6 + Np*3;

      c = 0.00001;
      should_continue = true;
      ittr = 0;
      MAX_ITTR = 45;
    }

    void init( vector<Point3d> points_in, vector<Point3d> cam_t_in, vector<Point3d> cam_rot_in);
    void init_with_first_image( vector< vector<Point2d> > captured_in, Size img_size, double mean_depth, double fov);
    double reprojection_error();
    void run_one_step();
    bool get_should_continue( double error_before, double error_after, double update_norm );

  }; // class Solver
} // namespace BundleAdjustment

/*
  bundle adjustment の計算をする上で必要な関数の集合
  状態を持たない。ba_ プレフィックス
*/

// うまくいったらcam_t, cam_rotをCameraに変更する
Point2d ba_reproject( Point3d pt, Point3d cam_t, Point3d cam_rot);
Point3d ba_reproject3d( Point3d pt, Point3d cam_t, Point3d cam_rot);
double ba_get_reproject_gradient_x( BundleAdjustment::Solver &s, int i, int j, int k);
double ba_get_reproject_gradient_y( BundleAdjustment::Solver &s, int i, int j, int k);
double ba_get_reproject_gradient_z( BundleAdjustment::Solver &s, int i, int j, int k);

// 出力用
cv::Mat1b print_point_to_image( vector<Point2d> pt_list,  cv::Size img_size ); // 適当に正規化する
void print_3d_point_to_file( vector<Point3d> pt_list, char filename[] , double s); // printf("%lf,%lf,%lf\n", s*pt.x/pt.z, s*pt.y/pt.z, s*1.0/pt.z) を出力

// 適当な解を作る
std::vector<Point3d> random_3d_points(int N, cv::Point3d min, cv::Point3d max);
std::vector<Point3d> random_3d_cam_t(int N);
std::vector<Point3d> random_3d_cam_rot(int N);

// 適当な解を作る(テスト用)
std::vector<Point3d> mock_3d_points(int N, cv::Point3d min, cv::Point3d max, int reduced_by);

std::vector<Point3d> mock_3d_cam_t(int N);
std::vector<Point3d> mock_3d_cam_rot(int N);

std::vector<Point3d> mock_sequential_cam_t(int N);
std::vector<Point3d> mock_sequential_cam_rot(int N);

// ３次元点を２次元へ射影
std::vector<Point2d> project_3d_to_2d( Point3d cam_t, Point3d cam_rot, std::vector<Point3d> &points);
std::vector<Point2d> project_3d_to_2d_normalize( Point3d cam_t, Point3d cam_rot, std::vector<Point3d> &points, int normalize); // 丸め処理を行う

// BundleAdjustment::Solver の初期解にノイズを加える
void add_noise_to_init_values( BundleAdjustment::Solver &s );


