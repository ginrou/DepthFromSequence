#ifndef __BUNDLE_ADJUTMENT_HPP__
#define __BUNDLE_ADJUTMENT_HPP__

#include <opencv2/opencv.hpp>
#include "Eigen/Core"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace BundleAdjustment {
  class Solver {

  public:
    
    size_t Nc, Np; // Nc = カメラの数, Np = 特徴点の数
    size_t K; // 変数の数

    // 観測した特徴点  i番目のカメラで撮影したj番目の点については captured[i][j] でアクセスする
    vector< vector<Point2d> > captured;

    // 特徴点
    vector<Point3d> points;

    // カメラの平行移動と回転
    vector<Point3d> cam_t_vec, cam_rot_vec;

    // 更新幅の正則化パラメータ
    double c;

    // run_one_stepのたびに更新される
    bool should_continue;

    Solver( vector< vector<Point2d> > captured_in )
      :captured( captured_in )
    {
      Nc = captured_in.size(); Np = captured_in[0].size();
      points = vector<Point3d>(Np);
      cam_t_vec = vector<Point3d>(Nc);
      cam_rot_vec = vector<Point3d>(Nc);

      K = Nc*6 + Np*3;

      c = 0.00001;
      should_continue = true;
    }

    void init( vector<Point3d> points_in, vector<Point3d> cam_t_in, vector<Point3d> cam_rot_in);
    double reprojection_error();
    void run_one_step();

  }; // class Solver
} // namespace BundleAdjustment

/*
  bundle adjustment の計算をする上で必要な関数の集合
  状態を持たない。ba_ プレフィックス
*/

Point2d ba_reproject( Point3d pt, Point3d cam_t, Point3d cam_rot);
Point3d ba_reproject3d( Point3d pt, Point3d cam_t, Point3d cam_rot);
double ba_get_reproject_gradient_x( BundleAdjustment::Solver &s, int i, int j, int k);
double ba_get_reproject_gradient_y( BundleAdjustment::Solver &s, int i, int j, int k);
double ba_get_reproject_gradient_z( BundleAdjustment::Solver &s, int i, int j, int k);
bool ba_should_continue( double error_before, double error_after, double update_norm );

cv::Mat1b print_point_to_image( vector<Point2d> pt_list,  cv::Size img_size ); // 適当に正規化する

 // 適当な解を作る
 std::vector<Point3d> mock_3d_points(int N, cv::Point3d min, cv::Point3d max, int reduced_by);
 std::vector<Point3d> mock_3d_cam_t(int N);
 std::vector<Point3d> mock_3d_cam_rot(int N);
 std::vector<Point2d> project_3d_to_2d( Point3d cam_t, Point3d cam_rot, std::vector<Point3d> &points);

#endif // __BUNDLE_ADJUTMENT_HPP__
