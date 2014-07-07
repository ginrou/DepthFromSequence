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

    // 観測した特徴点  i番目のカメラで撮影したj番目の点については captured[i][j] でアクセスする
    vector< vector<Point2f> > captured;

    // 特徴点
    vector<Point3f> points;

    // カメラの平行移動
    vector<Point3f> cam_t_vec;

    // カメラの回転
    vector<Point3f> cam_rot_vec;

    // 更新幅の正則化パラメータ
    double c;

    Solver( vector< vector<Point2f> > captured_in )
      :captured( captured_in )
    {
      Nc = captured_in.size(); Np = captured_in[0].size();
      points = vector<Point3f>(Np);
      cam_t_vec = vector<Point3f>(Nc);
      cam_rot_vec = vector<Point3f>(Nc);

      c = 0.00001;
    }

    void init( vector<Point3f> points_in, vector<Point3f> cam_t_in, vector<Point3f> cam_rot_in);
    double reprojection_error();


  }; // class Solver
} // namespace BundleAdjustment

/*
  bundle adjustment の計算をする上で必要な関数の集合
  状態を持たない。ba_ プレフィックス
*/

Point2f ba_reproject( Point3f pt, Point3f cam_t, Point3f cam_rot);

cv::Mat1b print_point_to_image( vector<Point2f> pt_list,  cv::Size img_size ); // 適当に正規化する

 // 適当な解を作る
 std::vector<Point3f> mock_3d_points(int N, cv::Point3f min, cv::Point3f max, int reduced_by);
 std::vector<Point3f> mock_3d_cam_t(int N);
 std::vector<Point3f> mock_3d_cam_rot(int N);
 std::vector<Point2f> project_3d_to_2d( Point3f cam_t, Point3f cam_rot, std::vector<Point3f> &points);

#endif // __BUNDLE_ADJUTMENT_HPP__
