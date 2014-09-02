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

    // 更新幅の正則化パラメータ
    double c;

    // run_one_stepのたびに更新される
    bool should_continue;
    int ittr;
    int MAX_ITTR;

    // for debug
    double update_norm;

    Solver( vector< vector<Point2d> > captured_in )
        :captured( captured_in )
        {
            Nc = captured_in.size(); Np = captured_in[0].size();
            points = vector<Point3d>(Np);
            camera_params = vector<Camera>(Nc);

            K = Nc*6 + Np*3;

            c = 0.00001;
            should_continue = true;
            ittr = 0;
            MAX_ITTR = 5;
        }

    void init_with_first_image( vector< vector<Point2d> > captured_in, cv::Size img_size, double focal_length, double mean_depth, double fov);
    void initialize(vector< vector<Point2d> > captured_in, double min_depth, double fov, cv::Size img_size, double focal_length);

    double reprojection_error();
    void run_one_step();
    bool get_should_continue( double error_before, double error_after, double update_norm );
    bool good_reporjection();

    vector<double> depth_variation(int resolution);

}; // class Solver
} // namespace BundleAdjustment

/*
  bundle adjustment の計算をする上で必要な関数の集合
  状態を持たない。ba_ プレフィックス
*/

// うまくいったらcam_t, cam_rotをCameraに変更する
inline Point2d ba_reproject( Point3d pt, Camera cam);
inline Point3d ba_reproject3d( Point3d pt, Camera cam);
inline double ba_get_reproject_gradient_x( BundleAdjustment::Solver &s, int i, int j, int k);
inline double ba_get_reproject_gradient_y( BundleAdjustment::Solver &s, int i, int j, int k);
inline double ba_get_reproject_gradient_z( BundleAdjustment::Solver &s, int i, int j, int k);

// デバッグ用関数
void print_params(BundleAdjustment::Solver &s); // Pointとcamera_paramsを表示
void print_ittr_status(BundleAdjustment::Solver &s); // 反復の状態を表示
template<typename T> void print_histogram(vector<T> v, T bin_size);
