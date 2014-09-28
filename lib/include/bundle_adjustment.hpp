#pragma once

#include <opencv2/opencv.hpp>
#include "depth_from_sequence.hpp"

using namespace cv;

namespace BundleAdjustment {
class Solver {

public:

    size_t Nc, Np; // Nc = number of cameras, Np = number of features
    size_t K; // number of variables

    /**
        inputs
     */

    /// tracked feature
    /// captured[i][j] should be j'th feature point in i'th camera
    vector< vector<Point2d> > captured;

    /**
        outputs
     */

    /// position of feature points in world coordinate
    vector<Point3d> points;

    /// extrinsic camera parameters
    vector<Camera> camera_params;

    /**
       parameters for solving
     */
    int MAX_ITTR;

    /// normalization term for update
    double c;

    /// updated for each itteration
    bool should_continue;
    int ittr;

    /// for debug
    double update_norm;

    Solver( vector< vector<Point2d> > captured_in )
        :captured( captured_in )
        {
            Nc = captured_in.size(); Np = captured_in.front().size();
            points = vector<Point3d>(Np);
            camera_params = vector<Camera>(Nc);

            /// each camera has 6 dof and points in world has 3 dof
            K = Nc*6 + Np*3;

            c = 0.00001; // default value
            should_continue = true;
            ittr = 0;
            MAX_ITTR = 5; // default value
        }

    void initialize(vector< vector<Point2d> > captured_in, double min_depth, double fov, cv::Size img_size, double focal_length);

    double reprojection_error();
    void run_one_step();
    bool get_should_continue( double error_before, double error_after, double update_norm );
    bool good_reporjection();

    /**
       returns depth sequence in this scene, assumed from depth of points
     */
    vector<double> depth_variation(int resolution);

}; // class Solver
} // namespace BundleAdjustment


/**
   functions for debugging
 */
void print_params(BundleAdjustment::Solver &s); // Pointとcamera_paramsを表示
void print_ittr_status(BundleAdjustment::Solver &s); // 反復の状態を表示
template<typename T> void print_histogram(vector<T> v, T bin_size);
