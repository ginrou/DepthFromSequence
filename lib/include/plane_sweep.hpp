#pragma once

#include "depth_from_sequence.hpp"

Matx33d ps_homography_matrix( Point3d trans, Point3d rot, cv::Size img_size, double depth);

class PlaneSweep {
public:

    static const double OutOfRangeIntensity;

    // inputs
    vector<Mat> _images;
    vector<Camera> _cameras;
    vector<double> _depth_variation;

    // ouput
    Mat1b _depth_smooth; // depth map smooth by dence_crf
    Mat1b _depth_raw; // raw depth map

    // for used in inside
    int _N; // number of images,
    vector< vector< Matx33d > > _homography_matrix; // homography_matrix[ img_index ][ depth_index];

    PlaneSweep(vector<Mat> images, vector<Camera> cameras, vector<double> depth_variation)
        :_images(images),
         _cameras(cameras),
         _depth_variation(depth_variation)
        {
            cv::Size img_size = images[0].size();

            _N = images.size();

            // allocate
            _depth_smooth = Mat(img_size, CV_8UC1);
            _depth_raw = Mat(img_size, CV_8UC1);

            // compute all homography matrix
            Camera ref_cam = cameras[0];
            _homography_matrix = vector< vector< Matx33d > >(_N);
            for(int i = 0; i < cameras.size(); ++i ) {
                _homography_matrix[i] = vector<Matx33d>(depth_variation.size());
                for(int d = 0; d < depth_variation.size(); ++d ) {
                    _homography_matrix[i][d] = PlaneSweep::homography_matrix(ref_cam, cameras[i], img_size, depth_variation[d]);
                }
            }
        }

    void sweep(Mat3b &img);

    float *compute_unary_energy();

    static Matx44d make_projection_matrix(Point3d trans, Point3d rot, cv::Size img_size) {
        double W = img_size.width, H = img_size.height;
        Matx44d intrinsic( W, 0, W/2.0, 0,
                           0, H, H/2.0, 0,
                           0, 0,     1, 0,
                           0, 0,     0, 1);
        Matx44d extrinsic( 1.0, -rot.z, rot.y, trans.x,
                           rot.z, 1.0, -rot.x, trans.y,
                           -rot.y, rot.x, 1.0, trans.z,
                           0, 0, 0, 1);
        return intrinsic * extrinsic;

    }

    // homography matrix to see target_cam image from ref_cam params assuming all points are on the depth
    static Matx33d homography_matrix( Camera ref_cam, Camera target_cam, cv::Size img_size, double depth ) {
        Matx33d homo_ref = ps_homography_matrix(ref_cam.t, ref_cam.rot, img_size, depth);
        Matx33d homo_target = ps_homography_matrix(target_cam.t, target_cam.rot, img_size, depth);
        return homo_target * homo_ref.inv();
    }

    static Matx33d homography_matrix_( Matx44d P_base,  Matx44d P_obj, Point3d trans, double disparity ) {
        Matx44d p = P_obj * P_base.inv();
        return Matx33d( p(0,0), p(0,1), p(0,2) + trans.x * disparity,
                        p(1,0), p(1,1), p(1,2) + trans.y * disparity,
                        p(2,0), p(2,1), p(2,2) + trans.z * disparity);

    }




}; // class PlaneSweep

Point2d ps_homogenious_point_2( Matx33d homo_mat, Point2d ref_point);


// 状態を持たない関数
Point2d ps_point_in_image( Point3d cam_trans, Point3d cam_rot, cv::Size img_size, Point3d point);

// カメラrefの画像上の点pt_ref が奥行き1.0/disparityにあるとき
// カメラobjで観測したときの画像上の点を返す
// Point2d ps_homogenious_point( Matx44d Proj_ref, // カメラrefの同次座標変換行列
// 			      Matx44d Proj_obj, // カメラobjの同次座標変換行列
// 			      Point2d pt_ref, // カメラrefで撮影した画像上の点
// 			      Size img_size,
// 			      Point3d trans_obj, // カメラobjの平行移動
// 			      double disparity);

Point2d ps_homogenious_point( Point3d trans_ref,
                              Point3d rot_ref,
                              Point3d trans_obj,
                              Point3d rot_obj,
                              Point2d pt_ref,
                              cv::Size img_size,
                              double depth);




double ps_intensity_at_depth(Mat img, Point3d trans_ref, Point3d rot_ref, Point3d trans_obj, Point3d rot_obj, Point2d pt_in_ref, double depth);

int ps_depth_index_for_point(vector<Mat> images, vector<Point3d> trans_vec, vector<Point3d> rot_vec, int row, int col, vector<double> depth_variation);
