#pragma once

#include "depth_from_sequence.hpp"

#include <fstream>

Matx33d ps_homography_matrix( Camera camera, double depth);
Point2d ps_homogenious_point( Matx33d homo_mat, Point2d ref_point);
Matx44d ps_projection_matrix(Camera c, double depth );

typedef void (*p_callback_t)(void *observer, float progress);

class PlaneSweep {
public:

    static const double OutOfRangeIntensity;

    // inputs
    vector<Mat3b> _images;
    vector<Camera> _cameras;
    vector<double> _depth_variation;
    cv::Rect _roi; //region to get depth

    // ouput
    Mat1b _depth_smooth; // depth map smooth by dence_crf
    Mat1b _depth_raw; // raw depth map
    cv::Rect _stable_region; // region where depth map is computed from sufficient images
    p_callback_t _p_callback;
    void *_callback_observer;

    // for used in inside
    int _N; // number of images,
    double _crf_threshold;
    double _sufficient_input;
    vector< vector< Matx33d > > _homography_matrix; // homography_matrix[ img_index ][ depth_index];

    PlaneSweep(vector<Mat3b> images, vector<Camera> cameras, vector<double> depth_variation, cv::Rect roi)
        :_images(images),
         _cameras(cameras),
         _depth_variation(depth_variation),
         _roi(roi)
        {
            cv::Size img_size = roi.size();

            _N = images.size();
            _crf_threshold = 0.15;
            _sufficient_input = 0.75;

            // allocate
            _depth_raw = Mat(img_size, CV_8UC1);
            _depth_smooth = Mat(img_size, CV_8UC1);

            // compute all homography matrix
            Camera ref_cam = cameras[0];
            _homography_matrix = vector< vector< Matx33d > >(_N);
            for(int i = 0; i < cameras.size(); ++i ) {
                _homography_matrix[i] = vector<Matx33d>(depth_variation.size());
                for(int d = 0; d < depth_variation.size(); ++d ) {
                    _homography_matrix[i][d] = PlaneSweep::homography_matrix(ref_cam, cameras[i], depth_variation[d]);
                }
            }

        }

    void set_callback(p_callback_t callback, void *observer) {
        _p_callback = callback;
        _callback_observer = observer;
    }

    void sweep(Mat3b &img);

    void compute_unary_energy(float *unary, cv::Rect &good_region);

    // homography matrix to see target_cam image from ref_cam params assuming all points are on the depth
    static Matx33d homography_matrix( Camera ref_cam, Camera dst_cam, double depth ) {
        Matx44d ref_projection = ps_projection_matrix(ref_cam, depth);
        Matx44d dst_projection = ps_projection_matrix(dst_cam, depth);
        Matx44d M10 = dst_projection * ref_projection.inv();
        return Matx33d( M10(0,0), M10(0,1), M10(0,2),
                        M10(1,0), M10(1,1), M10(1,2),
                        M10(2,0), M10(2,1), M10(2,2) );
    }

}; // class PlaneSweep

// デバッグ用関数
Mat1b warped_image(vector<Mat1b> images, vector<Camera> cameras, double depth);

//saved to tmp/warped-%02d
void save_warped_images(vector<Mat1b> images, vector<Camera> cameras, vector<double> depths);
