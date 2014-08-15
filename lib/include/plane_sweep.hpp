#pragma once

#include "depth_from_sequence.hpp"

Matx33d ps_homography_matrix( Camera camera, double depth);

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
                    _homography_matrix[i][d] = PlaneSweep::homography_matrix(ref_cam, cameras[i], depth_variation[d]);
                }
            }
        }

    void sweep(Mat3b &img);

    float *compute_unary_energy();

    // homography matrix to see target_cam image from ref_cam params assuming all points are on the depth
    static Matx33d homography_matrix( Camera ref_cam, Camera target_cam, double depth ) {
        Matx33d homo_ref = ps_homography_matrix(ref_cam, depth);
        Matx33d homo_target = ps_homography_matrix(target_cam, depth);
        return homo_target * homo_ref.inv();
    }

}; // class PlaneSweep
