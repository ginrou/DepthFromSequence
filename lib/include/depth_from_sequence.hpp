#pragma once

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

// extrinsic camera parameter
typedef struct {
    cv::Point3d t; // translaction of camera
    cv::Point3d rot; // rotation of camera
    cv::Size img_size;
    double f; // focal length
} Camera;

// for debug
void dump_camera(Camera camera);

#include "feature_tracking.hpp"
#include "bundle_adjustment.hpp"
#include "plane_sweep.hpp"

class DepthFromSequence {
public:
    enum EstimationStatus {
        Success,
        FeatureTrackingFailed,
        BundleAdjustmentFailed
    };

    /**
       inputs
     */
    std::vector<cv::Mat3b> _images;
    double _min_depth; // nearest point in capture images [mm]
    double _fov; // field of view for camera [deg]
    int _depth_resolution; // resolution of depth estiamtion
    cv::Rect _roi; // region of interest of depth estiamtion

    /** outputs
        only supported after estiamte completed
     */
    vector<Camera> _cameras;
    Mat1b _depth_smooth; // depth map smooth by dence_crf
    Mat1b _depth_raw; // raw depth map
    Mat3b _depth_color; // colord depth map

    DepthFromSequence(std::vector<cv::Mat3b> images, cv::Rect roi)
        :_images(images),
         _roi(roi)
        {
            // Default value
            _min_depth = 500.0;
            _fov = 50.0;
            _depth_resolution = 20;
        }

    EstimationStatus estimate();
};
