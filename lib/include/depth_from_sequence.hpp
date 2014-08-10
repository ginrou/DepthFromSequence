#pragma once

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

// extrinsic camera parameter
typedef struct {
  cv::Point3d t; // translaction of camera
  cv::Point3d rot; // rotation of camera
} Camera;

#include "feature_tracking.hpp"
#include "bundle_adjustment.hpp"
#include "plane_sweep.hpp"
