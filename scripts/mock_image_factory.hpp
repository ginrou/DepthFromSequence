#ifndef __MOCK_IMAGE_FACTORY__
#define __MOCK_IMAGE_FACTORY__

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Mat image_with_points(Size size, vector<Point2d> points);

#endif

