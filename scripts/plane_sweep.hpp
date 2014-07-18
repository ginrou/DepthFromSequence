#ifndef __PLANE_SWEEP__
#define __PLANE_SWEEP__

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class PlaneSweep {
public:

  static Matx44d make_projection_matrix(Point3d trans, Point3d rot, Size img_size) {
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

}; // class PlaneSweep


#endif
