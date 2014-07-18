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

  static Matx33d homography_matrix( Matx44d P_base,  Matx44d P_obj, Point3d trans, double disparity ) {
    Matx44d p = P_obj * P_base.inv();
    return Matx33d( p(0,0), p(0,1), p(0,2) + trans.x * disparity,
		    p(1,0), p(1,1), p(1,2) + trans.y * disparity,
		    p(2,0), p(2,1), p(2,2) + trans.z * disparity);

  }


}; // class PlaneSweep


#endif
