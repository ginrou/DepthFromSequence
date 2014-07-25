#ifndef __PLANE_SWEEP__
#define __PLANE_SWEEP__

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class PlaneSweep {
public:

  static const double OutOfRangeIntensity;

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

// 状態を持たない関数
Point2d ps_point_in_image( Point3d cam_trans, Point3d cam_rot, Size img_size, Point3d point);

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
			      Size img_size,
			      double depth);


Matx33d ps_homography_matrix( Point3d trans, Point3d rot, Size img_size, double depth);

double ps_intensity_at_depth(Mat img, Point3d trans_ref, Point3d rot_ref, Point3d trans_obj, Point3d rot_obj, Point2d pt_in_ref, double depth);

#endif
