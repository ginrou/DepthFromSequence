#include "plane_sweep.hpp"

Point2d ps_point_in_image( Point3d cam_trans, Point3d cam_rot, Size img_size, Point3d point)  {
  Matx44d proj_mat = PlaneSweep::make_projection_matrix( cam_trans, cam_rot, img_size);
  Matx41d homo_point( point.x, point.y, point.z , 1.0);
  Matx41d proj_point = proj_mat * homo_point;
  return Point2d( proj_point(0,0)/proj_point(0,2), proj_point(0,1)/proj_point(0,2) );
}

Point2d ps_homogenious_point( Point3d trans_ref,
			      Point3d rot_ref,
			      Point3d trans_obj,
			      Point3d rot_obj,
			      Point2d pt_ref,
			      Size img_size,
			      double depth)
{
  Matx33d homo_ref = ps_homography_matrix(trans_ref, rot_ref, img_size, depth);
  Matx33d homo_obj = ps_homography_matrix(trans_obj, rot_obj, img_size, depth);
  Matx33d homo = homo_obj * homo_ref.inv();
  Matx31d pt_obj = homo * Matx31d(pt_ref.x, pt_ref.y, 1.0);
  return Point2d(pt_obj(0,0)/pt_obj(0,2), pt_obj(0,1)/pt_obj(0,2));
}

Matx33d ps_homography_matrix( Point3d trans, Point3d rot, Size img_size, double depth) 
{
  double W = img_size.width, H = img_size.height;
  Matx33d intrinsic( W, 0, W/2.0,
		     0, H, H/2.0,
		     0, 0,     1);

  Matx33d extrinsic( 1.0, -rot.z, rot.y * depth + trans.x,
		     rot.z, 1.0, -rot.x * depth + trans.y,
		     -rot.y, rot.x, 1.0 * depth + trans.z);
  return intrinsic * extrinsic;
}
