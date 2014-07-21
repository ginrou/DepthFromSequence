#include "plane_sweep.hpp"

Point2d ps_point_in_image( Point3d cam_trans, Point3d cam_rot, Size img_size, Point3d point)  {
  Matx44d proj_mat = PlaneSweep::make_projection_matrix( cam_trans, cam_rot, img_size);
  Matx41d homo_point( point.x, point.y, point.z , 1.0);
  Matx41d proj_point = proj_mat * homo_point;
  return Point2d( proj_point(0,0)/proj_point(0,2), proj_point(0,1)/proj_point(0,2) );
}
