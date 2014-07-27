#include "plane_sweep.hpp"

const double PlaneSweep::OutOfRangeIntensity = -1;

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
  double W = -img_size.width, H = -img_size.height;
  Matx33d intrinsic( W, 0, W/2.0,
		     0, H, H/2.0,
		     0, 0,     1);

  Matx33d extrinsic( 1.0, -rot.z, rot.y * depth + trans.x,
		     rot.z, 1.0, -rot.x * depth + trans.y,
		     -rot.y, rot.x, 1.0 * depth + trans.z);
  return intrinsic * extrinsic;
}

double ps_intensity_at_depth(Mat img, Point3d trans_ref, Point3d rot_ref, Point3d trans_obj, Point3d rot_obj, Point2d pt_in_ref, double depth)
{
  Point2d pt = ps_homogenious_point(trans_ref, rot_ref, trans_obj, rot_obj, pt_in_ref, img.size(), depth);

  if( pt.x < 0 || pt.x >= img.cols || pt.y < 0 || pt.y >= img.rows ) return PlaneSweep::OutOfRangeIntensity;

  return (double)img.at<uchar>((int)pt.y  , (int)pt.x  );
}

int ps_depth_index_for_point(vector<Mat> images, vector<Point3d> trans_vec, vector<Point3d> rot_vec, int row, int col, vector<double> depth_variation)
{
  double min_var = DBL_MAX;
  int min_idx = -1;

  for (int d = 0; d < depth_variation.size(); ++d ) {
    
    vector<double> vals;
    double mean = 0.0;
    
    for (int i = 0; i < images.size(); ++i ) {
      double val = ps_intensity_at_depth(images[i], 
					 trans_vec[0], rot_vec[0], trans_vec[i], rot_vec[i],
					 Point2d(col, row), depth_variation[d]);

      if (val != PlaneSweep::OutOfRangeIntensity ) {
	vals.push_back(val);
	mean += val;
      }

    }

    mean /= (double)vals.size();
    double var = 0.0;
    for (int i = 0; i < vals.size(); ++i ) var += (vals[i]-mean)*(vals[i]-mean);

    var = sqrt(var/(double)vals.size());

    if( var < min_var ) {
      min_var = var;
      min_idx = d;
    }

  }

  return min_idx;

}
