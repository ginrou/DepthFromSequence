#include "plane_sweep.hpp"
#include "bundle_adjustment.hpp"

bool test_projection();
bool test_homo_projection();
bool test_depth_variation();
bool point_equals(Point2d pt1, Point2d pt2) {
  return fabs(pt1.x-pt2.x) + fabs(pt1.y-pt2.y) < 10e-10;
}

int main(int argc, char* argv[]) {

  if( test_projection() == false ) return 1;

  cout << "test_projection() ok" << endl;

  if( test_homo_projection() == false ) return 1;

  cout << "test_homo_projection() ok" << endl;

  if( test_depth_variation() == false ) return 1;

  cout << "test_depth_variation() ok" << endl;


  return 0;

}

bool test_projection() {

  { // 射影変換行列が正しいかのテスト
    Point3d trans(10, 0, 0), rot(20, 30, 0);
    Size sz(512, 512);
    Matx44d mat = PlaneSweep::make_projection_matrix( trans, rot, sz);
    Matx44d expected(-7168, 5120, 15616, 5120,
		     -7680, 5632, -9984, 0,
		     -30, 20, 1, 0,
		     0, 0, 0, 1);
    if( mat != expected ) return false;

  }


  { // 原点にカメラがあるパターン
    Point3d trans(0, 0, 0), rot(0, 0, 0);
    Size sz(512, 512);

    if( ps_point_in_image( trans, rot, sz, Point3d(0,0,100))
	!= Point2d(256,256)
	) return false;

    if( ps_point_in_image( trans, rot, sz, Point3d(100,100,200))
	!= Point2d(512, 512)
	) return false;

    if( ps_point_in_image( trans, rot, sz, Point3d(50,100,200))
	!= Point2d(384, 512)
	) return false;
  }

  { // カメラが水平方向に1だけ平行移動したパターン
    // 視差は tx * W / z になる
    Point3d trans(1, 0, 0), rot(0, 0, 0);
    Size sz(512, 512);

    if( ps_point_in_image( trans, rot, sz, Point3d(0,0,100))
	!= Point2d(256 + 512.0/100.0,256)
	) return false;

    if( ps_point_in_image( trans, rot, sz, Point3d(50,50,200))
	!= Point2d(384 + 512.0/200.0, 384)
	) return false;
  }

  { // カメラが垂直方向に10だけ平行移動したパターン
    // 視差は ty * H / z になる
    Point3d trans(0, 10, 0), rot(0, 0, 0);
    Size sz(512, 512);

    if( ps_point_in_image( trans, rot, sz, Point3d(0,0,100))
	!= Point2d(256,256 + 10.0*512.0/100.0)
	) return false;

    if( ps_point_in_image( trans, rot, sz, Point3d(50,50,200))
	!= Point2d(384, 384 + 10.0*512.0/200.0)
	) return false;
  }

  return true;
}

bool test_homo_projection() {

  { // 原点上の同じカメラでの比較の場合は同じ点になる
    Point3d trans(0, 0, 0), rot(0, 0, 0);
    Size sz(512, 512);

    Point2d pt1 = ps_point_in_image(trans, rot, sz, Point3d(50, 50, 200));
    Point2d pt2 = ps_homogenious_point(trans, rot, trans, rot, pt1, sz, 200);
    if( pt1 != pt2 ) return false;
  }

  { // 水平方向に1だけ平行移動したパターン
    // 移動量は tx * W / z になる
    Point3d trans(0, 0, 0), rot(0, 0, 0), trans_1(1,0,0);
    Size sz(512, 512);

    Point3d test_point(50, 50, 200);

    // カメラで直接test_pointを観測
    Point2d pt1 = ps_point_in_image(trans_1, rot, sz, test_point);

    // カメラ0との関係からtest_pointを観測
    Point2d pt_in_ref = ps_point_in_image(trans, rot, sz, test_point);
    Point2d pt2 = ps_homogenious_point(trans, rot, trans_1, rot, pt_in_ref, sz, test_point.z);

    if ( pt1 != pt2 ) return false;
  }

  { // パラメータをいろいろ調整した場合
    // 移動量は tx * W / z になる
    Point3d trans(0, 0, 0), rot(0, 0, 0), trans_1(100,30,10), rot1(0.001, 0.002, -0.001);
    Size sz(512, 512);

    Point3d test_point(50, 50, 200);

    Matx44d proj_ref = PlaneSweep::make_projection_matrix(trans, rot, sz);
    Matx44d proj_obj = PlaneSweep::make_projection_matrix(trans_1, rot1, sz);

    // カメラで直接test_pointを観測
    Point2d pt1 = ps_point_in_image(trans_1, rot1, sz, test_point);

    // カメラ0との関係からtest_pointを観測
    Point2d pt_in_ref = ps_point_in_image(trans, rot, sz, test_point);
    Point2d pt2 = ps_homogenious_point(trans, rot, trans_1, rot1, pt_in_ref, sz, test_point.z);

    if ( fabs(pt1.x-pt2.x) + fabs(pt1.y-pt2.y) > 10e-10  ) return false;
  }

  return true;
}

bool test_depth_variation() {
  Size sz(512, 512);
  Point3d test_point(50,60,70);

  vector<Point3d> cam_t(3);
  cam_t[0] = Point3d(0,0,0); cam_t[1] = Point3d(10,0,0); cam_t[2] = Point3d(20,-10,0.5);

  vector<Point3d> cam_rot(3);
  cam_rot[0] = Point3d(0,0,0); cam_rot[1] = Point3d(0.001,0,0); cam_rot[2] = Point3d(-0.02,0.1,0.5);

  Point2d pt_in_ref = ps_point_in_image( cam_t[0], cam_rot[0], sz, test_point);

  { // cam[1] の時
    Point2d pt1 = ps_point_in_image( cam_t[1], cam_rot[1], sz, test_point);
    Point2d pt2 = ps_homogenious_point( cam_t[0], cam_rot[0], cam_t[1], cam_rot[1], pt_in_ref, sz, test_point.z);
    if ( point_equals(pt1, pt2) == false ) return false;
  }

  { // cam[2] の時
    Point2d pt1 = ps_point_in_image( cam_t[2], cam_rot[2], sz, test_point);
    Point2d pt2 = ps_homogenious_point( cam_t[0], cam_rot[0], cam_t[2], cam_rot[2], pt_in_ref, sz, test_point.z);
    if ( point_equals(pt1, pt2) == false ) return false;
  }

  return true;
}
