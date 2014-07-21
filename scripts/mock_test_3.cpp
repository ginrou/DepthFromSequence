#include "plane_sweep.hpp"
#include "bundle_adjustment.hpp"

bool test_projection();

int main(int argc, char* argv[]) {

  if( test_projection() == false ) return 1;

  cout << "test_projection() ok" << endl;

  return 0;

}

bool test_projection() {

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

  return true;
}
