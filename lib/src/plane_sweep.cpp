#include "plane_sweep.hpp"
#include "densecrf.h"

const double PlaneSweep::OutOfRangeIntensity = -1;

void PlaneSweep::sweep(Mat3b &img) {
    int W = img.size().width, H = img.size().height;

    float *unary = this->compute_unary_energy();

    unsigned char *img_buf = new unsigned char[W*H*3];
    for( int h = 0; h < H; ++h ) {
        for( int w = 0; w < W; ++w ) {
            Vec3b intenisty = img.at<Vec3b>(h,w);
            img_buf[h*W*3 + w*3 + 0] = intenisty.val[0]; // b
            img_buf[h*W*3 + w*3 + 1] = intenisty.val[1]; // g
            img_buf[h*W*3 + w*3 + 2] = intenisty.val[2]; // r
        }
    }

    DenseCRF2D crf(W,H,_depth_variation.size());
    crf.setUnaryEnergy(unary);
    crf.addPairwiseGaussian(3,3,3);
    crf.addPairwiseBilateral(8, 8, 20, 20, 20, img_buf, 10);

    short *map = new short[W*H];
    crf.map(10, map);

    for( int h = 0; h < H; ++h ) {
        for( int w = 0; w < W; ++w ) {
            _depth_smooth.at<unsigned char>(h,w) = map[h*W+w];
        }
    }

    delete [] unary;
    delete [] img_buf;
    delete [] map;
}

float *PlaneSweep::compute_unary_energy() {
    Mat3b ref_img = _images[0];
    int W = ref_img.cols, H = ref_img.rows, M = _depth_variation.size();

    float *unary = new float[W*H*M];

    for( int h = 0; h < H; ++h ) {
        for( int w = 0; w < W; ++w ) {

            Vec3b ref_val = ref_img.at<Vec3b>(h,w);
            int min_idx;
            double min_val = DBL_MAX;

            for( int d = 0; d < M; ++d ) {

                double err = 0.0;
                for( int n = 1; n < _N; ++n ) { // skip ref image

                    //Point2d pt = ps_homogenious_point( _homography_matrix[n][d], Point2d(w, h));
                    Point2d pt = ps_homogenious_point_cam(_cameras[n], Point2d(w,h), _depth_variation[d]);
                    if( pt.x < 0 || pt.x >= W || pt.y < 0 || pt.y >= H ) {
                        err += 255*255; // とりあえずはずれの場合は最大誤差を入れる
                    } else {
                        Vec3b val = _images[n].at<Vec3b>((int)pt.y, (int)pt.x);
                        err += (ref_val.val[0] - val.val[0]) * (ref_val.val[0] - val.val[0]);
                        err += (ref_val.val[1] - val.val[1]) * (ref_val.val[1] - val.val[1]);
                        err += (ref_val.val[2] - val.val[2]) * (ref_val.val[2] - val.val[2]);
                    }

                }// n
                unary[h*W*M + w*M +d] = log(err/3.0);
                if ( err < min_val ) {
                    min_val = err;
                    min_idx = d;
                }
            }// d
            _depth_raw.at<uchar>(h,w) = min_idx;
        }//w
    }//h

    return unary;
}

Point2d ps_homogenious_point( Matx33d homo_mat, Point2d ref_point) {
    Matx31d dst = homo_mat * Matx31d(ref_point.x, ref_point.y, 1.0);
    return Point2d( dst(0,0)/dst(0,2), dst(0,1)/dst(0,2) );
}

Matx33d ps_homography_matrix( Camera camera, double depth) {
    double W = -camera.img_size.width, H = -camera.img_size.height, f = -camera.f;
    Point3d trans = camera.t, rot = camera.rot;
    Matx33d intrinsic( f, 0, W/2.0,
                       0, f, H/2.0,
                       0, 0,     1);
    Matx33d extrinsic( 1.0, -rot.z, rot.y * depth + trans.x,
                       rot.z, 1.0, -rot.x * depth + trans.y,
                       -rot.y, rot.x, 1.0 * depth + trans.z);
    return intrinsic * extrinsic;
}

Point2d ps_homogenious_point_cam( Camera cam, Point2d pt, double depth) {
    Point3d c, t = cam.t, r = cam.rot;
    c.x =  - ( 1.0 * t.x - r.z * t.y + r.y * t.z);
    c.y =  - ( r.z * t.x + 1.0 * t.y - r.x * t.z);
    c.z =  - (-r.y * t.x + r.x * t.y + 1.0 * t.z);
    double z0 = cam.f;
    double d = (depth - c.z) / (z0 - c.z);

    pt.x = (pt.x - cam.img_size.width/2.0) / cam.f;
    pt.y = (pt.y - cam.img_size.height/2.0) / cam.f;

    Point2d ret( (z0*d/depth)*pt.x + (1.0-d)*c.x, (z0*d/depth)*pt.y + (1.0-d)*c.y);
    ret.x = ret.x * cam.f + cam.img_size.width/2.0;
    ret.y = ret.y * cam.f + cam.img_size.height/2.0;

    return ret;
}
