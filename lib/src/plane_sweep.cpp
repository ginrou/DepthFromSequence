#include "plane_sweep.hpp"
#include "densecrf.h"

class PlaneSweepMetricFunction : public SemiMetricFunction {
public:
    double threshold;

    void apply( float * out_values, const float * in_values, int value_size ) const {
        for(int j = 0; j < value_size; ++j ) {
            out_values[j] = 0.0;
            for(int i = 0; i < value_size; ++i ){
                out_values[j] += MIN(threshold, fabs(i-j)) * in_values[i];
            }
        }
    }
};


const double PlaneSweep::OutOfRangeIntensity = -1;

void PlaneSweep::sweep(Mat3b &img) {
    int W = _roi.width, H = _roi.height, M = _depth_variation.size();

    float *unary = new float[W*H*M];
    this->compute_unary_energy(unary, _stable_region);
    cout << "unary energy computed" << endl;

    unsigned char *img_buf = new unsigned char[W*H*3];
    for( int h = 0; h < H; ++h ) {
        for( int w = 0; w < W; ++w ) {
            int y = h + _roi.y, x = w + _roi.x;
            Vec3b intenisty = img.at<Vec3b>(y,x);
            img_buf[h*W*3 + w*3 + 0] = intenisty.val[0]; // b
            img_buf[h*W*3 + w*3 + 1] = intenisty.val[1]; // g
            img_buf[h*W*3 + w*3 + 2] = intenisty.val[2]; // r
        }
    }

    DenseCRF2D crf(W,H,_depth_variation.size());
    crf.setUnaryEnergy(unary);
    crf.addPairwiseBilateral(10, 10, 30, 30, 30, img_buf, 256*256, NULL);

    short *map = new short[W*H];
    crf.map(5, map);
    cout << "smooth map computed" << endl;

    for( int h = 0; h < H; ++h ) {
        for( int w = 0; w < W; ++w ) {
            _depth_smooth.at<unsigned char>(h,w) = map[h*W+w];
        }
    }

    _depth_color = create_color_depth_map(_depth_smooth);

    if (_p_callback && _callback_observer) {
        _p_callback(_callback_observer, 1.0);
    }

    delete [] unary;
    delete [] img_buf;
    delete [] map;
}

void PlaneSweep::compute_unary_energy(float *unary, cv::Rect &good_region) {
    Mat3b ref_img = _images[0];
    int W = _roi.width, H = _roi.height, M = _depth_variation.size();

    int left = W, top = H, right = 0, bottom = 0;

    for( int h = _roi.y; h < _roi.br().y; ++h ) {
        for( int w = _roi.x; w < _roi.br().x; ++w ) {

            Vec3b ref_val = ref_img.at<Vec3b>(h,w);
            int out_of_ranges = 0;
            int min_idx = 0;
            double min_val = DBL_MAX;

            for( int d = 0; d < M; ++d ) {

                double err = 1.0; // offset
                for( int n = 1; n < _N; ++n ) { // skip ref image

                    Point2d pt = ps_homogenious_point( _homography_matrix[n][d], Point2d(w, h));
                    if(pt.x < 0 || pt.x >= _images[n].cols || pt.y < 0 || pt.y >= _images[n].rows) {
                        err += 1000 * 3*255*255; // とりあえずはずれの場合は最大誤差を入れる
                        out_of_ranges++;
                    } else {
                        Vec3b val = _images[n].at<Vec3b>((int)pt.y, (int)pt.x);
                        err += (ref_val.val[0] - val.val[0]) * (ref_val.val[0] - val.val[0]);
                        err += (ref_val.val[1] - val.val[1]) * (ref_val.val[1] - val.val[1]);
                        err += (ref_val.val[2] - val.val[2]) * (ref_val.val[2] - val.val[2]);
                    }

                }// n

                int idx = (h-_roi.y)*W*M + (w-_roi.x)*M + d;
                unary[idx] = err/(double)(_N*3);
                if ( err < min_val ) {
                    min_val = err;
                    min_idx = d;
                }
            }// d
            _depth_raw.at<uchar>(h-_roi.y, w-_roi.x) = min_idx;

            if ( 1.0 - (double)out_of_ranges / (double)M*_N >= _sufficient_input) {
                if ( w < W/2 && w < left   ) left = w;
                if ( w > W/2 && w > right  ) right = w;
                if ( h < H/2 && h < top    ) top = h;
                if ( h > H/2 && h > bottom ) bottom = h;
            }

        }//w
        if (_p_callback && _callback_observer) {
            float p = 0.7 * (h-_roi.y+1) / _roi.height;
            _p_callback(_callback_observer, p);
        }

    }//h

    good_region.x = left;
    good_region.y = top;
    good_region.width = right - left + 1;
    good_region.height = bottom - top + 1;
}

Mat3b PlaneSweep::create_color_depth_map(Mat1b &depth_map)
{
    Mat3b hsv(depth_map.size());

    int ds = (UCHAR_MAX+1)/(2*_depth_variation.size()), saturation = 180, lightness = 200;

    for ( int h = 0; h < hsv.rows; ++h ) {
        for ( int w = 0; w < hsv.cols; ++w ) {
            int depth = ds * depth_map.at<uchar>(h,w);
            hsv.at<Vec3b>(h,w) = Vec3b(depth, saturation, lightness);
        }
    }

    Mat3b rgb(hsv.size());
    cvtColor(hsv, rgb, CV_HSV2RGB);
    return rgb;
}

Point2d ps_homogenious_point( Matx33d homo_mat, Point2d ref_point) {
    Matx31d dst = homo_mat * Matx31d(ref_point.x, ref_point.y, 1.0);
    return Point2d( dst(0,0)/dst(0,2), dst(0,1)/dst(0,2) );
}

Matx44d ps_projection_matrix(Camera c, double depth ) {
    double W = c.img_size.width, H = c.img_size.height, f = c.f;
    Point3d trans = c.t, rot = c.rot;
    Matx44d intrinsic( f,  0, W/2.0, 0.0,
                       0, -f, H/2.0, 0.0,
                       0,  0,   1.0, 0.0,
                       0,  0,   0.0, 1.0);

    Matx44d extrinsic( 1.0, -rot.z, rot.y, trans.x,
                       rot.z, 1.0, -rot.x, trans.y,
                       -rot.y, rot.x, 1.0, trans.z,
                       0.0,   0.0,    1.0, -depth);

    return intrinsic * extrinsic;

}
