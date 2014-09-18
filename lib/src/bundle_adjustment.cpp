#include "bundle_adjustment.hpp"
#include "Eigen/LU"

#include <cstdlib>
#include <ctime>

template<typename T> void print_histogram(vector<T> v, T bin_size) {
    T min = v[0], max = v[0], sum = v[0];
    for ( int i = 1; i < v.size(); ++i ) {
        if ( v[i] < min ) min = v[i];
        if ( v[i] > max ) max = v[i];
        sum += v[i];
    }

    int size = (max-min)/bin_size + 1;
    vector<int> count(size);
    for ( int i = 0; i < v.size(); ++i ) {
        int bin = (v[i] - min)/bin_size;
        count[bin]++;
    }

    for ( int i = 0; i < size; ++i ) {
        printf("%3d - %3d : ", (int)(i*bin_size), (int)((i+1)*bin_size));
        for ( int j = 0; j < count[i]; ++j ) printf("#");
        printf("\n");
    }

    double mean = sum/(double)v.size(), var = 0.0;
    for ( int i = 0; i < v.size(); ++i ) var += (v[i]-mean)*(v[i]-mean);

    printf("variance = %lf\n", sqrt(var)/(double)v.size());
}


inline double drand() { return (double)rand()/RAND_MAX; }

std::vector<Camera> initial_camera_params(int N, cv::Size img_size, double focal_length) {
    std::vector<Camera> ret(N);

    std::srand(std::time(0));
    Point3d min_t(0.0, -0.5, -0.005), max_t(20.0, 0.5, 0.005);
    Point3d min_rot(-0.001, -0.001, -0.001), max_rot(0.001, 0.001, 0.001);
    for( int i = 0; i < N; ++i ) {
        ret[i].t.x = min_t.x + (max_t.x - min_t.x) * drand();
        ret[i].t.y = min_t.y + (max_t.y - min_t.y) * drand();
        ret[i].t.z = min_t.z + (max_t.z - min_t.z) * drand();
        ret[i].rot.x = min_rot.x + (max_rot.x - min_rot.x) * drand();
        ret[i].rot.y = min_rot.y + (max_rot.y - min_rot.y) * drand();
        ret[i].rot.z = min_rot.z + (max_rot.z - min_rot.z) * drand();
        ret[i].img_size = img_size;
        ret[i].f = focal_length;
    }

    // set first camera to the origin of world cordinate
    ret[0].t = Point3d(0, 0, 0);
    ret[0].rot = Point3d(0, 0, 0);

    // set translation between first camera and second camera to 5.0
    ret[1].t.x = 5.0;

    return ret;
}

void BundleAdjustment::Solver::init_with_first_image( vector< vector<Point2d> > captured_in,
                                                      cv::Size img_size,
                                                      double focal_length,
                                                      double mean_depth,
                                                      double fov
)
{
    double W = img_size.width, H = img_size.height;
    double tan_fov = tan(fov/(2.0*M_PI));

    // 1. initialize points
    for( int j = 0; j < Np; ++j ) {
        points[j].x = (2.0 * captured_in[0][j].x - W ) * tan_fov / focal_length;
        points[j].y = (2.0 * captured_in[0][j].y - H ) * tan_fov / focal_length;
        points[j].z = (1.0 + 0.01 * drand() ) / mean_depth;
    }

    // 2. initialize captured
    for( int i = 0; i < Nc; ++i ) {
        for( int j = 0; j < Np; ++j ) {
            captured[i][j].x =  (captured_in[i][j].x - W/2.0 ) / focal_length;
            captured[i][j].y = -(captured_in[i][j].y - H/2.0 ) / focal_length;
        }
    }

    // 3. initialize camera params
    camera_params = initial_camera_params(Nc, img_size, focal_length);
}

void BundleAdjustment::Solver::initialize(vector< vector<Point2d> > captured_in,
                                          double min_depth,
                                          double fov,
                                          cv::Size img_size,
                                          double focal_length)
{
    double W = img_size.width, H = img_size.height;
    double tan_fov = tan(fov/(2.0*M_PI));

    // captured : camera coordinate -> world coordinate
    for( int i = 0; i < Nc; ++i ) {
        for( int j = 0; j < Np; ++j ) {
            captured[i][j].x =  (captured_in[i][j].x - W/2.0 ) / focal_length;
            captured[i][j].y = -(captured_in[i][j].y - H/2.0 ) / focal_length;
        }
    }

    // 各特徴点の平均移動量を計算
    vector<double> pt_avg(Np, 0);
    double pt_max = 0.0;
    for ( int j = 0; j < Np; ++j ) {
        for ( int i = 0; i < Nc; ++i ) {
            double x = captured_in[i][j].x, y = captured_in[i][j].y;
            pt_avg[j] +=  sqrt(x*x + y*y)/ (double)Np;
        }

        if ( pt_max < pt_avg[j] ) pt_max = pt_avg[j];
    }

    double a = pt_max * min_depth; // 最も近い点と最も移動量が大きい点を対応付ける
    double z_avg = 0.0;
    for ( int j = 0; j < Np; ++j ) {
        points[j].x = (2.0 * captured_in[0][j].x - W ) * tan_fov / focal_length;
        points[j].y = (2.0 * captured_in[0][j].y - H ) * tan_fov / focal_length;
        points[j].z = a / pt_avg[j]; // 奥行きは最も近い点の逆数に比例する
        z_avg += points[j].z/(double)Np;
    }

    // カメラの外部パラメータなどの初期化
    Camera c;
    c.t = Point3d(0,0,0); c.rot = Point3d(0,0,0);
    c.f = focal_length, c.img_size = img_size;
    camera_params[0] = c;

    for ( int i = 1; i < Nc; ++i ) {
        Point2d trans_avg(0,0);
        for ( int j = 0; j < Np; ++j ) {
            trans_avg.x += (captured[0][j].x - captured[i][j].x)/(double)Np;
            trans_avg.y += (captured[0][j].y - captured[i][j].y)/(double)Np;
        }
        Camera c;
        c.t.x = -z_avg * trans_avg.x;
        c.t.y = -z_avg * trans_avg.y;
        c.t.z = 0.0;
        c.rot = Point3d(0,0,0);
        c.img_size = img_size;
        c.f = focal_length;
        camera_params[i] = c;

        camera_params[i].t.x /= fabs(0.2*camera_params[1].t.x); // 一つ目のカメラの平行移動を5に正規化
    }

    // zでパラメータ化しているので戻す
    for ( int j = 0; j < Np; ++j ) points[j].z = 1.0/points[j].z;

}


double BundleAdjustment::Solver::reprojection_error() {
    double error = 0.0;

    for( int i = 0; i < Nc; ++i ) {
        for( int j = 0; j < Np; ++j ) {
            Point2d reproj = ba_reproject(points[j], camera_params[i] );
            double ex = captured[i][j].x - reproj.x, ey = captured[i][j].y - reproj.y;
            error += ex*ex + ey*ey;
        }
    }

    return error;
}

void BundleAdjustment::Solver::run_one_step() {
    ittr++;

    // 領域確保
    int K = this->K -7;
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(2*Nc*Np, K);
    Eigen::VectorXd target_error = Eigen::VectorXd::Zero(2*Nc*Np);

    // 更新前の再投影エラー
    double error_before = this->reprojection_error();

    // Jacobianと誤差を計算
    for ( int i = 0; i < Nc; ++i ) {
        for ( int j = 0; j < Np; ++j ) {
            int nx = i*Np + j, ny = i*Np + j + Np*Nc;

            Point2d p = captured[i][j];
            Point3d q = ba_reproject3d(points[j], camera_params[i]);
            target_error[nx] = p.x - q.x/q.z;
            target_error[ny] = p.y - q.y/q.z;

            for ( int k = 0, l = 0; k < this->K; ++k ) {
                if ( k == 0 ) continue;
                if ( k == 1 ) continue;
                if ( k == Nc ) continue;
                if ( k == 2*Nc ) continue;
                if ( k == 3*Nc ) continue;
                if ( k == 4*Nc ) continue;
                if ( k == 5*Nc ) continue;

                Point3d grad;
                grad.x = ba_get_reproject_gradient_x( *this, i, j, k );
                grad.y = ba_get_reproject_gradient_y( *this, i, j, k );
                grad.z = ba_get_reproject_gradient_z( *this, i, j, k );

                Jacobian(nx, l) = (q.x * grad.z - q.z * grad.x ) / (q.z*q.z);
                Jacobian(ny, l) = (q.y * grad.z - q.z * grad.y ) / (q.z*q.z);
                l++;
            }
        }
    }

    // 更新方向の計算
    Eigen::VectorXd gradient = -Jacobian.transpose() * target_error;
    Eigen::MatrixXd Hessian = Jacobian.transpose() * Jacobian + this->c * Eigen::MatrixXd::Identity(K, K);

    Eigen::VectorXd sol = Hessian.fullPivLu().solve(gradient);
    Eigen::VectorXd update = Eigen::VectorXd::Zero(this->K);

    // 更新
    for ( int k = 0, l = 0; k < this->K; ++k ) {
        if ( k == 0 ) continue;
        if ( k == 1 ) continue;
        if ( k == Nc ) continue;
        if ( k == 2*Nc ) continue;
        if ( k == 3*Nc ) continue;
        if ( k == 4*Nc ) continue;
        if ( k == 5*Nc ) continue;

        update[k] = sol[l];
        l++;
    }

    for ( int i = 0; i < Nc; ++i ) {
        camera_params[i].t.x   += update[i];
        camera_params[i].t.y   += update[i+Nc];
        camera_params[i].t.z   += update[i+2*Nc];
        camera_params[i].rot.x += update[i+3*Nc];
        camera_params[i].rot.y += update[i+4*Nc];
        camera_params[i].rot.z += update[i+5*Nc];
    }

    for ( int j = 0; j < Np; ++j ) {
        points[j].x += update[ j + 0*Np + 6*Nc ];
        points[j].y += update[ j + 1*Np + 6*Nc ];
        points[j].z += update[ j + 2*Np + 6*Nc ];
    }

    double error_after = this->reprojection_error();

    // 正則化パラメータは更新しないほうが収束が早い
    error_before > error_after ? this->c *= 0.1 : this->c *= 10.0;
    update_norm = update.norm();

    this->should_continue = get_should_continue( error_before, error_after, update_norm);

}

bool BundleAdjustment::Solver::get_should_continue( double error_before, double error_after, double update_norm ) {

    if ( update_norm < 1.0e-5 ) return false;
    if ( fabs((error_after - error_before)/error_after) < 1.0e-4 ) return false;
    if ( ittr >= MAX_ITTR ) return false;

    return true;
}

bool BundleAdjustment::Solver::good_reporjection() {
    return this->reprojection_error() < 1.0; // 1.0より大きいと割と推定に失敗する
}

vector<double> BundleAdjustment::Solver::depth_variation(int resolution) {
    double min = DBL_MAX, max = 0.0;
    for ( int j = 0; j < Np; ++j ) {
        double z = 1.0/points[j].z;
        if ( z < min ) min = z;
        if ( z > max ) max = z;
    }
    min /= 1.5;
    max *= 2.0;

    vector<double> ret(resolution);
    double zmax = 1.0/min, zmin = 1.0/max;

    ret[0] = max;
    for ( int i = 1; i < resolution; ++i ) {
        ret[i] = 1.0/((i-1)*(zmax-zmin)/(double)(resolution-1) + zmin);
    }

    return ret;
}


// 単体の関数ここから
inline Point3d ba_reproject3d( Point3d pt, Camera cam) {
    Point3d ret;
    ret.x = ( pt.x - cam.rot.z * pt.y + cam.rot.y ) / pt.z + cam.t.x;
    ret.y = ( pt.x * cam.rot.z + pt.y - cam.rot.x ) / pt.z + cam.t.y;
    ret.z = (-pt.x * cam.rot.y + pt.y * cam.rot.x +1.0 ) / pt.z + cam.t.z;
    return ret;
}

inline Point2d ba_reproject( Point3d pt, Camera cam) {
    Point3d ret = ba_reproject3d(pt, cam);
    return Point2d( ret.x / ret.z, ret.y / ret.z );
}

inline double delta(int i, int j) { return i == j ? 1.0 : 0.0; }

inline double ba_get_reproject_gradient_x( BundleAdjustment::Solver &s, int i, int j, int k) {
    Point3d pt = s.points[j], cam_rot = s.camera_params[i].rot;

    if ( k < s.Nc ) // Txでの微分
        return delta(i,k);

    else if ( k < 2*s.Nc ) // Tyでの微分
        return 0;

    else if ( k < 3*s.Nc ) // Tzでの微分
        return 0;

    else if ( k < 4*s.Nc ) // pose_xでの微分
        return 0;

    else if ( k < 5*s.Nc ) // pose_yでの微分
        return delta(i,k -4*s.Nc) / pt.z;

    else if ( k < 6*s.Nc ) // pose_zでの微分
        return -delta(i,k-5*s.Nc) * pt.y / pt.z;

    else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
        return delta(j,k-6*s.Nc) / pt.z;

    else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
        return - delta(j, k - 6*s.Nc - s.Np) * cam_rot.z / pt.z;

    else // point_zでの微分
        return -delta(j, k - 6*s.Nc - 2*s.Np) *  ( pt.x - cam_rot.z * pt.y + cam_rot.y ) / ( pt.z*pt.z );

}

inline double ba_get_reproject_gradient_y( BundleAdjustment::Solver &s, int i, int j, int k) {
    Point3d pt = s.points[j], cam_rot = s.camera_params[i].rot;

    if ( k < s.Nc ) // Txでの微分
        return 0;

    else if ( k < 2*s.Nc ) // Tyでの微分
        return delta(i, k - s.Nc );

    else if ( k < 3*s.Nc ) // Tzでの微分
        return 0;

    else if ( k < 4*s.Nc ) // pose_xでの微分
        return -delta(i, k - 3*s.Nc) / pt.z;

    else if ( k < 5*s.Nc ) // pose_yでの微分
        return 0.0;

    else if ( k < 6*s.Nc ) // pose_zでの微分
        return delta(i, k - 5*s.Nc) * pt.x / pt.z;

    else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
        return delta(j, k - 6*s.Nc) * cam_rot.z / pt.z;

    else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
        return delta(j, k - 6*s.Nc - s.Np) / pt.z;

    else // point_zでの微分
        return -delta(j, k - 6*s.Nc - 2*s.Np) * ( cam_rot.z*pt.x + pt.y - cam_rot.x) / (pt.z*pt.z);

}


inline double ba_get_reproject_gradient_z( BundleAdjustment::Solver &s, int i, int j, int k) {
    Point3d pt = s.points[j], cam_rot = s.camera_params[i].rot;

    if ( k < s.Nc ) // Txでの微分
        return 0;

    else if ( k < 2*s.Nc ) // Tyでの微分
        return 0;

    else if ( k < 3*s.Nc ) // Tzでの微分
        return delta(i, k - 2*s.Nc);

    else if ( k < 4*s.Nc ) // pose_xでの微分
        return delta(i, k - 3*s.Nc) * pt.y / pt.z;

    else if ( k < 5*s.Nc ) // pose_yでの微分
        return -delta(i, k - 4*s.Nc) * pt.x / pt.z;

    else if ( k < 6*s.Nc ) // pose_zでの微分
        return 0;

    else if ( k < 6*s.Nc + s.Np ) // point_xでの微分
        return - delta(j, k - 6*s.Nc) * cam_rot.y / pt.z;

    else if ( k < 6*s.Nc + 2 * s.Np ) // point_yでの微分
        return delta(j, k - 6*s.Nc - s.Np) * cam_rot.x / pt.z;

    else // point_zでの微分
        return -delta(j, k - 6*s.Nc - 2*s.Np) * ( -cam_rot.y*pt.x + cam_rot.x*pt.y + 1.0 ) / (pt.z*pt.z);

}
