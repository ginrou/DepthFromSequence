#include "depth_from_sequence.hpp"

void test();

int main(int argc, char* argv[]) {

    test();
    return 0;

    // 画像をロード
    FeatureTracker tracker;
    vector<Mat3b> input_images;
    vector<Mat1b> gray_images;
    cout << "Feature Tracking, tracking points" << endl;
    for( int i = 1; i < argc-1; ++i ) {
        input_images.push_back( imread(argv[i],  CV_LOAD_IMAGE_COLOR) );
        gray_images.push_back( imread(argv[i],  CV_LOAD_IMAGE_GRAYSCALE) );
        tracker.add_image( imread(argv[i],  CV_LOAD_IMAGE_GRAYSCALE) );

        if ( i == 1 ) imwrite("tmp/ft-01.png",tracker.track_points_image());
        else if ( i == argc/2 ) imwrite("tmp/ft-02.png",tracker.track_points_image());
        else if ( i == argc-2 ) imwrite("tmp/ft-03.png",tracker.track_points_image());

        cout << tracker.count_track_points() << " " ;
    }
    cout << endl;

    vector< vector<Point2d> > track_points = tracker.pickup_stable_points();

    // Solver を初期化
    BundleAdjustment::Solver solver( track_points );
    double min_depth = 500.0; // [mm]
    double fov = 55.0; // [deg]
    cv::Size img_size = input_images.front().size();
    double f = MIN(img_size.width, img_size.height);
    solver.initialize(track_points, min_depth, fov, img_size, f);
    print_params(solver);

    // bundle adjustment を実行
    while ( solver.should_continue ) {
        solver.run_one_step();
        print_ittr_status(solver);
    }

    print_params(solver);

    // plane sweep の準備
    vector<double> depths = solver.depth_variation(32);

    for(int i = 0; i < depths.size(); ++i )
        cout << depths[i] << endl;

    for(int i = 0; i < depths.size(); ++i ) {
        Mat1b hoge = warped_image(gray_images, solver.camera_params, depths[i]);
        char filename[256];
        sprintf(filename, "tmp/warped-%02d.png", i);
        imwrite(filename, hoge);
    }
    return 0;

    // plane sweep + dencecrf で奥行きを求める
    PlaneSweep *ps = new PlaneSweep(input_images, solver.camera_params, depths);

    // 読み込みがカラー画像になるようにする
    Mat3b color_image(input_images[0].size(), CV_8UC3);
    if ( false ) {
        for( int h = 0; h < color_image.rows; ++h ) {
            for( int w = 0; w < color_image.cols; ++w ) {
                Vec3b intenisty(input_images[0].at<unsigned char>(h,w));
                color_image.at<Vec3b>(h,w) = intenisty;
            }
        }
    } else {
        color_image = imread(argv[1],  CV_LOAD_IMAGE_COLOR);
    }

    ps->sweep(color_image);

    // output
    imwrite(argv[argc-1], ps->_depth_smooth);
    imwrite("raw_depth.png", ps->_depth_raw);

    delete ps;
    return 0;
}


Point2d projection_point(Camera c, Point3d pt) {
    Matx41d pt4( pt.x, pt.y, pt.z, 1.0);
    Matx44d proj = ps_projection_matrix(c, 1.0);
    Matx41d pt_out = proj * pt4;
    return Point2d( pt_out(0,0)/pt_out(0,2), pt_out(0,1)/pt_out(0,2));
}

bool check(Point2d in, Point2d out) {
    bool ok = in == out;
    cout <<  in << " => " << out << ", " << ok << endl;
    return ok;
}

void test() {
    Point2d in, out;

    Camera ref_cam, dst_cam;
    ref_cam.f = 100.0;
    ref_cam.img_size = cv::Size(256, 256);
    ref_cam.t = Point3d(0, 0, 0);
    ref_cam.rot = Point3d(0, 0, 0);

    Point2d p = ps_homogenious_point_2(ref_cam, ref_cam, Point2d(128,128), 10);
    check(Point2d(128,128), p);
 
    dst_cam.f = 100.0;
    dst_cam.img_size = cv::Size(256, 256);
    dst_cam.t = Point3d(1000, -30, 40);
    dst_cam.rot = Point3d(0.001, 0.0003, -0.004);
    
    Point3d src(400, 500, 3000);
    in = projection_point(ref_cam, src);
    out = projection_point(dst_cam, src);
    p = ps_homogenious_point_2(ref_cam, dst_cam, in, src.z * 2);
    cout << in << endl;
    cout << out << endl;
    cout << p << endl;


}
