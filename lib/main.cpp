#include "depth_from_sequence.hpp"

int main(int argc, char* argv[]) {

    // 画像をロード
    FeatureTracker tracker;
    vector<Mat> input_images;
    cout << "Feature Tracking, tracking points" << endl;
    for( int i = 1; i < argc-1; ++i ) {
        input_images.push_back( imread(argv[i], CV_8UC1) );
        tracker.add_image(input_images.back());
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

    // plane sweep + dencecrf で奥行きを求める
    PlaneSweep *ps = new PlaneSweep(input_images, solver.camera_params, depths);

    // 読み込みがカラー画像になるようにする
    Mat3b color_image(input_images[0].size(), CV_8UC3);
    if ( true ) {
        for( int h = 0; h < color_image.rows; ++h ) {
            for( int w = 0; w < color_image.cols; ++w ) {
                Vec3b intenisty(input_images[0].at<unsigned char>(h,w));
                color_image.at<Vec3b>(h,w) = intenisty;
            }
        }
    } else {
        color_image = imread(argv[1], CV_8UC3);
    }

    ps->sweep(color_image);

    // output
    imwrite(argv[argc-1], ps->_depth_smooth);
    imwrite("raw_depth.png", ps->_depth_raw);

    delete ps;
    return 0;
}
