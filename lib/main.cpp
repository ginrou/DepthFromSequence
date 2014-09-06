#include "depth_from_sequence.hpp"
#include "refocus.hpp"

int main(int argc, char* argv[]) {

    // 画像をロード
    FeatureTracker tracker;
    vector<Mat3b> input_images;
    vector<Mat1b> gray_images;
    cout << "Feature Tracking, tracking points" << endl;
    for( int i = 1; i < argc; ++i ) {

        Mat1b input = imread(argv[i],  CV_LOAD_IMAGE_GRAYSCALE);
        if ( tracker.add_image(input) == true ) {
            input_images.push_back( imread(argv[i],  CV_LOAD_IMAGE_COLOR) );
            gray_images.push_back( imread(argv[i],  CV_LOAD_IMAGE_GRAYSCALE) );
            if ( i == 1 ) imwrite("tmp/ft-01.png",tracker.track_points_image());
            else if ( i == argc/2 ) imwrite("tmp/ft-02.png",tracker.track_points_image());
            else if ( i == argc-1 ) imwrite("tmp/ft-03.png",tracker.track_points_image());
            cout << tracker.count_track_points() << " " ;
        }

    }
    cout << endl;

    vector< vector<Point2d> > track_points = tracker.pickup_stable_points();

    // Solver を初期化
    BundleAdjustment::Solver solver( track_points );
    double min_depth = 500.0; // [mm]
    double fov = 55.0; // [deg]
    cv::Size img_size = input_images.front().size();
    double f = MIN(img_size.width, img_size.height)/2.0;
    solver.initialize(track_points, min_depth, fov, img_size, f);

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

    save_warped_images(gray_images, solver.camera_params, depths);

    // plane sweep + dencecrf で奥行きを求める
    cv::Rect roi;
    if ( img_size.width > img_size.height ) {
        roi = Rect( (img_size.width-img_size.height)/2, 0, img_size.height, img_size.height);
    } else {
        roi = Rect( 0, (img_size.height-img_size.width)/2, img_size.height, img_size.height);
    }

    PlaneSweep *ps = new PlaneSweep(input_images, solver.camera_params, depths, roi);
    // 読み込みがカラー画像になるようにする
    Mat3b color_image = imread(argv[1],  CV_LOAD_IMAGE_COLOR);
    ps->sweep(color_image);

    // output
    imwrite("depth_smooth.png", 8 * ps->_depth_smooth);
    imwrite("depth_raw.png", 8 * ps->_depth_raw);


    // refocus
    // aperture
    Mat1b aperture(64, 64);
    aperture.setTo(0);
    cv::circle(aperture, cv::Point2d(32,32), 32, cv::Scalar::all(255), -1);
    imwrite("aperture.png", aperture);

    // disparity sequence
    std::vector<double> disp_seq(depths.size());
    for(int i = 0; i < disp_seq.size(); ++i ) disp_seq[i] = i+1;

    // initialize
    Mat3b crop_image(roi.size());
    color_image(roi).copyTo(crop_image);
    Refocus refocus(crop_image, aperture, ps->_depth_smooth, disp_seq, 0.8);

    // compute
    Mat3b r = refocus.refocus_to(cv::Point2d(100, 200));
    imwrite("refocus.png", r);

    delete ps;
    return 0;
}
