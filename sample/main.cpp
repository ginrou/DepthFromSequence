#include "depth_from_sequence.hpp"
#include "refocus.hpp"

int main(int argc, char* argv[]) {

    // 画像をロード
    FeatureTracker tracker;
    vector<Mat3b> input_images;
    vector<Mat> gray_images;
    cout << "Feature Tracking, tracking points" << endl;
    for( int i = 1; i < argc; ++i ) {
        input_images.push_back( imread(argv[i],  CV_LOAD_IMAGE_COLOR) );
        gray_images.push_back( imread(argv[i],  CV_LOAD_IMAGE_GRAYSCALE) );
    }
    tracker.add_images_batch(gray_images);

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
    vector<double> depths = solver.depth_variation(20);

    for(int i = 0; i < depths.size(); ++i )
        cout << depths[i] << endl;

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
    imwrite("depth_color.png", ps->_depth_color);

    // refocus
    bool refocus = true;
    if(refocus){
        // aperture
        Mat1b aperture = Refocus::circuler_aperture(64);
        imwrite("aperture.png", aperture);

        // disparity sequence
        std::vector<double> disp_seq = Refocus::depth_to_disparity(depths, 48);

        // initialize
        Mat3b crop_image(roi.size());
        color_image(roi).copyTo(crop_image);
        Refocus refocus(crop_image, aperture, ps->_depth_smooth, disp_seq, 1.8);

        // compute
        Mat3b r = refocus.refocus_to(cv::Point2d(480, 300));
        imwrite("refocus.png", r);
    }

    delete ps;
    return 0;
}
