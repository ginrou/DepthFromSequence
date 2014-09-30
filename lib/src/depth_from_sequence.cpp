#include "depth_from_sequence.hpp"

DepthFromSequence::EstimationStatus DepthFromSequence::estimate()
{
    // convert input images to single channel for freature tracking
    vector<Mat> gray_images;
    for ( vector<Mat3b>::iterator it = _images.begin(); it != _images.end(); ++it ) {
        Mat1b gray(it->size());
        cv::cvtColor(*it, gray, CV_RGB2GRAY);
        gray_images.push_back(gray);
    }

    // FeatureTracking
    FeatureTracker tracker;
    tracker.add_images_batch(gray_images);
    vector< vector<Point2d> > track_points = tracker.pickup_stable_points();

    // bundle adjustment will tend to fail if feature points are less than 70
    if ( track_points.front().size() < 70 ) return FeatureTrackingFailed;

    // Initilaize BundleAdjustment
    BundleAdjustment::Solver solver( track_points );
    cv::Size img_size = _images.front().size();
    double f = MIN(img_size.width, img_size.height)/2.0;
    solver.initialize(track_points, _min_depth, _fov, img_size, f);

    // Run Bundle Adjustment
    while ( solver.should_continue ) {
        solver.run_one_step();
        print_ittr_status(solver);
    }

    this->_cameras = solver.camera_params;

    // bundle adjustment has failed if reprojection error is more than 1.0
    if ( solver.reprojection_error() >= 1.0 ) return BundleAdjustmentFailed;

    vector<double> depths = solver.depth_variation(_depth_resolution);

    PlaneSweep plane_sweep(_images, solver.camera_params, depths, _roi);
    plane_sweep.sweep(_images.front());

    // setup outputs
    this->_cameras = solver.camera_params;
    this->_depth_smooth = plane_sweep._depth_smooth;
    this->_depth_raw = plane_sweep._depth_raw;
    this->_depth_color = plane_sweep._depth_color;

    return Success;
}
