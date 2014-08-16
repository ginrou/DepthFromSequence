#include "feature_tracking.hpp"

void FeatureTracker::track() {
    // initialize
    initialize_tracker(images[0]);

    // track image sequence
    for(int i = 1; i < images.size(); ++i ) {
        std::vector<uchar> status;
        std::vector<Point2f> prev_point = all_track_points[i-1];

        std::vector<Point2f> new_points = track_for_image( images[i-1], images[i], prev_point, status );
        all_track_points.push_back(new_points);

        for(int j = 0; j < status.size(); ++j )
            total_status[j] &= status[j];
    }

}

void FeatureTracker::add_image(cv::Mat image) {
    if ( images.size() == 0 ) {
        initialize_tracker(image);
        images.push_back(image);
    } else {
        std::vector<uchar> status;
        std::vector<Point2f> prev_point = all_track_points.back();
        std::vector<Point2f> new_points = track_for_image(images.back(), image, prev_point, status);
        all_track_points.push_back(new_points);
        images.push_back(image);
        for(int j = 0; j < status.size(); ++j )
            total_status[j] &= status[j];
    }
}

int FeatureTracker::count_track_points() {
    int count = 0;
    for(int j = 0; j < total_status.size(); ++j ) {
        if(total_status[j]) count++;
    }
    return count;
}

void FeatureTracker::initialize_tracker(cv::Mat base_image) {
    vector<Point2f> points;
    cv::goodFeaturesToTrack(base_image, points, MAX_CORNERS, QUALITY_LEVEL, 10, noArray(), 5, true, 0.04);
    cv::cornerSubPix(base_image, points, sub_pix_win_size, cv::Size(-1,-1), term_crit);
    all_track_points.push_back(points);
    total_status = std::vector<uchar>( points.size(), 1 );
}

std::vector<Point2f> FeatureTracker::track_for_image( cv::Mat prev_image,
                                                      cv::Mat next_image,
                                                      std::vector<Point2f> prev_point,
                                                      std::vector<uchar> &status )
{
    std::vector<float> error;
    std::vector<Point2f> next_points;
    cv::calcOpticalFlowPyrLK( prev_image, next_image, prev_point, next_points, status, error);
    return next_points;
}

std::vector< std::vector<cv::Point2d> > FeatureTracker::pickup_stable_points()
{
    vector< vector<cv::Point2d> > ret;

    for( int i = 0; i < all_track_points.size(); ++i ) {
        std::vector<Point2d> pt_list;
        std::vector<Point2f> pt_list_2f;

        for( int j = 0; j < all_track_points[i].size(); ++j ) {
            if(total_status[j]) {
                Point2f pt = all_track_points[i][j];
                pt_list.push_back( Point2d(pt.x, pt.y) );
                pt_list_2f.push_back(pt);
            }
        }

        ret.push_back( pt_list );
        stable_track_points.push_back(pt_list_2f);
    }
    return ret;
}


void FeatureTracker::draw_correspondences(char prefix[]) {

    cv::Size img_size( images[0].cols, images[0].rows );

    for(int i = 1; i < images.size(); ++i ) {

        cv::Mat img( img_size.height ,img_size.width * 2,  CV_8UC1);

        // copy images
        images[i-1].copyTo( img(cv::Rect(0,0,img_size.width, img_size.height)));
        images[i  ].copyTo( img(cv::Rect(img_size.width,0,img_size.width, img_size.height)));

        // draw points and lines
        for(int j = 0; j < stable_track_points[i].size(); ++j ) {
            Point2f pt1 = stable_track_points[i-1][j];
            Point2f pt2 = stable_track_points[i  ][j];
            pt2.x += img_size.width;

            cv::Scalar color(255,0,0,0);

            int radius = 3.0;
            cv::circle(img, pt1, radius, color, -1, 8, 0 );
            cv::circle(img, pt2, radius, color, -1, 8, 0 );

            cv::line(img, pt1, pt2, color );

        }

        // save
        char filename[256];
        sprintf(filename, "%s%02d-%02d.png", prefix, i-1, i );
        imwrite(filename, img);

        cout << filename << " written" << endl;

    }
}
