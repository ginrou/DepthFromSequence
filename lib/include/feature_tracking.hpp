#pragma once

#include "depth_from_sequence.hpp"

class FeatureTracker {
public:
    vector<cv::Mat> images;
    vector< vector<cv::Point2f> > all_track_points;
    vector<cv::Point2f> last_try_points;
    void track();

    // params
    int MAX_CORNERS;
    int MAX_IMAGES;
    double QUALITY_LEVEL;
    double MIN_FEATURE_DISTANCE;
    double MIN_TRACK_DISTANCE;
    cv::Size sub_pix_win_size;
    cv::TermCriteria term_crit;

    // helper methods
    Mat1b track_points_image();

    FeatureTracker(std::vector<cv::Mat> images )
        :images(images)
        {}

    FeatureTracker() {
        sub_pix_win_size = cv::Size(10, 10);
        term_crit = cv::TermCriteria( cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03 );
        MAX_CORNERS = 150;
        QUALITY_LEVEL = 0.0075;
        MIN_FEATURE_DISTANCE = 10.0;
        MIN_TRACK_DISTANCE = 25.0;
        MAX_IMAGES = INT_MAX;
    }

    bool add_image(cv::Mat image); // return true if added
    bool points_moved_enough(vector<cv::Point2f> prev_points, vector<cv::Point2f> next_points, vector<uchar> status);
    int count_track_points();

    vector< vector<cv::Point2d> > pickup_stable_points();

    int good_features_to_track(cv::Mat1b img);

private:
    void initialize_tracker(cv::Mat base_image);
    std::vector<Point2f> track_for_image( cv::Mat prev_image,
                                          cv::Mat next_image,
                                          std::vector<Point2f> prev_point,
                                          std::vector<uchar> &status );

    std::vector<uchar> total_status;

};
