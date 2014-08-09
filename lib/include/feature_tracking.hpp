#ifndef __FEATURE_TRACKING_HPP__
#define __FEATURE_TRACKING_HPP__

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace BundleAdjustment {
  class FeatureTracker {
  public:
    vector<cv::Mat> images;
    vector< vector<cv::Point2f> > stable_track_points;
    vector< vector<cv::Point2f> > all_track_points;
    void track();

    // params
    int MAX_CORNERS;
    cv::Size sub_pix_win_size;
    cv::TermCriteria term_crit;

    // helper methods
    void draw_correspondences(char prefix[]);

    FeatureTracker(std::vector<cv::Mat> images )
      :images(images)
    {
      sub_pix_win_size = cv::Size(10, 10);
      term_crit = cv::TermCriteria( cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03 );
      MAX_CORNERS = 200;
    }

    vector< vector<cv::Point2d> > pickup_stable_points();

  private:
    void initialize_tracker(cv::Mat base_image);
    std::vector<Point2f> track_for_image( cv::Mat prev_image,
					  cv::Mat next_image,
					  std::vector<Point2f> prev_point,
					  std::vector<uchar> &status );

    std::vector<uchar> total_status;

  };
}

#endif // __FEATURE_TRACKING_HPP__



