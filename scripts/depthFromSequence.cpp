#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/stitcher.hpp>

using namespace std;
using namespace cv;

void bundle_adjustment( vector< vector<Point2f> > points , vector<Mat> &dst);

int main(int argc, char* argv[]) {

  static int MAX_CORNERS = 200;
  cv::Size sub_pix_win_size(10, 10);
  cv::TermCriteria term_crit( cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03 );
  cv::Size img_size;
  vector<Mat> input_images;
  vector< vector<Point2f> > track_points;

  // read images
  for (int i = 1; i < argc-1; ++i )
    input_images.push_back( imread(argv[i], CV_8UC1) );
  
  img_size = input_images[0].size();  

  // initialize trackers
  {
    vector<Point2f> points;
    cv::goodFeaturesToTrack( input_images[0], points, MAX_CORNERS, 0.01, 10);
    cv::cornerSubPix(input_images[0], points, sub_pix_win_size, cv::Size(-1,-1), term_crit );
    track_points.push_back(points);
  }

  std::vector<uchar> total_status(track_points[0].size(), 1);

  // tracking sequence
  for( int i = 1; i < input_images.size(); ++i ) {
    std::vector<uchar> status;
    std::vector<float> error;
    
    std::vector<Point2f> curr_points, prev_points = track_points[i-1];
    Mat curr_image = input_images[i], prev_image = input_images[i-1];

    cv::calcOpticalFlowPyrLK( prev_image, curr_image, prev_points, curr_points, status, error);

    // masking current status
    for(int j = 0; j < status.size(); ++j )
      total_status[j] &= status[j];

    track_points.push_back(curr_points);
    printf("%d end\n", i);
  }

  // construct fund matrix and rectify
  Mat fund_mat, rectified[2];
  {
    std::vector<Point2f> points1, points2;
    for( int i = 0; i < total_status.size(); ++i ){
      if( !total_status[i] ) continue;

      points1.push_back( track_points.front()[i] );
      points2.push_back( track_points.back()[i] );

    }

    std::vector<uchar> find_fund_mat_status;
    fund_mat = findFundamentalMat( points1, points2, find_fund_mat_status );

    Mat H1, H2;
    stereoRectifyUncalibrated( points1, points2, fund_mat, img_size, H1, H2);

    warpPerspective( input_images.front(), rectified[0], H1, img_size, INTER_LINEAR, BORDER_REPLICATE);
    warpPerspective( input_images.back() , rectified[1], H2, img_size, INTER_LINEAR, BORDER_REPLICATE);

    imwrite("tmp/img_00.png", rectified[0]);
    imwrite("tmp/img_01.png", rectified[1]);

  }

  // calc disparity
  Mat disp_map( rectified[0].size(), CV_32F );
  {
    cv::StereoSGBM stereo;
    stereo.minDisparity = 0.0;
    stereo.numberOfDisparities = 96;
    stereo.SADWindowSize = 11;
    stereo.uniquenessRatio = 12.0;
    stereo.P1 = 16 * 10 * 10;
    stereo.P2 = 64 * 10 * 10;
    stereo.preFilterCap = 20.0;
    stereo.disp12MaxDiff = 0;
    stereo( rectified[0], rectified[1], disp_map);
    disp_map /= 16.0;
  }


  double min, max;
  minMaxLoc( disp_map, &min, &max);
  cout << "min = " << min << endl;
  cout << "max = " << max << endl;
  
  Mat1b disp_1b(disp_map.size());
  disp_map.convertTo(disp_1b, CV_8UC1);
  imwrite("tmp/disp.png", disp_1b);

  return 0;
}

void bundle_adjustment( vector< vector<Point2f> > points , vector<Mat> &dst)
{
  
  // detail::BundleAdjusterReproj ba;
  // ba.estimate(features, pairewise_matches, &dst);
}
