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

  vector<Mat> pano;
  Stitcher stitcher = Stitcher::createDefault();
  Stitcher::Status status = stitcher.estimateTransform(input_images);

  if ( status != Stitcher::OK ) {
    cout << "stitch failed " << status << ";;" << endl;
    return -1;
  }

  for( int i = 0; i < stitcher.cameras_.size(); ++i ) {
    detail::CameraParams cam = stitcher.cameras_[i];
    cout << i << endl;
    cout << " f = " << cam.focal << ", aspect = " << cam.aspect << endl;
    cout << cam.R << endl; 
    cout << cam.t << endl;
    cout << cam.K() << endl;
    cout << cam.R.type() << "," << cam.K().type() << endl;
  }

  detail::CameraParams cam1 = stitcher.cameras_.front();
  detail::CameraParams cam2 = stitcher.cameras_.back();
  Mat1d R, T, R1, P1, R2, P2, Q;
  Mat1d distCoeffs(4,1);
  distCoeffs.setTo(0);
  R = cam1.R.inv() *cam2.R;
  T = cam1.t;
  cout << " R = " << R << endl;
  cout << " T = " << T << endl;
  cout << " R = " << R.type() << endl;
  cout << " T = " << T .type()<< endl;


  stereoRectify( cam1.K(), distCoeffs, cam2.K(), distCoeffs, img_size, 
		 R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size);
  cout << R1 << endl;
  cout << R2 << endl;

  Mat map11, map12, map21, map22;
  initUndistortRectifyMap(cam1.K(), distCoeffs, R1, P1, img_size, CV_16SC2, map11, map12);
  initUndistortRectifyMap(cam2.K(), distCoeffs, R2, P2, img_size, CV_16SC2, map21, map22);

  Mat img1r, img2r;
  remap(input_images.front(), img1r, map11, map12, INTER_LINEAR);
  remap(input_images.back() , img2r, map21, map22, INTER_LINEAR);

  imwrite("tmp/img_00.png", img1r);
  imwrite("tmp/img_01.png", img2r);

  {
    Mat disp_map( img1r.size(), CV_32F );
    cv::StereoBM stereo;
    stereo( img1r, img2r, disp_map);
    disp_map /= 16.0;
    Mat1b disp_1b(disp_map.size());
    disp_map.convertTo(disp_1b, CV_8UC1);
    imwrite("tmp/disp.png", disp_1b);
  }


  return 0;

  // initialize trackers
  {
    vector<Point2f> points;
    cv::goodFeaturesToTrack( input_images[0], points, MAX_CORNERS, 0.01, 10);
    cv::cornerSubPix(input_images[0], points, sub_pix_win_size, cv::Size(-1,-1), term_crit );
    track_points.push_back(points);
    img_size = input_images[0].size();
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

    std::vector<Mat> camera_mats;
    bundle_adjustment( track_points, camera_mats);
    imwrite("tmp/img_00.png", rectified[0]);
    imwrite("tmp/img_01.png", rectified[1]);

  }

  // calc disparity
  Mat disp_map( rectified[0].size(), CV_32F );
  {
    cv::StereoSGBM stereo;
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
