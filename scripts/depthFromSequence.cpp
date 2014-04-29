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
    cv::goodFeaturesToTrack( input_images[0], points, MAX_CORNERS, 0.01, 10, noArray(), 5, true, 0.04);
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
  Mat fund_mat, rectified[2], H1, H2;
  {
    std::vector<Point2f> points1, points2;
    for( int i = 0; i < total_status.size(); ++i ){
      if( !total_status[i] ) continue;

      points1.push_back( track_points.front()[i] );
      points2.push_back( track_points.back()[i] );

    }

    std::vector<uchar> find_fund_mat_status;
    fund_mat = findFundamentalMat( points1, points2, find_fund_mat_status );

    // fundmat -> P = [R|t]
    Mat K;
    cv::FileStorage fs;
    fs.open("out_camera_data.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> K;

    Mat E = K.t() * fund_mat * K;

    cv::SVD svd(E, SVD::FULL_UV);

    Mat1d W = (Mat1d(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
    Mat1d R = svd.u * W.inv() * svd.vt;
    Mat1d t = svd.u.col(2);

    cout << K << endl;
    cout << fund_mat << endl;
    cout << E << endl;
    cout << R << endl;
    cout << t << endl;

    Mat distCoeffs(4,1,CV_64FC1,Scalar(0));
    Mat R1, P1, R2, P2, Q;
    stereoRectify( K, distCoeffs, K, distCoeffs, img_size, R, t, R1, R2, P1, P2, Q, 0, -1, img_size);

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(K, distCoeffs, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(K, distCoeffs, R2, P2, img_size, CV_16SC2, map21, map22);

    remap(input_images.front(), rectified[0], map11, map12, INTER_LINEAR);
    remap(input_images.back() , rectified[1], map21, map22, INTER_LINEAR);

    imwrite("tmp/img_00.png", rectified[0]);
    imwrite("tmp/img_01.png", rectified[1]);

    // draw correspondences
    {
      Mat1b img( img_size.height, img_size.width*2);
      cout << img.size() << endl;
      input_images.front().copyTo( img(Rect(0,0,img_size.width, img_size.height)));
      input_images.back().copyTo( img(Rect(img_size.width,0,img_size.width, img_size.height)));

      cv::Scalar white( 255, 255, 255);

      for( int i = 0; i < points1.size(); ++i ) {
	Point2f pt1 = points1[i];
	circle(img, pt1, 2.0, white, 1, 8, 0);

	Point2f pt2 = points2[i];
	pt2.x += img_size.width;
	circle(img, pt2, 2.0, white, 1, 8, 0);

	line(img, pt1, pt2, white);

      }
      
      imwrite("tmp/matches.png", img);

    }

    // draw epi line
    {
      Mat1b img( img_size.height, img_size.width*2);
      cout << img.size() << endl;
      input_images.front().copyTo( img(Rect(0,0,img_size.width, img_size.height)));
      input_images.back().copyTo( img(Rect(img_size.width,0,img_size.width, img_size.height)));

      cv::Scalar white( 255, 255, 255);
      
      std::vector<Vec3f> lines;
      computeCorrespondEpilines(points1, 1, fund_mat, lines);

      for( int i = 0; i < lines.size(); ++i ) {

	Point2f pt1 = points1[i];
	circle(img, pt1, 2.0, white, 1, 8, 0);

	Vec3f line = lines[i];
	
	Point2f st, ed;
	st.x = img_size.width;
	st.y = -line[2]/line[1];
	ed.x = img_size.width*2-1;;
	ed.y = -(line[0]*img_size.width+line[2])/line[1];
	cv::line(img, st, ed, white);

      }
      
      imwrite("tmp/match-line.png", img);

    }


  }
				   
  // calc disparity
  Mat disp_map( img_size, CV_32F );
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
    Mat warped_disp_map;
    warpPerspective( disp_map , warped_disp_map, H1.inv(), img_size, INTER_LINEAR, BORDER_REPLICATE);

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
