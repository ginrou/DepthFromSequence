#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/stitcher.hpp>

using namespace std;
using namespace cv;

void factorization( vector< vector<Point2f> > points , vector<Mat> &camera_matrix, vector<Point3f> &pt_in_world );
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
    cv::goodFeaturesToTrack( input_images[0], points, MAX_CORNERS, 0.001, 10, noArray(), 5, true, 0.04);
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
  }

  // draw correspondences
  {
    Mat1b img( img_size.height, img_size.width*2);
    cout << img.size() << endl;
    input_images.front().copyTo( img(Rect(0,0,img_size.width, img_size.height)));
    input_images.back().copyTo( img(Rect(img_size.width,0,img_size.width, img_size.height)));

    cv::Scalar white( 255, 255, 255);

    std::vector<Point2f> points1, points2;
    for( int i = 0; i < total_status.size(); ++i ){
      if( !total_status[i] ) continue;

      points1.push_back( track_points.front()[i] );
      points2.push_back( track_points.back()[i] );

    }

    for( int i = 0; i < points1.size(); ++i ) {
      Point2f pt1 = points1[i];
      circle(img, pt1, 2.0, white, 1, 8, 0);

      Point2f pt2 = points2[i];
      pt2.x += img_size.width;
      circle(img, pt2, 2.0, white, 1, 8, 0);

      line(img, pt1, pt2, white);

    }

    cout << "num of lines : " << points1.size() << endl;      
    imwrite("tmp/matches.png", img);

  }

  // write to file
  {
    char filename[256];
    for( int i = 0; i < track_points.size(); ++i ){
      sprintf( filename, "data-%02d.txt", i );
      cout << filename << endl;

      FILE *fp = fopen(filename, "w");
      std::vector<Point2f> pt_list = track_points[i];
      fprintf(fp, "%d\n", pt_list.size());
      for( int j = 0; j < pt_list.size(); ++j ){
	fprintf(fp, "%lf %lf\n", pt_list[j].x, pt_list[j].y);
      }
      fclose(fp);

    }

  }

  for ( int d = 0; d < 30; ++d ) {
    Mat1d H = (Mat1d(3,3) << 1,0,d-50, 0,1,0, 0,0,1);
    Mat dst = input_images[0].clone();
    cv::warpPerspective( input_images[0], dst, H, dst.size(), INTER_LINEAR, BORDER_REPLICATE );
    char filename[256];
    sprintf(filename, "tmp/diff-%02d.png", d);
    imwrite(filename, dst-input_images[1]);
  }

  return 0;
}

void bundle_adjustment( vector< vector<Point2f> > points , vector<Mat> &dst)
{

}
