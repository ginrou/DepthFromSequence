#include "mock_image_factory.hpp"

Mat image_with_points(Size size, vector<Point2d> points) {
  Mat1b img(size.width, size.height);
  img.setTo(Scalar(0.0));
  
  for( int i = 0; i < points.size(); ++i ) {
    Point2d pt = points[i];
    if( pt.x < 0 || pt.x > size.width || pt.y < 0 || pt.y > size.height ) continue;
    img.at<uchar>((int)pt.y, (int)pt.x) = 255.0;
  }


  return img;
}
