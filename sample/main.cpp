#include "depth_from_sequence.hpp"
#include "refocus.hpp"

int main(int argc, char* argv[]) {

    // load images
    vector<Mat3b> input_images;
    for( int i = 1; i < argc; ++i ) {
        input_images.push_back( imread(argv[i],  CV_LOAD_IMAGE_COLOR) );
    }

    // setup ROI
    cv::Rect roi;
    cv::Size img_size = input_images.front().size();
    if ( img_size.width > img_size.height ) {
        roi = Rect( (img_size.width-img_size.height)/2, 0, img_size.height, img_size.height);
    } else {
        roi = Rect( 0, (img_size.height-img_size.width)/2, img_size.height, img_size.height);
    }

    DepthFromSequence dfs(input_images, roi);

    // run depth estimation
    DepthFromSequence::EstimationStatus result = dfs.estimate();

    switch(result) {
    case DepthFromSequence::Success:
        std::cout << "depth estimation has succeeded" << std::endl;
        imwrite("depth_smooth.png", 8 * dfs._depth_smooth);
        imwrite("depth_raw.png", 8 * dfs._depth_raw);
        imwrite("depth_color.png", dfs._depth_color);
        break;
    case DepthFromSequence::FeatureTrackingFailed:
        std::cout << "feature tracking has failed" << std::endl;
        break;
    case DepthFromSequence::BundleAdjustmentFailed:
        std::cout << "bundle adjsutmetn has failed" << std::endl;
        break;

    }

    return 0;

}
