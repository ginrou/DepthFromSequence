#include "refocus.hpp"

cv::Mat3b Refocus::refocus_to(cv::Point2d focal_point) {

    // prepare kernels
    std::vector<double> blur_sizes = get_blur_size(focal_point);
    std::vector<Mat1d> kernels = get_kernels(blur_sizes);

    // make blur images for each depth
    std::vector<Mat3b> blur_images(kernels.size());
    for ( int d = 0; d < kernels.size(); ++ d ) {
        blur_images[d] = Mat3b(ref_image_.size());
        cv::filter2D(ref_image_, blur_images[d], -1, kernels[d]);
    }

    // pickup blur images
    Mat3b dst(ref_image_.size());
    for ( int h = 0; h < dst.rows; ++h ) {
        for ( int w = 0; w < dst.cols; ++w ) {
            int d = disparity_map_.at<uchar>(h,w);
            dst.at<Vec3b>(h,w) = blur_images[d].at<Vec3b>(h,w);
        }
    }

    return dst;
}

std::vector<double> Refocus::get_blur_size(cv::Point2d focal_point) {
    int depth_index = disparity_map_.at<uchar>(focal_point);
    double focal_depth = disparity_sequence_[depth_index];
    std::vector<double> blur_size(disparity_sequence_.size());
    for ( int i = 0; i < blur_size.size(); ++i ) {
        blur_size[i] = aperture_size_ * (focal_depth - disparity_sequence_[i]);
    }

    return blur_size;
}

std::vector<Mat1d> Refocus::get_kernels(std::vector<double> kernel_sizes) {
    Mat1d kernel(aperture_shape_.size());
    aperture_shape_.convertTo(kernel, CV_64FC1);

    std::vector<Mat1d> kernels(kernel_sizes.size());
    for ( int i = 0; i < kernels.size(); ++i ) {
        int size = MAX(1, fabs(kernel_sizes[i])); // cannot make (0, 0) mat
        kernels[i] = Mat1d(size, size);
        cv::resize(kernel, kernels[i], cv::Size(size, size));
        kernels[i] /= cv::norm(kernels[i], NORM_L1);
    }

    return kernels;
}
