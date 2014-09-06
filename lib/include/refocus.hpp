#pragma once

#include "depth_from_sequence.hpp"

class Refocus {
public:
    cv::Mat3b ref_image_;
    cv::Mat1b aperture_shape_;
    cv::Mat1b disparity_map_;
    std::vector<double> disparity_sequence_;
    double aperture_size_;

    Refocus(cv::Mat3b ref_image,
            cv::Mat1b aperture_shape,
            cv::Mat1b disparity_map,
            std::vector<double> disparity_sequence,
            double aperture_size = 1.0)
        :ref_image_(ref_image),
         aperture_shape_(aperture_shape),
         disparity_map_(disparity_map),
         disparity_sequence_(disparity_sequence),
         aperture_size_(aperture_size)
        {}

    cv::Mat3b refocus_to(cv::Point2d focal_point);

    static cv::Mat1b circuler_aperture(int size) {
        cv::Mat1b img(size, size);
        img.setTo(0);
        cv::circle(img,
                   cv::Point2d(size/2, size/2), size/2,
                   cv::Scalar::all(255), -1);
        return img;
    }

private:
    std::vector<double> get_blur_size(cv::Point2d focal_point);
    std::vector<Mat1d> get_kernels(std::vector<double> kernel_sizes);
};
