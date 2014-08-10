//
//  TKDDepthEstimator.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/10.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDDepthEstimator.h"
#import <opencv.hpp>
#include "depth_from_sequence.hpp"
#import <ios.h>

using namespace std;
using namespace cv;

@interface TKDDepthEstimator ()
{
    vector<Mat3b> *full_color_images;
}
@property (nonatomic, strong) dispatch_queue_t queue;
@end

@implementation TKDDepthEstimator
- (instancetype)init {
    self = [super init];
    if (self) {
        full_color_images = new vector<Mat3b>;
        _queue = dispatch_queue_create("DepthEstimationQueue", NULL);
    }
    return self;
}

- (void)dealloc {
    delete full_color_images;
}

+ (Mat3b)sampleBufferToMat:(CMSampleBufferRef)sampleBuffer {
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

    CVPixelBufferLockBaseAddress( pixelBuffer, 0 );

    //Processing here
    int bufferWidth = (int)CVPixelBufferGetWidth(pixelBuffer);
    int bufferHeight = (int)CVPixelBufferGetHeight(pixelBuffer);
    unsigned char *pixel = (unsigned char *)CVPixelBufferGetBaseAddress(pixelBuffer);

    // put buffer in open cv, no memory copied
    Mat4b mat = cv::Mat(bufferHeight,bufferWidth,CV_8UC4,pixel);
    Mat3b mat3c(bufferHeight, bufferWidth);
    cvtColor(mat, mat3c, CV_BGRA2BGR);

    cv::Rect rect(0,0, 480, 480);
    Mat3b cropped = mat3c(rect).clone();


    //End processing
    CVPixelBufferUnlockBaseAddress( pixelBuffer, 0 );

    return cropped;
}

- (void)addImage:(CMSampleBufferRef)sampleBuffer {

    if (full_color_images->size() >= 20) {
        NSLog(@"%lu do not add anymore", full_color_images->size());
        return;
    }

    Mat3b new_iamge = [[self class] sampleBufferToMat:sampleBuffer];
    dispatch_async(_queue, ^{
        full_color_images->push_back(new_iamge);
    });

}

- (void)runEstimation {
    dispatch_async(_queue, ^{
        [self runEstimationImpl];
    });
}

- (void)runEstimationImpl {
    vector<Mat> gray_images(full_color_images->size());
    for (int i = 0; i < gray_images.size(); ++i) {
        gray_images[i] = Mat1b((*full_color_images)[i].size());
        cvtColor((*full_color_images)[i], gray_images[i], CV_BGR2GRAY);
    }

    FeatureTracker tracker(gray_images);
    tracker.track();
    vector<vector<Point2d>> track_points = tracker.pickup_stable_points();

    NSLog(@"tracking complete");

    BundleAdjustment::Solver ba_solver(track_points);
    ba_solver.init_with_first_image(track_points, gray_images[0].size(), 7500, 55);

    while (ba_solver.should_continue) {
        ba_solver.run_one_step();
        NSLog(@"ba_solver : %d", ba_solver.ittr);
    }

    vector<double> depths;
    for (int i = 0; i < 20; ++i) depths.push_back(1000.0 - 40*i);

    PlaneSweep *ps = new PlaneSweep(gray_images, ba_solver.camera_params, depths);
    NSLog(@"sweeping");
    ps->sweep((*full_color_images)[0]);

    convertScaleAbs(ps->_depth_smooth, ps->_depth_smooth, 10);

    UIImage *img = MatToUIImage(ps->_depth_smooth);
    dispatch_async(dispatch_get_main_queue(), ^{
        NSLog(@"done %@", img);
        [self.delegate depthEstimator:img];
    });


}

@end
