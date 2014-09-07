//
//  TKDDepthEstimator.mm
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/07.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDDepthEstimator.h"
#import <opencv.hpp>

#import "TKDImageConverter.h"

#import "depth_from_sequence.hpp"

inline cv::Rect cvRectFromCGRect(CGRect r) {
    return cv::Rect(r.origin.x, r.origin.y, r.size.width, r.size.height);
}

NSString *TKDDepthEstimatorErrorDomain = @"tkd.depthEstimator.error";
static const char kQueueName[] = "TKDDepthEstimator#Queue";

static int const kDefaultCapturingImages = 12;
static int const kDefaultDepthResolution = 32;

@interface TKDDepthEstimator ()
{
    std::vector<Mat3b> *captured_images;
    FeatureTracker *tracker;
}

@property (nonatomic, strong) dispatch_queue_t queue;

@end


@implementation TKDDepthEstimator

- (instancetype)init
{
    self = [super init];
    if (self) {
        _capturingImages = kDefaultCapturingImages;
        _depthResolution = kDefaultDepthResolution;

        captured_images = new std::vector<cv::Mat3b>;
        tracker = new FeatureTracker;
        tracker->MAX_IMAGES = _capturingImages;

        _queue = dispatch_queue_create(kQueueName, DISPATCH_QUEUE_SERIAL);
    }
    return self;
}

- (void)dealloc
{
    delete captured_images;
    delete tracker;
}

- (void)updateStabilityFromFeatures:(NSInteger)currentTrackedPoints
{
    _stability = (CGFloat)currentTrackedPoints/(CGFloat)tracker->MAX_CORNERS;
    dispatch_async(dispatch_get_main_queue(), ^{
        [self.captureDelegate depthEstimatorStabilityUpdated:self];
    });

}

- (NSInteger)capturedImages {
    return captured_images->size();
}

#pragma mark - FeatureTracking
- (void)checkStability:(CMSampleBufferRef)sampleBuffer
{
    Mat3b img = sampleBufferToMat(sampleBuffer);
    Mat1b gray(img.size());
    cvtColor(img, gray, CV_RGB2GRAY);
    int features = tracker->good_features_to_track(gray);
    [self updateStabilityFromFeatures:features];

}

- (void)trackImage:(CMSampleBufferRef)sampleBuffer
{
    Mat3b new_img = sampleBufferToMat(sampleBuffer);

    if (captured_images->size() >= self.capturingImages || self.stability < 0.7) {
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.captureDelegate depthEstimator:self didTrack:NO];
        });
    }

    Mat1b gray(new_img.size());
    cvtColor(new_img, gray, CV_RGB2GRAY);

    bool added = tracker->add_image(gray);
    [self updateStabilityFromFeatures:tracker->count_track_points()];

    if (self.stability < 0.7) {
        dispatch_async(dispatch_get_main_queue(), ^{
            NSError *e = [NSError errorWithDomain:TKDDepthEstimatorErrorDomain code:TKDDepthEstimatorFewFeaturesError userInfo:nil];
            [self.captureDelegate depthEstimator:self trackingFailed:e];
        });
    } else if (added) {
        captured_images->push_back(new_img);

        dispatch_async(dispatch_get_main_queue(), ^{
            [self.captureDelegate depthEstimator:self didTrack:YES];
        });

        if (captured_images->size() >= self.capturingImages) {
            dispatch_async(dispatch_get_main_queue(), ^{
                [self.captureDelegate depthEstimatorTrackingCompleted:self];
            });
        }
    }

}


@end
