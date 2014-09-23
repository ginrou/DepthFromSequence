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

// progress counter
@property (nonatomic ,assign) CGFloat baProgress;
@property (nonatomic ,assign) CGFloat psProgress;
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
    _stability = MAX(1.0,(CGFloat)currentTrackedPoints/(CGFloat)tracker->MAX_CORNERS);
    dispatch_async(dispatch_get_main_queue(), ^{
        [self.captureDelegate depthEstimatorStabilityUpdated:self];
    });

}

- (NSInteger)capturedImages {
    return captured_images->size();
}

- (UIImage *)referenceImage {
    if (captured_images->size() == 0) {
        return nil;
    } else {
        cv::Rect roi = cvRectFromCGRect(self.roi);
        Mat3b mat(roi.size());
        captured_images->front()(roi).copyTo(mat);
        return matToUIImage(mat, 1.0, UIImageOrientationRight);
    }
}

- (void)setDepthSequence:(std::vector<double>)depth
{
    NSMutableArray *array = [NSMutableArray array];
    for (int i = 0; i < depth.size(); ++i) {
        [array addObject:@(depth[i])];
    }
    _depthSequence = [NSArray arrayWithArray:array];
}

- (BOOL)isComputed {
    return self.smoothDisparityMap != nil;
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

- (void)runEstimation
{
    if (self.isRunning) return;
    dispatch_async(self.queue, ^{
        [self runEstimationImpl];
    });
}

- (void)runEstimationImpl
{

    // feature trackingの結果を得る
    std::vector<std::vector<cv::Point2d>> track_points = tracker->pickup_stable_points();

    // Solverの初期化
    BundleAdjustment::Solver solver(track_points);
    double min_depth = 500.0; // [mm]
    double fov = 55.0; // [deg]
    cv::Size img_size = captured_images->front().size();
    double f = MIN(img_size.width, img_size.height)/2.0;
    solver.initialize(track_points, min_depth, fov, img_size, f);

    // bundle adjustment を実行
    while (solver.should_continue) {
        solver.run_one_step();
        self.baProgress = (CGFloat)(solver.ittr+1) / (CGFloat)solver.MAX_ITTR;
    }

    // bundle adjustment 終了
    self.baProgress = 1.0;
    print_params(solver);

    if (solver.good_reporjection() == false) {
        print_ittr_status(solver);
        NSError *e = [NSError errorWithDomain:TKDDepthEstimatorErrorDomain code:TKDDepthEstimatorBundleAdjustmentFailed userInfo:nil];
        [self notifyErrorOnMainQueue:e];
        return;
    }

    // plane sweep の準備
    std::vector<double> depths = solver.depth_variation((int)self.depthResolution);

    // plane sweep + densecrf で奥行きを求める
    cv::Rect roi = cvRectFromCGRect(self.roi);
    PlaneSweep ps(*captured_images,
                  solver.camera_params,
                  depths,
                  roi);

    ps.set_callback(*progress_callback, (__bridge void *)self);

    ps.sweep(captured_images->front());

    // plane sweep 完了
    self.psProgress = 1.0;

    // complete
    _rawDisparityMap = matToUIImage(ps._depth_raw, 1.0, UIImageOrientationRight);
    _smoothDisparityMap = matToUIImage(ps._depth_smooth, 1.0, UIImageOrientationRight);
    _colorDisparityMap = matToUIImage(ps._depth_color, 1.0, UIImageOrientationRight);
    [self setDepthSequence:depths];

    dispatch_async(dispatch_get_main_queue(), ^{
        [self.estimationDelegate depthEstimator:self
                            estimationCompleted:self.smoothDisparityMap];
    });

}

- (void)notifyErrorOnMainQueue:(NSError *)error
{
    dispatch_async(dispatch_get_main_queue(), ^{
        [self.estimationDelegate depthEstimator:self
                               estimationFailed:error];
    });
}


#pragma mark - progress handling

- (void)setBaProgress:(CGFloat)baProgress {
    if (_baProgress != baProgress) {
        _baProgress = baProgress;
        [self notifyProgressOnMainQueue];
    }
}

- (void)setPsProgress:(CGFloat)psProgress {
    if (_psProgress != psProgress) {
        _psProgress = psProgress;
        [self notifyProgressOnMainQueue];
    }
}

- (void)notifyProgressOnMainQueue
{
    CGFloat p = 0.3 * self.baProgress + 0.7 * self.psProgress;
    dispatch_async(dispatch_get_main_queue(), ^{
        NSString *status = self.baProgress < 1.0 ? @"bundleAdjustment" : @"planeSweep";
        [self.estimationDelegate depthEstimator:self estimationProceeded:p status:status];
    });
}

void progress_callback(void *obj, float progress) {
    TKDDepthEstimator *estimator = (__bridge TKDDepthEstimator *)obj;

    if ([estimator respondsToSelector:@selector(setPsProgress:)]) {
        [estimator setPsProgress:progress];
    }
}

@end
