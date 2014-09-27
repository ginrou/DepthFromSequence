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
static double const process_size_ratio = 0.5;

@interface TKDDepthEstimator ()
{
    std::vector<Mat3b> *captured_images;

    /// original reference image which is not corpped nor downsampled
    Mat3b *ref_mat;

    /// images are down sampled to this size
    cv::Size process_size;
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
        tracker->MAX_IMAGES = (int)_capturingImages;
        tracker->MIN_FEATURE_DISTANCE = 5.0;
        tracker->MIN_TRACK_DISTANCE = 10.0;
        ref_mat = NULL;

        _queue = dispatch_queue_create(kQueueName, DISPATCH_QUEUE_SERIAL);
    }
    return self;
}

- (void)dealloc
{
    delete captured_images;
    delete ref_mat;
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
        (*ref_mat)(roi).copyTo(mat);
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

- (void)setInputBufferSize:(CGSize)inputBufferSize
{
    _inputBufferSize = inputBufferSize;
    process_size = cv::Size(inputBufferSize.width * process_size_ratio,
                            inputBufferSize.height * process_size_ratio);
}

#pragma mark - private
/// returns processing size considerd rect
- (cv::Rect)processingROI
{
    double r = process_size.width / _inputBufferSize.width;
    cv::Rect roi = cvRectFromCGRect(self.roi);
    roi.x *= r; roi.y *= r;
    roi.width *= r; roi.height *= r;
    return roi;
}

/// bufferが来たらまずここに入れる
- (Mat3b)createProcessingMat:(Mat3b)mat
{
    Mat3b dst(process_size);
    cv::resize(mat, dst, process_size);
    return dst;
}

- (Mat)upsampleToOriginalSize:(Mat)mat
{
    Mat dst(cv::Size(_inputBufferSize.width, _inputBufferSize.height), mat.channels());
    cv::resize(mat, dst, dst.size());
    return dst;
}

- (Mat1b)toGray:(Mat3b)mat
{
    Mat1b dst(mat.size());
    cvtColor(mat, dst, CV_RGB2GRAY);
    return dst;
}

- (Mat1b)createProcessing1bMat:(Mat3b)mat
{
    return [self toGray:[self createProcessingMat:mat]];
}

- (Mat1b)createProcessing1bMatFromBuffer:(CMSampleBufferRef)sampleBuffer
{
    return [self createProcessing1bMat:sampleBufferToMat(sampleBuffer)];
}

- (Mat3b)createProcessingMatFromBuffer:(CMSampleBufferRef)sampleBuffer
{
    return [self createProcessingMat:sampleBufferToMat(sampleBuffer)];
}

- (void)setRefMat:(CMSampleBufferRef)sampleBuffer
{
    Mat3b img = sampleBufferToMat(sampleBuffer);
    ref_mat = new cv::Mat3b(img.size());
    img.copyTo(*ref_mat);
}

- (void)setResults:(PlaneSweep *)ps depths:(std::vector<double>)depths
{
    _rawDisparityMap = matToUIImage([self upsampleToOriginalSize:ps->_depth_raw], 1.0, UIImageOrientationRight);
    _smoothDisparityMap = matToUIImage([self upsampleToOriginalSize:ps->_depth_smooth], 1.0, UIImageOrientationRight);
    _colorDisparityMap = matToUIImage([self upsampleToOriginalSize:ps->_depth_color], 1.0, UIImageOrientationRight);
    [self setDepthSequence:depths];
}

#pragma mark - FeatureTracking
- (void)checkStability:(CMSampleBufferRef)sampleBuffer
{
    Mat1b gray = [self createProcessing1bMatFromBuffer:sampleBuffer];
    int features = tracker->good_features_to_track(gray);
    [self updateStabilityFromFeatures:features];
}

- (void)trackImage:(CMSampleBufferRef)sampleBuffer
{
    if (captured_images->size() >= self.capturingImages || self.stability < 0.7) {
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.captureDelegate depthEstimator:self didTrack:NO];
        });
    }

    Mat1b track_mat = [self createProcessing1bMatFromBuffer:sampleBuffer];
    bool added = tracker->add_image(track_mat);
    [self updateStabilityFromFeatures:tracker->count_track_points()];

    if (self.stability < 0.7) {
        dispatch_async(dispatch_get_main_queue(), ^{
            NSError *e = [NSError errorWithDomain:TKDDepthEstimatorErrorDomain code:TKDDepthEstimatorFewFeaturesError userInfo:nil];
            [self.captureDelegate depthEstimator:self trackingFailed:e];
        });
    } else if (added) {

        if (ref_mat == NULL) {
            [self setRefMat:sampleBuffer];
        }

        captured_images->push_back([self createProcessingMatFromBuffer:sampleBuffer]);

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

    if (solver.good_reporjection() == false && NO) {
        print_ittr_status(solver);
        NSError *e = [NSError errorWithDomain:TKDDepthEstimatorErrorDomain code:TKDDepthEstimatorBundleAdjustmentFailed userInfo:nil];
        [self notifyErrorOnMainQueue:e];
        return;
    }

    // plane sweep の準備
    std::vector<double> depths = solver.depth_variation((int)self.depthResolution);

    // plane sweep + densecrf で奥行きを求める
    // cv::Rect roi = cvRectFromCGRect(self.roi);
    cv::Rect roi = [self processingROI];
    PlaneSweep ps(*captured_images,
                  solver.camera_params,
                  depths,
                  roi);

    ps.set_callback(*progress_callback, (__bridge void *)self);

    ps.sweep(captured_images->front());

    // plane sweep 完了
    self.psProgress = 1.0;

    // complete
    [self setResults:&ps depths:depths];

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
