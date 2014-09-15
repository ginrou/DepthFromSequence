//
//  TKDDepthEstimator.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/07.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

@protocol TKDDepthEstimatorCaptureDelegate;
@protocol TKDDepthEstimatorEstimationDelegate;

@interface TKDDepthEstimator : NSObject

/*
 Capture And Feature Tracking
 */
// input
@property (nonatomic, weak) id<TKDDepthEstimatorCaptureDelegate> captureDelegate;
@property (nonatomic, assign) NSInteger capturingImages;

// output
/* stability of feature tracking
 0 ~ 0.7 : unstable
 0.7 ~ 0.85 : not stable but may be success to bundle adjustment
 0.85 ~ 1.0 : stable
 */
@property (nonatomic, readonly) CGFloat stability;

// current captured images
@property (nonatomic, readonly) NSInteger capturedImages;

- (void)checkStability:(CMSampleBufferRef)sampleBuffer;
- (void)trackImage:(CMSampleBufferRef)sampleBuffer;

/*
 Bundle Adjustment and Plane Sweep
 */
// input
@property (nonatomic, weak) id<TKDDepthEstimatorEstimationDelegate> estimationDelegate;
@property (nonatomic, assign) NSInteger depthResolution;
@property (nonatomic, assign) CGRect roi;

// output
@property (nonatomic, readonly) UIImage *referenceImage;
@property (nonatomic, readonly) UIImage *rawDisparityMap;
@property (nonatomic, readonly) UIImage *smoothDisparityMap;
@property (nonatomic, readonly) UIImage *colorDisparityMap;

// property
@property (nonatomic, readonly) BOOL isComputed;
@property (atomic, assign) BOOL isRunning;


- (void)runEstimation;

@end

@protocol TKDDepthEstimatorCaptureDelegate <NSObject>
- (void)depthEstimatorStabilityUpdated:(TKDDepthEstimator *)estimator;
- (void)depthEstimator:(TKDDepthEstimator *)estimator didTrack:(BOOL)added;
- (void)depthEstimator:(TKDDepthEstimator *)estimator trackingFailed:(NSError *)error;
- (void)depthEstimatorTrackingCompleted:(TKDDepthEstimator *)estimator;

@end

@protocol TKDDepthEstimatorEstimationDelegate <NSObject>
- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationProceeded:(CGFloat)progress status:(NSString *)status;
- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationCompleted:(UIImage *)disparityMap;
- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationFailed:(NSError *)error;
@end

FOUNDATION_EXTERN NSString *TKDDepthEstimatorErrorDomain;

typedef NS_ENUM(NSInteger, TKDDepthEstimatorErrorCode) {
    TKDDepthEstimatorInvalidInputError = 0,
    TKDDepthEstimatorFewFeaturesError,
    TKDDepthEstimatorBundleAdjustmentFailed,
    TKDDepthEstimatorAlredyRunning,
    TKDDepthEstimatorErrorCodeCount
};

