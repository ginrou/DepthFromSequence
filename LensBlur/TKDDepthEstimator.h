//
//  TKDDepthEstimator.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/10.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

@protocol TKDDepthEstimatorDelegate;

@interface TKDDepthEstimator : NSObject
@property (nonatomic, weak) id<TKDDepthEstimatorDelegate> delegate;
@property (nonatomic, assign) BOOL captureLog;

// outputs
@property (nonatomic, strong) NSMutableString *log;
@property (nonatomic, strong) UIImage *rawDepthMap;
@property (nonatomic, strong) UIImage *smoothDepthMap;
@property (nonatomic, readonly) NSDictionary *computationLog;

- (void)checkStability:(CMSampleBufferRef)sampleBuffer block:(void(^)(CGFloat stability))block;

- (void)addImage:(CMSampleBufferRef)sampleBuffer block:(void(^)(BOOL added, BOOL prepared))block;
- (NSInteger)numberOfRquiredImages;
- (NSArray *)convertToUIImages;

- (NSProgress *)runEstimationOnSuccess:(void(^)(UIImage *depthMap))onSuccess
                               onError:(void(^)(NSError *error))onError;


@end

@protocol TKDDepthEstimatorDelegate <NSObject>
@optional
- (void)depthEstimator:(TKDDepthEstimator *)estimator getLog:(NSString *)newLine;
@end

FOUNDATION_EXTERN NSString *TKDDepthEstimatorErrorDomain;

typedef NS_ENUM(NSInteger, TKDDepthEstimatorErrorCode) {
    TKDDepthEstimatorInvalidInputError = 0,
    TKDDepthEstimatorFewFeaturesError,
    TKDDepthEstimatorBundleAdjustmentFailed,
    TKDDepthEstimatorErrorCodeCount
};