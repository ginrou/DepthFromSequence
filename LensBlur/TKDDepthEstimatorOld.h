//
//  TKDDepthEstimatorOld.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/10.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

@protocol TKDDepthEstimatorOldDelegate;

@interface TKDDepthEstimatorOld : NSObject
@property (nonatomic, weak) id<TKDDepthEstimatorOldDelegate> delegate;
@property (nonatomic, assign) BOOL captureLog;

// outputs
@property (nonatomic, strong) NSMutableString *log;
@property (nonatomic, readonly) UIImage *referenceImage;
@property (nonatomic, strong) UIImage *rawDepthMap;
@property (nonatomic, strong) UIImage *smoothDepthMap;
@property (nonatomic, readonly) NSDictionary *computationLog;
@property (nonatomic, readonly) NSUInteger depthResolution;

- (instancetype)initWithImageSize:(CGSize)size roi:(CGRect)roi;

- (void)checkStability:(CMSampleBufferRef)sampleBuffer block:(void(^)(CGFloat stability))block;

- (void)addImage:(CMSampleBufferRef)sampleBuffer block:(void(^)(BOOL added, BOOL prepared))block;
- (NSInteger)numberOfRquiredImages;
- (NSArray *)convertToUIImages;

- (void)runEstimationOnSuccess:(void(^)(UIImage *depthMap))onSuccess
                    onProgress:(void(^)(CGFloat fraction))onProgress
                       onError:(void(^)(NSError *error))onError;


@end

@protocol TKDDepthEstimatorOldDelegate <NSObject>
@optional
- (void)depthEstimator:(TKDDepthEstimatorOld *)estimator getLog:(NSString *)newLine;
@end

FOUNDATION_EXTERN NSString *TKDDepthEstimatorOldErrorDomain;

typedef NS_ENUM(NSInteger, TKDDepthEstimatorOldErrorCode) {
    TKDDepthEstimatorOldInvalidInputError = 0,
    TKDDepthEstimatorOldFewFeaturesError,
    TKDDepthEstimatorOldBundleAdjustmentFailed,
    TKDDepthEstimatorOldAlredyRunning,
    TKDDepthEstimatorOldErrorCodeCount
};