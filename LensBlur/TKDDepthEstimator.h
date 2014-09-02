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
@property (nonatomic, strong) NSMutableString *log;
@property (nonatomic, strong) UIImage *rawDepthMap;
@property (nonatomic, strong) UIImage *smoothDepthMap;

- (void)checkStability:(CMSampleBufferRef)sampleBuffer;
- (void)addImage:(CMSampleBufferRef)sampleBuffer;
- (void)runEstimation;
- (NSArray *)convertToUIImages;

@end

@protocol TKDDepthEstimatorDelegate <NSObject>
- (void)depthEstimator:(UIImage *)estimatedDepthMap;
- (void)depthEstimator:(TKDDepthEstimator *)estimator stabilityUpdated:(CGFloat)stability;
- (void)depthEstimator:(TKDDepthEstimator *)estimator statusUpdated:(NSString *)status;
- (void)depthEstimator:(TKDDepthEstimator *)estimator getLog:(NSString *)newLine;
- (void)depthEstimatorImagesPrepared:(TKDDepthEstimator *)estimator;
- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationCompleted:(UIImage *)smoothDepthMap;
@end