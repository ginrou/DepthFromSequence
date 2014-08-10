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

- (void)addImage:(CMSampleBufferRef)sampleBuffer;
- (void)runEstimation;

@end

@protocol TKDDepthEstimatorDelegate <NSObject>
- (void)depthEstimator:(UIImage *)estimatedDepthMap;

@end