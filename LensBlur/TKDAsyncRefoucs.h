//
//  TKDAsyncRefoucs.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/06.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <Foundation/Foundation.h>

@protocol TKDAsyncRefocusDelegate;

@interface TKDAsyncRefoucs : NSObject

@property (nonatomic, weak) id<TKDAsyncRefocusDelegate> delegate;

// inputs
@property (nonatomic, strong) UIImage *referenceImage;
@property (nonatomic, strong) UIImage *aperture;
@property (nonatomic, strong) UIImage *disparityMap;
@property (nonatomic, strong) NSArray *disparitySequence;
@property (nonatomic, assign) CGFloat apertureSize;

// output
@property (nonatomic, strong, readonly) UIImage *result;

- (void)refocusTo:(CGPoint)point;

@end

@protocol TKDAsyncRefocusDelegate <NSObject>
@required
- (void)asyncRefocus:(TKDAsyncRefoucs *)asyncRefocus refocusCompleted:(UIImage *)refocusedImage;
- (void)asyncRefocus:(TKDAsyncRefoucs *)asyncRefocus refocusFailed:(NSError *)error;
@end