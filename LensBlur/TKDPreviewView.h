//
//  TKDPreviewView.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/10.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <UIKit/UIKit.h>
@import AVFoundation;

@interface TKDPreviewView : UIView
@property (nonatomic, strong) AVCaptureSession* session;
- (void)setVideoGravity:(NSString *)gravity;
@end
