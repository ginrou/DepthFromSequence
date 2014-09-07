//
//  TKDLensBlurEditViewController.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "TKDDepthEstimatorOld.h"

@interface TKDLensBlurEditViewController : UIViewController <TKDDepthEstimatorOldDelegate>
@property (nonatomic, strong) TKDDepthEstimatorOld *depthEstimator;
@end
