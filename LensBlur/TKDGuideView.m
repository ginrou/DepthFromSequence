//
//  TKDGuideView.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDGuideView.h"

@implementation TKDGuideView
+(instancetype)guideViewWithXibName:(NSString *)xibName {
    UINib *nib = [UINib nibWithNibName:xibName bundle:nil];
    return [nib instantiateWithOwner:nil options:nil][0];
}

- (void)showOnView:(UIView *)view removeAutomatically:(BOOL)autoRemove
{
    self.alpha = 0.0;
    self.center = view.center;

    [view addSubview:self];
    [UIView animateWithDuration:0.7 animations:^{
        self.alpha = 1.0;
    } completion:^(BOOL finished) {
        if (autoRemove) {
            [UIView animateWithDuration:0.7 delay:3.0 options:0 animations:^{
                self.alpha = 0.0;
            } completion:^(BOOL finished) {
                [self removeFromSuperview];
            }];
        }
    }];
}
@end
