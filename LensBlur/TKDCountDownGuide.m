//
//  TKDCountDownGuide.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDCountDownGuide.h"

@interface TKDCountDownGuide ()
@property (weak, nonatomic) IBOutlet UILabel *label;

@end

@implementation TKDCountDownGuide

- (void)awakeFromNib {
    [super awakeFromNib];
    self.layer.cornerRadius = 5.0;
}

- (void)setNumber:(NSInteger)num {
    self.label.text = [NSString stringWithFormat:@"%d", num];
}

@end
