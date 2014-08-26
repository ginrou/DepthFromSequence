//
//  TKDRecordViewerViewController.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/26.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "TKDRecordDataSource.h"

@interface TKDRecordViewerViewController : UITableViewController
@property (nonatomic, strong) TKDDepthEstimationRecord *record;
@end
