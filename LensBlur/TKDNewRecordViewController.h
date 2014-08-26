//
//  TKDNewRecordViewController.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/24.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "TKDRecordDataSource.h"

@interface TKDNewRecordViewController : UITableViewController

// avaiable after estimation
@property (nonatomic, strong) TKDDepthEstimationRecord *createdRecord;
@end
