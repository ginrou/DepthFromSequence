//
//  TKDRecordViewerViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/26.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDRecordViewerViewController.h"

@interface TKDRecordViewerViewController ()
@property (weak, nonatomic) IBOutlet UIImageView *refernceImageView;
@property (weak, nonatomic) IBOutlet UIImageView *rawDepthMap;
@property (weak, nonatomic) IBOutlet UIImageView *smoothDepthMap;
@property (weak, nonatomic) IBOutlet UITextView *logTextView;

@end

@implementation TKDRecordViewerViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.clearsSelectionOnViewWillAppear = NO;

    self.title = self.record.key;
    self.refernceImageView.image = self.record.capturedImages.firstObject;
    self.rawDepthMap.image = self.record.rawDepthMap;
    self.smoothDepthMap.image = self.record.smoothDepthMap;
    self.logTextView.text = self.record.log;

}


/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}
*/

@end
