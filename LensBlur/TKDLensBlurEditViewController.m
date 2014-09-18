//
//  TKDLensBlurEditViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <SVProgressHUD.h>

#import "TKDLensBlurEditViewController.h"
#import "TKDAsyncRefoucs.h"

@interface TKDLensBlurEditViewController () <TKDAsyncRefocusDelegate>
@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (weak, nonatomic) IBOutlet UINavigationBar *myNavigationBar;
@property (weak, nonatomic) IBOutlet UINavigationItem *navigationTitle;
@property (weak, nonatomic) IBOutlet UIButton *flipButton;
@property (weak, nonatomic) IBOutlet UISlider *apertureSizeSlider;

@property (assign, nonatomic) BOOL depthMapMode;
@property (assign, nonatomic) BOOL isComputing;

@property (assign, nonatomic) BOOL hasLastTouchPoint;
@property (assign, nonatomic) CGPoint lastTouchPoint;

// refocus engine
@property (strong, nonatomic) TKDAsyncRefoucs *asyncRefocus;

@end

@implementation TKDLensBlurEditViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.hasLastTouchPoint = NO;
}

- (void)viewDidAppear:(BOOL)animated {

    [super viewDidAppear:animated];

    self.imageView.image = [self.depthEstimator referenceImage];
    self.depthMapMode = NO;

    self.flipButton.enabled = NO;
    self.apertureSizeSlider.enabled = NO;

    if (self.depthEstimator.isComputed == NO) {
        [SVProgressHUD showProgress:0.0 status:@"計算開始"];
        self.isComputing = YES;
        self.depthEstimator.estimationDelegate = self;
        [self.depthEstimator runEstimation];
    }

}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
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

- (void)setupViews {
    self.imageView.image = self.depthEstimator.referenceImage;
    [self.flipButton setImage:self.depthEstimator.colorDisparityMap
                     forState:UIControlStateNormal];
    self.apertureSizeSlider.enabled = YES;
    self.flipButton.enabled = YES;

    // imageview にtapGestureRecognizerを入れる
    UITapGestureRecognizer *r = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(imageViewTapped:)];
    self.imageView.userInteractionEnabled = YES;
    [self.imageView addGestureRecognizer:r];
}

- (void)setupAsyncRefocus:(UIImage *)dispMap {
    self.asyncRefocus = [TKDAsyncRefoucs new];
    self.asyncRefocus.referenceImage = self.depthEstimator.referenceImage;
    self.asyncRefocus.disparityMap = dispMap;
    self.asyncRefocus.depthSequence = self.depthEstimator.depthSequence;
    self.asyncRefocus.delegate = self;
}

#pragma mark - User Interaction
- (IBAction)saveButtonTapped:(id)sender {
    if (self.isComputing) return;

    self.view.userInteractionEnabled = NO;
    UIImageWriteToSavedPhotosAlbum(self.imageView.image, self, @selector(image:didFinishSavingWithError:contextInfo:), NULL);
}

- (IBAction)sliderValueChanged:(id)sender {
    if (self.isComputing) return;

    NSLog(@"%f", self.apertureSizeSlider.value);
    self.asyncRefocus.apertureSize = 1.5 * self.apertureSizeSlider.value + 0.5;
    if (self.hasLastTouchPoint) {
        [self.asyncRefocus refocusTo:self.lastTouchPoint];
        [SVProgressHUD showWithStatus:@"リフォーカス中"];
    }

}

- (IBAction)flipButtonTapped:(id)sender {
    if (self.depthMapMode) {
        self.depthMapMode = NO;
        self.apertureSizeSlider.enabled = YES;
        self.imageView.image = self.depthEstimator.referenceImage;
        [self.flipButton setImage:self.depthEstimator.colorDisparityMap
                         forState:UIControlStateNormal];
    } else {
        self.depthMapMode = YES;
        self.apertureSizeSlider.enabled = NO;
        self.imageView.image = self.depthEstimator.colorDisparityMap;
        [self.flipButton setImage:self.depthEstimator.referenceImage
                         forState:UIControlStateNormal];
    }
}

- (void)imageViewTapped:(UITapGestureRecognizer *)r
{
    if (self.isComputing) return;
    if (self.depthMapMode) return;
    if (r.state != UIGestureRecognizerStateRecognized) return;

    CGPoint touchPoint = [r locationInView:self.imageView];
    self.lastTouchPoint = touchPoint;
    CGFloat sx = self.imageView.image.size.width / self.imageView.frame.size.width;
    CGFloat sy = self.imageView.image.size.height / self.imageView.frame.size.height;
    NSLog(@"%f, %f", sx, sy);
    [self.asyncRefocus refocusTo:CGPointMake(sx * touchPoint.x, sy * touchPoint.y)];

    UIImageView *imgView = [[UIImageView alloc] initWithImage:[UIImage imageNamed:@"focus"]];
    imgView.center = touchPoint;
    [self.imageView addSubview:imgView];

    [UIView animateWithDuration:0.1 delay:0.6 options:0 animations:^{
        imgView.alpha = 0.f;
    } completion:^(BOOL finished) {
        [imgView removeFromSuperview];
        [SVProgressHUD showWithStatus:@"リフォーカス中"];
    }];

}

#pragma mark - DepthEstimater Estimation Delegate
- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationProceeded:(CGFloat)progress status:(NSString *)status
{
    NSString *displayStatus = @"計算中";
    if ([status isEqualToString:@"bundleAdjustment"]) displayStatus = @"カメラの位置の計算中";
    else if ([status isEqualToString:@"planeSweep"]) displayStatus = @"シーンの奥行きを計算中";

    [SVProgressHUD showProgress:progress status:displayStatus];
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationCompleted:(UIImage *)disparityMap
{
    self.isComputing = NO;
    [SVProgressHUD dismiss];
    [self setupViews];
    [self setupAsyncRefocus:disparityMap];
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationFailed:(NSError *)error
{
    NSString *message;
    if (error.code == TKDDepthEstimatorBundleAdjustmentFailed) {
        message = @"カメラの位置を正しく推定できませんでした。";
    } else {
        message = @"シーンの奥行きを推定できませんでした。";
    }

    [SVProgressHUD showErrorWithStatus:message];
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.5 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
        [self performSegueWithIdentifier:@"editCompletedSegue" sender:self];
    });
}

#pragma mark - AsyncRefocus Delegate
- (void)asyncRefocus:(TKDAsyncRefoucs *)asyncRefocus refocusCompleted:(UIImage *)refocusedImage
{
    NSLog(@"done");
    self.imageView.image = refocusedImage;
    [SVProgressHUD dismiss];
}

- (void)asyncRefocus:(TKDAsyncRefoucs *)asyncRefocus refocusFailed:(NSError *)error
{
    NSLog(@"%@", error);
}

- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo
{
    if (error) {
        [SVProgressHUD showErrorWithStatus:error.localizedFailureReason];
    } else {
        [self performSegueWithIdentifier:@"editCompletedSegue" sender:self];
    }
}

@end
