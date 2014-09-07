//
//  TKDLensBlurEditViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDLensBlurEditViewController.h"
#import "TKDAsyncRefoucs.h"

@interface TKDLensBlurEditViewController () <TKDAsyncRefocusDelegate>
@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (weak, nonatomic) IBOutlet UINavigationBar *myNavigationBar;
@property (weak, nonatomic) IBOutlet UINavigationItem *navigationTitle;
@property (weak, nonatomic) IBOutlet UIButton *flipButton;
@property (weak, nonatomic) IBOutlet UISlider *apertureSizeSlider;

@property (assign, nonatomic) BOOL depthMapMode;

@property (weak, nonatomic) IBOutlet UILabel *label; //あとで消す

// refocus engine
@property (strong, nonatomic) TKDAsyncRefoucs *asyncRefocus;

@end

@implementation TKDLensBlurEditViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
}

- (void)viewDidAppear:(BOOL)animated {

    [super viewDidAppear:animated];

    self.imageView.image = [self.depthEstimator referenceImage];
    self.depthMapMode = NO;

    self.flipButton.enabled = NO;
    self.apertureSizeSlider.enabled = NO;

    self.depthEstimator.estimationDelegate = self;
    [self.depthEstimator runEstimation];
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
    [self.flipButton setImage:self.depthEstimator.smoothDisparityMap
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

    NSInteger depthCount = self.depthEstimator.depthResolution;
    NSMutableArray *array = [NSMutableArray arrayWithCapacity:depthCount];
    for (int i = 0; i < depthCount; ++i) [array addObject:@(i+1)];
    self.asyncRefocus.disparitySequence = array;

    self.asyncRefocus.delegate = self;
}

#pragma mark - User Interaction
- (IBAction)saveButtonTapped:(id)sender {
    self.view.userInteractionEnabled = NO;
    UIImageWriteToSavedPhotosAlbum(self.imageView.image, self, @selector(image:didFinishSavingWithError:contextInfo:), NULL);
}

- (IBAction)sliderValueChanged:(id)sender {
    NSLog(@"%f", self.apertureSizeSlider.value);
    self.asyncRefocus.apertureSize = 1.5 * self.apertureSizeSlider.value + 0.5;
}

- (IBAction)flipButtonTapped:(id)sender {
    if (self.depthMapMode) {
        self.depthMapMode = NO;
        self.apertureSizeSlider.enabled = YES;
        self.imageView.image = self.depthEstimator.referenceImage;
    } else {
        self.depthMapMode = YES;
        self.apertureSizeSlider.enabled = NO;
        self.imageView.image = self.depthEstimator.smoothDisparityMap;
    }
}

- (void)imageViewTapped:(UITapGestureRecognizer *)r
{
    NSLog(@"%@", r);

    if (self.depthMapMode) return;
    if (r.state != UIGestureRecognizerStateRecognized) return;

    CGPoint touchPoint = [r locationInView:self.imageView];
    NSLog(@"%@", NSStringFromCGPoint(touchPoint));
    [self.asyncRefocus refocusTo:touchPoint];
}

#pragma mark - DepthEstimater Estimation Delegate
- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationProceeded:(CGFloat)progress
{
    int persentage = 100.0 * progress;
    self.label.text = [NSString stringWithFormat:@"%d %%", persentage];
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationCompleted:(UIImage *)disparityMap
{
    [self setupViews];
    [self setupAsyncRefocus:disparityMap];
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationFailed:(NSError *)error
{
    NSString *message = [NSString stringWithFormat:@"code : %ld", (long)error.code];
    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"error" message:message delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil];
    [alert show];
}

#pragma mark - AsyncRefocus Delegate
- (void)asyncRefocus:(TKDAsyncRefoucs *)asyncRefocus refocusCompleted:(UIImage *)refocusedImage
{
    NSLog(@"done");
    self.imageView.image = refocusedImage;
}

- (void)asyncRefocus:(TKDAsyncRefoucs *)asyncRefocus refocusFailed:(NSError *)error
{
    NSLog(@"%@", error);
}

- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo
{
    
}

@end
