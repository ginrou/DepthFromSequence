//
//  TKDLensBlurEditViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDLensBlurEditViewController.h"

@interface TKDLensBlurEditViewController ()
@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (weak, nonatomic) IBOutlet UINavigationBar *myNavigationBar;
@property (weak, nonatomic) IBOutlet UINavigationItem *navigationTitle;
@property (weak, nonatomic) IBOutlet UIButton *flipButton;
@property (weak, nonatomic) IBOutlet UISlider *apertureSizeSlider;

@property (assign, nonatomic) BOOL depthMapMode;

@property (weak, nonatomic) IBOutlet UILabel *label; //あとで消す

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

    __weak typeof(self) weakSelf = self;
    [self.depthEstimator runEstimationOnSuccess:^(UIImage *depthMap) {
        [weakSelf setupViews];
    } onProgress:^(CGFloat fraction) {
        int persentage = 100.0 * fraction;
        weakSelf.label.text = [NSString stringWithFormat:@"%d %%", persentage];
    } onError:^(NSError *error) {
        NSString *message = [NSString stringWithFormat:@"code : %d", error.code];
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"error" message:message delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alert show];
    }];

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
    [self.flipButton setImage:self.depthEstimator.smoothDepthMap forState:UIControlStateNormal];
    self.apertureSizeSlider.enabled = YES;
    self.flipButton.enabled = YES;

    // imageview にtapGestureRecognizerを入れる
}


#pragma mark - User Interaction
- (IBAction)saveButtonTapped:(id)sender {
}

- (IBAction)sliderValueChanged:(id)sender {
}

- (IBAction)flipButtonTapped:(id)sender {
    if (self.depthMapMode) {
        self.depthMapMode = NO;
        self.apertureSizeSlider.enabled = YES;
        self.imageView.image = self.depthEstimator.referenceImage;
    } else {
        self.depthMapMode = YES;
        self.apertureSizeSlider.enabled = NO;
        self.imageView.image = self.depthEstimator.smoothDepthMap;
    }
}



@end
