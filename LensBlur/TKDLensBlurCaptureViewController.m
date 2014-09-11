//
//  TKDLensBlurCaptureViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

@import AVFoundation;

#import "TKDLensBlurCaptureViewController.h"
#import "TKDLensBlurEditViewController.h"
#import "TKDDepthEstimator.h"
#import "TKDHowToUseGuide.h"
#import "TKDCountDownGuide.h"

@interface TKDLensBlurCaptureViewController () <
AVCaptureVideoDataOutputSampleBufferDelegate,
TKDDepthEstimatorCaptureDelegate
>
@property (weak, nonatomic) IBOutlet UILabel *stabilityLabel;
@property (weak, nonatomic) IBOutlet UIView *stabilityIcon;
@property (weak, nonatomic) IBOutlet UIButton *captureButton;
@property (weak, nonatomic) IBOutlet TKDPreviewView *previewView;

// Guide
@property (weak, nonatomic) TKDGuideView *guide;

// AVCaptures
@property (assign, nonatomic) BOOL readyToCapture;
@property (nonatomic, strong) AVCaptureSession *session;
@property (nonatomic, strong) dispatch_queue_t sessionQueue;
@property (nonatomic, strong) dispatch_queue_t videoDataQueue;
@property (atomic, assign) BOOL trackingMode;

// depth estimator
@property (nonatomic, assign) int round;
@property (nonatomic, strong) TKDDepthEstimator *depthEstimator;

@end

static const char * sessionQueueLabel = "TKDLensBlurCaptureViewController#sessionQueue";
static const char * videoDataQueueLabel = "TKDLensBlurCaptureViewController#videoDataQueue";
static const CGFloat kNotReadyStability = -1;
static const CGSize kImageSize = {1280, 720};
static const CGRect kROI = {{320, 40}, {640, 640}};

@implementation TKDLensBlurCaptureViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.stabilityIcon.layer.cornerRadius = self.stabilityIcon.frame.size.width/2.0;
    self.captureButton.enabled = NO;
    [self setupAVCapture];
    self.depthEstimator = [TKDDepthEstimator new];
    self.depthEstimator.captureDelegate = self;
//    self.depthEstimator.roi = kROI;
    self.depthEstimator.roi = CGRectMake(320, 40, 640, 640);

}

- (void)viewDidAppear:(BOOL)animated {
    [super viewDidAppear:animated];
    dispatch_async(self.sessionQueue, ^{
        if (self.session.isRunning == NO) {
            [self.session startRunning];
        }
    });
    self.guide = [TKDGuideView guideViewWithXibName:@"TKDHowToUseGuide"];
    [self.guide showOnView:self.view removeAutomatically:YES];

}

- (void)viewWillDisappear:(BOOL)animated {
    [super viewWillDisappear:animated];
    dispatch_async(self.sessionQueue, ^{
        if (self.session.isRunning) {
            [self.session stopRunning];
        }
    });
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)setReadyToCapture:(BOOL)readyToCapture {
    if (_readyToCapture != readyToCapture) {
        [self willChangeValueForKey:@"readyToCapture"];
        _readyToCapture = readyToCapture;
        [self didChangeValueForKey:@"readyToCapture"];

        self.captureButton.enabled = readyToCapture;
    }
}

- (void)setStability:(CGFloat)stability {
    if (stability == kNotReadyStability) {
        self.stabilityLabel.text = @"Preparing...";
        self.stabilityIcon.backgroundColor = [UIColor grayColor];
    } else {

        self.stabilityLabel.text = [NSString stringWithFormat:@"stability : %d%%", (int)(100.0*stability)];

        if (stability < 0.7) self.stabilityIcon.backgroundColor = [UIColor redColor];
        else if (stability < 0.85) self.stabilityIcon.backgroundColor = [UIColor yellowColor];
        else self.stabilityIcon.backgroundColor = [UIColor greenColor];

        self.readyToCapture = stability > 0.5;

    }
}

- (void)updateGuideView {
    NSInteger n = self.depthEstimator.capturingImages - self.depthEstimator.capturedImages;
    [(TKDCountDownGuide *)self.guide setNumber:n];
}

#pragma mark - User Interactions
- (IBAction)captureButtonTouchDown:(id)sender {

    dispatch_async(self.sessionQueue, ^{

        // lock focus, exposure, whitebalanace
        [self.session beginConfiguration];
        AVCaptureDevice *device = [[self class] defaultDevice];
        [device lockForConfiguration:nil];
        device.whiteBalanceMode = AVCaptureWhiteBalanceModeLocked;
        device.exposureMode = AVCaptureExposureModeLocked;
        device.focusMode = AVCaptureFocusModeLocked;
        [device unlockForConfiguration];
        [self.session commitConfiguration];

        // start tracking
        dispatch_async(self.videoDataQueue, ^{
            self.trackingMode = YES;
        });
    });

    self.guide = [TKDGuideView guideViewWithXibName:@"TKDCountDownGuide"];
    [self.guide showOnView:self.view removeAutomatically:NO];
    [self updateGuideView];
}

- (IBAction)captureButtonTouchUp:(id)sender {

    dispatch_async(self.sessionQueue, ^{

        // unlock focus, exposure, whitebalanace
        [self.session beginConfiguration];
        AVCaptureDevice *device = [[self class] defaultDevice];
        [device lockForConfiguration:nil];
        device.whiteBalanceMode = AVCaptureWhiteBalanceModeAutoWhiteBalance;
        device.exposureMode = AVCaptureExposureModeAutoExpose;
        device.focusMode = AVCaptureFocusModeAutoFocus;
        [device unlockForConfiguration];
        [self.session commitConfiguration];

        dispatch_async(self.videoDataQueue, ^{
            // stop tracking
            self.trackingMode = NO;

            // reset depth estimator
            self.depthEstimator = [TKDDepthEstimator new];
            self.depthEstimator.captureDelegate = self;
            self.depthEstimator.roi = kROI;
            self.depthEstimator.roi = CGRectMake(320, 40, 640, 640);
        });
    });

    [self.guide removeFromSuperview];
}


#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    if ([segue.identifier isEqualToString:@"showEditViewController"]) {
        TKDLensBlurEditViewController *vc = segue.destinationViewController;
        vc.depthEstimator = self.depthEstimator;
    }

}

- (IBAction)editCompletedSegue:(UIStoryboardSegue *)segue {

}

#pragma mark - AVFoundation and About DepthEstimator

- (void)setupAVCapture {
    [self setStability:kNotReadyStability];

    self.session = [AVCaptureSession new];
    if ([self.session canSetSessionPreset:AVCaptureSessionPreset1280x720]) {
        [self.session setSessionPreset:AVCaptureSessionPreset1280x720];
    } else {
        // TODO: 無理だったときのハンドリング
        NSLog(@"OMG!!!!");
    }
    self.previewView.session = self.session;
    self.sessionQueue = dispatch_queue_create(sessionQueueLabel, DISPATCH_QUEUE_SERIAL);
    dispatch_async(self.sessionQueue, ^{

        NSError *error = nil;

        AVCaptureDevice *device = [[self class] defaultDevice];
        AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];

        // TODO: 無理だったときのハンドリング
        if (error) NSLog(@"%@", error);

        if ([self.session canAddInput:input]) {
            [self.session addInput:input];
            dispatch_async(dispatch_get_main_queue(), ^{
                [[(AVCaptureVideoPreviewLayer *)[[self previewView] layer] connection] setVideoOrientation:(AVCaptureVideoOrientation)[self interfaceOrientation]];
            });
        }

        AVCaptureVideoDataOutput *output = [AVCaptureVideoDataOutput new];
        output.videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey: @(kCMPixelFormat_32BGRA)};
        output.alwaysDiscardsLateVideoFrames = YES;

        self.videoDataQueue = dispatch_queue_create(videoDataQueueLabel, DISPATCH_QUEUE_SERIAL);

        [output setSampleBufferDelegate:self queue:self.videoDataQueue];
        [[output connectionWithMediaType:AVMediaTypeVideo] setEnabled:YES];

        if ([self.session canAddOutput:output]) {
            [self.session addOutput:output];
        }

        [self.previewView setVideoGravity:AVLayerVideoGravityResizeAspect];
        [self.session startRunning];

        dispatch_async(dispatch_get_main_queue(), ^{
            self.readyToCapture = YES;
        });

    });

}

+ (AVCaptureDevice *)defaultDevice {
    NSArray *devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
    for (AVCaptureDevice *d in devices) {
        if (d.position == AVCaptureDevicePositionBack) return d;
    }
    return devices.firstObject;
}

#pragma mark - AVCaptureVideoDataOutputSampleBufferDelegate
- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    if (self.trackingMode) {
        [self.depthEstimator trackImage:sampleBuffer];
    } else {
        if (_round++%5 == 0) {
            [self.depthEstimator checkStability:sampleBuffer];
        }
    }
}

#pragma mark - TKDDepthEstimatorCaptureDelegate
- (void)depthEstimatorStabilityUpdated:(TKDDepthEstimator *)estimator
{
    [self setStability:estimator.stability];
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator didTrack:(BOOL)added
{
    if (added) {
        [self updateGuideView];
    }
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator trackingFailed:(NSError *)error
{
    NSLog(@"%@", error);

    // TODO: いい感じのライブラリに置き換える
    // TODO: 2回呼ばれてしまうので対応する
    if (error.code == TKDDepthEstimatorFewFeaturesError) {
        NSString *message = @"特徴点追跡に失敗しました。明るいところでお試しください。";
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"エラー" message:message delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alert show];

        self.depthEstimator = [TKDDepthEstimator new];
        self.depthEstimator.captureDelegate = self;
        dispatch_async(self.videoDataQueue, ^{
            self.trackingMode = NO;
        });
    }

}

- (void)depthEstimatorTrackingCompleted:(TKDDepthEstimator *)estimator
{
    [self updateGuideView];
    [self performSegueWithIdentifier:@"showEditViewController" sender:self];
}

@end
