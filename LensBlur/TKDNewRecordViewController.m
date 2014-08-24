//
//  TKDNewRecordViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/24.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

@import AVFoundation;

#import "TKDNewRecordViewController.h"

#import "TKDDepthEstimator.h"
#import "TKDPreviewView.h"

@interface TKDNewRecordViewController () <
AVCaptureVideoDataOutputSampleBufferDelegate,
TKDDepthEstimatorDelegate
>

// UIParts
@property (weak, nonatomic) IBOutlet UILabel *statusLabel;
@property (weak, nonatomic) IBOutlet TKDPreviewView *previewView;
@property (weak, nonatomic) IBOutlet UIImageView *rawDepthMap;
@property (weak, nonatomic) IBOutlet UIImageView *smoothDepthMap;
@property (weak, nonatomic) IBOutlet UITextView *logTextView;


// Buttons
@property (weak, nonatomic) IBOutlet UIBarButtonItem *cancelButton;
@property (weak, nonatomic) IBOutlet UIBarButtonItem *startButton;
@property (weak, nonatomic) IBOutlet UIBarButtonItem *saveButton;

// AVCaptures
@property (nonatomic, assign) BOOL shouldCapture;
@property (nonatomic, strong) AVCaptureSession *session;
@property (nonatomic, strong) AVCaptureDeviceInput *deviceInput;
@property (nonatomic, strong) AVCaptureVideoDataOutput *videoOutput;
@property (nonatomic, strong) dispatch_queue_t sessionQueue;
@property (nonatomic, strong) dispatch_queue_t videoDataQueue;


// Depth Estimator
@property (nonatomic, strong) TKDDepthEstimator *depthEstimator;

@end

@implementation TKDNewRecordViewController

- (void)viewDidLoad
{
    [super viewDidLoad];

    self.startButton.enabled = NO;
    self.saveButton.enabled = NO;
    self.statusLabel.text = @"Preparing...";

    self.depthEstimator = [TKDDepthEstimator new];
    self.depthEstimator.delegate = self;
    self.shouldCapture = NO;

    self.session = [AVCaptureSession new];
    [self.session setSessionPreset:AVCaptureSessionPreset640x480];
    self.previewView.session = self.session;
    self.sessionQueue = dispatch_queue_create("TKDNewRecordViewController#sessionQueue", DISPATCH_QUEUE_SERIAL);

    dispatch_async(self.sessionQueue, ^{

        NSError *error;

        AVCaptureDevice *device = [[self class] defaultDevice];
        self.deviceInput = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];

        if (error) NSLog(@"%@", error);

        if ([self.session canAddInput:self.deviceInput]) {
            [self.session addInput:self.deviceInput];
            dispatch_async(dispatch_get_main_queue(), ^{
                [[(AVCaptureVideoPreviewLayer *)[[self previewView] layer] connection] setVideoOrientation:(AVCaptureVideoOrientation)[self interfaceOrientation]];
            });
        }

        self.videoOutput = [AVCaptureVideoDataOutput new];
        self.videoOutput.videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey: @(kCMPixelFormat_32BGRA)};
        self.videoOutput.alwaysDiscardsLateVideoFrames = YES;

        self.videoDataQueue = dispatch_queue_create("TKDNewRecordViewController#videoDataQueue", DISPATCH_QUEUE_SERIAL);
        [self.videoOutput setSampleBufferDelegate:self queue:self.videoDataQueue];
        [[self.videoOutput connectionWithMediaType:AVMediaTypeVideo] setEnabled:YES];
        if ([self.session canAddOutput:self.videoOutput]) {
            [self.session addOutput:self.videoOutput];
        }
        [self.session startRunning];
        dispatch_async(dispatch_get_main_queue(), ^{
            self.statusLabel.text = @"ready to capture";
            self.startButton.enabled = YES;
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

- (IBAction)startButtonTapped:(id)sender {
    dispatch_async(self.videoDataQueue, ^{
        [self.session beginConfiguration];
        AVCaptureDevice *device = self.deviceInput.device;
        [device lockForConfiguration:nil];
        [device setFocusMode:AVCaptureFocusModeLocked];
        [device unlockForConfiguration];
        [self.session commitConfiguration];
        self.shouldCapture = YES;
        self.startButton.enabled = NO;
    });

}

- (IBAction)saveButtonTapped:(id)sender {
}

#pragma mark - Delegate
- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    if (self.shouldCapture) {
        [self.depthEstimator addImage:sampleBuffer];
    }
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator statusUpdated:(NSString *)status
{
    self.statusLabel.text = status;
}

- (void)depthEstimatorImagesPrepared:(TKDDepthEstimator *)estimator
{
    dispatch_async(self.videoDataQueue, ^{
        self.shouldCapture = NO;
        self.cancelButton.enabled = NO;
        self.saveButton.enabled = NO;
        dispatch_async(self.sessionQueue, ^{
            [self.session stopRunning];
        });
        [self.depthEstimator runEstimation];
    });
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator estimationCompleted:(UIImage *)smoothDepthMap
{
    self.rawDepthMap.image = estimator.rawDepthMap;
    self.smoothDepthMap.image = estimator.smoothDepthMap;
    self.statusLabel.text = @"completed";
    self.saveButton.enabled = YES;
}

- (void)depthEstimator:(TKDDepthEstimator *)estimator getLog:(NSString *)newLine
{
    self.logTextView.text = newLine;
}

@end
