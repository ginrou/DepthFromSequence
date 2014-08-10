//
//  TKDViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/04/23.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

@import AVFoundation;

#import "TKDViewController.h"
#import "TKDDepthEstimator.h"
#import "TKDPreviewView.h"

@interface TKDViewController () <AVCaptureVideoDataOutputSampleBufferDelegate, TKDDepthEstimatorDelegate>
@property (weak, nonatomic) IBOutlet TKDPreviewView *previewView;
@property (weak, nonatomic) IBOutlet UIImageView *depthmap;
@property (atomic, assign) BOOL shouldCapture;
@property (nonatomic, strong) AVCaptureSession *session;
@property (nonatomic, strong) AVCaptureDeviceInput *deviceInput;
@property (nonatomic, strong) AVCaptureVideoDataOutput *videoOutput;
@property (nonatomic, strong) dispatch_queue_t sessionQueue;
@property (nonatomic, strong) dispatch_queue_t videoDataQueue;
@property (nonatomic, strong) TKDDepthEstimator *depthEstimator;
@end

@implementation TKDViewController

- (void)viewDidLoad
{
    [super viewDidLoad];

    self.depthEstimator = [TKDDepthEstimator new];
    self.depthEstimator.delegate = self;

    self.shouldCapture = NO;

    self.session = [AVCaptureSession new];
    self.previewView.session = self.session;
    self.sessionQueue = dispatch_queue_create("TKDViewController#sessionQueue", DISPATCH_QUEUE_SERIAL);

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

        self.videoDataQueue = dispatch_queue_create("TKDViewController#videoDataQueue", DISPATCH_QUEUE_SERIAL);
        [self.videoOutput setSampleBufferDelegate:self queue:self.videoDataQueue];
        [[self.videoOutput connectionWithMediaType:AVMediaTypeVideo] setEnabled:YES];
        if ([self.session canAddOutput:self.videoOutput]) {
            [self.session addOutput:self.videoOutput];
        }
        [self.session startRunning];
    });

}

- (void)viewDidAppear:(BOOL)animated {
    [super viewDidAppear:animated];
}


- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
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
    });
}

- (IBAction)stopButtonTapped:(id)sender {
    dispatch_async(self.videoDataQueue, ^{
        self.shouldCapture = NO;
        [self.depthEstimator runEstimation];
    });
}


+ (AVCaptureDevice *)defaultDevice
{
    NSArray *devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
    for (AVCaptureDevice *d in devices) {
        if (d.position == AVCaptureDevicePositionBack) return d;
    }
    return devices.firstObject;
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    if (self.shouldCapture) {
        [self.depthEstimator addImage:sampleBuffer];
    }
}

- (void)depthEstimator:(UIImage *)estimatedDepthMap
{
    self.depthmap.image = estimatedDepthMap;
}

@end
