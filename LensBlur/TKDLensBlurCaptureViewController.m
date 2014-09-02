//
//  TKDLensBlurCaptureViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/02.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

@import AVFoundation;

#import "TKDLensBlurCaptureViewController.h"

@interface TKDLensBlurCaptureViewController () <
AVCaptureVideoDataOutputSampleBufferDelegate
>
@property (weak, nonatomic) IBOutlet UILabel *stabilityLabel;
@property (weak, nonatomic) IBOutlet UIView *stabilityIcon;
@property (weak, nonatomic) IBOutlet UIButton *captureButton;
@property (weak, nonatomic) IBOutlet TKDPreviewView *previewView;
@property (assign, nonatomic) BOOL readyToCapture;

// AVCaptures
@property (nonatomic, strong) AVCaptureSession *session;
@property (nonatomic, strong) dispatch_queue_t sessionQueue;
@property (nonatomic, strong) dispatch_queue_t videoDataQueue;

@end

static const char * sessionQueueLabel = "TKDLensBlurCaptureViewController#sessionQueue";
static const char * videoDataQueueLabel = "TKDLensBlurCaptureViewController#videoDataQueue";

@implementation TKDLensBlurCaptureViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    self.stabilityIcon.layer.cornerRadius = self.stabilityIcon.frame.size.width/2.0;
    self.captureButton.enabled = NO;
    [self setupAVCapture];
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

- (void)setStabilityLabelText:(NSString *)text color:(UIColor *)color {
    self.stabilityLabel.text = text;
    self.stabilityIcon.backgroundColor = color;
}

- (void)setupAVCapture {
    [self setStabilityLabelText:@"preparing..." color:[UIColor redColor]];

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
