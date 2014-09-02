//
//  TKDDepthEstimator.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/10.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDDepthEstimator.h"
#import <opencv.hpp>
#include "depth_from_sequence.hpp"
#import <ios.h>

static int const kMaxImages = 20;

using namespace std;
using namespace cv;

@interface TKDDepthEstimator ()
{
    vector<Mat3b> *full_color_images;
    FeatureTracker *tracker;
}
@property (nonatomic, strong) dispatch_queue_t queue;
@property (atomic) BOOL running;
@property NSFileHandle *pipeReadHandle;
@end

@implementation TKDDepthEstimator
- (instancetype)init {
    self = [super init];
    if (self) {
        full_color_images = new vector<Mat3b>;
        tracker = new FeatureTracker;
        _queue = dispatch_queue_create("DepthEstimationQueue", NULL);

        self.log = [NSMutableString string];
        NSPipe *pipe = [NSPipe pipe];
        self.pipeReadHandle = [pipe fileHandleForReading];
        dup2([[pipe fileHandleForWriting] fileDescriptor], fileno(stdout));
        [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(readSTDOUT:) name:NSFileHandleReadCompletionNotification object:nil];
        [self.pipeReadHandle readInBackgroundAndNotify];
    }
    return self;
}

- (void)dealloc {
    delete full_color_images;
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    dup2(fileno(stdout), self.pipeReadHandle.fileDescriptor);
}

+ (Mat3b)sampleBufferToMat:(CMSampleBufferRef)sampleBuffer {
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

    CVPixelBufferLockBaseAddress( pixelBuffer, 0 );

    //Processing here
    int bufferWidth = (int)CVPixelBufferGetWidth(pixelBuffer);
    int bufferHeight = (int)CVPixelBufferGetHeight(pixelBuffer);
    unsigned char *pixel = (unsigned char *)CVPixelBufferGetBaseAddress(pixelBuffer);

    // put buffer in open cv, no memory copied
    Mat4b mat = cv::Mat(bufferHeight,bufferWidth,CV_8UC4,pixel);
    Mat3b mat3b(bufferHeight, bufferWidth);
    cvtColor(mat, mat3b, CV_BGRA2RGB);

    //End processing
    CVPixelBufferUnlockBaseAddress( pixelBuffer, 0 );

    return mat3b;
}

- (NSArray *)convertToUIImages {
    NSMutableArray *array = [NSMutableArray array];
    for (int i = 0; i < full_color_images->size(); ++i) {
        Mat3b mat = (*full_color_images)[i];
        UIImage *img = MatToUIImage(mat);
        [array addObject:[UIImage imageWithCGImage:img.CGImage scale:2.0 orientation:UIImageOrientationRight]];
    }
    return array;
}

- (void)readSTDOUT:(NSNotification *)n {
    [self.pipeReadHandle readInBackgroundAndNotify];
    NSString *str = [[NSString alloc] initWithData:n.userInfo[NSFileHandleNotificationDataItem] encoding:NSASCIIStringEncoding];
    [self.log appendString:str];

    dispatch_async(dispatch_get_main_queue(), ^{
        [self.delegate depthEstimator:self getLog:self.log];
    });

}

- (void)checkStability:(CMSampleBufferRef)sampleBuffer {
    Mat3b mat = [[self class] sampleBufferToMat:sampleBuffer];
    dispatch_async(_queue, ^{
        Mat1b *gray = new Mat1b(mat.size());
        cvtColor(mat, *gray, CV_RGB2GRAY);
        int features = tracker->good_features_to_track(*gray);
        dispatch_async(dispatch_get_main_queue(), ^{
            CGFloat st = (CGFloat)features/(CGFloat)tracker->MAX_CORNERS;
            [self.delegate depthEstimator:self stabilityUpdated:st];
        });
        delete gray;
    });
}

- (void)addImage:(CMSampleBufferRef)sampleBuffer {

    if (full_color_images->size() >= kMaxImages) {
        NSLog(@"%lu do not add anymore", full_color_images->size());
        dispatch_async(dispatch_get_main_queue(), ^{
            [self.delegate depthEstimatorImagesPrepared:self];
        });
        return;
    }

    Mat3b new_image = [[self class] sampleBufferToMat:sampleBuffer];

    dispatch_async(_queue, ^{

        Mat1b *gray = new Mat1b(new_image.size());
        cvtColor(new_image, *gray, CV_BGR2GRAY);
        bool added = tracker->add_image(*gray);

        if (added) {
            full_color_images->push_back(new_image);
            cout << tracker->count_track_points() << endl;
        }

        delete gray;
    });

}

- (void)runEstimation {
    dispatch_async(_queue, ^{
        if (self.running == NO) {
            self.running = YES;
            [self runEstimationImpl];
        }
    });
}

- (void)runEstimationImpl {

    vector< vector<Point2d> > track_points = tracker->pickup_stable_points();

    // Solver を初期化
    BundleAdjustment::Solver solver( track_points );
    double min_depth = 500.0; // [mm]
    double fov = 55.0; // [deg]
    cv::Size img_size = full_color_images->front().size();
    double f = MIN(img_size.width, img_size.height)/2.0;
    solver.initialize(track_points, min_depth, fov, img_size, f);

    // bundle adjustment を実行
    while ( solver.should_continue ) {
        solver.run_one_step();
        print_ittr_status(solver);
    }

    print_params(solver);
    // plane sweep の準備
    vector<double> depths = solver.depth_variation(32);

    for(int i = 0; i < depths.size(); ++i )
        cout << depths[i] << endl;

    // plane sweep + dencecrf で奥行きを求める
    PlaneSweep *ps = new PlaneSweep(*full_color_images, solver.camera_params, depths);
    ps->sweep(full_color_images->front()); // ref imageは最初の画像

    UIImage *raw = MatToUIImage(8*(ps->_depth_raw));
    self.rawDepthMap = [UIImage imageWithCGImage:raw.CGImage scale:2.0 orientation:UIImageOrientationRight];

    UIImage *smooth = MatToUIImage(8*(ps->_depth_smooth));
    self.smoothDepthMap = [UIImage imageWithCGImage:smooth.CGImage scale:2.0 orientation:UIImageOrientationRight];

    dispatch_async(dispatch_get_main_queue(), ^{
        [self.delegate depthEstimator:self
                  estimationCompleted:self.smoothDepthMap];
    });

    delete ps;

}

@end
