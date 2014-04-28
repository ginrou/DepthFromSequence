//
//  TKDViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/04/23.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <opencv2/opencv.hpp>
#import <ios.h>
#import <opencv2/highgui/cap_ios.h>
#import <opencv2/stitching/stitcher.hpp>
#import <opencv2/stitching/detail/motion_estimators.hpp>

#import "TKDViewController.h"

using namespace std;
using namespace cv;

@interface TKDViewController () <CvVideoCameraDelegate>
{

    vector<vector<Point2f>> *points;
    vector<Mat> *images;

}

@property (nonatomic, strong) CvVideoCamera *videoCamera;
@property (strong, nonatomic) IBOutlet UIView *videoView;
@property (nonatomic, assign) BOOL needToInit;
@property (nonatomic, strong) NSMutableArray *imageArray;
@end

@implementation TKDViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    Stitcher st = Stitcher::createDefault();

    points = new vector<vector<Point2f>>();
    images = new vector<Mat>();
    self.imageArray = [NSMutableArray array];

    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:self.videoView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    self.videoCamera.defaultFPS = 30;
    self.videoCamera.grayscaleMode = YES;
    self.needToInit = YES;
}

- (void)viewDidAppear:(BOOL)animated {
    [super viewDidAppear:animated];
    [self.videoCamera start];
}


- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)processImage:(cv::Mat &)image {

    static int MAX_COUNT = 500;
    cv::Size subPixWinSize(10,10), winSize(31,31);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

    UIImage *hoge = MatToUIImage(image);
    [self.imageArray addObject:hoge];

    vector<Point2f> prev_points, cur_points;
    (*images).push_back(image.clone());

    if (_needToInit) {
        cv::goodFeaturesToTrack(image, cur_points, MAX_COUNT, 0.01, 10);
        cv::cornerSubPix(image, cur_points, subPixWinSize, cv::Size(-1,-1), termcrit);
        _needToInit = NO;
    }
    else if (!(*points).empty()) {
        std::vector<uchar> status;
        std::vector<float> err;

        prev_points = (*points).back();
        Mat prev_image = (*images).back();

        cv::calcOpticalFlowPyrLK(prev_image, image, prev_points, cur_points, status, err, winSize, 3, termcrit, 0, 0.001);

        size_t i;
        for( i = 0; i < cur_points.size(); i++ ) {
            if( !status[i] )
                continue;

            cv::circle( image, cur_points[i], 3, cv::Scalar(0,255,0), -1, 8);
        }

    }

    (*points).push_back(cur_points);

    if ((*points).size() > 30) {
        [self.videoCamera stop];
        [self estimateDepth];
    }

}

- (void)estimateDepth
{
    vector<Point2f> first_pt = (*points).front(), last_pt = (*points).back();
    Mat img = (*images).front();
    Mat fund_mat = findFundamentalMat(first_pt, last_pt);
    Mat homo1, homo2;
    stereoRectifyUncalibrated(first_pt, last_pt, fund_mat, img.size(), homo1, homo2);

    cout << fund_mat << endl;
    cout << homo1 << endl;
    cout << homo2 << endl;

    Mat warp1, warp2;
    Mat first_img = (*images).front(), last_img = (*images).back();

    StereoBM stereo;
    Mat disp_map;
    stereo(last_img, first_img, disp_map);
    Mat1b mat1b(disp_map.size());

    convertScaleAbs(disp_map, mat1b);

    UIImage *uiImage = MatToUIImage(mat1b);

    dispatch_async(dispatch_get_main_queue(), ^{
        UIImageView *v = [[UIImageView alloc] initWithImage:uiImage];
        [self.view addSubview:v];
    });

    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *pathBase = paths[0];
    for (int i = 0; i < self.imageArray.count; ++i) {

        NSString *filename = [NSString stringWithFormat:@"img-%02d.jpg", i];
        NSString *filepath = [pathBase stringByAppendingPathComponent:filename];
        [UIImageJPEGRepresentation(self.imageArray[i], 1.0) writeToFile:filepath atomically:YES];

    }


}

@end
