//
//  TKDAsyncRefoucs.mm
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/06.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDAsyncRefoucs.h"
#import "refocus.hpp"

#import "TKDImageConverter.h"

@interface TKDAsyncRefoucs () {
    Mat3b *ref_img_mat;
    Mat1b *disp_mat;
    Mat1b aperture_mat;
    std::vector<double> *disp_seq;
}
@property (nonatomic, strong) dispatch_queue_t queue;
@end

static const char queue_label[] = "TKDAsyncRefocus#processingQueue";

@implementation TKDAsyncRefoucs
- (instancetype)init {
    self = [super init];
    if (self) {
        _apertureSize = 1.0;
        _queue = dispatch_queue_create(queue_label, DISPATCH_QUEUE_SERIAL);
        ref_img_mat = NULL;
        disp_mat = NULL;
        aperture_mat = Refocus::circuler_aperture(30);
        disp_seq = NULL;
    }
    return self;
}

- (void)dealloc {
    delete ref_img_mat;
    delete disp_mat;
    delete disp_seq;
}


- (void)setReferenceImage:(UIImage *)referenceImage {
    if (_referenceImage != referenceImage) {
        _referenceImage = referenceImage;

        dispatch_async(self.queue, ^{
            if (ref_img_mat != NULL) delete ref_img_mat;
            ref_img_mat = new Mat3b(referenceImage.size.height, referenceImage.size.height);
            UIImageToMat3b(referenceImage, *ref_img_mat);
        });
    }
}

- (void)setDisparityMap:(UIImage *)disparityMap {
    if (_disparityMap != disparityMap) {
        _disparityMap = disparityMap;

        dispatch_async(self.queue, ^{
            if (disp_mat != NULL) delete disp_mat;
            disp_mat = new Mat1b(disparityMap.size.height, disparityMap.size.height);
            UIImageToMat(disparityMap, *disp_mat);
        });
    }
}

- (void)setDepthSequence:(NSArray *)depthSequence {
    if (_depthSequence != depthSequence) {
        _depthSequence = depthSequence;

        dispatch_async(self.queue, ^{

            std::vector<double> depth;
            for (NSNumber *n in depthSequence) {
                depth.push_back([n doubleValue]);
            }

            if (disp_seq != NULL) delete disp_seq;
            disp_seq = new std::vector<double>(depth.size());
            *disp_seq = Refocus::depth_to_disparity(depth, 32);
        });
    }
}

- (void)refocusTo:(CGPoint)point {

    NSAssert(self.referenceImage, @"insufficient input, referenceImage");
    NSAssert(self.disparityMap, @"insufficient input, disparityMap");
    NSAssert(self.depthSequence, @"insufficient input, disparitySequence");

    NSLog(@"refocus to %@", NSStringFromCGPoint(point));

    __weak typeof(self) weakSelf = self;
    dispatch_async(self.queue, ^{

        Refocus *refocus = new Refocus(*ref_img_mat, aperture_mat, *disp_mat, *disp_seq, _apertureSize);

        cv::Point2d focal_point(point.x, point.y);
        Mat3b dst = refocus->refocus_to(focal_point);
        UIImage *refocused = matToUIImage(dst, 1.0, UIImageOrientationRight);
        _result = refocused;

        dispatch_async(dispatch_get_main_queue(), ^{
            [weakSelf.delegate asyncRefocus:weakSelf refocusCompleted:refocused];
        });

        delete refocus;
    });

}

@end
