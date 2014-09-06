//
//  TKDImageConverter.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/06.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <opencv.hpp>
#import <ios.h>

cv::Mat3b sampleBufferToMat(CMSampleBufferRef sampleBuffer);
UIImage* matToUIImage(const cv::Mat& mat, CGFloat scale, UIImageOrientation orientation);
void UIImageToMat3b(UIImage *img, cv::Mat3b &mat);