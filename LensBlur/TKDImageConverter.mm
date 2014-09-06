//
//  TKDImageConverter.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/06.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDImageConverter.h"

cv::Mat3b sampleBufferToMat(CMSampleBufferRef sampleBuffer)
{
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

    CVPixelBufferLockBaseAddress( pixelBuffer, 0 );

    //Processing here
    int bufferWidth = (int)CVPixelBufferGetWidth(pixelBuffer);
    int bufferHeight = (int)CVPixelBufferGetHeight(pixelBuffer);
    unsigned char *pixel = (unsigned char *)CVPixelBufferGetBaseAddress(pixelBuffer);

    // put buffer in open cv, no memory copied
    cv::Mat4b mat = cv::Mat(bufferHeight,bufferWidth,CV_8UC4,pixel);
    cv::Mat3b mat3b(bufferHeight, bufferWidth);
    cvtColor(mat, mat3b, CV_BGRA2RGB);

    //End processing
    CVPixelBufferUnlockBaseAddress( pixelBuffer, 0 );

    return mat3b;

}

UIImage* matToUIImage(const cv::Mat& mat, CGFloat scale, UIImageOrientation orientation)
{
    UIImage *img;
    if (mat.channels() == 3) {
        cv::Mat4b mat4b(mat.size());
        cvtColor(mat, mat4b, CV_RGB2RGBA);
        img = MatToUIImage(mat4b);
    } else {
        img = MatToUIImage(mat);
    }

    return [UIImage imageWithCGImage:img.CGImage
                               scale:scale
                         orientation:orientation];
}

void UIImageToMat3b(UIImage *img, cv::Mat3b &mat) {
    cv::Mat4b mat4b;
    UIImageToMat(img, mat4b);

    mat.create(mat4b.size());
    cvtColor(mat4b, mat, CV_RGBA2RGB);
}
