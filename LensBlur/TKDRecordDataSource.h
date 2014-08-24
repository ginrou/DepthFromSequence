//
//  TKDRecordDataSource.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/24.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface TKDDepthEstimationRecord : NSObject
@property NSString *key;
@property NSString *log;
@property NSArray *capturedImages;
@property UIImage *rawDepthMap;
@property UIImage *smoothDepthMap;

+ (TKDDepthEstimationRecord *)recordFromURL:(NSURL *)url;
+ (TKDDepthEstimationRecord *)saveRecordTo:(NSURL *)dirURL
                                       log:(NSString *)log
                            capturedImages:(NSArray *)capturedImages
                               rawDepthMap:(UIImage *)rawDepthMap
                            smoothDepthMap:(UIImage *)smoothDepthMap;
@end

@interface TKDRecordDataSource : NSObject
+ (void)fetchDepthEstimationRecord:(void(^)(NSArray *records, NSError *error))block;
+ (void)writeDepthEstimationRecord:(NSString *)log
                    capturedImages:(NSArray *)capturedImages
                       rawDepthMap:(UIImage *)rawDepthMap
                    smoothDepthMap:(UIImage *)smoothDepthMap
                        completion:(void(^)(TKDDepthEstimationRecord *recored, NSError *error))block;
@end
