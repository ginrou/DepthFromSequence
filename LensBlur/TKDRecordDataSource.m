//
//  TKDRecordDataSource.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/24.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDRecordDataSource.h"

static NSString * const kdepthEstimationRecordDirectoryPathComponent = @"depthEstimationRecord";

@implementation TKDDepthEstimationRecord


+ (TKDDepthEstimationRecord *)recordFromURL:(NSURL *)dirUrl {

    TKDDepthEstimationRecord *recored = [TKDDepthEstimationRecord new];
    recored.key = dirUrl.pathComponents.lastObject;

    NSFileManager *fm = [NSFileManager defaultManager];
    NSMutableArray *images = [NSMutableArray array];
    NSArray *contentsInDirectory = [fm contentsOfDirectoryAtURL:dirUrl
                                     includingPropertiesForKeys:@[]
                                                        options:NSDirectoryEnumerationSkipsHiddenFiles
                                                          error:nil];

    NSPredicate *logFilePredicate = [NSPredicate predicateWithFormat:@"self = 'log.txt'"];
    NSPredicate *capturedPredicate = [NSPredicate predicateWithFormat:@"self matches 'captured-[0-9]{2}.jpg$'"];
    NSPredicate *rawDepthPredicate = [NSPredicate predicateWithFormat:@"self = 'depth-raw.png'"];
    NSPredicate *smoothDepthPredicate = [NSPredicate predicateWithFormat:@"self = 'depth-smooth.png'"];

    for (NSURL *url in contentsInDirectory) {
        NSString *filename = url.pathComponents.lastObject;

        if ([@[filename] filteredArrayUsingPredicate:logFilePredicate].count == 1) {

            recored.log = [NSString stringWithContentsOfURL:url encoding:NSASCIIStringEncoding error:nil];

        } else if ([@[filename] filteredArrayUsingPredicate:capturedPredicate].count == 1) {

            [images addObject:[UIImage imageWithContentsOfFile:url.path]];

        } else if ([@[filename] filteredArrayUsingPredicate:rawDepthPredicate].count == 1) {

            recored.rawDepthMap = [UIImage imageWithContentsOfFile:url.path];

        } else if ([@[filename] filteredArrayUsingPredicate:smoothDepthPredicate].count == 1) {

            recored.smoothDepthMap = [UIImage imageWithContentsOfFile:url.path];

        }

    }

    recored.capturedImages = images;

    return recored;
}

+ (TKDDepthEstimationRecord *)saveRecordTo:(NSURL *)dirURL
                                       log:(NSString *)log
                            capturedImages:(NSArray *)capturedImages
                               rawDepthMap:(UIImage *)rawDepthMap
                            smoothDepthMap:(UIImage *)smoothDepthMap
{

    // write log file
    NSString *logFilePath = [[dirURL.path stringByAppendingPathComponent:@"log"] stringByAppendingPathExtension:@"txt"];
    [log writeToFile:logFilePath atomically:YES encoding:NSASCIIStringEncoding error:nil];

    // write captured images
    NSMutableArray *imageFiles = [NSMutableArray array];
    for (NSInteger i = 0; i < capturedImages.count; ++i) {
        NSData *jpeg = UIImageJPEGRepresentation(capturedImages[i], 0.6);
        NSString *fileName = [NSString stringWithFormat:@"captured-%02d.jpg", (int)i];
        NSString *filePath = [dirURL.path stringByAppendingPathComponent:fileName];
        [jpeg writeToFile:filePath atomically:YES];
        [imageFiles addObject:filePath];
    }

    // write raw depth map
    NSData *rawDepthPNG = UIImagePNGRepresentation(rawDepthMap);
    NSString *rawDepthPath = [[dirURL.path stringByAppendingPathComponent:@"depth-raw"] stringByAppendingPathExtension:@"png"];
    [rawDepthPNG writeToFile:rawDepthPath atomically:YES];

    // write smooth depth map
    NSData *smoothDepthPNG = UIImagePNGRepresentation(smoothDepthMap);
    NSString *smoothDepthPath = [[dirURL.path stringByAppendingPathComponent:@"depth-smooth"] stringByAppendingPathExtension:@"png"];
    [smoothDepthPNG writeToFile:smoothDepthPath atomically:YES];

    TKDDepthEstimationRecord *record = [TKDDepthEstimationRecord new];
    record.key = [dirURL.path componentsSeparatedByString:@"/"].lastObject;
    record.log = log;
    record.capturedImages = capturedImages;
    record.rawDepthMap = rawDepthMap;
    record.smoothDepthMap = smoothDepthMap;

    return record;
}

@end


@implementation TKDRecordDataSource

+ (dispatch_queue_t)fileIOQueue {
    static dispatch_queue_t ioQueue;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        ioQueue = dispatch_queue_create("TKDRecoredDataSource#fileIOQueue", DISPATCH_QUEUE_SERIAL);
    });
    return ioQueue;
}

+ (NSString *)depthEstimationRecordDirectory {
    NSString *documentPath = [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES) firstObject];
    return [documentPath stringByAppendingPathComponent:kdepthEstimationRecordDirectoryPathComponent];
}

+ (void)fetchDepthEstimationRecord:(void (^)(NSArray *, NSError *))block {

    dispatch_async([self fileIOQueue], ^{
        NSFileManager *fileManager = [NSFileManager defaultManager];

        NSString *directoryPath = [self depthEstimationRecordDirectory];
        NSURL *directoryURL = [NSURL fileURLWithPath:directoryPath isDirectory:YES];
        NSLog(@"%@", directoryURL);

        NSDirectoryEnumerator *enumerator = [fileManager enumeratorAtURL:directoryURL
                                              includingPropertiesForKeys:@[NSURLIsDirectoryKey, NSURLNameKey]
                                                                 options:NSDirectoryEnumerationSkipsSubdirectoryDescendants
                                                            errorHandler:^BOOL(NSURL *url, NSError *error) {
                                                                block(nil, error);
                                                                return NO;
                                                            }];

        NSMutableArray *result = [NSMutableArray array];
        for (NSURL *url in enumerator) {
            NSLog(@"%@", url);
            TKDDepthEstimationRecord *record = [TKDDepthEstimationRecord recordFromURL:url];
            if (record) [result addObject:record];
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            block(result, nil);
        });

    });

}

+ (void)writeDepthEstimationRecord:(NSString *)log
                    capturedImages:(NSArray *)capturedImages
                       rawDepthMap:(UIImage *)rawDepthMap
                    smoothDepthMap:(UIImage *)smoothDepthMap
                        completion:(void (^)(TKDDepthEstimationRecord *, NSError *))block
{
    dispatch_async([self fileIOQueue], ^{

        NSError *error;

        NSFileManager *fileManager = [NSFileManager defaultManager];
        NSString *key = [[NSDate date] description];
        NSString *directoryPath = [[self depthEstimationRecordDirectory] stringByAppendingPathComponent:key];
        [fileManager createDirectoryAtPath:directoryPath withIntermediateDirectories:YES attributes:nil error:&error];

        if (error) {
            dispatch_async(dispatch_get_main_queue(), ^{
                block(nil, error);
            });
            return;
        }

        NSURL *url = [NSURL fileURLWithPath:directoryPath isDirectory:YES];
        TKDDepthEstimationRecord *record = [TKDDepthEstimationRecord saveRecordTo:url
                                                                               log:log
                                                                    capturedImages:capturedImages
                                                                       rawDepthMap:rawDepthMap
                                                                    smoothDepthMap:smoothDepthMap];
        dispatch_async(dispatch_get_main_queue(), ^{
            block(record, nil);
        });
    });
}
@end
