//
//  TKDFileWatcher.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/11.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDFileWatcher.h"

@interface TKDFileWatcher ()

@end

@implementation TKDFileWatcher
- (instancetype)initWithFileName:(NSString *)filename
{
    self = [self init];
    if (self) {
        _filename = filename;
        _stringEncoding = NSASCIIStringEncoding;
    }
    return self;
}

- (NSString *)filepath {
    NSString *dir = [NSSearchPathForDirectoriesInDomains(NSCachesDirectory, NSUserDomainMask, YES) firstObject];
    NSString *path = [dir stringByAppendingPathComponent:_filename];

    return path;
}

- (void)startWatching
{
    NSFileManager *fm = [NSFileManager defaultManager];
    if ([fm fileExistsAtPath:self.filepath]) {
        [fm removeItemAtPath:self.filepath error:nil];
    }

    self.fileHandle = [NSFileHandle fileHandleForUpdatingAtPath:self.filepath];

    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(didWrittenToFile:)
                                                 name:NSFileHandleDataAvailableNotification
                                               object:self.fileHandle];

    [self.fileHandle waitForDataInBackgroundAndNotify];
}

- (void)didWrittenToFile:(NSNotification *)n
{
    NSFileHandle *fh = n.object;
    NSString *str = [[NSString alloc] initWithData:fh.availableData encoding:self.stringEncoding];

    [self.delegate fileWatcher:self stringWritten:str];

    [fh waitForDataInBackgroundAndNotify];
}

- (void)stopWatching
{
    [[NSNotificationCenter defaultCenter] removeObserver:self name:NSFileHandleDataAvailableNotification object:self.fileHandle];
}

@end
