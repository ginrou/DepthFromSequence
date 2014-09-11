//
//  TKDFileWatcher.h
//  LensBlur
//
//  Created by 武田 祐一 on 2014/09/11.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import <Foundation/Foundation.h>

@protocol TKDFileWatcherDelegate;

@interface TKDFileWatcher : NSObject
@property (readonly) NSString *filename;
@property (readonly) NSString *filepath;
@property (weak) id<TKDFileWatcherDelegate> delegate;
@property NSFileHandle *fileHandle;
@property NSStringEncoding stringEncoding;

- (instancetype)initWithFileName:(NSString *)filename;

- (void)startWatching;
- (void)stopWatching;

@end

@protocol TKDFileWatcherDelegate <NSObject>
- (void)fileWatcher:(TKDFileWatcher *)watcher stringWritten:(NSString *)writtenString;
@end