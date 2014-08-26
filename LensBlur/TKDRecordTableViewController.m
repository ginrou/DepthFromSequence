//
//  TKDRecordTableViewController.m
//  LensBlur
//
//  Created by 武田 祐一 on 2014/08/24.
//  Copyright (c) 2014年 Yuichi Takeda. All rights reserved.
//

#import "TKDRecordDataSource.h"
#import "TKDRecordTableViewController.h"
#import "TKDNewRecordViewController.h"
#import "TKDRecordViewerViewController.h"

@interface TKDRecordTableViewController ()
@property (nonatomic, strong) NSMutableArray *records;
@end

@implementation TKDRecordTableViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    self.clearsSelectionOnViewWillAppear = NO;
    self.records = [NSMutableArray array];

    [TKDRecordDataSource fetchDepthEstimationRecord:^(NSArray *records, NSError *error) {

        if (error || records == nil) {
            NSLog(@"records = %@", records);
            NSLog(@"error = %@", error);
        } else {
            [self.records removeAllObjects];
            [self.records addObjectsFromArray:records];
            [self.tableView reloadData];
        }

    }];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Table view data source
- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    return self.records.count;
}


- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:@"cell" forIndexPath:indexPath];
    TKDDepthEstimationRecord *record = self.records[indexPath.row];
    cell.textLabel.text = record.key;
    return cell;
}


#pragma mark - Navigation

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    if ([segue.identifier isEqualToString:@"showRecordViewer"]) {
        TKDRecordViewerViewController *dst = segue.destinationViewController;
        NSInteger index = self.tableView.indexPathForSelectedRow.row;
        dst.record = self.records[index];
    }
}


- (IBAction)createNewRecordCanceled:(UIStoryboardSegue *)segue{
    // storyboad close modal automatically
}

- (IBAction)createNewRecordCaptured:(UIStoryboardSegue *)segue {
    TKDDepthEstimationRecord *record = [(TKDNewRecordViewController *)segue.sourceViewController createdRecord];

    [self.tableView beginUpdates];
    [self.records addObject:record];
    NSIndexPath *ip = [NSIndexPath indexPathForRow:self.records.count-1 inSection:0];
    [self.tableView insertRowsAtIndexPaths:@[ip] withRowAnimation:UITableViewRowAnimationAutomatic];
    [self.tableView endUpdates];
}


@end
