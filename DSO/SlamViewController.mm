//
//  SlamViewController.m
//  DSO
//
//  Created by Neil on 2019/3/23.
//  Copyright © 2019 Neil. All rights reserved.
//

#import "PointCloud.h"
#import "SlamViewController.h"

@interface SlamViewController ()

@end

@implementation SlamViewController

Test t;
- (void)viewDidLoad {
    [super viewDidLoad];
    printf("2 did load\n");
    printf("there are %lu points\n", [[PointCloud getInstance].points count]);
    // Do any additional setup after loading the view.
}

- (void)viewWillDisappear:(BOOL)animated {
    printf("will disappear, %d\n", t.a);
}
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
    
}

@end
