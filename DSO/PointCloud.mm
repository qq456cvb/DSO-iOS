//
//  PointCloud.m
//  DSO
//
//  Created by Neil on 2019/9/12.
//  Copyright © 2019 Neil. All rights reserved.
//
#include "PointCloud.h"
#import <Foundation/Foundation.h>
@implementation PointCloud : NSObject

static PointCloud *instance = nil;
- (void) pushPoint:(CIVector*) point
{
    [self.points addObject:point];
}
+ (PointCloud *) getInstance
{
    @synchronized(self)
    {
        if(instance==nil)
        {
            instance= [PointCloud new];
            instance.points = [NSMutableArray array];
        }
    }
    return instance;
}
@end

