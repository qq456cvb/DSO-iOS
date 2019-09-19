//
//  PointCloud.h
//  DSO
//
//  Created by Neil on 2019/9/12.
//  Copyright © 2019 Neil. All rights reserved.
//
#include <vector>
#include <mutex>
#include <Eigen/Core>
#import <Foundation/Foundation.h>
#import <CoreImage/CIVector.h>

#ifndef PointCloud_h
#define PointCloud_h
@interface PointCloud : NSObject
@property (atomic) NSMutableArray* points;
- (void) pushPoint:(CIVector*) point;
+ (PointCloud*)getInstance;
@end

#endif /* PointCloud_h */
