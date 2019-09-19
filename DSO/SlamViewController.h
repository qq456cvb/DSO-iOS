//
//  SlamViewController.h
//  DSO
//
//  Created by Neil on 2019/3/23.
//  Copyright © 2019 Neil. All rights reserved.
//

#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

class Test {
    
public:
    int a = 0;
    Test() {
        printf("construct\n");
    }
    ~Test() {
        printf("distruct\n");
    }
};
@interface SlamViewController : UIViewController
@end

NS_ASSUME_NONNULL_END
