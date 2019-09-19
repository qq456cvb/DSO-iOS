//
//  ViewController.h
//  DSO
//
//  Created by Neil on 2019/3/15.
//  Copyright © 2019 Neil. All rights reserved.
//
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/cap_ios.h>
#import <UIKit/UIKit.h>
#include <thread>
#include "MinimalImage.h"
#include "Output3DWrapper.h"



#include "HessianBlocks.h"
#include "FrameShell.h"
#include "FullSystem.h"
#include <mutex>
#include <vector>
#include <CoreImage/CIVector.h>
#include <SceneKit/SceneKit.h>

@interface ViewController : UIViewController<CvVideoCameraDelegate>

@property (strong, nonatomic) SCNScene *scene;
@property (strong, nonatomic) SCNNode *lookat;

@property (weak, nonatomic) IBOutlet SCNView *scnView;
@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (weak, nonatomic) IBOutlet UIImageView *depthView;
@property (retain, nonatomic) CvVideoCamera *videoCamera;

@end

class MyOutputWrapper : public dso::IOWrap::Output3DWrapper
{
    ViewController *controller;
    int w, h;
    bool img_added;
public:
    MyOutputWrapper(int w, int h, ViewController *controller);
    virtual void pushDepthImage(dso::MinimalImageB3* image);
    virtual bool needPushDepthImage();
    virtual void publishKeyframes(std::vector<dso::FrameHessian *> &frames, bool final, dso::CalibHessian *HCalib);
    virtual void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib);
    void run();
};
