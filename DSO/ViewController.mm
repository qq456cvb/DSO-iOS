//
//  ViewController.m
//  DSO
//
//  Created by Neil on 2019/3/15.
//  Copyright © 2019 Neil. All rights reserved.
//

#import "ViewController.h"
#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "Output3DWrapper.h"
#include "ImageDisplay.h"

#include <thread>
#import <AVFoundation/AVFoundation.h>
#include "settings.h"
#include "globalFuncs.h"
#include "DatasetReader.h"
#include "globalCalib.h"

#include "NumType.h"
#include "FullSystem.h"
#include "MatrixAccumulators.h"
#include "PixelSelector2.h"
#include "PointCloud.h"


/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */


//#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
//#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include <sys/time.h>

static char *img_data;
static char *bgr_img_data;
static bool started = false;
static std::mutex mtx;
static int points_count = 0;
static SCNVector3 points_center = SCNVector3Make(0, 0, 0);
static SCNVector3 last_pos = SCNVector3Make(0, 0, 0);
static std::vector<cv::Mat> key_frames;

MyOutputWrapper::MyOutputWrapper(int w, int h, ViewController *controller) {
    this->w = w;
    this->h = h;
    this->controller = controller;
}

bool  MyOutputWrapper::needPushDepthImage() {
    return true;
}

void MyOutputWrapper::publishCamPose(FrameShell* frame, CalibHessian* HCalib) {
    auto const& m = frame->camToWorld.matrix3x4();
    Eigen::Vector4d origin(0.f, 0.f, 0.f, 1.f);
    Eigen::Vector3d cam_pos = m * origin;
    if (last_pos.x * last_pos.x + last_pos.y * last_pos.y + last_pos.z * last_pos.z > 1e-7) {
        
        SCNVector3 positions[] = {
            SCNVector3Make(last_pos.x, last_pos.y, last_pos.z),
            SCNVector3Make(cam_pos.x(), cam_pos.y(), cam_pos.z())
        };
        
        int indices[] = {0, 1};
        
        SCNGeometrySource *vertexSource = [SCNGeometrySource geometrySourceWithVertices:positions
                                                                                  count:2];
        
        NSData *indexData = [NSData dataWithBytes:indices
                                           length:sizeof(indices)];
        
        SCNGeometryElement *element = [SCNGeometryElement geometryElementWithData:indexData
                                                                    primitiveType:SCNGeometryPrimitiveTypeLine
                                                                   primitiveCount:1
                                                                    bytesPerIndex:sizeof(int)];
        
        SCNGeometry *line = [SCNGeometry geometryWithSources:@[vertexSource]
                                                    elements:@[element]];
        
        SCNNode *lineNode = [SCNNode nodeWithGeometry:line];
        
//        dispatch_async(dispatch_get_main_queue(), ^{
        [this->controller.scene.rootNode addChildNode:lineNode];
//        });
    }
    last_pos = SCNVector3Make(cam_pos.x(), cam_pos.y(), cam_pos.z());
}

void MyOutputWrapper::publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib) {
    float fx, fy, cx, cy;
    float fxi, fyi, cxi, cyi;
    //float colorIntensity = 1.0f;
    fx = HCalib->fxl();
    fy = HCalib->fyl();
    cx = HCalib->cxl();
    cy = HCalib->cyl();
    fxi = 1 / fx;
    fyi = 1 / fy;
    cxi = -cx / fx;
    cyi = -cy / fy;
    
    if (final)
    {
        for (FrameHessian* f : frames)
        {
            if (f->shell->poseValid)
            {
                auto const& m = f->shell->camToWorld.matrix3x4();
                
                // use only marginalized points.
                auto const& points = f->pointHessiansMarginalized;
                
                printf("marginalized %lu points for %d\n", f->pointHessiansMarginalized.size(), f->shell->incoming_id);
//                printf("active %lu points\n", f->pointHessians.size());
                
                std::vector<SCNVector3> vis_points, vis_colors;
                for (auto const* p : points)
                {
                    float depth = 1.0f / p->idepth;
                    auto const x = (p->u * fxi + cxi) * depth;
                    auto const y = (p->v * fyi + cyi) * depth;
//                    auto const z = depth * (1 + 2 * fxi);
                    auto const z = depth;
                    
                    Eigen::Vector4d camPoint(x, y, z, 1.f);
                    Eigen::Vector3d worldPoint = m * camPoint;
                    
                    vis_points.push_back(SCNVector3Make(worldPoint[0], worldPoint[1], worldPoint[2]));
                    points_count++;
                    points_center.x += (worldPoint[0] - points_center.x) / points_count;
                    points_center.y += (worldPoint[1] - points_center.y) / points_count;
                    points_center.z += (worldPoint[2] - points_center.z) / points_count;
                    Vec3b color = f->debugImage->at(p->u, p->v);
                    vis_colors.push_back(SCNVector3Make(color[2] / 255.f, color[1] / 255.f, color[0] / 255.f));
//                    [[PointCloud getInstance] pushPoint:[CIVector vectorWithX:worldPoint[0] Y:worldPoint[1] Z:worldPoint[2]] ];
                }
                SCNGeometrySource *positionSource = [SCNGeometrySource geometrySourceWithVertices:vis_points.data() count:vis_points.size()];
                NSData *colorData = [NSData dataWithBytes:vis_colors.data() length:vis_colors.size() * sizeof(SCNVector3)];
                SCNGeometrySource *colorSource = [SCNGeometrySource geometrySourceWithData:colorData semantic:SCNGeometrySourceSemanticColor vectorCount:vis_colors.size() floatComponents:true componentsPerVector:3 bytesPerComponent:sizeof(float) dataOffset:0 dataStride:sizeof(SCNVector3)];
//let colorSource = SCNGeometrySource(data: colorData, semantic: SCNGeometrySourceSemanticColor, vectorCount: colors.count, floatComponents: true, componentsPerVector: 3, bytesPerComponent: sizeof(Float), dataOffset: 0, dataStride: sizeof(SCNVector3))
                SCNGeometryElement *elements = [SCNGeometryElement geometryElementWithData:nil primitiveType:SCNGeometryPrimitiveTypePoint primitiveCount:vis_points.size() bytesPerIndex:sizeof(int)];
                elements.pointSize = 3.;
                elements.minimumPointScreenSpaceRadius = 3;
                elements.maximumPointScreenSpaceRadius = 3;
                SCNGeometry *pointsGeometry = [SCNGeometry geometryWithSources:@[positionSource, colorSource] elements:@[elements]];
//                dispatch_async(dispatch_get_main_queue(), ^{
                [this->controller.scene.rootNode addChildNode:[SCNNode nodeWithGeometry:pointsGeometry]];
//                    std::cout << points_center.x << ", " << points_center.y << ", " << points_center.z << std::endl;
//                    this->controller.lookat.position = points_center;
//                });
            }
        }
    }
}

void MyOutputWrapper::pushDepthImage(dso::MinimalImageB3* image) {
    static unsigned char*data = (unsigned char *)malloc(w * h * 4);
    for (size_t i = 0; i < w; i++) {
        for (size_t j = 0; j < h; j++) {
            auto rgb = image->at(static_cast<int>(i), static_cast<int>(j));
            data[(j * w + i) * 4] = rgb(0, 0);
            data[(j * w + i) * 4 + 1] = rgb(1, 0);
            data[(j * w + i) * 4 + 2] = rgb(2, 0);
            data[(j * w + i) * 4 + 3] = 255;
        }
    }
    int bitsPerComponent = 8;
    int bitsPerPixel = 32;
    int bytesPerRow = w * 4;
    CGDataProviderRef provider = CGDataProviderCreateWithData(NULL, data, w*h*4, NULL);
    CGColorSpaceRef colorSpaceRef = CGColorSpaceCreateDeviceRGB();
    CGBitmapInfo bitmapInfo = kCGImageAlphaNoneSkipLast;
    CGColorRenderingIntent renderingIntent = kCGRenderingIntentDefault;
    
    CGImageRef imageRef = CGImageCreate( w, h, bitsPerComponent, bitsPerPixel,
                                        bytesPerRow, colorSpaceRef, bitmapInfo, provider, NULL, NO, renderingIntent);
    UIImage *myImage = [UIImage imageWithCGImage:imageRef];
    dispatch_async(dispatch_get_main_queue(), ^{
        [controller.depthView setImage:myImage];
    });
//    free(data);
}

void MyOutputWrapper::run() {

}

@interface ViewController () {
}

@end

@implementation ViewController



std::string vignette = "";
std::string gammaCalib = "";
std::string source = "";
std::string calib = "";
double rescale = 1;
bool reverse = false;
bool disableROS = false;
int start=0;
int end=100000;
bool prefetch = false;
float playbackSpeed=0;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload=false;
bool useSampleOutput=false;
int mode=0;

bool firstRosSpin=false;

using namespace dso;


void settingsDefault(int preset)
{
    printf("\n=============== PRESET Settings: ===============\n");
    if(preset == 0 || preset == 1)
    {
        printf("DEFAULT settings:\n"
               "- %s real-time enforcing\n"
               "- 2000 active points\n"
               "- 5-7 active frames\n"
               "- 1-6 LM iteration each KF\n"
               "- original image resolution\n", preset==0 ? "no " : "1x");
        
        playbackSpeed = (preset==0 ? 0 : 1);
        preload = preset == 0;
        setting_desiredImmatureDensity = 1500;
        setting_desiredPointDensity = 2000;
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations=6;
        setting_minOptIterations=1;
        
        setting_logStuff = false;
    }
    
    if(preset == 2 || preset == 3)
    {
        printf("FAST settings:\n"
               "- %s real-time enforcing\n"
               "- 800 active points\n"
               "- 4-6 active frames\n"
               "- 1-4 LM iteration each KF\n"
               "- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");
        
        playbackSpeed = (preset==2 ? 0 : 5);
        preload = preset==3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations=4;
        setting_minOptIterations=1;
        
        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;
        
        setting_logStuff = false;
    }
    
    printf("==============================================\n");
}

void parseArgument(const char* arg)
{
    int option;
    float foption;
    char buf[1000];
    
    
    if(1==sscanf(arg,"sampleoutput=%d",&option))
    {
        if(option==1)
        {
            useSampleOutput = true;
            printf("USING SAMPLE OUTPUT WRAPPER!\n");
        }
        return;
    }
    
    if(1==sscanf(arg,"quiet=%d",&option))
    {
        if(option==1)
        {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }
    
    if(1==sscanf(arg,"preset=%d",&option))
    {
        settingsDefault(option);
        return;
    }
    
    
    if(1==sscanf(arg,"rec=%d",&option))
    {
        if(option==0)
        {
            disableReconfigure = true;
            printf("DISABLE RECONFIGURE!\n");
        }
        return;
    }
    
    
    
    if(1==sscanf(arg,"noros=%d",&option))
    {
        if(option==1)
        {
            disableROS = true;
            disableReconfigure = true;
            printf("DISABLE ROS (AND RECONFIGURE)!\n");
        }
        return;
    }
    
    if(1==sscanf(arg,"nolog=%d",&option))
    {
        if(option==1)
        {
            setting_logStuff = false;
            printf("DISABLE LOGGING!\n");
        }
        return;
    }
    if(1==sscanf(arg,"reverse=%d",&option))
    {
        if(option==1)
        {
            reverse = true;
            printf("REVERSE!\n");
        }
        return;
    }
    if(1==sscanf(arg,"nogui=%d",&option))
    {
        if(option==1)
        {
            disableAllDisplay = true;
            printf("NO GUI!\n");
        }
        return;
    }
    if(1==sscanf(arg,"nomt=%d",&option))
    {
        if(option==1)
        {
            multiThreading = false;
            printf("NO MultiThreading!\n");
        }
        return;
    }
    if(1==sscanf(arg,"prefetch=%d",&option))
    {
        if(option==1)
        {
            prefetch = true;
            printf("PREFETCH!\n");
        }
        return;
    }
    if(1==sscanf(arg,"start=%d",&option))
    {
        start = option;
        printf("START AT %d!\n",start);
        return;
    }
    if(1==sscanf(arg,"end=%d",&option))
    {
        end = option;
        printf("END AT %d!\n",start);
        return;
    }
    
    if(1==sscanf(arg,"files=%s",buf))
    {
        source = buf;
        printf("loading data from %s!\n", source.c_str());
        return;
    }
    
    if(1==sscanf(arg,"calib=%s",buf))
    {
        calib = buf;
        printf("loading calibration from %s!\n", calib.c_str());
        return;
    }
    
    if(1==sscanf(arg,"vignette=%s",buf))
    {
        vignette = buf;
        printf("loading vignette from %s!\n", vignette.c_str());
        return;
    }
    
    if(1==sscanf(arg,"gamma=%s",buf))
    {
        gammaCalib = buf;
        printf("loading gammaCalib from %s!\n", gammaCalib.c_str());
        return;
    }
    
    if(1==sscanf(arg,"rescale=%f",&foption))
    {
        rescale = foption;
        printf("RESCALE %f!\n", rescale);
        return;
    }
    
    if(1==sscanf(arg,"speed=%f",&foption))
    {
        playbackSpeed = foption;
        printf("PLAYBACK SPEED %f!\n", playbackSpeed);
        return;
    }
    
    if(1==sscanf(arg,"save=%d",&option))
    {
        if(option==1)
        {
            debugSaveImages = true;
            printf("SAVE IMAGES!\n");
        }
        return;
    }
    
    if(1==sscanf(arg,"mode=%d",&option))
    {
        
        mode = option;
        if(option==0)
        {
            printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
        }
        if(option==1)
        {
            printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        }
        if(option==2)
        {
            printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd=3;
        }
        return;
    }
    
    printf("could not parse argument \"%s\"!!!!\n", arg);
}

std::string type2str(int type) {
    std::string r;
    
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    
    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }
    
    r += "C";
    r += (chans+'0');
    
    return r;
}

- (void)processImage:(cv::Mat&)image
{
    cv::Mat gray_img, rgb_img;
    cv::cvtColor(image, gray_img, CV_BGR2GRAY);
    cv::cvtColor(image, rgb_img, CV_BGRA2BGR);
//    std::cout << type2str( image.type() ) << std::endl;
    assert(gray_img.type() == CV_8U);
    {
        std::lock_guard<std::mutex> lock_guard(mtx);
        memcpy(img_data, gray_img.data, gray_img.rows * gray_img.cols);
        memcpy(bgr_img_data, rgb_img.data, rgb_img.rows * rgb_img.cols * 3);
    }
    // Do some OpenCV stuff with the image
}
- (IBAction)onStart:(id)sender {
    [self.videoCamera start];
    started = true;
}

- (IBAction)onToggle:(id)sender {
    if (!_imageView.hidden) {
        _imageView.hidden = !_imageView.hidden;
        _depthView.hidden = !_depthView.hidden;
    } else if (!_depthView.hidden) {
        _depthView.hidden = !_depthView.hidden;
        _scnView.hidden = !_scnView.hidden;
    } else {
        _scnView.hidden = !_scnView.hidden;
        _imageView.hidden = !_imageView.hidden;
    }

}

- (void)viewDidLoad {
    [super viewDidLoad];
    _depthView.hidden = true;
    _scnView.hidden = true;
    
    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:_imageView];
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationLandscapeLeft;
    self.videoCamera.defaultFPS = 30;
    self.videoCamera.grayscaleMode = false;
    self.videoCamera.delegate = self;
    
    // SceneView
    _scene = [[SCNScene alloc] init];
    
    // create and add a camera to the scene
    SCNNode *cameraNode = [[SCNNode alloc] init];
    cameraNode.camera = [[SCNCamera alloc] init];
    cameraNode.camera.zNear = 0.0;
    cameraNode.camera.zFar = 100.0;
//    cameraNode.camera.fieldOfView = 120;
    [_scene.rootNode addChildNode:cameraNode];
    
    // place the camera
    cameraNode.position = SCNVector3Make(0, 0, 5.);
    
    // create and add a light to the scene
//    SCNNode *lightNode = [[SCNNode alloc] init];
//    lightNode.light = [[SCNLight alloc] init];
//    lightNode.light.type = SCNLightTypeOmni;
//    lightNode.position = SCNVector3Make(0, 3, 3);
//    [_scene.rootNode addChildNode:lightNode];
    
    // create and add an ambient light to the scene
    SCNNode *ambientLightNode = [[SCNNode alloc] init];
    ambientLightNode.light = [[SCNLight alloc] init];
    ambientLightNode.light.type = SCNLightTypeAmbient;
    ambientLightNode.light.color = UIColor.whiteColor;
    [_scene.rootNode addChildNode:ambientLightNode];
    
//    _lookat = [[SCNNode alloc] init];
//    _lookat.position = SCNVector3Make(0, 0, 1.);
//    cameraNode.constraints = @[[SCNLookAtConstraint lookAtConstraintWithTarget:_lookat]];
    
    // set the scene to the view
    _scnView.scene = _scene;
    
    // allows the user to manipulate the camera
    _scnView.allowsCameraControl = true;
    
    // show statistics such as fps and timing information
    _scnView.showsStatistics = true;
    
    // configure the view
    _scnView.backgroundColor = UIColor.blackColor;
    
    
//    NSString *filePath = [[NSBundle mainBundle] pathForResource:@"images" ofType:@""];
//    filePath = [@"files=" stringByAppendingString:filePath];
    NSString *filePath = @"files=";
    std::string image_fn = [filePath cStringUsingEncoding: NSUTF8StringEncoding];
    const char *image_option = image_fn.c_str();
    
    filePath = [[NSBundle mainBundle] pathForResource:@"camera" ofType:@"txt"];
    filePath = [@"calib=" stringByAppendingString:filePath];
    std::string calib_fn = [filePath cStringUsingEncoding: NSUTF8StringEncoding];
    const char *calib_option = calib_fn.c_str();
    
    filePath = [[NSBundle mainBundle] pathForResource:@"pcalib" ofType:@"txt"];
    filePath = [@"gamma=" stringByAppendingString:filePath];
    std::string gamma_fn = [filePath cStringUsingEncoding: NSUTF8StringEncoding];
    const char *gamma_option = gamma_fn.c_str();
    
    filePath = [[NSBundle mainBundle] pathForResource:@"vignette" ofType:@"png"];
    filePath = [@"vignette=" stringByAppendingString:filePath];
    std::string vignette_fn = [filePath cStringUsingEncoding: NSUTF8StringEncoding];
    const char *vignette_option = vignette_fn.c_str();
    
    int argc = 10;
    const char *argv[] = {(const char *)"dummy", image_option, calib_option, (const char *)"gamma=", (const char *)"vignette=", (const char *)"mode=1", (const char *)"preset=1", (const char *)"nogui=1", (const char *)"nolog=1", (const char *)"quiet=1"};
    
    // Do any additional setup after loading the view, typically from a nib.
    //setlocale(LC_ALL, "");
    for(int i=1; i<argc;i++)
        parseArgument(argv[i]);


    // to make MacOS happy: run this in dedicated thread -- and use this one to run the GUI.
    dispatch_async( dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        ImageFolderReader* reader = new ImageFolderReader(source,calib, gammaCalib, vignette);
        reader->setGlobalCalibration();
        
        
        
        if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
        {
            printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
            exit(1);
        }
        
        int lstart=start;
        int lend = end;
        int linc = 1;
        if(reverse)
        {
            printf("REVERSE!!!!");
            lstart=end-1;
            if(lstart >= reader->getNumImages())
                lstart = reader->getNumImages()-1;
            lend = start;
            linc = -1;
        }
        
        img_data = new char[wG[0] * hG[0]];
        bgr_img_data = new char[wG[0] * hG[0] * 3];
        while (true) {
            FullSystem* fullSystem = new FullSystem();
            fullSystem->setGammaFunction(reader->getPhotometricGamma());
            fullSystem->linearizeOperation = (playbackSpeed==0);
            
            auto viewer = new MyOutputWrapper(wG[0], hG[0], self);
            fullSystem->outputWrapper.push_back(viewer);
            
            Undistort *undistort = Undistort::getUndistorterForFile(calib, gammaCalib, vignette);
            
            int i = 0;
            while (true) {
                //            printf("%d\n", started);
                if (!started) {
                    sleep(1);
                    continue;
                }
                MinimalImageB* img = new MinimalImageB(wG[0], hG[0]);
                MinimalImageB3* bgr_img = new MinimalImageB3(wG[0], hG[0]);
                {
                    std::lock_guard<std::mutex> lock_guard(mtx);
                    memcpy(img->data, img_data, wG[0]*hG[0]);
                    memcpy(bgr_img->data, bgr_img_data, wG[0]*hG[0]*3);
                }
                ImageAndExposure* frame = undistort->undistort<unsigned char>(img, 1.f, 0.f);
                delete img;
                fullSystem->addActiveFrame(frame, bgr_img,  i++);
                delete frame;
                
                if(fullSystem->isLost)
                {
                    printf("LOST!!\n");
                    break;
                }
            }
            
            //        fullSystem->blockUntilMappingIsFinished();
            
            delete viewer;
            
            printf("DELETE FULLSYSTEM!\n");
            delete fullSystem;
        }
    });
}




@end
