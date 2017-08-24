//
//  ViewController.m
//  UCSVQEDemo
//
//  Created by VintonLiu on 2017/8/16.
//  Copyright © 2017年 ucpaas. All rights reserved.
//

#import "ViewController.h"
#import "ucs_vqe.h"

@interface ViewController ()

@end

@implementation ViewController
@synthesize m_pLabelTips;
@synthesize m_pButtonStart;

+ (NSString*) GetTestFilePath:(NSString *)name {
    NSString* path = [[NSBundle mainBundle]pathForResource:name ofType:@"pcm"];
    NSLog(@"filepath = %@", path);
    return path;
}

+ (NSString*) GetOutputPath {
    NSArray* directoryPath = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString* doucumentDir = [directoryPath objectAtIndex:0];
    NSString* filepath = [doucumentDir stringByAppendingPathComponent:@"aec_out.pcm"];
    NSLog(@"GetOutputPath %@", filepath);
    
    NSFileManager* fileManager = [NSFileManager defaultManager];
    if (![fileManager fileExistsAtPath:filepath]) {
        [fileManager createFileAtPath:filepath contents:nil attributes:nil];
    }
    
    return filepath;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (IBAction)startVqeProc:(id)sender {
    NSLog(@"startVqeProc");
    int res = 0;
    char* nearPath = NULL;
    char* farPath = NULL;
    char* outputPath = NULL;
    FILE* nearfp = NULL;
    FILE* farfp = NULL;
    FILE* outfp = NULL;
    UcsVqeConfig config;
    const int kFrameLen = 80;
    short nearbuf[kFrameLen] = {0};
    short farbuf[kFrameLen] = { 0 };
    short outdata[kFrameLen] = { 0 };
    
    nearPath = (char*) [[ViewController GetTestFilePath:@"near2"] UTF8String];
    farPath = (char*) [[ViewController GetTestFilePath:@"far2"] UTF8String];
    outputPath = (char*) [[ViewController GetOutputPath] UTF8String];
    
    if (nearPath != NULL) {
        nearfp = fopen(nearPath, "rb");
    }
    if (farPath) {
        farfp = fopen(farPath, "rb");
    }
    if (outputPath) {
        outfp = fopen(outputPath, "wb");
    }
    
    if (nearfp == NULL ||
        farfp == NULL ||
        outfp == NULL) {
        m_pLabelTips.text = @"测试文件打开失败";
        return;
    }
    m_pLabelTips.text = @"已打开测试文件";
    NSLog(@"%@", m_pLabelTips.text);
    
    memset(&config, 0x00, sizeof(UcsVqeConfig));
    config.aec_enable = ucs_true;
    config.agc_enable = ucs_true;
    config.ns_enable = ucs_true;
    res = UCSVQE_Init(kUcsSampleRate8kHz, &config);
    if (res != 0) {
        m_pLabelTips.text = @"初始化失败";
        return;
    }
    m_pLabelTips.text = @"初始化成功";
    NSLog(@"%@", m_pLabelTips.text);
    
    while ((kFrameLen == fread(farbuf, 2, kFrameLen, farfp)) &&
           (kFrameLen == fread(nearbuf, 2, kFrameLen, nearfp))) {
        UCSVQE_FarendAnalysis(farbuf);
        res = UCSVQE_Process(nearbuf, 0, outdata);
        if (res == 0) {
            fwrite(outdata, kFrameLen, 2, outfp);
        } else {
            NSLog(@"UCSVQE_Process failed(%d).", res);
        }
    }
    UCSVQE_Closed();
    m_pLabelTips.text = @"处理完成";
}


@end
