//
//  ViewController.h
//  UCSVQEDemo
//
//  Created by VintonLiu on 2017/8/16.
//  Copyright © 2017年 ucpaas. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface ViewController : UIViewController
@property (retain, nonatomic) IBOutlet UILabel* m_pLabelTips;
@property (retain, nonatomic) IBOutlet UIButton* m_pButtonStart;

+ (NSString*) GetTestFilePath: (NSString*)name;
+ (NSString*) GetOutputPath;

@end

