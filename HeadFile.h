//
// Created by wsc on 2022/7/19.
//

#ifndef SIMQUESTION_HEADFILE_H
#define SIMQUESTION_HEADFILE_H

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/ml.hpp"

using namespace std;
using namespace cv;
using namespace xfeatures2d;
using namespace ml;

void mywarpPerspective(Mat &srcImage,Mat &objImage);
void get_ORBMatchNums(Mat &InputImage,Mat &TemplateDescriptor,int &matchNums);
void get_SURFMatchNums(Mat &InputImage,Mat &TemplateDescriptor,int &matchNums);
void calibWarp(Mat &src,Point2f src_point[],Point2f obj_point[],Mat &H);
void LabBinary(Mat &src,Mat &dst);
void cut4num(Mat &src,Mat dst[]);
void get_obstacle(Mat src[],vector<int> &Obstacle);

#endif //SIMQUESTION_HEADFILE_H
