//
// Created by wsc on 2022/7/19.
//

#include "HeadFile.h"

void get_ORBMatchNums(Mat &InputImage,Mat &TemplateDescriptor,int &matchNums)
{
    Mat InputGrayImage;

    vector<KeyPoint> InputKeyPoints,DstKeyPoints;
    Mat InputDescriptors;

    if(InputImage.channels()!=1)
    {
        cvtColor(InputImage,InputGrayImage,COLOR_BGR2GRAY);
    }
    else
    {
        InputGrayImage=InputImage;
    }


    Ptr<ORB> featureDetector=ORB::create();
    Ptr<DescriptorExtractor> featureExtractor=ORB::create();

    featureDetector->detect(InputGrayImage,InputKeyPoints);

    featureExtractor->compute(InputGrayImage,InputKeyPoints,InputDescriptors);

    if(!InputDescriptors.empty())
    {
        flann::Index flannIndex(InputDescriptors, flann::LshIndexParams(12, 10, 2), cvflann::FLANN_DIST_HAMMING);
        Mat matchIndex(TemplateDescriptor.rows, 2, CV_32SC1), matchDistance(TemplateDescriptor.rows, 2, CV_32FC1);
        flannIndex.knnSearch(TemplateDescriptor, matchIndex, matchDistance, 2, flann::SearchParams());

        vector<DMatch> goodMatches;

        for (int i = 0; i < matchDistance.rows; i++)
        {
            if (matchDistance.at<float>(i, 0) < 0.6* matchDistance.at<float>(i, 1))
            {
                DMatch dmatches(i, matchIndex.at<int>(i, 0), matchDistance.at<float>(i, 0));
                goodMatches.push_back(dmatches);
            }
        }
        matchNums=goodMatches.size();
    }
    else
    {
        matchNums=0;
    }
}

void get_SURFMatchNums(Mat &InputImage,Mat &TemplateDescriptor,int &matchNums)
{
        Ptr<SurfFeatureDetector> featureDetector=SURF::create();
        featureDetector->setHessianThreshold(10);

        Ptr<SurfDescriptorExtractor> featureExtractor=SURF::create();

        FlannBasedMatcher matcher;
        vector<Mat> template_desc_collection(1,TemplateDescriptor);
        matcher.add(template_desc_collection);
        matcher.train();

        Mat gray_InputImage;
        cvtColor(InputImage,gray_InputImage,COLOR_BGR2GRAY);
        vector<KeyPoint> InputImage_keypoint;
        Mat InputImageDescriptor;
        featureDetector->detect(gray_InputImage,InputImage_keypoint);
        featureExtractor->compute(gray_InputImage,InputImage_keypoint,InputImageDescriptor);

        vector<vector<DMatch>> matches;
        matcher.knnMatch(InputImageDescriptor,matches,2);

        vector<DMatch> goodMatches;
        for(int i=0;i<matches.size();i++)
        {
            if(matches[i][0].distance<0.6*matches[i][1].distance)
            {
                goodMatches.push_back(matches[i][0]);
            }
        }

        matchNums=matches.size();
}