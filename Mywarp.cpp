//
// Created by wsc on 2022/7/19.
//
#include "HeadFile.h"

void mywarpPerspective(Mat &srcImage,Mat &objImage)
{

    int width=400;
    int high=400;

    Point2f src_point[4];
    src_point[0]=Point2f (430,200);
    src_point[1]=Point2f (850,190);
    src_point[2]=Point2f (280,610);
    src_point[3]=Point2f (1000,610);

    Point2f obj_point[4];
    obj_point[0]=Point2f (0,0);
    obj_point[1]=Point2f (width,0);
    obj_point[2]=Point2f (0,high);
    obj_point[3]=Point2f (width,high);
    Mat H=getPerspectiveTransform(src_point,obj_point);
    //cout<<H<<endl;
    warpPerspective(srcImage,objImage,H,Size(400,400));

}

void GetCorners(Mat &InputImage,Point2f Corners[])
{
    Mat img_gray;
    vector<Vec2f> temp_corners;
    Size winSize = Size( 5, 5 );
    Size zeroZone = Size( -1, -1 );
    //迭代终止条件
    TermCriteria criteria = TermCriteria( TermCriteria::Type::EPS+TermCriteria::Type::MAX_ITER,40,0.001 );

    cvtColor(InputImage,img_gray,COLOR_BGR2GRAY);//转为灰度图

    goodFeaturesToTrack(img_gray,temp_corners,20,0.08,200,noArray(),5, false,0.1);//初步检测角点

    //cornerSubPix( img_gray, temp_corners, winSize, zeroZone, criteria );//对初步检测到的角点进行精细化检测

    for(int i=0;i<temp_corners.size();i++)
    {
        Point2f center;
        if(temp_corners[i][0]<30&&temp_corners[i][1]<30)
        {
            Corners[0]=Point2f (temp_corners[i][0],temp_corners[i][1]);
            //cout<<"corners 0 :"<<Corners[0]<<endl;
            center=Point2f (temp_corners[i][0],temp_corners[i][1]);
            circle(InputImage,center,10,Scalar(255,0,0),-1);
        }
        else if(temp_corners[i][0]>200&&temp_corners[i][1]<30)
        {
            Corners[1]=Point2f (temp_corners[i][0],temp_corners[i][1]);
            //cout<<"corners 1 :"<<Corners[1]<<endl;
            center=Point2f (temp_corners[i][0],temp_corners[i][1]);
            circle(InputImage,center,10,Scalar(0,255,0),-1);
        }
        else if(temp_corners[i][0]<30&&temp_corners[i][1]>250)
        {
            Corners[2]=Point2f (temp_corners[i][0],temp_corners[i][1]);
            //cout<<"corners 2 :"<<Corners[2]<<endl;
            center=Point2f (temp_corners[i][0],temp_corners[i][1]);
            circle(InputImage,center,10,Scalar(0,0,255),-1);
        }
        else if(temp_corners[i][0]>200&temp_corners[i][1]>250)
        {
            Corners[3]=Point2f (temp_corners[i][0],temp_corners[i][1]);
            //cout<<"corners 3 :"<<Corners[3]<<endl;
            center=Point2f (temp_corners[i][0],temp_corners[i][1]);
            circle(InputImage,center,10,Scalar(0,0,0),-1);
        }
        else
        {
            ;
        }
    }
}

void calibWarp(Mat &src,Point2f src_point[],Point2f obj_point[],Mat &H)
{
    Mat hsv,binary;
    cvtColor(src,hsv,COLOR_BGR2HSV);
    inRange(hsv,Scalar(35,80,90),Scalar(77,255,255),binary);
    vector<vector<Point>> conturs;
    vector<Vec4i> hierarchy;
    Point2f center;
    float radius;
    findContours(binary,conturs,hierarchy,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if(!conturs.empty())
    {
        for(auto &contour:conturs)
        {
            minEnclosingCircle(contour,center,radius);
            //cout<<radius<<endl;
            if(radius>=8){
                //circle(frame,center,radius,Scalar(0,0,255),3);
                cout<<radius<<endl;
                if(center.y<240){
                    if(center.x<320){
                        src_point[0]=center;
                    }
                    else{
                        src_point[1]=center;
                    }
                }
                else{
                    if(center.x<320){
                        src_point[2]=center;
                    }
                    else{
                        src_point[3]=center;
                    }
                }
            }
        }
    }
    H=getPerspectiveTransform(src_point,obj_point);

}

void toBinary(Mat &src,Mat &dst)
{
    Mat gray;
    cvtColor(src,gray,COLOR_BGR2GRAY);
    threshold(gray,dst,170,255,THRESH_BINARY);
}

void LabBinary(Mat &src,Mat &dst)
{
    Mat Lab;
    vector<Mat> channel;
    cvtColor(src,Lab,COLOR_BGR2Lab);
    split(Lab,channel);
    adaptiveThreshold(channel.at(0),dst,255,ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 39, 19);
}

void cut4num(Mat &src,Mat dst[])
{
    Rect cut=Rect (10,10,80,80);
    for(int i=0;i<4;i++)
    {
        cut.y=10+i*100;
        for(int j=0;j<4;j++)
        {
            cut.x=10+j*100;
            dst[i*4+j]=src(cut);

        }
    }
}

void get_obstacle(Mat src[],vector<int> &Obstacle)
{
    Obstacle.clear();
    for(int i=0;i<16;i++)
    {
        int nums=0;
        Mat temp;
        LabBinary(src[i],temp);
        int rownum=temp.rows;
        int colnum=temp.cols;
        for(int j=0;j<rownum;j++)
        {
            uchar *pixel=src[i].ptr<uchar>(j);
            for(int k=0;k<colnum;k++)
            {
                if(pixel[k]<100)
                {
                    nums++;
                }
            }

        }
        //cout<<nums<<endl;
        if (nums>=5000)
        {
            Obstacle.push_back(i);
        }
    }
}

bool get_road(int &target,vector<int> &road,vector<int> &obstacle)
{
    if(target==0)
        return 0;
    vector<int> field;
    if(target%4!=0)
        field.push_back(target-1);
    if(target-4>=0)
        field.push_back(target-4);
    if(target%4!=3)
        field.push_back(target+1);
    if(target+4<=15)
        field.push_back(target+4);
    //建立目标点的领域
   for(vector<int>::iterator i=obstacle.begin();i!=obstacle.end();i++)
   {
       for(vector<int>::iterator j=field.begin();j!=field.end();j++)
       {
           if(*j==*i)
               field.erase(j);
       }
   }
   //删除不可达领域

   road.push_back(field[0]);
   obstacle.push_back(target);

    bool temp=get_road(field[0],road,obstacle);
    for(auto &roads:road)
    {
        cout<<roads<<endl;
    }
}