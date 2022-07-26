#include "HeadFile.h"


int j=225;


int main() {

    int width=400;
    int high=400;

    Point2f src_point[4];
    Point2f obj_point[4];
    obj_point[0]=Point2f (0,0);
    obj_point[1]=Point2f (width,0);
    obj_point[2]=Point2f (0,high);
    obj_point[3]=Point2f (width,high);
#if 0
        Mat trainImage;
        vector<int> trainLabel;
        for(int i=0;i<3600;i++)
        {
            Mat src= imread("/home/wsc/cutnum/"+ to_string(i)+".png");
            //imwrite("/home/wsc/new/"+ to_string(i%16)+"/"+ to_string(i)+".png",src);
            Mat gray;
            cvtColor(src,gray,COLOR_BGR2GRAY);
            Mat temp,convert;
            resize(gray,temp,Size(30,30));
            temp.convertTo(convert,CV_32F,1.0);
            gray=convert.reshape(0,1);
            trainImage.push_back(gray);
            trainLabel.push_back(i%16);
        }
    Ptr<SVM> svm=SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);//核函数
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER,100,1e-6));
    svm->trainAuto(trainImage,ROW_SAMPLE,trainLabel);
  //svm->save("svm.xml");
  //svm->load("svm.xml");
#endif
    VideoCapture capture(0,CAP_V4L2);
    Mat frame,warp,rotate_img,hsv,binary,H;
    capture.set(CAP_PROP_FRAME_WIDTH,640);
    capture.set(CAP_PROP_FRAME_HEIGHT,480);
    for(int i=0;i<10;i++)
    {
        capture>>frame;
    }
    calibWarp(frame,src_point,obj_point,H);
    namedWindow("frame",0);
    namedWindow("warp",0);
#if 0
    Mat moban= imread("/home/wsc/autoThresholdnum/4.png");
    Mat gray_moban;
    vector<KeyPoint> keypoints;
    Mat descriptor;
    cvtColor(moban,gray_moban,COLOR_BGR2GRAY);
    Ptr<ORB> featureDetector=ORB::create();
    Ptr<DescriptorExtractor> featureExtractor=ORB::create();

    featureDetector->detect(gray_moban,keypoints);

    featureExtractor->compute(gray_moban,keypoints,descriptor);
#endif

//下面是数字识别部分，测试代码
#if 0
    while(true)

    {
        #if 1
        double time0=getTickCount();
        capture>>frame;

        warpPerspective(frame,warp,H,Size(400,400));

        Rect cut=Rect(10,10,80,80);

                Mat temp0;
                rotate(warp,temp0,ROTATE_180);
                Mat temp=temp0(cut);
                Mat temp_gray;
                cvtColor(temp,temp_gray,COLOR_BGR2GRAY);
                Mat temp_resize;
                resize(temp_gray,temp_resize,Size(30,30));
                Mat convert;
                temp_resize.convertTo(convert,CV_32F);
                temp=convert.reshape(0,1);
                Mat temp_binary;
                //LabBinary(temp,temp_binary);
                int matchnum=0;
               cout<<svm->predict(temp)<<endl;

      imshow("frame",frame);
//
        imshow("warp",temp0);
//

        if(waitKey(50)==32)
        {
            cout<<j<<endl;
            imwrite("/home/wsc/moniWarp/"+ to_string(j)+".png",temp0);
            j++;
        } else if(waitKey(20)==65)
        {
            calibWarp(frame,src_point,obj_point,H);
        }

    }
        #endif
#endif

//下面是识别障碍区域的代码
#if 0

    while (true)
    {
        vector<int> obstacle;
        capture>>frame;
        warpPerspective(frame,warp,H,Size(400,400));
        Mat rotate_warp;
        rotate(warp,rotate_warp,ROTATE_180);
        Mat lab;
        cvtColor(rotate_warp,lab,COLOR_BGR2Lab);
        vector<Mat> channels;
        split(lab,channels);
        Mat Threshold;
        threshold(channels.at(0),Threshold,128,255,THRESH_BINARY);
        //cout<<Threshold.size<<endl;
        Mat cut_num[16];
        cut4num(Threshold,cut_num);
        get_obstacle(cut_num,obstacle);
        //cout<<obstacle.size()<<endl;
        for(auto &num:obstacle)
        {
            cout<<num<<endl;
        }
        imshow("warp",Threshold);
        if(waitKey(10)==27)
        {
            break;
        }


    }

#endif

while(true)
{

        Mat frame,warp,num16[16];
        capture>>frame;
    warpPerspective(frame,warp,H,Size(400,400));
    cut4num(warp,num16);
    vector<int> ons;
    Mat temp;
    LabBinary(num16[5],temp);
    imshow("frame",num16[5]);
    imshow("warp",temp);
    get_obstacle(num16,ons);
    for(auto &x:ons)
    {
        cout<<x<<endl;
    }
    if(waitKey(1)==17)
    {
        break;
    }

}

    return 0;
}
