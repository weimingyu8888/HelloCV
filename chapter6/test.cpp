#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void getContours(Mat input,Mat output,int tpye)
{
    vector<vector<Point> >contours;
    vector<Vec4i> hierarchy;
    
    findContours(input,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > Poly(contours.size());
    vector<Rect> bdRect(contours.size());
    
    for(int i=0;i<contours.size();i++)
    {
        int S=contourArea(contours[i]);
        if(S>30000)
        {
            double peri =arcLength(contours[i],1);
            approxPolyDP(contours[i],Poly[i],0.02*peri,1);
            bdRect[i]=boundingRect(Poly[i]);
            if(tpye==1)
            {
                putText(output,"Green",Point(10,50),FONT_HERSHEY_COMPLEX,1.5,Scalar(0,255,0),3);
                rectangle(output,bdRect[i].tl(),bdRect[i].br(),Scalar(0,0,255),3);
            }
            else 
            {
                putText(output,"Red",Point(10,50),FONT_HERSHEY_COMPLEX,1.5,Scalar(0,0,255),3);
                rectangle(output,bdRect[i].tl(),bdRect[i].br(),Scalar(0,255,0),3);
            }
        }
    }
}

int main()
{

    VideoCapture cap("22/TrafficLight.mp4");  
    if(!cap.isOpened()) {
        cout << "无法打开视频文件！请检查文件路径" << endl;
        return -1;
    }
    
 
    double fps = cap.get(CAP_PROP_FPS);
    if(fps <= 0) fps = 30.0;  
    
    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    
    cout << "视频信息: " << width << "x" << height << " FPS:" << fps << endl;
    

    VideoWriter wt("output.avi", VideoWriter::fourcc('M','J','P','G'), fps, Size(width, height));
    if(!wt.isOpened()) {
        cout << "无法创建输出视频文件！" << endl;
        return -1;
    }
    
    while(1)
    {
        Mat ori,img_green,img_red;
        cap.read(ori);
        if(ori.empty()) break;
        
        inRange(ori,Scalar(100,129,0),Scalar(207,247,80),img_green);
        inRange(ori,Scalar(13,14,131),Scalar(116,120,255),img_red);

        GaussianBlur(img_green,img_green,Size(5,5),0,0);
        GaussianBlur(img_red,img_red,Size(5,5),0,0);
        Canny(img_green,img_green,100,200);
        Canny(img_red,img_red,100,200);
        Mat K=getStructuringElement(MORPH_RECT,Size(5,5));
        dilate(img_green,img_green,K);
        erode(img_green,img_green,K);
        dilate(img_red,img_red,K);
        erode(img_red,img_red,K);
        
        getContours(img_green,ori,1);
        getContours(img_red,ori,2);
        
        imshow("Ori",ori);
        wt.write(ori);
        
        if(waitKey(1)>=0) break;
    }
    
    cap.release();
    wt.release();
    cout << "处理完成" << endl;
    return 0;
}
