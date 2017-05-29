#include "opencv2/opencv.hpp"  
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <iostream>  
  
using namespace cv;  
using namespace std;  
  
  
  
int main(int, char**)  
{  
    
    Rect roi1, roi2;
    Mat Q;
    StereoBM bm;
    float scale = 1.f;
    int SADWindowSize = 0, numberOfDisparities = 0;

  // reading intrinsic parameters
        FileStorage fs("intrinsics.yml", CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            //printf("Failed to open file %s\n", "intrinsics.yml");
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open("extrinsics.yml", CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            //printf("Failed to open file %s\n", "extrinsics.yml");
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;




    VideoCapture cap1(0);  
    cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    if (!cap1.isOpened())  
    {  
  //      printf("Can't Open First Camera. \n");  
    }

    VideoCapture cap2(2);  
    cap2.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap2.isOpened())  
    {  
  //      printf("Can't Open First Camera. \n");  
    }

    Mat frame1, frame2, disparity;  

    namedWindow("camera1", 1);  
    namedWindow("camera2", 1);  
    namedWindow("disparity", 1);  
       
    Mat img1, img2;        
    
    Mat disp, disp8;

    for (;;)  
    {  

  
        //웹캡으로부터 한 프레임을 읽어옴  
        cap1 >> frame1;  
        cap2 >> frame2;  
        cap2 >> disparity;  

        frame1.convertTo(img1, CV_8U);
        frame2.convertTo(img2, CV_8U);

        cvtColor(img1,img1,CV_8U);
        cvtColor(img2,img2,CV_8U);

        if (scale != 1.f)
        {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
        }

        Size img_size = img1.size();


        //cout << " img_size.width = " << img_size.width << endl;


        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;


        Mat image1, image2; 

        img1.convertTo(image1, CV_8U);
        img2.convertTo(image2, CV_8U);


        cvtColor(img1,image1,COLOR_BGR2GRAY);
        cvtColor(img2,image2,COLOR_BGR2GRAY);

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;


//cout << " image_type "  << image1.type() << endl;


        bm(image1, image2, disp);


        imshow("camera1", image1);  
        imshow("camera2", image2);  
        imshow("disparity", disp);  
  
        //  
        if (waitKey(20) >= 0) break;  
    }  
  
  
    return 0;  
}  
