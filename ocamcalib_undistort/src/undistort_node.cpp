#include "ocamcalib_undistort/ocam_functions.h"
#include "ocamcalib_undistort/Parameters.h"

#include <iostream>
#include <string>
#include <exception>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/opencv.hpp"
#include <ocamcalib_undistort/PhantomVisionNetMsg.h>

#include <pcl_ros/point_cloud.h>

using namespace cv;
using namespace std;

ros::Publisher Pub_image_center;
ros::Publisher Pub_image_rear;
ros::Publisher Pub_image_left;
ros::Publisher Pub_image_right;
ros::Publisher Pub_AVM_image;

cv::Mat AVM_left = cv::Mat::zeros(400,400, CV_8UC3);// = cv::Mat::zeros(450,450, CV_8UC3);
cv::Mat AVM_right = cv::Mat::zeros(400,400, CV_8UC3);// = cv::Mat::zeros(450,450, CV_8UC3);
cv::Mat AVM_front = cv::Mat::zeros(400,400, CV_8UC3);
cv::Mat AVM_rear = cv::Mat::zeros(400,400, CV_8UC3);

cv::Mat AVM_seg_front= cv::Mat::zeros(400,400, CV_8UC3);
cv::Mat AVM_seg_rear= cv::Mat::zeros(400,400, CV_8UC3);
cv::Mat AVM_seg_left= cv::Mat::zeros(400,400, CV_8UC3);
cv::Mat AVM_seg_right= cv::Mat::zeros(400,400, CV_8UC3);

cv::Mat ORG_image_front= cv::Mat::zeros(320, 200, CV_8UC3); 
cv::Mat ORG_image_rear= cv::Mat::zeros(320, 200, CV_8UC3); 
cv::Mat ORG_image_left= cv::Mat::zeros(320, 200, CV_8UC3); 
cv::Mat ORG_image_right= cv::Mat::zeros(320, 200, CV_8UC3); 

cv::Mat result = cv::Mat::zeros(1200, 400, CV_8UC3);
cv::Mat result_seg = cv::Mat::zeros(1200, 400, CV_8UC3);
cv::Mat temp = cv::Mat::zeros(1200, 800, CV_8UC3);
cv::Mat temp2 = cv::Mat::zeros(1520, 800, CV_8UC3);

cv::Mat ORG = cv::Mat::zeros(320, 200, CV_8UC3);
cv::Mat ORG2= cv::Mat::zeros(320, 200, CV_8UC3);

cv::Mat result1 = cv::Mat::zeros(400, 400, CV_8UC3);
cv::Mat result2 = cv::Mat::zeros(400, 400 , CV_8UC3);
cv::Mat result3 = cv::Mat::zeros(400, 400 , CV_8UC3); 
cv::Mat result4 = cv::Mat::zeros(400, 400 , CV_8UC3); 
cv::Mat result5 = cv::Mat::zeros(400, 400 , CV_8UC3); 
cv::Mat result6 = cv::Mat::zeros(400, 400 , CV_8UC3); 

ocam_model front_model;
ocam_model  left_model;
ocam_model right_model;
ocam_model  rear_model;

int flag=0;
double start;
double endd;

#define M_DEG2RAD  3.1415926 / 180.0
double M_front_param[6] = {0.688 * M_DEG2RAD,  21.631 * M_DEG2RAD,   3.103* M_DEG2RAD   ,1.905,   0.033, 0.707 };
double M_left_param[6] =  {1.133 * M_DEG2RAD,  19.535 * M_DEG2RAD,   92.160* M_DEG2RAD  ,0.0,     1.034, 0.974 };
double M_right_param[6] = {3.440 * M_DEG2RAD,  18.273 * M_DEG2RAD,  -86.127* M_DEG2RAD  ,0.0,    -1.034, 0.988 };
double M_back_param[6] =  {0.752 * M_DEG2RAD,  31.238 * M_DEG2RAD,  -178.189* M_DEG2RAD ,-2.973, -0.065, 0.883 };

Parameters getParameters(ros::NodeHandle& nh)
{
    Parameters params;

    nh.param<std::string>("camera_type", params.cameraType, "fisheye");
    // nh.param<std::string>("base_in_topic", params.inTopic, 'camera/image_center');
    // nh.param<std::string>("base_out_topic", params.outTopic, "/ocamcalib_undistorted");
    nh.param<std::string>("calibration_file_path", params.calibrationFile, "include/calib_results_left.txt"); //center.txt");

    nh.param<std::string>("transport_hint", params.transportHint, "raw");
    nh.param<double>("scale_factor", params.scaleFactor, 15);
    nh.param<int>("left_bound", params.leftBound,   0);
    nh.param<int>("right_bound", params.rightBound, 0);
    nh.param<int>("top_bound", params.topBound, 0);
    nh.param<int>("bottom_bound", params.bottomBound, 0);

    return params;
}
void printOcamModel(const struct ocam_model& model)
{
    std::cout << "OCamCalib model parameters" << std::endl
              << "pol: " << std::endl;
    for (int i=0; i < model.length_pol; i++)
    {
        std::cout << "\t" << model.pol[i] << "\n";
    }

    std::cout << "\ninvpol: " << std::endl;
    for (int i=0; i < model.length_invpol; i++)
    {
        std::cout << "\t" << model.invpol[i] << "\n";
    };
    std::cout << std::endl;

    std::cout << "xc:\t" << model.xc << std::endl
              << "yc:\t" << model.yc << std::endl
              << "width:\t" << model.width << std::endl
              << "height:\t" << model.height << std::endl;
}

void seg2rgb(cv::Mat input_img, cv::Mat& output_img) {
    for (int i = 0 ; i < input_img.rows ; i++) {
        for (int j = 0 ; j < input_img.cols ; j++) {
            switch((int)input_img.at<uchar>(i, j)){
                case 1 : // vehicle
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 2 : // wheel
                    output_img.at<Vec3b>(i, j)[0] = 255;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 0;
                break;  
                case 3 : 
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 123;
                break;  
                case 4 : 
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 125;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 5 : 
                    output_img.at<Vec3b>(i, j)[0] = 255;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 6 : 
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 7 : // human
                    output_img.at<Vec3b>(i, j)[0] = 255;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 8 :    // road 
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 0;
                break;
                case 10 : 
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 165;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 11 : // cart?
                    output_img.at<Vec3b>(i, j)[0] = 193;
                    output_img.at<Vec3b>(i, j)[1] = 182;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                default :
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 0;
            }
        }
    }
}

void CallbackPhantom_seg_front(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg) {
    cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->image_seg, msg->image_seg.encoding )->image;
    seg2rgb(cv_frame_seg, cv_frame_raw_new);
    
    //resize
    cv::Mat cv_frame_resize;
    cv::resize( cv_frame_raw_new, cv_frame_resize, Size(1920, 1080), 0, 0, INTER_LINEAR );

    cv::Mat cv_frame_resize_pad;
    cv::copyMakeBorder(cv_frame_resize, cv_frame_resize_pad, 0, 128, 0, 0, BORDER_CONSTANT, Scalar(0,0,0) );

    XY_coord xy;
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // imshow("test",cv_frame_resize_pad);                                                              //
    // waitKey(1);                                                                                      //
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    int u,v;
        AVM_seg_front = cv::Mat::zeros(400,400, CV_8UC3);
        for(int i=0; i< (cv_frame_resize_pad.size().height)  ;i++)
            for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){
                xy = InvProjGRD(4*j,4*i, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);
                // xy = InvProjGRD(3*j,3*i, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);
                // xy = InvProjGRD(2*j,2*i, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);
                // xy = InvProjGRD(j,i, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);


                if(xy.x < 0){
                    xy.y =0;
                    xy.x =0;
                }
                if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                    v = 200 - 20*xy.x;  // width
                    u = 200 - 20*xy.y;  // height

                    AVM_seg_front.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                    AVM_seg_front.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                    AVM_seg_front.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r
                }
            }
}
void CallbackPhantom_seg_rear(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg){
    cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->image_seg, msg->image_seg.encoding )->image;
    seg2rgb(cv_frame_seg, cv_frame_raw_new);
    
    //resize
    cv::Mat cv_frame_resize;
    cv::resize( cv_frame_raw_new, cv_frame_resize, Size(1920, 1080), 0, 0, INTER_LINEAR );

    cv::Mat cv_frame_resize_pad;
    cv::copyMakeBorder(cv_frame_resize, cv_frame_resize_pad, 0, 128, 0, 0, BORDER_CONSTANT, Scalar(0,0,0) );

    XY_coord xy;
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );
    int u,v;
    AVM_seg_rear = cv::Mat::zeros(400,400, CV_8UC3);
    for(int i=0; i< (cv_frame_resize_pad.size().height)  ;i++)
        for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){
            xy = InvProjGRD(4*j,4*i, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);
            // xy = InvProjGRD(3*j,3*i, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);
            // xy = InvProjGRD(2*j,2*i, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);
            // xy = InvProjGRD(j,i, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);


            if(xy.x > 0){
                xy.y =0;
                xy.x =0;
            }
            if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                v = 200 - 20*xy.x;  // width
                u = 200 - 20*xy.y;  // height
                
                AVM_seg_rear.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                AVM_seg_rear.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                AVM_seg_rear.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r
            }
        }
}
void CallbackPhantom_seg_left(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg){
    cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->image_seg, msg->image_seg.encoding )->image;
    seg2rgb(cv_frame_seg, cv_frame_raw_new);
    
    //resize
    cv::Mat cv_frame_resize;
    cv::resize( cv_frame_raw_new, cv_frame_resize, Size(1920, 1080), 0, 0, INTER_LINEAR );

    cv::Mat cv_frame_resize_pad;
    cv::copyMakeBorder(cv_frame_resize, cv_frame_resize_pad, 0, 128, 0, 0, BORDER_CONSTANT, Scalar(0,0,0) );

    XY_coord xy;
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // imshow("test",cv_frame_resize_pad);                                                              //
    // waitKey(1);                                                                                      //
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    int u,v;
        AVM_seg_left = cv::Mat::zeros(400,400, CV_8UC3);
        for(int i=0; i< (cv_frame_resize_pad.size().height)  ;i++)
            for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){
                xy = InvProjGRD(4*j,4*i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);
                // xy = InvProjGRD(3*j,3*i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);
                // xy = InvProjGRD(2*j,2*i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);
                // xy = InvProjGRD(j,i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);

                if(xy.y < 0){
                    xy.y =0;
                    xy.x =0;
                }
                if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                    v = 200 - 20*xy.x;  // width
                    u = 200 - 20*xy.y;  // height
                
                    AVM_seg_left.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                    AVM_seg_left.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                    AVM_seg_left.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r
                }
        }

}
void CallbackPhantom_seg_right(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg){
    cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->image_seg, msg->image_seg.encoding )->image;
    seg2rgb(cv_frame_seg, cv_frame_raw_new);
    
    //resize
    cv::Mat cv_frame_resize;
    cv::resize( cv_frame_raw_new, cv_frame_resize, Size(1920, 1080), 0, 0, INTER_LINEAR );

    cv::Mat cv_frame_resize_pad;
    cv::copyMakeBorder(cv_frame_resize, cv_frame_resize_pad, 0, 128, 0, 0, BORDER_CONSTANT, Scalar(0,0,0) );

    XY_coord xy;
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );

    int u,v;
        AVM_seg_right = cv::Mat::zeros(400,400, CV_8UC3);
        for(int i=0; i< (cv_frame_resize_pad.size().height) ;i++)
        for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){
            xy = InvProjGRD(4*j,4*i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);
            // xy = InvProjGRD(3*j,3*i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);
            // xy = InvProjGRD(2*j,2*i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);
            // xy = InvProjGRD(j,i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);

            if(xy.y > 0){
                xy.y =0;
                xy.x =0;
            }
            
            if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                v = 200 - 20*xy.x; 
                u = 200 - 20*xy.y; 

                AVM_seg_right.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                AVM_seg_right.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                AVM_seg_right.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r
            }

        }
}

void CallbackPhantom_center(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg) 
{   
    cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;
    //resize
    // cv::Mat cv_frame_resize;
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, ORG_image_front, cv::Size(320, 200), 0, 0, cv::INTER_LINEAR );
    
    // padding
    // cv::Mat cv_frame_resize_pad;
    cv::Mat temp;
    cv::copyMakeBorder(cv_frame_resize_pad, cv_frame_resize_pad, 0, 128, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    // sensor_msgs::ImagePtr msg_img_center = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_frame_resize_pad).toImageMsg();
    XY_coord xy; 
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );
    int u,v;

    string myText = "Front";
    cv::Point myPoint;
    myPoint.x = 10;
    myPoint.y = 20;
    cv::putText( ORG_image_front, myText, myPoint, 2, 0.7, Scalar(0, 255, 255) );

    AVM_front = cv::Mat::zeros(400,400, CV_8UC3);
    for(int i=0; i< (cv_frame_resize_pad.size().height)  ;i++)
        for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){
            
            // std::cout<<cv_frame_resize_pad.size().height<<cv_frame_resize_pad.size().width<<std::endl;
            xy = InvProjGRD(4*j,4*i, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);
            // xy = InvProjGRD(3*j+1,3*i+1, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);
            // xy = InvProjGRD(2*j,2*i, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);
            // xy = InvProjGRD(j,i, M_front_param[0], M_front_param[1], M_front_param[2], M_front_param[3], M_front_param[4] ,M_front_param[5], &front_model);

            // if(xy.x < 0){
            //     xy.y = 0;
            //     xy.x = 0;
            // }

            if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                v = 200 - 20*xy.x;  // width
                u = 200 - 20*xy.y;  // height

                // if ((AVM.at<cv::Vec3b>(int(v), int(u))[0] == 0) && (AVM.at<cv::Vec3b>(int(v), int(u))[1] == 0) && (AVM.at<cv::Vec3b>(int(v), int(u))[2] == 0)){
                AVM_front.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                AVM_front.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                AVM_front.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r
            }
        }
}
void CallbackPhantom_rear(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg) 
{
    cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;
    //resize
    // cv::Mat cv_frame_resize;
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, ORG_image_rear, cv::Size(320, 200), 0, 0, cv::INTER_LINEAR );

    //padding
    // cv::Mat cv_frame_resize_pad;
    cv::Mat temp;
    cv::copyMakeBorder(cv_frame_resize_pad, cv_frame_resize_pad, 0, 128, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    // sensor_msgs::ImagePtr msg_img_rear = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_frame_resize_pad).toImageMsg();

    XY_coord xy; 
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );

    int u,v;

    string myText = "Rear";
    cv::Point myPoint;
    myPoint.x = 10;
    myPoint.y = 20;
    cv::putText( ORG_image_rear, myText, myPoint, 2, 0.7,Scalar(0, 255, 255) );
    
    // double end = ros::Time::now().toSec();
    // std::cout<<end-start<<std::endl;
    AVM_rear = cv::Mat::zeros(400,400, CV_8UC3);

    for(int i=0; i< (cv_frame_resize_pad.size().height)  ;i++)
        for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){
            xy = InvProjGRD(4*j,4*i, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);
            // xy = InvProjGRD(3*j+1,3*i+1, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);            
            // xy = InvProjGRD(2*j,2*i, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);
            // xy = InvProjGRD(j,i, M_back_param[0], M_back_param[1], M_back_param[2], M_back_param[3], M_back_param[4] ,M_back_param[5], &rear_model);
          
            // if(xy.x > 0){
            //     xy.y =0;
            //     xy.x =0;
            // }
            if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                v = 200 - 20*xy.x;  // width
                u = 200 - 20*xy.y;  // height

                AVM_rear.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                AVM_rear.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                AVM_rear.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r

            }
        }
}
void CallbackPhantom_left(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg) 
{
    cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;
    
    // //resize
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, ORG_image_left, cv::Size(320, 200), 0, 0, cv::INTER_LINEAR );
    
    // //padding
    // cv::Mat cv_frame_resize_pad;
    cv::Mat temp;
    cv::Rect bounds(0,0,cv_frame_resize_pad.cols,cv_frame_resize_pad.rows);
    cv::Rect r(0,12,1920,1080 - 12);
    cv::Mat roi = cv_frame_resize_pad( r & bounds );
    cv::copyMakeBorder(roi, cv_frame_resize_pad, 0, 140, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );   

    XY_coord xy; 

    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );

    int u,v;

    string myText = "Left";
    cv::Point myPoint;
    myPoint.x = 10;
    myPoint.y = 20;
    cv::putText( ORG_image_left, myText, myPoint, 2, 0.7, Scalar(0, 255, 255));
    
    AVM_left = cv::Mat::zeros(400,400, CV_8UC3);
    for(int i=0; i< (cv_frame_resize_pad.size().height)  ;i++)
        for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){ 
            xy = InvProjGRD(4*j,4*i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);
            // xy = InvProjGRD(3*j,3*i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);
            // xy = InvProjGRD(2*j,2*i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);
            // xy = InvProjGRD(j,i, M_left_param[0], M_left_param[1], M_left_param[2], M_left_param[3], M_left_param[4] ,M_left_param[5], &left_model);

            if(xy.y < 0){
                xy.y =0;
                xy.x =0;
            }

            if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                v = 200 - 20*xy.x;  // width
                u = 200 - 20*xy.y;  // height
                // if ((AVM.at<cv::Vec3b>(int(v), int(u))[0] == 0) && (AVM.at<cv::Vec3b>(int(v), int(u))[1] == 0) && (AVM.at<cv::Vec3b>(int(v), int(u))[2] == 0)){
                AVM_left.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                AVM_left.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                AVM_left.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r
            }
        }
}
void CallbackPhantom_right(const ocamcalib_undistort::PhantomVisionNetMsg::ConstPtr& msg) 
{
    cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy( msg->image_raw, msg->image_raw.encoding )->image;

    //resize
    // cv::Mat cv_frame_resize;
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, ORG_image_right, cv::Size(320, 200), 0, 0, cv::INTER_LINEAR );
    
    //padding
    // cv::Mat cv_frame_resize_pad;
    cv::Mat temp;

    cv::Rect bounds(0,0,cv_frame_resize_pad.cols,cv_frame_resize_pad.rows);
    cv::Rect r(0,12,1920,1080 - 12);
    cv::Mat roi = cv_frame_resize_pad( r & bounds );
    cv::copyMakeBorder(roi, cv_frame_resize_pad, 0, 140, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );

    // sensor_msgs::ImagePtr msg_img_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_frame_resize_pad).toImageMsg();
    
    XY_coord xy; 
    
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(960, 604), 0, 0, cv::INTER_LINEAR );
    // cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(640, 402), 0, 0, cv::INTER_LINEAR );
    cv::resize( cv_frame_resize_pad, cv_frame_resize_pad, cv::Size(480, 302), 0, 0, cv::INTER_LINEAR );

    int u,v;

    string myText = "Right";
    cv::Point myPoint;
    myPoint.x = 10;
    myPoint.y = 20;
    cv::putText( ORG_image_right, myText, myPoint, 2, 0.7, Scalar(0, 255, 255) );
    int a = 0;

    AVM_right = cv::Mat::zeros(400,400, CV_8UC3);
    for(int i=0; i< (cv_frame_resize_pad.size().height) ;i++)
        for(int j=0 ;j< cv_frame_resize_pad.size().width ;j++){
            
            xy = InvProjGRD(4*j,4*i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);
            // xy = InvProjGRD(3*j,3*i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);
            // xy = InvProjGRD(2*j,2*i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);
            // xy = InvProjGRD(j,i, M_right_param[0], M_right_param[1], M_right_param[2], M_right_param[3], M_right_param[4] ,M_right_param[5], &right_model);

            if(xy.y > 0){
                xy.y =0;
                xy.x =0;
            }
            
            if( !(xy.x ==0) && (xy.y ==0) || !((abs(xy.x) >=10) || (abs(xy.y) >=10)) ){
                v = 200 - 20*xy.x; 
                u = 200 - 20*xy.y; 

                AVM_right.at<cv::Vec3b>(int(v), int(u))[0] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[0]);   //b
                AVM_right.at<cv::Vec3b>(int(v), int(u))[1] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[1]);   //g
                AVM_right.at<cv::Vec3b>(int(v), int(u))[2] = static_cast<uint8_t>(cv_frame_resize_pad.at<cv::Vec3b>(i,j)[2]);   //r
            }
        }
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "undistort_node");
    ros::NodeHandle nodeHandle("~");
    auto params = getParameters(nodeHandle); 

    std::string calibration_front ="/home/dyros-vehicle/catkin_ws/src/ocamcalib_undistort/include/calib_results_phantom_190_028_front.txt";
    std::string calibration_left = "/home/dyros-vehicle/catkin_ws/src/ocamcalib_undistort/include/calib_results_phantom_190_022_left.txt";
    std::string calibration_right ="/home/dyros-vehicle/catkin_ws/src/ocamcalib_undistort/include/calib_results_phantom_190_023_right.txt";
    std::string calibration_rear = "/home/dyros-vehicle/catkin_ws/src/ocamcalib_undistort/include/calib_results_phantom_190_029_rear.txt" ;        

    if(flag == 0)
    {
        if(!get_ocam_model(&front_model, calibration_front.c_str()))
        {
            return 2;
        }
        if(!get_ocam_model(&left_model, calibration_left.c_str()))
        {
            return 2;
        }
        if(!get_ocam_model(&right_model, calibration_right.c_str()))
        {
            return 2;
        }
        if(!get_ocam_model(&rear_model, calibration_rear.c_str()))
        {
            return 2;
        }
        flag =1;
    }

    ros::Subscriber Sub_phantom_front_center_svm = nodeHandle.subscribe("/phantomnet/output/front_center_svm", 1 , CallbackPhantom_center);
    ros::Subscriber Sub_phantom_rear_center_svm = nodeHandle.subscribe("/phantomnet/output/rear_center_svm", 1, CallbackPhantom_rear);
    ros::Subscriber Sub_phantom_side_left = nodeHandle.subscribe("/phantomnet/output/side_left", 1, CallbackPhantom_left);
    ros::Subscriber Sub_phantom_side_right = nodeHandle.subscribe("/phantomnet/output/side_right", 1, CallbackPhantom_right);

    ros::Subscriber Sub_phantom_front_seg = nodeHandle.subscribe("/phantomnet/output/front_center_svm", 1 , CallbackPhantom_seg_front);
    ros::Subscriber Sub_phantom_rear_seg = nodeHandle.subscribe("/phantomnet/output/rear_center_svm", 1 , CallbackPhantom_seg_rear);
    ros::Subscriber Sub_phantom_left_seg = nodeHandle.subscribe("/phantomnet/output/side_left", 1 , CallbackPhantom_seg_left);
    ros::Subscriber Sub_phantom_right_seg = nodeHandle.subscribe("/phantomnet/output/side_right", 1 , CallbackPhantom_seg_right);


    Pub_AVM_image = nodeHandle.advertise<sensor_msgs::Image>("/AVM_image", 1);
    
    // Pub_image = nodeHandle.advertise<sensor_msgs::Image>("/ocamcalib_undistorted_front", 1);
    // Pub_image_center = nodeHandle.advertise<sensor_msgs::Image>("/ocamcalib_undistorted_front", 1);
    // Pub_image_rear = nodeHandle.advertise<sensor_msgs::Image>("/ocamcalib_undistorted_rear", 1);
    // Pub_image_left = nodeHandle.advertise<sensor_msgs::Image>("/ocamcalib_undistorted_left", 1);
    // Pub_image_right = nodeHandle.advertise<sensor_msgs::Image>("/ocamcalib_undistorted_right", 1);

    // Pub_cam_left = nodeHandle.advertise<VPointImage>("grid_data_cam_left", 1);
    // Pub_cam_right = nodeHandle.advertise<VPointImage>("grid_data_cam_right", 1);

    // ros::spin();
    // ros::Rate loop_rate(10);
    while(ros::ok())        //half -> 0.03
    {
        ros::AsyncSpinner spinner(8);
        spinner.start();

        result1 = AVM_front + AVM_right + AVM_left + AVM_rear;
        result2 = AVM_front + AVM_rear;
        result3 = AVM_right + AVM_left;
        rectangle(result1, Point(179, 161.9), Point(220.68, 259.46), Scalar(0, 0, 255), 2);
        rectangle(result2, Point(179, 161.9), Point(220.68, 259.46), Scalar(0, 0, 255), 2);
        rectangle(result3, Point(179, 161.9), Point(220.68, 259.46), Scalar(0, 0, 255), 2);
        cv::hconcat(result1, result2, result1);
        cv::hconcat(result1, result3, result1);

        result4 = AVM_seg_front + AVM_seg_right + AVM_seg_left + AVM_seg_rear;
        result5 = AVM_seg_front + AVM_seg_rear;
        result6 = AVM_seg_right + AVM_seg_left;

        cv::hconcat(result4, result5, result4);
        cv::hconcat(result4, result6, result4);


        cv::vconcat(result1,result4,result1);

        //org
        cv::vconcat(ORG_image_front, ORG_image_rear,  ORG);
        cv::vconcat(ORG, ORG_image_left,  ORG);
        cv::vconcat(ORG, ORG_image_right, ORG);

        if (ORG.size().height == result1.size().height) {
            cv::hconcat(ORG, result1,  temp2);
            cv::imshow("temp",temp2);
            cv::waitKey(1);
        }
    }

    return 0;
}
