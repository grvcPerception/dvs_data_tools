#include<ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include<signal.h>

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

#include <vector>
#include <utility>      // std::pair, std::make_pair

#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

#include <fstream>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>

// Globla variables
ros::Publisher pubPerformance;
ros::Publisher pubDetectorInfo;

ros::Publisher corner_pub, tracker_pub;
image_transport::Publisher saePositivePub, saeNegativePub, clusterPub, imageClusterPub, pubBlobImage, counterPub;
sensor_msgs::ImagePtr sae_positive_msg, sae_negative_msg, image_cluster_msg, image_cluster_image_msg, blobImage_msg,counterMat_msg;

std::vector<std::vector<dvs_msgs::Event>> eventLeft, eventRight;
dvs_msgs::Event eLeft, eRight;
std::vector<cv::Mat> imageLeft, imageRight;
int counterLeft = 0, counterRight = 0;
bool tFlag = false;
double tZero = 0.0;
int limit = 5;

std::vector <double> timeR, timeL;

void shutdown(int s);

void callSync(){
    static int counter = 0;
    std::cout<<"sync, counter : "<<counter<<std::endl;
    if (counter == 1){
        std::cout<<"Ready to sync"<<std::endl;
        
        double boh = eventLeft[1][1].ts.toSec(); 
        eLeft = eventLeft[1][1]; 
        for(int jj = 0; jj<eventRight.size(); jj++){
            std::vector <dvs_msgs::Event> vecLeft = eventLeft[jj];
            for(int ll =0; ll<vecLeft.size(); ll++){
                //std::vector <dvs_msgs::Event> vecLeft = eventLeft[kk];
                for (int kk = 0; kk<eventRight.size(); kk++){
                    //std::vector <dvs_msgs::Event> vecLeft = eventLeft[kk];
                    std::vector <dvs_msgs::Event> vecRight = eventRight[kk];

                    // std::cout<<"LEFT"<<std::endl;
                    // for(int ii = 0; ii<vecLeft.size(); ii++)
                    //     std::cout<<"event Left : "<<vecLeft[ii].ts.toSec()-tZero<<std::endl;
                    //std::cout<<"RIGHT"<<std::endl;
                    for(int ii = 0; ii<vecRight.size(); ii++){
                        if(vecRight[ii].ts.toSec()-vecLeft[ll].ts.toSec() <= 0.00005 && vecRight[ii].ts.toSec()-vecLeft[ll].ts.toSec()>= -0.00005)
                            std::cout<<"Data Association"<<std::endl;
                        //double aux = vecRight[ii].ts.toSec()-boh;
                        //std::cout<<"event Right : "<<aux<<std::endl;
                        // if(aux <= 0.0003 && aux>= -0.0003){
                        //     std::cout<<" Found "<<std::endl;
                        //     std::cout<<"event Right : "<<aux<<std::endl;
                        //     eRight = vecRight[ii]; 
                        // }

                    } 
                }
            }
        }
        // cv::circle(imageRight[0],cv::Point(eRight.x, eRight.y), 5, cv::Scalar(0,255,0), -1);
        // //imageRight[0].at<cv::Vec3b>(cv::Point(eRight.x, eRight.y)) = cv::Vec3b(255,0,0);
        // cv::imwrite("/home/juan/Right.png",imageRight[0]);
        // cv::circle(imageLeft[0],cv::Point(eLeft.x, eLeft.y), 5, cv::Scalar(0,255,0), -1);
        // //imageLeft[0].at<cv::Vec3b>(cv::Point(eLeft.x, eLeft.y)) = cv::Vec3b(0,255,255);
        // cv::imwrite("/home/juan/Left.png",imageLeft[0]);
    }
    counter++;

}

void eventCallbackLeft(const dvs_msgs::EventArray::ConstPtr &msg){
    //std::cout<<"HOLA LEFT"<<std::endl;
    if (!tFlag){
        dvs_msgs::Event e = msg->events[0]; 
        tZero = e.ts.toSec();
        tFlag = true;
    }

    if (counterLeft<= limit )
        eventLeft.push_back(msg->events); 
    if (counterLeft == limit )
        callSync();
    counterLeft++;
}

void eventCallbackRight(const dvs_msgs::EventArray::ConstPtr &msg){
    //std::cout<<"HOLA RIGHT"<<std::endl;
    if (!tFlag){
        dvs_msgs::Event e = msg->events[0]; 
        tZero = e.ts.toSec();
        tFlag = true;
    }

    if (counterRight<= limit )
        eventRight.push_back(msg->events);
    if (counterRight == limit )
        callSync();
    counterRight++;
}

void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg){
    try{
        if (counterRight<= limit )
            imageRight.push_back(cv_bridge::toCvShare(msg, "rgb8")->image);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    } 
}

void imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg){
    try{
        if (counterLeft<= limit )
            imageLeft.push_back(cv_bridge::toCvShare(msg, "rgb8")->image);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_eClustering_node");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh_public;

    //nh.getParam("usePencentageOfEvents", useRateOfEvents); // pencentage of events that we used (choosing randomly)
    
    ros::Subscriber subEventsLeft = nh.subscribe("/davis_left/events", 1, &eventCallbackLeft);
    ros::Subscriber subEventsRight = nh.subscribe("/davis_right/events", 1, &eventCallbackRight);

    // JP
    tracker_pub = nh_public.advertise<dvs_msgs::EventArray>("tracker_corners", 1);
    corner_pub = nh_public.advertise<dvs_msgs::EventArray>("corner_events", 1);

    image_transport::ImageTransport it(nh_public);
    //pubBlobImage = it.advertise("blobImage", 1);
    //clusterPub = it.advertise("clusterImage", 1);
    //counterPub = it.advertise("counterImage", 1);


    image_transport::Subscriber imageSubRight = it.subscribe("davis_right/image_raw", 5, imageCallbackRight);
    image_transport::Subscriber imageSubLeft = it.subscribe("davis_left/image_raw", 5, imageCallbackLeft);

    //capture Ctrl-C
    signal(SIGINT, shutdown);
    ros::spin();
}

void shutdown(int s)
{
    //std::cout << "Shutting down ..." << std::endl;
    exit(0);
}



