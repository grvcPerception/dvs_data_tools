#include<ros/ros.h>
#include<signal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>

#include <math.h>
#include <iostream>
#include <utility> 
#include <string>
#include <map>

ros::Publisher corner_pub, tracker_pub;
image_transport::Publisher eventFramePub;
sensor_msgs::ImagePtr eventFramePub_msg;
int visualizationType = 0;
int nEvents = 200;
double deltaTime = 0.1;
int events_counter = 0;
double timeStart;
bool firstFlag = true;
cv::Mat eventFrame;
int sensorWidth, sensorHeight;
/*
enum visualization {TIME, NUMBEROFEVENTS, DEFAULT};

visualization visualizationTipe_boh = "default";

std::map<visualization,std::string> visualizationType;
// Map definition 
visualizationType[TIME] = "time";
visualizationType[NUMBEROFEVENTS] = "number_of_events";
visualizationType[DEFAULT] = "default";*/

void shutdown(int s);

void initParameters(int width, int height, double time){
    sensorWidth = width;
    sensorHeight = height;
    firstFlag = false;
    eventFrame = cv::Mat::zeros(sensorHeight,sensorWidth, CV_8UC3);
    timeStart = ros::Time::now().toSec();;
}

void publishEventImage(){
    eventFramePub_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", eventFrame).toImageMsg();
    eventFramePub.publish(eventFramePub_msg);
    eventFrame = cv::Mat::zeros(sensorHeight,sensorWidth, CV_8UC3);
}

void eventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg){

    const int n_event = event_msg->events.size();
    if (n_event == 0) 
        return;

    if (firstFlag == true)
        initParameters(event_msg->width, event_msg->height, event_msg->events[0].ts.toSec());
           
    for (int ii = 0; ii<event_msg->events.size(); ++ii){
    //for (const auto& e : event_msg->events) {
      
        if(event_msg->events[ii].polarity)
            eventFrame.at<cv::Vec3b>(cv::Point(event_msg->events[ii].x,event_msg->events[ii].y))=cv::Vec3b(0,0,255);
        else
            eventFrame.at<cv::Vec3b>(cv::Point(event_msg->events[ii].x,event_msg->events[ii].y))=cv::Vec3b(255,0,0);

        switch(visualizationType) {
                case 0 :
                    if (ii == n_event-1){
                        publishEventImage();
                    }
                    break;
                case 1 :
                    if (events_counter >= nEvents){
                        events_counter = 0;
                        publishEventImage();
                    }
                    else
                        events_counter++;
                    break;
                case 2 :
                    double currentTime = event_msg->events[ii].ts.toSec()-timeStart;
                    if(currentTime >= deltaTime){
                        timeStart = event_msg->events[ii].ts.toSec();
                        publishEventImage();
                  
                    }
                    break;
        }
    }  
}

int main(int argc, char** argv){
    ros::init(argc, argv, "eventFrame_node");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh_public;
    // Parameters
    nh.getParam("visualization", visualizationType); 
    nh.getParam("nEvents", nEvents); 
    nh.getParam("deltaTime", deltaTime); 

    // Subscribers
    ros::Subscriber subPC = nh_public.subscribe("events_topic", 10, &eventCallback);
    // Publishers
    image_transport::ImageTransport it(nh_public);
    eventFramePub = it.advertise("events_frame", 1);

    //capture Ctrl-C
    signal(SIGINT, shutdown);
    ros::spin();
 
}

void shutdown(int s){
    exit(0);
}
