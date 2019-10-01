#include "event_data_tools/events2frames.h"

Events2Frames::Events2Frames(ros::NodeHandle & nh, ros::NodeHandle nh_private): nh_(nh){
    nh_private.getParam("visualization", visualizationType_); 
    nh_private.getParam("nEvents", nEvents_); 
    nh_private.getParam("backgroundColor", bColor_);
    nh_private.getParam("deltaTime", deltaTime_); 

    loadDefinitions();
    
    // Subscribers
    event_sub_ = nh_.subscribe("events_topic", 10, &Events2Frames::eventCallback, this);
    // Publishers
    image_transport::ImageTransport it(nh_);
    eventFramePub_ = it.advertise("events_frame", 1);
}

Events2Frames::~Events2Frames(){}

void Events2Frames::publishEventImage(){
    eventFramePub_msg_ = cv_bridge::CvImage(std_msgs::Header(), "rgb8", eventFrame_).toImageMsg();
    eventFramePub_.publish(eventFramePub_msg_);
    eventFrame_ = cv::Mat::zeros(sensorHeight_,sensorWidth_, CV_8UC3);
    if (bColorListVec_[bColor_]==WHITE)
        eventFrame_ = cv::Scalar(255, 255, 255);
}

void Events2Frames::loadDefinitions(){
    // Define map
    visualizationTypeVec_["default"]=DEFAULT;
    visualizationTypeVec_["number_of_events"]=NUMBEROFEVENTS;
    visualizationTypeVec_["time"]=TIME;
    visualizationTypeVec_["experimental"]=EXPERIMENTAL;
    // Define color map
    bColorListVec_["black"] = BLACK;
    bColorListVec_["white"] = WHITE;

}

void Events2Frames::initParameters(int width, int height, double time, double timeEvent){
    sensorWidth_ = width;
    sensorHeight_ = height;
    firstFlag_ = false;    
    timeStart_ = time;
    timeStartEvent_ = timeEvent;
    eventFrame_ = cv::Mat::zeros(sensorHeight_,sensorWidth_, CV_8UC3);
    if (bColorListVec_[bColor_]==WHITE)
        eventFrame_ = cv::Scalar(255, 255, 255);
}

void Events2Frames::eventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg){
    const int n_event = event_msg->events.size();
    if (n_event == 0) 
        return;

    if (firstFlag_ == true)
        initParameters(event_msg->width, event_msg->height, ros::Time::now().toSec(), event_msg->events[0].ts.toSec());
           
    for (int ii = 0; ii<event_msg->events.size(); ++ii){
    //for (const auto& e : event_msg->events) {
      
        if(event_msg->events[ii].polarity)
            eventFrame_.at<cv::Vec3b>(cv::Point(event_msg->events[ii].x,event_msg->events[ii].y))=cv::Vec3b(0,0,255);
        else
            eventFrame_.at<cv::Vec3b>(cv::Point(event_msg->events[ii].x,event_msg->events[ii].y))=cv::Vec3b(255,0,0);

        double currentTime;
        switch(visualizationTypeVec_[visualizationType_]){
                case DEFAULT :
                    if (ii == n_event-1){
                        publishEventImage();
                    }
                    break;
                case NUMBEROFEVENTS :
                    if (events_counter_ >= nEvents_){
                        events_counter_ = 0;
                        publishEventImage();
                    }
                    else
                        events_counter_++;
                    break;
                case TIME :
                    currentTime = ros::Time::now().toSec()-timeStart_;
                    if(currentTime >= deltaTime_){
                        timeStart_ = ros::Time::now().toSec();
                        publishEventImage();
                  
                    }
                    break;
                case EXPERIMENTAL :
                    currentTime = event_msg->events[ii].ts.toSec()-timeStartEvent_;
                    if(currentTime >= deltaTime_){
                        timeStartEvent_ = event_msg->events[ii].ts.toSec();
                        publishEventImage();
                    }
                    break;
        }
    }

}

cv::Mat Events2Frames::getEventFrame(){
    if (eventFrame_.empty()){
        eventFrame_ = cv::Mat::zeros(sensorHeight_,sensorWidth_, CV_8UC3);
        if (bColorListVec_[bColor_]==WHITE)
            eventFrame_ = cv::Scalar(255, 255, 255);
    }
    else
        return eventFrame_;
}