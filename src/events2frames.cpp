#include "dvs_data_tools/events2frames.h"


Events2Frames::Events2Frames(ros::NodeHandle & nh, ros::NodeHandle nh_private): nh_(nh){
    nh_private.getParam("visualization", visualizationType_);
    nh_private.getParam("nEventsFrame", nEventsFrame_);
    nh_private.getParam("nEvents", nEvents_);
    nh_private.getParam("displayType", displayType_);
    nh_private.getParam("backgroundColor", bColor_);
    nh_private.getParam("deltaTime", deltaTime_);
    nh_private.getParam("filterFlag", filterFlag_);

    loadDefinitions();

    // Subscribers
    event_sub_ = nh_.subscribe("events_topic", 10, &Events2Frames::eventCallback, this);
    // Publishers
    image_transport::ImageTransport it(nh_);
    eventFramePub_ = it.advertise("events_frame", 1);
    eventPub_ = nh_.advertise<dvs_msgs::EventArray>("events", 1, true);

    //dvsUtils::dvsUtils utils;
}

Events2Frames::~Events2Frames(){}

void Events2Frames::publishEventImage(){

    switch(displayTypeVec_[displayType_] ){
        case GRAYSCALE :
            cv::normalize(on_events_, on_events_, 0, 128, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(off_events_, off_events_, 0, 127, cv::NORM_MINMAX, CV_8UC1);

            // Add positive an negative to the output image
            eventFrame_ += on_events_;
            eventFrame_ -= off_events_;

            eventFramePub_msg_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", eventFrame_).toImageMsg();
            eventFramePub_.publish(eventFramePub_msg_);

            // reset the image
            //eventFrame_ = cv::Mat(sensorHeight_,sensorWidth_, CV_8U);
            eventFrame_ = cv::Scalar(128);

            // reset the event on - off images
            //on_events_ = cv::Mat(sensorHeight_,sensorWidth_, CV_8U);
            on_events_ = cv::Scalar(0);

            //off_events_ = cv::Mat(sensorHeight_,sensorWidth_, CV_8U);
            off_events_ = cv::Scalar(0);
            break;

        case BLACK_WHITE_NON_POLARITY :

            if (bColorListVec_[bColor_]==BLACK)
              cv::threshold(eventFrame_, eventFrame_, 0, 255, cv::THRESH_BINARY);
            else
              cv::threshold(eventFrame_, eventFrame_, 254, 255, cv::THRESH_BINARY);

            eventFramePub_msg_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", eventFrame_).toImageMsg();
            eventFramePub_.publish(eventFramePub_msg_);

            if (bColorListVec_[bColor_]==WHITE)
                eventFrame_ = cv::Scalar(255, 255, 255);
            else
                eventFrame_ = cv::Scalar(0);
            break;

        case RED_BLUE :
            eventFramePub_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", eventFrame_).toImageMsg();
            eventFramePub_.publish(eventFramePub_msg_);
            // reset the event image
            eventFrame_ = cv::Mat::zeros(sensorHeight_,sensorWidth_, CV_8UC3);
            if (bColorListVec_[bColor_]==WHITE)
                eventFrame_ = cv::Scalar(255, 255, 255);
            break;

        default :
            ROS_INFO("Cannot publish frame, invalid displayType argument");

    }
}

void Events2Frames::publishEventSet(){
    eventPub_msg_.header.stamp = ros::Time::now();
    eventPub_msg_.header.frame_id = std::to_string(frameIdEvents_);
    eventPub_msg_.height =  sensorHeight_;
    eventPub_msg_.width = sensorWidth_;
    eventPub_msg_.events = eventBuffer_;

    eventPub_.publish(eventPub_msg_);

    eventBuffer_.clear();
    frameIdEvents_++;

}

void Events2Frames::loadDefinitions(){
    // Define map
    visualizationTypeVec_["batch"]=BATCH;
    visualizationTypeVec_["number_of_events"]=NUMBEROFEVENTS;
    visualizationTypeVec_["time"]=TIME;
    visualizationTypeVec_["experimental"]=EXPERIMENTAL;
    // Define color map
    bColorListVec_["black"] = BLACK;
    bColorListVec_["white"] = WHITE;
    // drawing type
    displayTypeVec_["grayscale"] = GRAYSCALE;
    displayTypeVec_["blackwithe-non-polarity"] = BLACK_WHITE_NON_POLARITY;
    displayTypeVec_["red-blue"] = RED_BLUE;

}

void Events2Frames::initParameters(int width, int height, double time, double timeEvent){
    sensorWidth_ = width;
    sensorHeight_ = height;
    firstFlag_ = false;
    timeStart_ = time;
    timeStartEvent_ = timeEvent;

    // Initialize the type to publish
    switch(displayTypeVec_[displayType_] ){
        case GRAYSCALE :
            eventFrame_ = cv::Mat(sensorHeight_,sensorWidth_, CV_8U);
            eventFrame_ = cv::Scalar(128);

            on_events_ = cv::Mat(sensorHeight_,sensorWidth_, CV_8U);
            on_events_ = cv::Scalar(0);

            off_events_ = cv::Mat(sensorHeight_,sensorWidth_, CV_8U);
            off_events_ = cv::Scalar(0);

            break;
        case BLACK_WHITE_NON_POLARITY :
            eventFrame_ = cv::Mat(sensorHeight_,sensorWidth_, CV_8U);
            if (bColorListVec_[bColor_]==WHITE)
                eventFrame_ = cv::Scalar(255, 255, 255);
            else
                eventFrame_ = cv::Scalar(0);
            break;

        case RED_BLUE :
            eventFrame_ = cv::Mat::zeros(sensorHeight_,sensorWidth_, CV_8UC3);
            if (bColorListVec_[bColor_]==WHITE)
                eventFrame_ = cv::Scalar(255, 255, 255);
            break;

        default :
            ROS_INFO("Cannot initialize frame, invalid displayType argument");
    }
}

void Events2Frames::eventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg){
    const int n_event = event_msg->events.size();
    if (n_event == 0)
        return;

    if (firstFlag_ == true)
        initParameters(event_msg->width, event_msg->height, ros::Time::now().toSec(), event_msg->events[0].ts.toSec());

    for (int ii = 0; ii<event_msg->events.size(); ++ii){

        if(!filterFlag_){
            updateEventFrame(event_msg->events[ii]);
            checkEventPublishing(event_msg->events[ii], n_event, ii);
        }
        else{
            if(!utils_.inEdge(event_msg->events[ii].x, event_msg->events[ii].y)){
                updateEventFrame(event_msg->events[ii]);
                checkEventPublishing(event_msg->events[ii], n_event, ii);
            }
        }
    }
}

void Events2Frames::updateEventFrame(dvs_msgs::Event event){


    // Accumulate events for displaying
    switch(displayTypeVec_[displayType_] ){
    case GRAYSCALE :
        if (event.polarity)
            on_events_.at<uint8_t>(cv::Point(event.x, event.y))++;
        else
            off_events_.at<uint8_t>(cv::Point(event.x, event.y))++;
        break;

    case BLACK_WHITE_NON_POLARITY :
        if (bColorListVec_[bColor_]==BLACK)
            eventFrame_.at<uint8_t>(cv::Point(event.x, event.y))++;
        else{
            if(eventFrame_.at<uint8_t>(cv::Point(event.x, event.y)) > 0)
                eventFrame_.at<uint8_t>(cv::Point(event.x, event.y))--;
        }
        break;

    case RED_BLUE :
        if(event.polarity)
            eventFrame_.at<cv::Vec3b>(cv::Point(event.x,event.y))=cv::Vec3b(255,0,0);
        else
            eventFrame_.at<cv::Vec3b>(cv::Point(event.x,event.y))=cv::Vec3b(0,0,255);
        break;

    default :
        ROS_INFO("Invalid displayType argument");
    }
}

void Events2Frames::checkEventPublishing(dvs_msgs::Event event, int n_event, int current_event_index){

    eventBuffer_.push_back(event);
    double currentTime;
    switch(visualizationTypeVec_[visualizationType_]){
        case BATCH :
            if (current_event_index == n_event-1){
                publishEventImage();
                publishEventSet();
            }
            break;
        case NUMBEROFEVENTS :
            if (events_counter_ >= nEventsFrame_){
                events_counter_ = 0;
                publishEventImage();
            }
            else
                events_counter_++;
            if (eventBuffer_.size() > nEvents_-1)
                publishEventSet();
            break;
        case TIME :
            currentTime = ros::Time::now().toSec()-timeStart_;
            if(currentTime >= deltaTime_){
                timeStart_ = ros::Time::now().toSec();
                publishEventImage();
                publishEventSet();
            }
            break;
        case EXPERIMENTAL :
            currentTime = event.ts.toSec()-timeStartEvent_;
            if(currentTime >= deltaTime_){
                timeStartEvent_ = event.ts.toSec();
                publishEventImage();
                publishEventSet();
            }
            break;
        default :
            ROS_INFO("Invalid visualizationType_ argument");
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
