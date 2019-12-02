#include<ros/ros.h>

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

class Events2Frames{    
    private:
    ros::NodeHandle nh_;
    image_transport::Publisher eventFramePub_;
    sensor_msgs::ImagePtr eventFramePub_msg_;
    int sensorWidth_ = 346;
    int sensorHeight_ = 260;
    double timeStart_,timeStartEvent_;
    cv::Mat eventFrame_;
    ros::Subscriber event_sub_;

    // Default values
    std::string visualizationType_ = "default";
    std::string bColor_ = "black";
    int nEvents_ = 200;
    double deltaTime_ = 0.1;
    
    // Fixed parameters
    bool firstFlag_ = true;
    int events_counter_ = 0;

    void publishEventImage();
    void initParameters(int width, int height, double time, double timeEvent);
    void eventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg);

    enum visualization {DEFAULT = 0, NUMBEROFEVENTS = 1, TIME = 2, EXPERIMENTAL = 3};
    enum bColorList {BLACK = 0, WHITE = 1};
    std::map<std::string,visualization> visualizationTypeVec_;
    std::map<std::string,bColorList> bColorListVec_;
    void loadDefinitions();

    public:
    Events2Frames(ros::NodeHandle & nh, ros::NodeHandle nh_private);
    virtual ~Events2Frames();

    cv::Mat getEventFrame();
};
