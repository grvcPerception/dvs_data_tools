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

#include "dvs_data_tools/utils.h"

class Events2Frames{    
    private:
        ros::NodeHandle nh_;
        image_transport::Publisher eventFramePub_;
        sensor_msgs::ImagePtr eventFramePub_msg_;
        dvs_msgs::EventArray eventPub_msg_;

        int sensorWidth_ = 346;
        int sensorHeight_ = 260;
        double timeStart_,timeStartEvent_;
        cv::Mat eventFrame_, on_events_, off_events_;
        ros::Subscriber event_sub_;
        ros::Publisher eventPub_;

        // Default values
        std::string visualizationType_ = "batch";
        std::string bColor_ = "black";
        std::string displayType_ = "red-blue";
        int nEventsFrame_ = 2000, nEvents_ = 400, frameIdEvents_ = 0;
        double deltaTime_ = 0.1;
        
        // Fixed parameters
        bool firstFlag_ = true;
        int events_counter_ = 0;

        enum visualization {BATCH = 0, NUMBEROFEVENTS = 1, TIME = 2, EXPERIMENTAL = 3};
        enum bColorList {BLACK = 0, WHITE = 1};
        enum displayType {GRAYSCALE = 0, BLACK_WHITE_NON_POLARITY = 1, RED_BLUE = 2};
        std::map<std::string,visualization> visualizationTypeVec_;
        std::map<std::string,bColorList> bColorListVec_;
        std::map<std::string,displayType> displayTypeVec_;
        

        std::vector<dvs_msgs::Event> eventBuffer_;

        dvsUtils utils_;

        // Methods
        void loadDefinitions();
        void publishEventImage();
        void publishEventSet();
        void initParameters(int width, int height, double time, double timeEvent);
        void eventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg);
        void updateEventFrame(dvs_msgs::Event event);
        void checkEventPublishing(dvs_msgs::Event event, int n_event, int current_event);


    public:
        Events2Frames(ros::NodeHandle & nh, ros::NodeHandle nh_private);
        virtual ~Events2Frames();

        cv::Mat getEventFrame();
};
