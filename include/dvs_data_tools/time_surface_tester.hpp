#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include <std_msgs/Time.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <map>

#include "dvs_data_tools/time_surface.hpp"

class timeSurfaceTester{
  private:

    ros::Publisher pubTimeSurface_;
    ros::Subscriber eventSub_;
    int cameraWidth_ = 346, cameraHeight_ = 260, accNevents_ = 0;
    double time_constant_, tZero_ = -1.0, t_ref_ = -1.0, accTime_ = -1.0;
    std::string updateType_;
    enum accumlationType {NUMBEROFEVENTS = 0, TIME = 1};
    std::map<std::string,accumlationType> updateTypeVec_;

    // Frames
    cv::Mat imageTimeSurface_;
    // Classes
    timeSurface tSurface_;

    // Methods
    void publishTimeSufaces(double t, dvs_msgs::Event e);
    void loadDefinitions();
    void initFrames(int width, int height, double time);
    bool frameTrigger(dvs_msgs::Event e);
    void cleanTimeSurfacesFrames();

  public:
    timeSurfaceTester(ros::NodeHandle &n, ros::NodeHandle &npublic);
    ~timeSurfaceTester(void);

    void eventCallback(const dvs_msgs::EventArray::ConstPtr& msg);
};
