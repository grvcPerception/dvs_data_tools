#include<ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>

#include <math.h>
#include <iostream>
#include <utility>
#include <string>
#include <map>

#include <dvs_data_tools/dvs_data_toolsConfig.h>
#include <dvs_data_tools/utils.h>

namespace dvs_tools
{
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

          // Filtering options
          bool filterFlag_ = false;
          bool undistorEventsFlag_ = false;

          enum visualization {BATCH = 0, NUMBEROFEVENTS = 1, TIME = 2, EXPERIMENTAL = 3};
          enum bColorList {BLACK = 0, WHITE = 1};
          enum displayType {GRAYSCALE = 0, BLACK_WHITE_NON_POLARITY = 1, RED_BLUE = 2};
          std::map<std::string,visualization> visualizationTypeVec_;
          std::map<std::string,bColorList> bColorListVec_;
          std::map<std::string,displayType> displayTypeVec_;


          std::vector<dvs_msgs::Event> eventBuffer_;

          dvs_tools::utils utils_;

          // Methods
          void loadDefinitions();
          void publishEventImage();
          void publishEventSet();
          void initParameters(int width, int height, double time, double timeEvent);
          bool updateEventFrame(dvs_msgs::Event event);
          void checkEventPublishing(dvs_msgs::Event event, int n_event, int current_event, bool valid_event);
          void updateTimeReference(double time);

          // Subscribers
          void eventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg);

          // Dynamic reconfigure
          void reconfigureCallback(dvs_data_tools::dvs_data_toolsConfig &config, uint32_t level);
          dynamic_reconfigure::Server<dvs_data_tools::dvs_data_toolsConfig> drServer_;
          dynamic_reconfigure::Server<dvs_data_tools::dvs_data_toolsConfig>::CallbackType drCallback_;

          // boost::shared_ptr<dynamic_reconfigure::Server<dvs_data_tools::dvs_data_toolsConfig> > server_;
          // dynamic_reconfigure::Server<dvs_data_tools::dvs_data_toolsConfig>::CallbackType dynamic_reconfigure_callback_;

      public:
          Events2Frames(ros::NodeHandle & nh, ros::NodeHandle nh_private);
          ~Events2Frames();

          cv::Mat getEventFrame();
  };

}// namespace
