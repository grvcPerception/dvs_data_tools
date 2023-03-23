#include "dvs_data_tools/time_surface_tester.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "time_surface_tester_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;

  timeSurfaceTester rc(nh, nh_public);
  ros::spin();
}

timeSurfaceTester::timeSurfaceTester(ros::NodeHandle &nh, ros::NodeHandle &nh_public){

  eventSub_ = nh_public.subscribe("events_topic", 0, &timeSurfaceTester::eventCallback, this);
  pubTimeSurface_ = nh_public.advertise<sensor_msgs::Image>("time_surface", 1);

  std::string accumlationType;
  // Parameter time surface
  nh.getParam("time_constant",time_constant_);
  // Parameters event accumulator
  nh.getParam("accumulation_type", updateType_);
  loadDefinitions();
  nh.getParam("accumulation_time", accTime_);
  nh.getParam("number_of_accumulated_events", accNevents_);
}

timeSurfaceTester::~timeSurfaceTester(){}

void timeSurfaceTester::initFrames(int width, int height, double time){
  cameraWidth_ = width;
  cameraHeight_ = height;

  tZero_ = time;
  t_ref_ = time;
  // Initialize parameters in each object
  tSurface_.initSurface(cameraWidth_,cameraHeight_, 0.0);
  imageTimeSurface_ = cv::Mat(cameraHeight_, cameraWidth_, CV_8UC1, cv::Scalar(0));
}

void timeSurfaceTester::eventCallback(const dvs_msgs::EventArray::ConstPtr& msg){
  const int n_events = msg->events.size();
  if (n_events == 1)
    return;

  if(tZero_ < 0.0)
    initFrames(msg->width,msg->height, msg->events[0].ts.toSec());

  for(auto e : msg->events) {
    if(!e.polarity)
      continue;

    // Get the time of the event in seconds
    double currentT = e.ts.toSec();
    tSurface_.update(e.x, e.y, currentT-tZero_);

    if(frameTrigger(e))
      publishTimeSufaces(currentT, e);

  }
}

void timeSurfaceTester::loadDefinitions(){
  updateTypeVec_["number_of_events"]=NUMBEROFEVENTS;
  updateTypeVec_["time"]=TIME;
}

bool timeSurfaceTester::frameTrigger(dvs_msgs::Event e){
  static int event_counter = 0;

  switch(updateTypeVec_[updateType_]){
    case NUMBEROFEVENTS :
      event_counter++;
      if(event_counter >= accNevents_){
        event_counter = 0;
        return true;
      }
      break;
    case TIME :
      if (e.ts.toSec()-t_ref_ > accTime_){
        t_ref_ = e.ts.toSec();
        return true;
      }
      break;
  }
  return false;
}

void timeSurfaceTester::publishTimeSufaces(double t, dvs_msgs::Event e){
  static sensor_msgs::ImagePtr imMsg;
  static std_msgs::Header header_msg;

  tSurface_.computeTimeDecayFrame(imageTimeSurface_, time_constant_, t-tZero_);

  header_msg.stamp = e.ts;
  ROS_INFO_STREAM("Last event timestamp "<<e.ts.toSec() - tZero_);

  imMsg = cv_bridge::CvImage(header_msg, "mono8", imageTimeSurface_).toImageMsg();
  pubTimeSurface_.publish(imMsg);

  cleanTimeSurfacesFrames();
  // tSurface_.resetTimeSurface();
}

void timeSurfaceTester::cleanTimeSurfacesFrames(){
  imageTimeSurface_ = cv::Mat(cameraHeight_, cameraWidth_, CV_8UC1, cv::Scalar(0));
}
