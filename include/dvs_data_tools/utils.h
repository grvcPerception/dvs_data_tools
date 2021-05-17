
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui.hpp>

namespace dvs_tools
{
  class utils{
      public:
        //dvsUtils(int width, int height, Eigen::MatrixXd K, Eigen::VectorXd D);
        utils();
        ~utils();

        bool inEdge(unsigned int xx, unsigned int yy);
        cv::Point undistortEvent(int u, int v);
        void loadCameraCalibration(std::vector <double> Kc, std::vector <double> Dc);

      private:
        Eigen::MatrixXd K_ = Eigen::MatrixXd::Zero(3,3);
        Eigen::VectorXd D_ = Eigen::VectorXd::Zero(5);
        int sensorWidth_ = 346;
        int sensorHeight_ = 260;

  };

} //namespace
