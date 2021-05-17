#include "dvs_data_tools/utils.h"


namespace dvs_tools
{
  utils::utils(){
  }

  utils::~utils(){}

  void utils::loadCameraCalibration(std::vector <double> Kc, std::vector <double> Dc){

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++)
            K_(j,i) =  Kc[i+j*3];
    }
    for (int i = 0; i < Dc.size(); i++)
        D_(i) = Dc[i];
  }


  bool utils::inEdge(unsigned int xx, unsigned int yy){
          double x = (double)xx;
          double y = (double)yy;
          return ( \
                         y<115 && x>179 && sqrt(pow(y-115,2) + pow(x-179,2)) > 182 || \
                         y>135 && x>245 && sqrt(pow(y-135,2) + pow(x-245,2)) > 131 || \
                         y>135 && x>260 && sqrt(pow(y-135,2) + pow(x-260,2)) > 124 || \
                         y<115 && x<160 && sqrt(pow(y-115,2) + pow(x-160,2)) > 165 || \
                         y>115 && x<160 && sqrt(pow(y-115,2) + pow(x-160,2)) > 178 || \
                         y>115 && x<105 && sqrt(pow(y-115,2) + pow(x-105,2)) > 144 \
                         );
  }

  cv::Point utils::undistortEvent(int u, int v){
      static bool firstIteration = true;
      static double ki[8]={0,0,0,0,0,0,0,0};
      static double fx = 0.0;
      static double fy = 0.0;
      static double ifx = 0.0;
      static double ify = 0.0;
      static double cx = 0.0;
      static double cy = 0.0;

      if(firstIteration){
          // Camera matrix parameter
          fx = K_(0,0);
          fy = K_(1,1);
          ifx = 1./fx;
          ify = 1./fy;
          cx = K_(0,2);
          cy = K_(1,2);

          for(int i = 0; i < 5; i++ )
              ki[i] = D_(i);
          firstIteration = false;
      }

      double x = u, y = v;
      // Map into 3D (inverse of camera matrix)
      double x0 = x = (x - cx)*ifx;
      double y0 = y = (y - cy)*ify;

      int iters = 2; // It depends of the lens distortion, the higher the distortion the higher the number of iterations
      for(int j = 0; j < iters; j++ ){
              double r2 = x*x + y*y;
              double icdist = (1 + ((ki[7]*r2 + ki[6])*r2 + ki[5])*r2)/(1 + ((ki[4]*r2 + ki[1])*r2 + ki[0])*r2);
              double deltaX = 2*ki[2]*x*y + ki[3]*(r2 + 2*x*x);
              double deltaY = ki[2]*(r2 + 2*y*y) + 2*ki[3]*x*y;
              x = (x0 - deltaX)*icdist;
              y = (y0 - deltaY)*icdist;
      }

      // Apply the camera matrix
      x = x*fx + cx;
      y = y*fy + cy;

      if (x>0 && x<sensorWidth_ && y>0 && y<sensorHeight_)
          return (cv::Point((int)x,(int)y));
      else
          return (cv::Point(-1,-1));

  }
} //namespace
