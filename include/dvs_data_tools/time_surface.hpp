#ifndef TIME_SURFACE_H
#define TIME_SURFACE_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <utility>
#include <algorithm>
#include <iostream>

class timeSurface{
  protected:
    int cameraWidth_;
    int cameraHeight_;
    std::vector <std::vector <double> > time_surface_;

  public:
    timeSurface();
    ~timeSurface(void);

    void initSurface(int width, int height, double init_value);
    void update(int x, int y, double ts);
    void updateBilinearInterpolation(double ev_warped_x, double ev_warped_y, double ts);
    void trunkTime(int x, int y, double t);
    void computeTimeDecayFrame(cv::Mat &im, double time_constant, double t_ref);
    std::vector<std::vector<double>> getTimeSurface();
    void resetTimeSurface();
};
#endif //TIME_SURFACE_H
