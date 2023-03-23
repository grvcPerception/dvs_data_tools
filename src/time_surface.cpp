#include "dvs_data_tools/time_surface.hpp"

timeSurface::timeSurface(){}
timeSurface::~timeSurface(){}

void timeSurface::initSurface(int width, int height, double init_value){
  cameraWidth_ = width;
  cameraHeight_ = height;

  std::vector<std::vector<double>> aux(cameraWidth_, std::vector<double> (cameraHeight_, init_value));
  time_surface_ = aux;
  // im_time_surface_ =
}

void timeSurface::update(int x, int y, double ts){
  time_surface_[x][y] = ts;
}

void timeSurface::updateBilinearInterpolation(double ev_warped_x, double ev_warped_y, double ts){
  static double nxx, nyy, dx, dy, dtx, dty, deltat_x, deltat_y;
  static int xx_voted, yy_voted;
  // Accumulate warped events, using bilinear voting (polarity or count)
  const int xx = ev_warped_x, yy = ev_warped_y;

  // Avoid going out of the frame boundaries
  if (xx  < 1 || xx  < 1 || xx > cameraWidth_-1 || yy > cameraHeight_-1)
    return;

  nxx = (ev_warped_x - floor(ev_warped_x)) - 0.5;
  nyy = (ev_warped_y - floor(ev_warped_y)) - 0.5;

  // set the location of the pixels affected by the interpolation
  if (nxx > 0) xx_voted = xx+1; else xx_voted = xx-1;
  if (nyy > 0) yy_voted = yy+1; else yy_voted = yy-1;

  dx = abs(nxx);
  dy = abs(nyy);

  std::vector<std::pair<int,int>> vecLoc = {std::make_pair(xx,yy),
    std::make_pair(xx,yy_voted), std::make_pair(xx_voted,yy), std::make_pair(xx_voted,yy_voted)};

  std::vector<double> vecVal = {(1-dx)+(1-dy), (1-dx)+dy, dx+(1-dy), dx+dy};
  // Normalize time vector
  double max  = *max_element(vecVal.begin(),vecVal.end());
  for (int i = 0; i<vecVal.size(); i++)
      vecVal[i] = vecVal[i]/max;

  trunkTime(vecLoc[0].first,vecLoc[0].second, vecVal[0]*ts);
  trunkTime(vecLoc[1].first,vecLoc[1].second, vecVal[1]*ts);
  trunkTime(vecLoc[2].first,vecLoc[2].second, vecVal[2]*ts);
  trunkTime(vecLoc[3].first,vecLoc[3].second, vecVal[3]*ts);

  // time_surface_[xx][yy] += polarity*(1-dx)*(1-dy);
  // time_surface_[xx][yy_voted] += polarity*(1-dx)*dy;
  // time_surface_[xx_voted][yy] += polarity*dx*(1-dy);
  // time_surface_[xx_voted][yy_voted] += polarity*dx*dy;
}

void timeSurface::trunkTime(int x, int y, double ts){
  if(time_surface_[x][y] < ts)
    time_surface_[x][y] = ts;
}

std::vector<std::vector<double>> timeSurface::getTimeSurface(){
  return time_surface_;
}

void timeSurface::resetTimeSurface(){
  std::vector<std::vector<double>> aux(cameraWidth_, std::vector<double> (cameraHeight_, 0.0));
  time_surface_ = aux;
}

void timeSurface::computeTimeDecayFrame(cv::Mat &im, double time_constant, double t_ref){

  for(int y=0; y<cameraHeight_; y++){
    // Iterate per colum
    uint8_t* x_ptr;
    x_ptr = im.ptr<uint8_t>(y);
    for(int x=0; x<cameraWidth_; x++)
      x_ptr[x] = 255.0*exp(-((t_ref-time_surface_[x][y])/time_constant));
  }
}
