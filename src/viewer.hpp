#ifdef USE_VIEWER

#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>

#include "logic-velocity.hpp"

class Viewer {

  public:
    Viewer();
    ~Viewer();

  public:
    void run();

  private:
    void drawPath();
    void drawAimPoint();
    void drawSpeedProfile();

  public:
    VelocityControl *ptrVelocityControl;

};

#endif // VIEWER_H

#endif // USE_VIEWER