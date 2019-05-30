#ifdef USE_VIEWER

#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <thread>
#include <chrono>
#include <mutex>

#include "logic-velocity.hpp"

class Viewer {

  public:
    Viewer();
    ~Viewer();

  public:
    void run();

  public:
    VelocityControl *ptrVelocityControl;

};

#endif // VIEWER_H

#endif // USE_VIEWER