#ifdef USE_VIEWER

#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <cstdio>

#include "logic-velocity.hpp"

class Viewer {

  public:
    Viewer();
    ~Viewer();

  public:
    void run();

  private:
    void drawPath(const Eigen::MatrixXf &path);
    void drawAimPoint(const opendlv::logic::action::AimPoint &aimPoint);
    void drawSpeedProfile(const Eigen::MatrixXf &speedProfile, const Eigen::MatrixXf &path);
    void drawCar();

  public:
    VelocityControl *ptrVelocityControl;

};

#endif // VIEWER_H

#endif // USE_VIEWER