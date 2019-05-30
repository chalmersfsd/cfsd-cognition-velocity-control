#ifdef USE_VIEWER

#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>
#include <thread>
#include <chrono>

class Viewer {

  public:
    Viewer();
    ~Viewer();

  public:
    void run();

  private:
    void setUp();
    void tearDown();

};

#endif // VIEWER_H

#endif // USE_VIEWER