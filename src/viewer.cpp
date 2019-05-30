#ifdef USE_VIEWER
#include <viewer.hpp>

Viewer::Viewer()
{
  setUp();
}

Viewer::~Viewer()
{
  tearDown();
}

void Viewer::setUp() {}
void Viewer::tearDown() {}

void Viewer::run() 
{
  
  pangolin::CreateWindowAndBind("Main", 640, 480);
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
      pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                              .SetHandler(&handler);

  while (!pangolin::ShouldQuit())
  {
    // Let thread update slowly
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(50ms);

    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    // Render OpenGL Cube
    pangolin::glDrawColouredCube();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
}

#endif