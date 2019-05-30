#ifdef USE_VIEWER
#include <viewer.hpp>

Viewer::Viewer() {}
Viewer::~Viewer() {}

void Viewer::run() 
{
  // Create OpenGL window in single line
  const uint16_t width = 1024;
  const uint16_t height = 768;
  pangolin::CreateWindowAndBind("Main",width,height);
  
  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  const int MENU_WIDTH = 180;

  // Create panel at left side of window
  pangolin::CreatePanel("menu")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(MENU_WIDTH));

  // Menu variables
  pangolin::Var<bool> menuShowPath("menu.ShowPath", true, true);
  pangolin::Var<bool> menuShowAimPoint("menu.ShowAimPoint", true, true);
  pangolin::Var<bool> menuShowSpeedProfile("menu.ShowSpeedProfile", true, true);
  pangolin::Var<bool> menuExit("menu.Exit", false, false);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(width,height,2000,2000,width/2,height/2,0.1,1000),
                pangolin::ModelViewLookAt(0,-100,100, 0,0,0,1.0,0.0, 0.0)
  );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(MENU_WIDTH), 1.0, -float(width)/float(height))
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (!pangolin::ShouldQuit())
  {
    // Limit thread update
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(30ms);
    
    Eigen::MatrixXf path;
    opendlv::logic::action::AimPoint aimPoint;
    {
      std::lock_guard<std::mutex> lock1(ptrVelocityControl->m_pathMutex);
      std::lock_guard<std::mutex> lock2(ptrVelocityControl->m_aimPointMutex);

      path = ptrVelocityControl->m_path;
      aimPoint = ptrVelocityControl->m_aimPoint;
    }

    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

    // Activate efficiently by object
    d_cam.Activate(s_cam);
    glClearColor(0.0f,0.0f,0.0f,1.0f);

    if (menuShowPath)
      drawPath();

    if (menuShowAimPoint)
      drawAimPoint();
    
    if (menuShowSpeedProfile)
      drawSpeedProfile();

    // Swap frames and Process Events
    pangolin::FinishFrame();

    if(menuExit)
      break;
  }
}

// ########################## DRAW FUNCTIONS ###################################

void Viewer::drawPath()
{
   // Safely copy path from velocity control
  Eigen::MatrixXf path;
  {
    std::lock_guard<std::mutex> lock(ptrVelocityControl->m_pathMutex);
    path = ptrVelocityControl->m_path;
  }
  if (path.rows() == 0) {
    return;
  }

  glColor3f(1.0f, 1.0f, 1.0f);
  glPointSize(2);

  glBegin(GL_POINTS);

  int length = path.rows();
  for (int i = 0; i < length; i++){
    glVertex2f(path(i,0), path(i,1));
  }
  glEnd();

  glLineWidth(3);
  glBegin(GL_LINES);
  glVertex2f(path(0,0), path(0,0));
  for (int i = 0; i < length; i++) {
    glVertex2f(path(i,0), path(i,0));
    glVertex2f(path(i,0), path(i,0));
  }
  glVertex2f(path(length-1,0), path(length-1,1));
  glEnd();
}

void Viewer::drawAimPoint()
{

  // Safely copy aim point from velocity control
  opendlv::logic::action::AimPoint aimPoint;
  {
    std::lock_guard<std::mutex> lock(ptrVelocityControl->m_aimPointMutex);
    aimPoint = ptrVelocityControl->m_aimPoint;
  }
  float x = aimPoint.distance() * std::cos(aimPoint.azimuthAngle());
  float y = aimPoint.distance() * std::sin(aimPoint.azimuthAngle());

  glColor3f(1.0f, 0.0f, 0.0f);
  glPointSize(5);

  glBegin(GL_POINTS);
    glVertex2f(x, y);
  glEnd();
}

void Viewer::drawSpeedProfile()
{
  opendlv::logic::action::AimPoint aimPoint;
  {
    std::lock_guard<std::mutex> lock(ptrVelocityControl->m_aimPointMutex);
    aimPoint = ptrVelocityControl->m_aimPoint;
  }

  // Change text color to white
  glColor3f(1.0f, 1.0f, 1.0f);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  float x = aimPoint.distance() * std::cos(aimPoint.azimuthAngle());
  float y = aimPoint.distance() * std::sin(aimPoint.azimuthAngle());

  pangolin::GlFont::I().Text(
        "(%.2f, %.2f)",
        aimPoint.distance(), aimPoint.azimuthAngle()
  ).DrawWindow(1024.0f/2.0f, 768.0f/2.0f);
  glDisable(GL_BLEND);
}

#endif