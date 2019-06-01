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
  pangolin::Var<bool> menuShowCar("menu.ShowCar", true, true);
  pangolin::Var<bool> menuExit("menu.Exit", false, false);

  const int cameraPosZ = 30;
  const int offsetY = 3;

  // Define Camera Render Object (for view / scene browsing)
  // y-axis up
  pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(width,height,2000,2000,width/2,height/2,0.1,1000),
                pangolin::ModelViewLookAt(0,0,cameraPosZ, 0,offsetY,0, pangolin::AxisY)
  );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(MENU_WIDTH), 1.0, -float(width)/float(height))
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (!pangolin::ShouldQuit())
  {
    // Limit thread update
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);

    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

    // Activate efficiently by object
    d_cam.Activate(s_cam);
    glClearColor(0.0f,0.0f,0.0f,1.0f);

    // Safely copy from velocity Control
    Eigen::MatrixXf path;
    Eigen::MatrixXf speedProfile;
    opendlv::logic::action::AimPoint aimPoint;
    {
      std::lock_guard<std::mutex> lock1(ptrVelocityControl->m_pathMutex);
      std::lock_guard<std::mutex> lock2(ptrVelocityControl->m_speedProfileMutex);
      std::lock_guard<std::mutex> lock3(ptrVelocityControl->m_aimPointMutex);

      path = ptrVelocityControl->m_path;
      speedProfile = ptrVelocityControl->m_speedProfile;
      aimPoint = ptrVelocityControl->m_aimPoint;
    }  

    if (menuShowPath)
      drawPath(path);

    if (menuShowAimPoint)
      drawAimPoint(aimPoint);
    
    if (menuShowSpeedProfile)
      drawSpeedProfile(speedProfile, path);
    
    if (menuShowCar)
      drawCar();

    // Swap frames and Process Events
    pangolin::FinishFrame();

    if(menuExit)
      break;
  }
}

// ########################## DRAW FUNCTIONS ###################################

void Viewer::drawPath(const Eigen::MatrixXf &path)
{
  if (path.rows() == 0) {
    return;
  }

  glColor3f(1.0f, 1.0f, 1.0f);
  glPointSize(6);

  // Draw discrete path points
  glBegin(GL_POINTS);
  int length = path.rows();
  for (int i = 0; i < length; i++){
    glVertex2f(path(i,0), path(i,1));
  }
  glEnd();

  // Fill in path line between points
  glLineWidth(3);
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < length; i++) {
    glVertex2f(path(i,0), path(i,1));
  }
  glEnd();
  
}

void Viewer::drawAimPoint(const opendlv::logic::action::AimPoint &aimPoint)
{
  float x = aimPoint.distance() * std::cos(aimPoint.azimuthAngle());
  float y = aimPoint.distance() * std::sin(aimPoint.azimuthAngle());

  glColor3f(1.0f, 0.0f, 0.0f);
  glPointSize(9);

  glBegin(GL_POINTS);
  glVertex2f(x, y);
  glEnd();
}

void Viewer::drawSpeedProfile(const Eigen::MatrixXf &speedProfile, const Eigen::MatrixXf &path)
{
  if (speedProfile.rows() == 0) {
    return;
  }
  
  glColor3f(1.0f, 1.0f, 1.0f);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Offset text to left/right depending on turn direction
  const bool leftTurn = path.col(0).sum() < 0.0f;
  float textOffsetX = -0.8f;
  float textOffsetY = 0.1f;
  if (leftTurn) {
    textOffsetX = 0.3f;
  }

  // Print speed values at path points
  for (int i = 0; i < speedProfile.rows(); i++) {
    std::ostringstream strStream;
    strStream.precision(2);
    strStream << std::fixed << speedProfile(i);
    pangolin::GlFont::I()
      .Text(strStream.str().c_str())
      .Draw(path(i,0) + textOffsetX, path(i,1) + textOffsetY, 0.0f);
  }

  glDisable(GL_BLEND);
}

void Viewer::drawCar()
{
  // Specify two corners of a rectangle
  glColor3f(0.0f, 0.0f, 1.0f);

  const float carLength = 0.3f;
  const float x1 = -carLength * 0.5f;
  const float y1 = carLength;
  const float x2 = -x1;
  const float y2 = -y1;

  glRectf(x1, y1, x2, y2);
}

#endif