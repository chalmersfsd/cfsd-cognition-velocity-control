#ifdef USE_VIEWER
#include <viewer.hpp>

Viewer::Viewer() {}
Viewer::~Viewer() {}

void Viewer::run() 
{
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("Main",640,480);
  
  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
    pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
  );

  const int UI_WIDTH = 180;

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  // Safe and efficient binding of named variables.
  // Specialisations mean no conversions take place for exact types
  // and conversions between scalar types are cheap.
  pangolin::Var<bool> a_button("ui.A_Button",false,false);
  pangolin::Var<double> a_double("ui.A_Double",3,0,5);
  pangolin::Var<int> an_int("ui.An_Int",2,0,5);
  pangolin::Var<double> a_double_log("ui.Log_scale var",3,1,1E4, true);
  pangolin::Var<bool> a_checkbox("ui.A_Checkbox",false,true);
  pangolin::Var<int> an_int_no_input("ui.An_Int_No_Input",2);

  pangolin::Var<bool> save_window("ui.Save_Window",false,false);
  pangolin::Var<bool> save_cube("ui.Save_Cube",false,false);


  // Demonstration of how we can register a keyboard hook to alter a Var
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', pangolin::SetVarFunctor<double>("ui.A_Double", 3.5));

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

    if( pangolin::Pushed(a_button) )
      std::cout << "You Pushed a button!" << std::endl;

    // Overloading of Var<T> operators allows us to treat them like
    // their wrapped types, eg:
    if( a_checkbox )
      an_int = (int)a_double;

    an_int_no_input = an_int;

    if( pangolin::Pushed(save_window) )
        pangolin::SaveWindowOnRender("window");

    if( pangolin::Pushed(save_cube) )
        d_cam.SaveOnRender("cube");

    // Activate efficiently by object
    d_cam.Activate(s_cam);

    // Render some stuff
    glColor3f(1.0,1.0,1.0);
    pangolin::glDrawColouredCube();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
}

#endif