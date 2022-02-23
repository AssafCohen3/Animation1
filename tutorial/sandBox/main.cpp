
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "Game.h"
int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Wellcome");
  Renderer renderer;

  Game viewer; // initiate with 3 cylinders
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  
  Init(*disp, menu);
  renderer.init(&viewer,2,menu);

  disp->SetRenderer(&renderer);
  //renderer.core().trackball_angle.normalize();
  renderer.core().camera_zoom = 0.3;
  renderer.core().animation_max_fps = 30.;
  renderer.core().camera_eye = Eigen::Vector3f(0, 5, 1);
  //renderer.core().camera_base_translation = Eigen::Vector3f(0, 0, -1);
  renderer.core().camera_view_angle = 45;
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
