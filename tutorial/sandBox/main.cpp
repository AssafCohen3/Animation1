
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
  //renderer.core().camera_base_translation = Eigen::Vector3f(0, 0, -1);
  renderer.core().camera_eye = Eigen::Vector3f(0, 7, 1);
  renderer.core().camera_view_angle = 270; // look down
  //core.camera_center = Eigen::Vector3f(0, 0, 0);
  //core.camera_up = Eigen::Vector3f(0, 1, 0);
  viewer.SetVisibilities();
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
