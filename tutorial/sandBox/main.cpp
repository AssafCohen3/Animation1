
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
  renderer.core().trackball_angle.normalize();
  renderer.core().camera_zoom = 2.5;
  renderer.core().animation_max_fps = 30.;
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
