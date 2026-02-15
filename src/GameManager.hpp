#ifndef GAMEMANAGER_HPP
#define GAMEMANAGER_HPP

#include <GLFW/glfw3.h>
#include "utils/wgpuBundle.hpp"

struct RenderInfo
{
    uint32_t width;
    uint32_t height;
    double time;
    bool resizeNeeded;
};

//================================//
class GameManager
{
public:
    GameManager();
    ~GameManager();

    void RunMainLoop();

private:
    std::unique_ptr<WgpuBundle> wgpuBundle;
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window;

    RenderInfo renderInfo;
    bool correctlyInitialized = false;

};

#endif // GAMEMANAGER_HPP