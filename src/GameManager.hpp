#ifndef GAMEMANAGER_HPP
#define GAMEMANAGER_HPP

#include <GLFW/glfw3.h>
#include "utils/wgpuBundle.hpp"
#include "rendering/RenderEngine.hpp"
#include "constants.hpp"

//================================//
class GameManager
{
public:
    GameManager();
    ~GameManager();

    void RunMainLoop();
    void InitGraphics()
    {
        if (this->renderEngine)
        {
            this->renderEngine->Initialize();
        }
    }

private:
    std::unique_ptr<RenderEngine> renderEngine;
    std::unique_ptr<WgpuBundle> wgpuBundle;
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window;

    RenderInfo renderInfo;
    bool correctlyInitialized = false;

    float lastFrameTime = 0.0f;
    float deltaTime = 0.0f;
    float frameRate = 0.0f;
    std::vector<float> frameRateAccumulator;

    float lastMouseX = 0.0f;
    float lastMouseY = 0.0f;
    bool mouseClicked = false;

    //================================//
    void ProcessEvents(float deltaTime);
    void UpdateCurrentTime();
    void AccumulateFrameRate();
};

#endif // GAMEMANAGER_HPP