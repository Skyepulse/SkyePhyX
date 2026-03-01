#ifndef GAMEMANAGER_HPP
#define GAMEMANAGER_HPP

#include <GLFW/glfw3.h>
#include "utils/wgpuBundle.hpp"
#include "rendering/RenderEngine.hpp"
#include "constants.hpp"
#include "physics/solver.hpp"

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

    void ChangeLevel(int levelIndex);
    int GetCurrentLevel() const { return this->currentLevel; }

private:
    std::unique_ptr<RenderEngine> renderEngine;
    std::unique_ptr<Solver> solver;
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

    bool paused = false;
    bool pKeyWasPressed = false;

    bool nextPass = false;
    bool nextPassKeyWasPressed = false;

    bool restartKeyWasPressed = false;

    bool randomBoxSpawnedPressed = false;

    //================================//
    void ProcessEvents(float deltaTime);
    void UpdateCurrentTime();
    void AccumulateFrameRate();

    //================================//
    void SpawnRandomBox();

    //================================//
    int currentLevel = 0;
};

#endif // GAMEMANAGER_HPP