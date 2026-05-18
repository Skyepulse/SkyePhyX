#ifndef GAMEMANAGER_HPP
#define GAMEMANAGER_HPP

#include <GLFW/glfw3.h>
#include "utils/wgpuBundle.hpp"
#include "rendering/RenderEngine.hpp"
#include "constants.hpp"
#include "physics/solver.hpp"
#include "levels.h"

//================================//
struct ObjectPicker
{
    bool active = false;
    float screenX = 0.0f;
    float screenY = 0.0f;

    Mesh* pickedMesh = nullptr;
    Eigen::Vector3f localHitPoint = Eigen::Vector3f::Zero();
    float hitDistance = 0.0f;

    Joint* mouseJoint = nullptr;
};

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
    
    LevelParameters levelParameters = { 8000.0f, 0.3f, 1.0f };

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
    bool rMouseClicked = false;
    bool lMouseClicked = false;

    bool paused = false;
    bool pKeyWasPressed = false;

    bool nextPass = false;
    bool nextPassKeyWasPressed = false;

    bool restartKeyWasPressed = false;

    bool randomBoxSpawnedPressed = false;
    bool shootBallPressed = false;
    bool mainLoopStarted = false;
    float physicsAccumulator = 0.0f;

    //================================//
    void ProcessEvents(float deltaTime);
    void UpdateCurrentTime();
    void AccumulateFrameRate();
    void TickFrame();

    //================================//
    void SpawnRandomBox();
    void SpawnShootingSphere();
    void ReleaseObjectPicker();
    Eigen::Vector3f ProjectMouseToPickDepth(float screenX, float screenY, float depth);
    bool RaycastFromMouse(float screenX, float screenY, Eigen::Vector3f& outHitPoint, Mesh*& outHitMesh, float& outHitDistance);

    //================================//
    int currentLevel = 1;
    ObjectPicker objectPicker;
};

#endif // GAMEMANAGER_HPP
