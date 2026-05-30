#ifndef GAMEMANAGER_HPP
#define GAMEMANAGER_HPP

#include <GLFW/glfw3.h>
#include "utils/wgpuBundle.hpp"
#include "rendering/RenderEngine.hpp"
#include "constants.hpp"
#include "physics/solver.hpp"
#include "levels.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/html5.h>
#endif

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

#ifdef __EMSCRIPTEN__
//================================//
struct TouchInputState
{
    bool active = false;
    bool wasActive = false;
    bool consumedByUi = false;
    bool pinchActive = false;
    int touchCount = 0;

    float primaryX = 0.0f;
    float primaryY = 0.0f;
    float previousPrimaryX = 0.0f;
    float previousPrimaryY = 0.0f;

    float pinchDistance = 0.0f;
    float previousPinchDistance = 0.0f;
};
#endif

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
    void NextStep() { this->nextPass = true; }
    
    LevelParameters levelParameters = { 8000.0f, 0.3f, 1.0f };

    bool paused = false;
    bool shootSpheres = false;
    bool pickMeshes = true;

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

    bool pKeyWasPressed = false;

    bool nextPass = false;
    bool nextPassKeyWasPressed = false;

    bool restartKeyWasPressed = false;

    bool randomBoxSpawnedPressed = false;
    bool mainLoopStarted = false;
    float physicsAccumulator = 0.0f;
    double lastSphereDragSpawnTime = -1000.0;
    float lastSphereDragX = 0.0f;
    float lastSphereDragY = 0.0f;

#ifdef __EMSCRIPTEN__
    TouchInputState touchInput;
#endif

    //================================//
    void ProcessEvents(float deltaTime);
    void UpdateCurrentTime();
    void AccumulateFrameRate();
    void TickFrame();

    //================================//
    void SpawnRandomMesh();
    void SpawnShootingSphere(float mouseX = 0.0f, float mouseY = 0.0f);
    void ProcessPrimaryPointer(bool pressed, float screenX, float screenY, bool capturedByUi);
    bool ShouldSpawnDragSphere(float screenX, float screenY);
    void ReleaseObjectPicker();
    Eigen::Vector3f ProjectMouseToPickDepth(float screenX, float screenY, float depth);
    bool RaycastFromMouse(float screenX, float screenY, Eigen::Vector3f& outHitPoint, Mesh*& outHitMesh, float& outHitDistance);

#ifdef __EMSCRIPTEN__
    void RegisterMobileTouchCallbacks();
    bool ProcessTouchEvents(float deltaTime);
    static EM_BOOL HandleTouchEvent(int eventType, const EmscriptenTouchEvent* touchEvent, void* userData);
#endif

    //================================//
    int currentLevel = 1;
    ObjectPicker objectPicker;
};

#endif // GAMEMANAGER_HPP
