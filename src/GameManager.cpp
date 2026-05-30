#include "GameManager.hpp"
#include "helpers/math.hpp"
#include "helpers/time.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#endif

//================================//
static bool GetMouseRay(Camera* camera, GLFWwindow* window, uint32_t width, uint32_t height, float screenX, float screenY, Eigen::Vector3f& outOrigin, Eigen::Vector3f& outDirection)
{
    if (width == 0 || height == 0)
        return false;

    int windowWidth = 0;
    int windowHeight = 0;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    if (windowWidth > 0 && windowHeight > 0)
    {
        screenX *= static_cast<float>(width) / static_cast<float>(windowWidth);
        screenY *= static_cast<float>(height) / static_cast<float>(windowHeight);
    }

    Eigen::Matrix4f view, projection;
    camera->GetViewMatrix(view);
    camera->GetProjectionMatrix(projection);

    const Eigen::Matrix4f invViewProjection = (projection * view).inverse();
    const float ndcX = 2.0f * screenX / static_cast<float>(width) - 1.0f;
    const float ndcY = 1.0f - 2.0f * screenY / static_cast<float>(height);

    Eigen::Vector4f nearPoint = invViewProjection * Eigen::Vector4f(ndcX, ndcY, 0.0f, 1.0f);
    Eigen::Vector4f farPoint = invViewProjection * Eigen::Vector4f(ndcX, ndcY, 1.0f, 1.0f);
    if (std::abs(nearPoint.w()) <= 1e-6f || std::abs(farPoint.w()) <= 1e-6f)
        return false;

    nearPoint /= nearPoint.w();
    farPoint /= farPoint.w();

    outOrigin = nearPoint.head<3>();
    outDirection = (farPoint.head<3>() - outOrigin).normalized();
    return outDirection.allFinite();
}

//================================//
static bool RaycastConvexHull(const ConvexHull& hull, const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, float& outDistance)
{
    float enter = 0.0f;
    float exit = std::numeric_limits<float>::infinity();

    for (const HullFace& face : hull.faces)
    {
        const float distance = face.normal.dot(origin) - face.planeOffset;
        const float denom = face.normal.dot(direction);

        if (std::abs(denom) <= 1e-6f)
        {
            if (distance > 0.0f)
                return false;
            continue;
        }

        const float t = -distance / denom;
        if (denom < 0.0f)
            enter = std::max(enter, t);
        else
            exit = std::min(exit, t);

        if (enter > exit)
            return false;
    }

    outDistance = enter;
    return outDistance >= 0.0f;
}

//================================//
GameManager::GameManager(): window(nullptr, &glfwDestroyWindow)
{
    if (!glfwInit())
    {
        std::cout << "[ERROR][GameManager] Failed to initialize GLFW." << std::endl;
        return;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    this->window.reset(glfwCreateWindow(INITIAL_WINDOW_WIDTH, INITIAL_WINDOW_HEIGHT, "SkyePhyX", nullptr, nullptr));

    GLFWwindow* window = this->window.get();

    if (!window)
    {
        std::cerr << "[ERROR][GameManager] Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return;
    }

    WindowFormat windowFormat = { this->window.get(), INITIAL_WINDOW_WIDTH, INITIAL_WINDOW_HEIGHT, false };
    this->renderInfo.width = static_cast<uint32_t>(INITIAL_WINDOW_WIDTH);
    this->renderInfo.height = static_cast<uint32_t>(INITIAL_WINDOW_HEIGHT);

    this->wgpuBundle = std::make_unique<WgpuBundle>(windowFormat);
    this->solver = std::make_unique<Solver>();
    this->renderEngine = std::make_unique<RenderEngine>(this->wgpuBundle.get(), this->solver.get(), this);

#ifdef __EMSCRIPTEN__
    this->RegisterMobileTouchCallbacks();
#endif

    this->correctlyInitialized = true;
}

//================================//
GameManager::~GameManager()
{
    this->wgpuBundle.reset();
    this->window.reset();
    glfwTerminate();
}

//================================//
void GameManager::TickFrame()
{
    if (!this->correctlyInitialized || glfwWindowShouldClose(this->window.get()))
    {
#ifdef __EMSCRIPTEN__
        emscripten_cancel_main_loop();
#endif
        return;
    }

    const int MAX_PHYSICS_STEPS = 5;
    RenderTimings& rt = this->renderEngine->renderTimings;
    Time::TimePoint frameStart = Time::Clock::now();

    this->UpdateCurrentTime();
    Time::TimePoint timeUpdated = Time::Clock::now();

    this->ProcessEvents(this->deltaTime);
    Time::TimePoint eventsProcessed = Time::Clock::now();

    this->physicsAccumulator += this->deltaTime;

    int steps = 0;
    while (this->physicsAccumulator >= this->solver->stepValue && steps < MAX_PHYSICS_STEPS)
    {
        if (!this->paused || this->nextPass)
        {
            this->solver->Step();
            this->nextPass = false;
        }
        this->physicsAccumulator -= this->solver->stepValue;
        steps++;
    }

    if (steps == MAX_PHYSICS_STEPS)
        this->physicsAccumulator = 0.f;

    Time::TimePoint physicsDone = Time::Clock::now();

    Time::TimePoint t0 = Time::Clock::now();
    bool success = this->renderEngine->AcquireSwapchainTexture();
    if (!success)
    {
        this->wgpuBundle->RequestSurfaceReconfigure();
        this->renderInfo.resizeNeeded = true;
        return;
    }

    Time::TimePoint t1 = Time::Clock::now();
    this->renderEngine->SetSolverStepTime();
    this->renderEngine->UpdateInstanceBuffer();

    Time::TimePoint t2 = Time::Clock::now();
    this->renderEngine->UpdateLineBuffer();
    this->renderEngine->UpdateSoftBodySurfaceBuffer();

    Time::TimePoint t3 = Time::Clock::now();
    this->renderEngine->UpdateDebugPointBuffer();

    Time::TimePoint t4 = Time::Clock::now();
    this->renderEngine->Render(static_cast<void*>(&this->renderInfo));

    Time::TimePoint t5 = Time::Clock::now();
#ifndef __EMSCRIPTEN__
    this->wgpuBundle->GetSurface().Present();
#endif
    this->wgpuBundle->GetInstance().ProcessEvents();

    Time::TimePoint t6 = Time::Clock::now();

    rt.updateTime.push(Time::MillisecondsBetween(frameStart, timeUpdated));
    rt.processEvents.push(Time::MillisecondsBetween(timeUpdated, eventsProcessed));
    rt.physics.push(Time::MillisecondsBetween(eventsProcessed, physicsDone));
    rt.physicsSteps.push(static_cast<float>(steps));
    rt.acquireSwapchain.push(Time::MillisecondsBetween(t0, t1));
    rt.updateInstances.push(Time::MillisecondsBetween(t1, t2));
    rt.updateLines.push(Time::MillisecondsBetween(t2, t3));
    rt.updateDebug.push(Time::MillisecondsBetween(t3, t4));
    rt.render.push(Time::MillisecondsBetween(t4, t5));
    rt.present.push(Time::MillisecondsBetween(t5, t6));
    rt.totalFrame.push(Time::MillisecondsBetween(t0, t6));
    rt.fullFrame.push(Time::MillisecondsBetween(frameStart, t6));

    this->AccumulateFrameRate();
}

//================================//
void GameManager::RunMainLoop()
{
    if (!this->correctlyInitialized || this->mainLoopStarted)
        return;

    this->mainLoopStarted = true;
    std::cout << "[INFO][GameManager] Entering main loop...\n";

    this->solver->Start();
    ChangeLevel(1);

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(
        [](void* userData)
        {
            static_cast<GameManager*>(userData)->TickFrame();
        },
        this,
        0,
        true
    );
#else
    while (!glfwWindowShouldClose(this->window.get()))
    {
        this->TickFrame();
    }
#endif
}

//================================//
void GameManager::ProcessEvents(float deltaTime)
{
    glfwPollEvents();

    Camera* camera = this->renderEngine->GetCamera();

    bool isShiftPressed = glfwGetKey(this->window.get(), GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(this->window.get(), GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    float speed = isShiftPressed ? 60.0f : 10.0f;

    double mouseX, mouseY;
    glfwGetCursorPos(this->window.get(), &mouseX, &mouseY);
    bool mouseCapturedByUi = this->renderEngine->IsScreenPointInsideUi(static_cast<float>(mouseX), static_cast<float>(mouseY));

    bool rightMousePressed = glfwGetMouseButton(this->window.get(), GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    bool leftMousePressed = glfwGetMouseButton(this->window.get(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

#ifdef __EMSCRIPTEN__
    const bool touchHandled = this->ProcessTouchEvents(deltaTime);
#else
    const bool touchHandled = false;
#endif

    if (!touchHandled && !mouseCapturedByUi && rightMousePressed)
    {
        if (!this->rMouseClicked)
        {
            this->lastMouseX = static_cast<float>(mouseX);
            this->lastMouseY = static_cast<float>(mouseY);
            this->rMouseClicked = true;
        }
        else
        {
            float deltaX = static_cast<float>(mouseX) - this->lastMouseX;
            float deltaY = static_cast<float>(mouseY) - this->lastMouseY;

            camera->RotateByMouseMovement(-deltaX, -deltaY);

            this->lastMouseX = static_cast<float>(mouseX);
            this->lastMouseY = static_cast<float>(mouseY);
        }
    }
    else
    {
        this->rMouseClicked = false;
    }

    if (!touchHandled)
        this->ProcessPrimaryPointer(leftMousePressed, static_cast<float>(mouseX), static_cast<float>(mouseY), mouseCapturedByUi);
    else if (!leftMousePressed)
    {
        this->lMouseClicked = false;
    }

    Eigen::Vector3f movementDelta(0.0f, 0.0f, 0.0f);
    if (glfwGetKey(this->window.get(), GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(this->window.get(), GLFW_KEY_Z) == GLFW_PRESS)
        movementDelta.z() += deltaTime * speed;
    if (glfwGetKey(this->window.get(), GLFW_KEY_S) == GLFW_PRESS)
        movementDelta.z() -= deltaTime * speed;
    if (glfwGetKey(this->window.get(), GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(this->window.get(), GLFW_KEY_Q) == GLFW_PRESS)
        movementDelta.x() -= deltaTime * speed;
    if (glfwGetKey(this->window.get(), GLFW_KEY_D) == GLFW_PRESS)
        movementDelta.x() += deltaTime * speed;
    //Z, x for up and down
    if (glfwGetKey(this->window.get(), GLFW_KEY_DOWN) == GLFW_PRESS)
        movementDelta.y() -= deltaTime * speed;
    if (glfwGetKey(this->window.get(), GLFW_KEY_UP) == GLFW_PRESS)
        movementDelta.y() += deltaTime * speed;
    if (glfwGetKey(this->window.get(), GLFW_KEY_LEFT) == GLFW_PRESS)
        camera->RotateByMouseMovement(deltaTime * 6.0f * speed, 0.0f);
    if (glfwGetKey(this->window.get(), GLFW_KEY_RIGHT) == GLFW_PRESS)
        camera->RotateByMouseMovement(-deltaTime * 6.0f * speed, 0.0f);
    
    camera->MoveLocal(movementDelta);

    WindowFormat currentFormat = this->wgpuBundle->GetWindowFormat();
    this->renderInfo.width = static_cast<uint32_t>(currentFormat.width);
    this->renderInfo.height = static_cast<uint32_t>(currentFormat.height);
    this->renderInfo.resizeNeeded = currentFormat.resizeNeeded;

    bool pKeyPressed = glfwGetKey(this->window.get(), GLFW_KEY_P) == GLFW_PRESS;
    if (pKeyPressed && !this->pKeyWasPressed)
        this->paused = !this->paused;
    this->pKeyWasPressed = pKeyPressed;

    bool nextPassKeyPressed = glfwGetKey(this->window.get(), GLFW_KEY_N) == GLFW_PRESS;
    if (nextPassKeyPressed && !this->nextPassKeyWasPressed)
        this->nextPass = true;
    this->nextPassKeyWasPressed = nextPassKeyPressed;

    bool restartKeyPressed = glfwGetKey(this->window.get(), GLFW_KEY_R) == GLFW_PRESS;
    if (restartKeyPressed && !this->restartKeyWasPressed)
        ChangeLevel(this->currentLevel);
    this->restartKeyWasPressed = restartKeyPressed;

    bool randomBoxSpawnedKeyPressed = glfwGetKey(this->window.get(), GLFW_KEY_B) == GLFW_PRESS;
    if (randomBoxSpawnedKeyPressed && !this->randomBoxSpawnedPressed)
        this->SpawnRandomMesh();
    this->randomBoxSpawnedPressed = randomBoxSpawnedKeyPressed;
}

//================================//
void GameManager::UpdateCurrentTime()
{
#ifdef __EMSCRIPTEN__
    double currentTime = emscripten_get_now() / 1000.0;
#else
    double currentTime = static_cast<double>(glfwGetTime());
#endif

    if (this->lastFrameTime == 0.0f)
            this->lastFrameTime = static_cast<float>(currentTime);

    this->deltaTime = static_cast<float>(currentTime - this->lastFrameTime);
    this->lastFrameTime = static_cast<float>(currentTime);

    this->renderInfo.time = currentTime;

#ifdef __EMSCRIPTEN__
    this->deltaTime = std::min(this->deltaTime, 0.1f);
#endif
}

//================================//
void GameManager::AccumulateFrameRate()
{
    if (this->deltaTime <= 0.0f)
        return;

    this->frameRateAccumulator.push_back(1.0f / this->deltaTime);
    if (this->frameRateAccumulator.size() >= 100)
    {
        float sum = 0.0f;
        for (float fr : this->frameRateAccumulator)
            sum += fr;
        this->frameRate = sum / static_cast<float>(this->frameRateAccumulator.size());
        this->frameRateAccumulator.clear();
    }
}

//================================//
void GameManager::ChangeLevel(int levelIndex)
{
    if (levelIndex < 0 || levelIndex >= numLevels)
        return;

    ReleaseObjectPicker();
    this->currentLevel = levelIndex;
    this->solver->Clear();

    levels[levelIndex](this->solver.get(), this->renderEngine->GetCamera(), this->levelParameters);
}

//================================//
void GameManager::SpawnRandomMesh()
{
    int numBodies = static_cast<int>(this->solver->bodyPtrs.size());

    ModelType modelType;
    float choseModel = rand01();
    if (choseModel < 0.25f)
        modelType = ModelType_Cube;
    else if (choseModel < 0.5f)
        modelType = ModelType_Sphere;
    else if (choseModel < 0.75f)
        modelType = ModelType_Capsule;
    else
        modelType = ModelType_TestConvexMesh;

    float maxScalePerAxis = 3.f;
    float minScalePerAxis = 1.f;

    float randomScaleX = rand01() * maxScalePerAxis;
    if (randomScaleX < minScalePerAxis) randomScaleX = minScalePerAxis;

    float randomScaleY = rand01() * maxScalePerAxis;
    if (randomScaleY < minScalePerAxis) randomScaleY = minScalePerAxis;

    float randomScaleZ = rand01() * maxScalePerAxis;
    if (randomScaleZ < minScalePerAxis) randomScaleZ = minScalePerAxis;

    Eigen::Vector3f randomScale = Eigen::Vector3f(randomScaleX, randomScaleY, randomScaleZ);
    if (modelType == ModelType_Capsule || modelType == ModelType_Sphere)
    {
        float maxUniformScale = std::min({ randomScaleX, randomScaleY, randomScaleZ });
        randomScale = Eigen::Vector3f(maxUniformScale, maxUniformScale, maxUniformScale);
    }

    float minBounds[3] = { -10.f, 3.f, -10.f };
    float maxBounds[3] = { 10.f, 10.f, 10.f };

    Eigen::Vector3f randomPos = genRandomPos(minBounds, maxBounds);

    Quaternionf randomRotation = Quaternionf::UnitRandom();

    Eigen::Vector3f randomColor = Eigen::Vector3f(rand01(), rand01(), rand01());

    Mesh* mesh = this->solver->AddBody(modelType, 1.0f, 0.6f, randomPos, randomScale, Eigen::Vector3f(0.0f, 0.0f, 0.0f), randomRotation, Eigen::Vector3f(0.0f, 0.0f, 0.0f), false, randomColor);
    mesh->name = "Random " + std::to_string(modelType) + " " + std::to_string(numBodies);
}

//================================//
void GameManager::SpawnShootingSphere(float mouseX, float mouseY)
{
    int numBodies = static_cast<int>(this->solver->bodyPtrs.size());

    Eigen::Vector3f spawnPos = this->renderEngine->GetCamera()->GetPosition();

    float sphereRadius = 1.0f;

    // Shoot in direction of mouseX mouseY as if it was a ray from center to a plane 1m in front of the camera
    Eigen::Vector3f rayOrigin, rayDirection;
    WindowFormat currentFormat = this->wgpuBundle->GetWindowFormat();
    if (!GetMouseRay(this->renderEngine->GetCamera(), this->window.get(), static_cast<uint32_t>(currentFormat.width), static_cast<uint32_t>(currentFormat.height), static_cast<float>(mouseX), static_cast<float>(mouseY), rayOrigin, rayDirection))
        rayDirection = this->renderEngine->GetCamera()->GetForwardDirection();
    
    Eigen::Vector3f shootingDirection = rayDirection.normalized();

    float shootSpeed = 50.0f;
    Eigen::Vector3f smallUpDrift = Eigen::Vector3f(0.0f, 0.1f, 0.0f);
    Eigen::Vector3f initialVelocity = (shootingDirection + smallUpDrift).normalized() * shootSpeed;

    Eigen::Vector3f randomColor = Eigen::Vector3f(rand01(), rand01(), rand01());

    Mesh* mesh = this->solver->AddBody(ModelType_Sphere, 1.0f, 0.5f, spawnPos, Eigen::Vector3f(sphereRadius, sphereRadius, sphereRadius), initialVelocity, Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), false, randomColor);
    mesh->name = "Shooting Sphere " + std::to_string(numBodies);
}

//================================//
bool GameManager::ShouldSpawnDragSphere(float screenX, float screenY)
{
    constexpr double kSpawnIntervalSeconds = 0.075;
    constexpr float kMinDragDistancePixels = 18.0f;

    const double now = this->renderInfo.time;
    const float dx = screenX - this->lastSphereDragX;
    const float dy = screenY - this->lastSphereDragY;
    const float distanceSquared = dx * dx + dy * dy;

    return (now - this->lastSphereDragSpawnTime) >= kSpawnIntervalSeconds &&
        distanceSquared >= kMinDragDistancePixels * kMinDragDistancePixels;
}

//================================//
void GameManager::ProcessPrimaryPointer(bool pressed, float screenX, float screenY, bool capturedByUi)
{
    if (!pressed || capturedByUi)
    {
        ReleaseObjectPicker();
        this->lMouseClicked = false;
        return;
    }

    if (this->pickMeshes)
    {
        this->objectPicker.screenX = screenX;
        this->objectPicker.screenY = screenY;

        if (!this->lMouseClicked)
        {
            this->objectPicker.active = RaycastFromMouse(
                screenX,
                screenY,
                this->objectPicker.localHitPoint,
                this->objectPicker.pickedMesh,
                this->objectPicker.hitDistance
            );

            if (this->objectPicker.active)
            {
                Eigen::Vector3f target = ProjectMouseToPickDepth(screenX, screenY, this->objectPicker.hitDistance);
                Vector6f stiffness = Vector6f::Zero();
                stiffness.head<3>().setConstant(8000.0f);
                this->objectPicker.mouseJoint = static_cast<Joint*>(this->solver->AddForce(std::make_unique<Joint>(
                    this->solver.get(),
                    nullptr,
                    target,
                    this->objectPicker.pickedMesh,
                    this->objectPicker.localHitPoint,
                    stiffness
                )));
                for (int i = 0; i < 3; ++i)
                    this->objectPicker.mouseJoint->constraintPoints[i].penalty = stiffness[i];
            }
        }

        if (this->objectPicker.active && this->objectPicker.mouseJoint)
            this->objectPicker.mouseJoint->rA = ProjectMouseToPickDepth(screenX, screenY, this->objectPicker.hitDistance);
    }
    else if (this->shootSpheres)
    {
        if (!this->lMouseClicked || this->ShouldSpawnDragSphere(screenX, screenY))
        {
            SpawnShootingSphere(screenX, screenY);
            this->lastSphereDragSpawnTime = this->renderInfo.time;
            this->lastSphereDragX = screenX;
            this->lastSphereDragY = screenY;
        }
    }

    this->lMouseClicked = true;
}

#ifdef __EMSCRIPTEN__
//================================//
void GameManager::RegisterMobileTouchCallbacks()
{
    emscripten_set_touchstart_callback("#canvas", this, false, &GameManager::HandleTouchEvent);
    emscripten_set_touchmove_callback("#canvas", this, false, &GameManager::HandleTouchEvent);
    emscripten_set_touchend_callback("#canvas", this, false, &GameManager::HandleTouchEvent);
    emscripten_set_touchcancel_callback("#canvas", this, false, &GameManager::HandleTouchEvent);
}

//================================//
EM_BOOL GameManager::HandleTouchEvent(int eventType, const EmscriptenTouchEvent* touchEvent, void* userData)
{
    GameManager* manager = static_cast<GameManager*>(userData);
    if (!manager || !touchEvent)
        return EM_FALSE;

    TouchInputState& touch = manager->touchInput;
    touch.touchCount = std::min(touchEvent->numTouches, 2);
    touch.active = touch.touchCount > 0 && eventType != EMSCRIPTEN_EVENT_TOUCHCANCEL;

    if (!touch.active)
    {
        touch.touchCount = 0;
        touch.pinchActive = false;
        return EM_TRUE;
    }

    const EmscriptenTouchPoint& firstTouch = touchEvent->touches[0];
    touch.primaryX = static_cast<float>(firstTouch.targetX);
    touch.primaryY = static_cast<float>(firstTouch.targetY);

    if (!touch.wasActive || eventType == EMSCRIPTEN_EVENT_TOUCHSTART)
    {
        touch.previousPrimaryX = touch.primaryX;
        touch.previousPrimaryY = touch.primaryY;
        touch.consumedByUi = manager->renderEngine &&
            manager->renderEngine->IsScreenPointInsideUi(touch.primaryX, touch.primaryY);
    }

    if (touch.touchCount >= 2)
    {
        const EmscriptenTouchPoint& secondTouch = touchEvent->touches[1];
        const float secondX = static_cast<float>(secondTouch.targetX);
        const float secondY = static_cast<float>(secondTouch.targetY);
        const float dx = secondX - touch.primaryX;
        const float dy = secondY - touch.primaryY;
        touch.pinchDistance = std::sqrt(dx * dx + dy * dy);
    }

    return EM_TRUE;
}

//================================//
bool GameManager::ProcessTouchEvents(float deltaTime)
{
    (void)deltaTime;

    if (!this->touchInput.active && !this->touchInput.wasActive)
        return false;

    Camera* camera = this->renderEngine->GetCamera();

    if (!this->touchInput.active)
    {
        this->ProcessPrimaryPointer(false, this->touchInput.previousPrimaryX, this->touchInput.previousPrimaryY, false);
        this->touchInput.wasActive = false;
        this->touchInput.consumedByUi = false;
        this->touchInput.pinchActive = false;
        return true;
    }

    if (this->touchInput.consumedByUi)
    {
        this->ProcessPrimaryPointer(false, this->touchInput.primaryX, this->touchInput.primaryY, true);
        this->touchInput.previousPrimaryX = this->touchInput.primaryX;
        this->touchInput.previousPrimaryY = this->touchInput.primaryY;
        this->touchInput.wasActive = true;
        return true;
    }

    if (this->touchInput.touchCount >= 2)
    {
        ReleaseObjectPicker();
        this->lMouseClicked = false;

        if (!this->touchInput.pinchActive || this->touchInput.previousPinchDistance <= 1.0f)
        {
            this->touchInput.previousPinchDistance = this->touchInput.pinchDistance;
            this->touchInput.pinchActive = true;
        }
        else if (this->touchInput.pinchDistance > 1.0f)
        {
            const float ratio = this->touchInput.pinchDistance / this->touchInput.previousPinchDistance;
            const float zoomDelta = std::clamp(std::log(ratio) * 25.0f, -8.0f, 8.0f);
            if (std::isfinite(zoomDelta))
                camera->MoveForwardBy(zoomDelta);
            this->touchInput.previousPinchDistance = this->touchInput.pinchDistance;
        }

        this->touchInput.previousPrimaryX = this->touchInput.primaryX;
        this->touchInput.previousPrimaryY = this->touchInput.primaryY;
        this->touchInput.wasActive = true;
        return true;
    }

    this->touchInput.pinchActive = false;
    this->ProcessPrimaryPointer(true, this->touchInput.primaryX, this->touchInput.primaryY, false);
    this->touchInput.previousPrimaryX = this->touchInput.primaryX;
    this->touchInput.previousPrimaryY = this->touchInput.primaryY;
    this->touchInput.wasActive = true;
    return true;
}
#endif

//================================//
void GameManager::ReleaseObjectPicker()
{
    if (this->objectPicker.mouseJoint)
        this->solver->RemoveForce(this->objectPicker.mouseJoint);

    this->objectPicker = ObjectPicker{};
}

//================================//
Eigen::Vector3f GameManager::ProjectMouseToPickDepth(float screenX, float screenY, float depth)
{
    Eigen::Vector3f origin, direction;
    WindowFormat currentFormat = this->wgpuBundle->GetWindowFormat();
    if (!GetMouseRay(this->renderEngine->GetCamera(), this->window.get(), static_cast<uint32_t>(currentFormat.width), static_cast<uint32_t>(currentFormat.height), screenX, screenY, origin, direction))
        return origin;

    return origin + direction * depth;
}

//================================//
bool GameManager::RaycastFromMouse(float screenX, float screenY, Eigen::Vector3f& outHitPoint, Mesh*& outHitMesh, float& outHitDistance)
{
    Eigen::Vector3f rayOrigin, rayDirection;
    WindowFormat currentFormat = this->wgpuBundle->GetWindowFormat();
    if (!GetMouseRay(this->renderEngine->GetCamera(), this->window.get(), static_cast<uint32_t>(currentFormat.width), static_cast<uint32_t>(currentFormat.height), screenX, screenY, rayOrigin, rayDirection))
        return false;

    float bestDistance = std::numeric_limits<float>::infinity();
    Mesh* bestMesh = nullptr;
    Eigen::Vector3f bestHitPoint = Eigen::Vector3f::Zero();

    for (Mesh* mesh : this->solver->bodyPtrs)
    {
        if (!mesh || mesh->isStatic || mesh->isInvisible)
            continue;

        const ConvexHull hull = this->solver->GetModelConvexHull(mesh->modelType);
        if (hull.faces.empty())
            continue;

        const Eigen::Vector3f scale = mesh->transform.GetScale();
        if (std::abs(scale.x()) <= 1e-6f || std::abs(scale.y()) <= 1e-6f || std::abs(scale.z()) <= 1e-6f)
            continue;

        const Quaternionf invRotation = mesh->transform.GetRotation().conjugate();
        const Eigen::Vector3f localOrigin = (invRotation * (rayOrigin - mesh->transform.GetPosition())).cwiseQuotient(scale);
        const Eigen::Vector3f localDirection = (invRotation * rayDirection).cwiseQuotient(scale);

        float hitDistance = 0.0f;
        if (!RaycastConvexHull(hull, localOrigin, localDirection, hitDistance))
            continue;

        if (hitDistance < bestDistance)
        {
            bestDistance = hitDistance;
            bestMesh = mesh;
            bestHitPoint = localOrigin + localDirection * hitDistance;
        }
    }

    if (!bestMesh)
        return false;

    outHitPoint = bestHitPoint;
    outHitMesh = bestMesh;
    outHitDistance = bestDistance;
    return true;
}
