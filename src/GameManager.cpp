#include "GameManager.hpp"

#include <iostream>
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

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
    this->renderEngine = std::make_unique<RenderEngine>(this->wgpuBundle.get(), this->solver.get());

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
void GameManager::RunMainLoop()
{
    if (!this->correctlyInitialized)
        return;

    std::cout << "[INFO][GameManager] Entering main loop...\n";   

    const int MAX_PHYSICS_STEPS = 5;
    float accumulator = 0.f;

    this->solver->Start();

    while (!glfwWindowShouldClose(this->window.get()))
    {
        this->UpdateCurrentTime();
        this->ProcessEvents(this->deltaTime);

        accumulator += this->deltaTime;

        int steps = 0;
        while (accumulator >= this->solver->stepValue && steps < MAX_PHYSICS_STEPS)
        {
            if (!this->paused || this->nextPass)
            {
                this->solver->Step();
                this->nextPass = false;
            }
            accumulator -= this->solver->stepValue;
            steps++;
        }

        if (steps == MAX_PHYSICS_STEPS)
            accumulator = 0.f;

        using Clock = std::chrono::high_resolution_clock;
        RenderTimings& rt = this->renderEngine->renderTimings;

        auto t0 = Clock::now();
        this->renderEngine->AcquireSwapchainTexture();

        auto t1 = Clock::now();
        this->renderEngine->SetSolverStepTime(this->solver->averageStepTime);
        this->renderEngine->UpdateInstanceBuffer(this->solver->solverBodies);

        auto t2 = Clock::now();
        this->renderEngine->UpdateLineBuffer(this->solver->lineData);

        auto t3 = Clock::now();
        this->renderEngine->UpdateDebugPointBuffer(this->solver->debugPointData);

        auto t4 = Clock::now();
        this->renderEngine->Render(static_cast<void*>(&this->renderInfo));

        auto t5 = Clock::now();
        this->wgpuBundle->GetSurface().Present();
        this->wgpuBundle->GetInstance().ProcessEvents();

        auto t6 = Clock::now();

        auto ms = [](auto start, auto end) {
            return std::chrono::duration<float, std::milli>(end - start).count();
        };

        rt.acquireSwapchain.push(ms(t0, t1));
        rt.updateInstances.push(ms(t1, t2));
        rt.updateLines.push(ms(t2, t3));
        rt.updateDebug.push(ms(t3, t4));
        rt.render.push(ms(t4, t5));
        rt.present.push(ms(t5, t6));
        rt.totalFrame.push(ms(t0, t6));

        this->AccumulateFrameRate();
    }
}

//================================//
void GameManager::ProcessEvents(float deltaTime)
{
    glfwPollEvents();

    Camera* camera = this->renderEngine->GetCamera();

    bool isShiftPressed = glfwGetKey(this->window.get(), GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(this->window.get(), GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    float speed = isShiftPressed ? 60.0f : 10.0f;

    if (glfwGetMouseButton(this->window.get(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        double mouseX, mouseY;
        glfwGetCursorPos(this->window.get(), &mouseX, &mouseY);

        if (!this->mouseClicked)
        {
            this->lastMouseX = static_cast<float>(mouseX);
            this->lastMouseY = static_cast<float>(mouseY);
            this->mouseClicked = true;
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
        this->mouseClicked = false;
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
        camera->RotateByMouseMovement(-deltaTime * 60.0f, 0.0f);
    if (glfwGetKey(this->window.get(), GLFW_KEY_RIGHT) == GLFW_PRESS)
        camera->RotateByMouseMovement(deltaTime * 60.0f, 0.0f);
    
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
        this->solver->Start();
    this->restartKeyWasPressed = restartKeyPressed;
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