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

    this->renderInfo.width = static_cast<uint32_t>(INITIAL_WINDOW_WIDTH);
    this->renderInfo.height = static_cast<uint32_t>(INITIAL_WINDOW_HEIGHT);

    this->wgpuBundle = std::make_unique<WgpuBundle>(windowFormat);

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
GameManager::RunMainLoop()
{
    if (!this->correctlyInitialized)
        return;

    std::cout << "[INFO][GameManager] Entering main loop...\n";   

    while (!glfwWindowShouldClose(this->window.get()))
    {
        this->wgpuBundle->GetSurface().Present();
        this->wgpuBundle->GetInstance().ProcessEvents();
    }
}