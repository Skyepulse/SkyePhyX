#include <iostream>
#include "src/GameManager.hpp"
#include <charconv>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

std::unique_ptr<GameManager> g_manager;

//================================//
int main(int argc, char** argv)
{
    std::cout << "Starting SkyePhyX..." << std::endl;
    
    g_manager = std::make_unique<GameManager>();
    g_manager->InitGraphics();
    g_manager->RunMainLoop();

#ifndef __EMSCRIPTEN__
    g_manager.reset();
#endif

    std::cout << "Exiting application." << std::endl;
    return 0;
}
