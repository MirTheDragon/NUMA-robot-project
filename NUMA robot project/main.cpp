#include "face_renderer.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <random>



// You can replace this later with your own Robot class or modules
void updateRobotLogic() {
    // TODO: Add your movement, sensing, decision logic here
    //std::cout << "Running control logic..." << std::endl;
}

int main() {
    std::cout << "Starting robot controller..." << std::endl;

    // === Loop Control ===
    const int loopRateHz = 60; // 60 times per second
    const auto loopInterval = std::chrono::milliseconds(1000 / loopRateHz);

    std::atomic<bool> running(true);

    // === Initialize Graphics ===
    
    if (!initGraphics()) return 1;
    
    bool quit = false;
    SDL_Event e;

    // === Optional Signal Handling (e.g., Ctrl+C) ===
    // Can add signal(SIGINT, ...) here later if needed

    // === Main Loop ===
    while (running) {
        auto loopStart = std::chrono::steady_clock::now();

        //updateRobotLogic();


        // Process SDL events (e.g., quit, keys)
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running = false;
            } else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) {
                running = false;
            }
        }



        // Render the robot's face
        updateAndDrawEyes();
        

        // Wait for remainder of loop time
        std::this_thread::sleep_until(loopStart + loopInterval);
    }



    std::cout << "Shutting down robot controller." << std::endl;
    
    shutdownGraphics();
    std::cout << "Graphics shutdown complete." << std::endl;
    // Cleanup and exit

    SDL_Quit();
    std::cout << "SDL cleanup complete." << std::endl;
    std::cout << "Exiting program." << std::endl;
    // Exit gracefully
    if (SDL_WasInit(SDL_INIT_VIDEO)) {
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        std::cout << "SDL video subsystem cleaned up." << std::endl;
    }

    return 0;
}
