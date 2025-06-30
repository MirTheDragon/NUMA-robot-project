#include "face_renderer.hpp"
#include "gamepad.hpp" // We'll define this next
#include <vector>
#include <string>
#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstdlib> // for system()


std::atomic<bool> running(true);
std::atomic<bool> faceVisible(true);

// You can replace this later with your own Robot class or modules
void updateRobotLogic() {
    // TODO: Add your movement, sensing, decision logic here
    //std::cout << "Running control logic..." << std::endl;
}

//Combination event definition tests
std::vector<ComboEvent> comboDefinitions = {
    ComboEvent{"A", {"DOWN", "DOWN"}, "Sleep"},
    ComboEvent{"A", {"UP", "UP"}, "Wake"},
    ComboEvent{"A", {"LEFT", "RIGHT", "LEFT"}, "Shake"},
    ComboEvent{"A", {"DOWN", "LEFT", "RIGHT"}, "Scared"},
    ComboEvent{"A", {"UP", "LEFT", "RIGHT"}, "Dance"}
};


int main() {
    std::cout << "Starting robot controller..." << std::endl;

    // === Loop Control ===
    const int loopRateHz = 60; // 60 times per second
    const auto loopInterval = std::chrono::milliseconds(1000 / loopRateHz);

     // === Initialize Controller ===

    GamepadController pad;
    GamepadState state;
    std::cout << "GamepadController created." << std::endl;

    if (!pad.initialize(comboDefinitions)) {
        std::cerr << "Warning: Gamepad not detected!\n";
    } else {
        std::cout << "Gamepad initialized successfully.\n";
    }
    

    // === Initialize Graphics ===
    
    if (!initGraphics()) return 1;
    
    bool quit = false;
    SDL_Event e;


    // === Optional Signal Handling (e.g., Ctrl+C) ===
    // Can add signal(SIGINT, ...) here later if needed

    // === Main Loop ===
    std::cout << "Main Loop Running.\n";
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

        
        pad.update(state);

        pad.printJoystickAndTriggerChanges();

        // Get events (including combos, button press/releases)
        auto events = pad.pollEvents();
        for (const auto& ev : events) {
            if (ev.type == "button") {
                // Example: react to some buttons
                if (ev.name == "A" && ev.action == "pressed") {
                    std::cout << "Button A was pressed\n";
                } else if (ev.name == "B" && ev.action == "pressed") {
                    std::cout << "Button B was pressed\n";
                } else if (ev.name == "X" && ev.action == "pressed") {
                    std::cout << "Button X was pressed\n";
                } else if (ev.name == "Y" && ev.action == "pressed") {
                    std::cout << "Button Y was pressed\n";

                } else if (ev.name == "Down" && ev.action == "pressed") {
                    std::cout << "Button Down was pressed\n";
                } else if (ev.name == "Up" && ev.action == "pressed") {
                    std::cout << "Button Up was pressed\n";
                } else if (ev.name == "Left" && ev.action == "pressed") {
                    std::cout << "Button Left was pressed\n";
                } else if (ev.name == "Right" && ev.action == "pressed") {
                    std::cout << "Button Right was pressed\n";
                    
                } else if (ev.name == "Minus" && ev.action == "pressed") {
                    std::cout << "Minus button event, toggling face.\n";
                    faceVisible = !faceVisible;
                    if (faceVisible) {
                        initGraphics();
                    } else {
                        shutdownGraphics();
                    }
                }
                // Add more buttons as needed

            } else if (ev.type == "combo") {
                std::cout << "Combo triggered: " << ev.name << "\n";

                // React to combos by name
                if (ev.name == "Sleep") {
                    std::cout << "Sleep combo activated!\n";
                } else if (ev.name == "Wake") {
                    std::cout << "Wake combo activated!\n";
                } else if (ev.name == "Shake") {
                    std::cout << "Shake combo activated!\n";
                } else if (ev.name == "Scared") {
                    std::cout << "Scared combo activated!\n";
                } else if (ev.name == "Dance") {
                    std::cout << "Dance combo activated!\n";
                }
                // Add more combos as needed
            }
        }

        /*
        for (const auto& [buttonName, event] : state.buttonEvents) {
            if (event == ButtonEvent::Pressed) {
                std::cout << "Pressed: " << buttonName << "\n";
            } else if (event == ButtonEvent::Released) {
                std::cout << "Released: " << buttonName << "\n";
            }
        }


        // Render the robot's face
        if (state.buttonEvents["Minus"] == ButtonEvent::Pressed) {
            faceVisible = !faceVisible;
            if (faceVisible) {
                initGraphics();
            } else {
                shutdownGraphics();
            }
        }

        */

        if (faceVisible) {
            updateAndDrawEyes();
        }
        

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
