#include "face_renderer.hpp"
#include "gamepad.hpp"
#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

std::atomic<bool> running(true);
std::atomic<bool> faceVisible(true);

void updateRobotLogic() {
    // Put your robot movement/logic here
}

int main() {
    std::cout << "Starting robot controller..." << std::endl;

    const int loopRateHz = 60;
    const auto loopInterval = std::chrono::milliseconds(1000 / loopRateHz);

    // === Initialize Controller ===
    GamepadController pad;
    GamepadState state;
    std::cout << "GamepadController created.\n";

    if (!pad.initialize({})) {
        std::cerr << "⚠️ No controller found at startup.\n";
    }

    // === Initialize Graphics ===
    if (!initGraphics()) return 1;

    SDL_Event e;
    std::cout << "🟢 Main Loop Running\n";

    while (running) {
        auto loopStart = std::chrono::steady_clock::now();

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT || 
                (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)) {
                running = false;
            }
        }

        pad.update(state);
        pad.printJoystickAndTriggerChanges();

        // Get all button or combo events
        auto events = pad.pollEvents();
        for (const auto& ev : events) {
            if (ev.type == "button") {
                std::cout << "🔘 Button " << ev.name << " " << ev.action << "\n";

                if (ev.name == "Minus" && ev.action == "pressed") {
                    faceVisible = !faceVisible;
                    std::cout << (faceVisible ? "🟢 Showing face\n" : "⚫ Hiding face\n");
                    if (faceVisible) {
                        initGraphics();
                    } else {
                        shutdownGraphics();
                    }
                }

                // Add logic for button presses here
            }

            else if (ev.type == "combo") {
                std::cout << "🎯 Combo detected: " << ev.name << "\n";

                // Handle by name
                if (ev.name == "A+Left") std::cout << "👈 A+Left triggered!\n";
                else if (ev.name == "B+Down") std::cout << "👇 B+Down triggered!\n";
                else if (ev.name == "X+Up") std::cout << "☝️ X+Up triggered!\n";
                else if (ev.name == "Y+Right") std::cout << "👉 Y+Right triggered!\n";

                // Add other combos here easily
            }
        }

        if (faceVisible) updateAndDrawEyes();

        std::this_thread::sleep_until(loopStart + loopInterval);
    }

    std::cout << "Shutting down...\n";
    shutdownGraphics();
    SDL_Quit();
    return 0;
}
