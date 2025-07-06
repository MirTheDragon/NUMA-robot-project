#include "face_renderer.hpp"
#include "gamepad.hpp"
#include "PathPlanner.hpp"  // Add your PathPlanner header
#include <vector>
#include <string>
#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <csignal>
#include <cstdlib> // for system()


// Globals for path planning
PathPlanner* pathPlanner = nullptr;
std::vector<WalkCycle*> walkCycles;
size_t currentWalkCycleIndex = 0;

// Gamepad event queue with thread safety
std::queue<GamepadEvent> gamepadEventQueue;
std::mutex eventQueueMutex;
std::condition_variable eventQueueCV;

// Gamepad instance (global so accessible in threads)
GamepadController pad;

// Globals
GamepadState sharedState;
std::mutex stateMutex;

// Drawing synchronization
std::mutex drawMutex;
std::condition_variable drawCV;
bool drawRequested = false;
std::atomic<bool> running(true);


void gamepadPollingThread() {
    while (running) {
        pad.update(sharedState);  // Update joystick state here
        auto events = pad.pollEvents();

        {
            std::lock_guard<std::mutex> lock(eventQueueMutex);
            for (auto& ev : events) {
                gamepadEventQueue.push(ev);
            }
        }
        eventQueueCV.notify_one();

        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // adjust as needed
    }
}

// Drawing thread function
void drawingThread() {
    const auto drawInterval = std::chrono::milliseconds(33); // 30 FPS approx
    auto nextDrawTime = std::chrono::steady_clock::now();

    while (running) {
        std::unique_lock<std::mutex> lock(drawMutex);

        // Wait until either draw requested or timeout for next frame
        drawCV.wait_until(lock, nextDrawTime, []() { return drawRequested || !running; });

        if (!running) break;

        if (drawRequested || std::chrono::steady_clock::now() >= nextDrawTime) {
            lock.unlock();  // Unlock during drawing to avoid blocking main loop

            auto drawStart = std::chrono::steady_clock::now();

            updateAndDrawEyes();  // Your drawing function

            auto drawEnd = std::chrono::steady_clock::now();
            auto drawDuration = std::chrono::duration_cast<std::chrono::microseconds>(drawEnd - drawStart).count();
            std::cout << "Drawing time (us): " << drawDuration << std::endl;

            lock.lock();
            drawRequested = false;
            nextDrawTime = drawStart + drawInterval;
        }
    }
}



int main() {
    std::cout << "Starting robot controller..." << std::endl;

    const int loopRateHz = 120;
    const auto loopInterval = std::chrono::milliseconds(1000 / loopRateHz);

    std::cout << "GamepadController created." << std::endl;

    if (!pad.initialize({ /* your combo definitions if needed */ })) {
        std::cerr << "Warning: Gamepad not detected!\n";
    } else {
        std::cout << "Gamepad initialized successfully.\n";
    }

    if (!initGraphics()) return 1;

    // Initialize walk cycles
    static WalkCycle walkCycle3Set({ {0, 2, 4}, {1, 3, 5} }, 1.f, 1.0f, 0.05f);
    static WalkCycle walkCycle2Set({ {0, 3}, {1, 4}, {2, 5} }, 3.f, 0.5f, 0.05f);
    static WalkCycle walkCycle1Set({ {0}, {1}, {2}, {3}, {4}, {5} }, 6.f, 0.2f, 0.05f);

    walkCycles = { &walkCycle3Set, &walkCycle2Set, &walkCycle1Set };

    RobotController robot(6);
    if (robot.initialize() != 0) {
        std::cerr << "Failed to initialize robot controller." << std::endl;
        return 1;
    }

    robot.Body.position = { 0.f, 0.f, robot.Body.originStartingHeight + 20.0f };
    pathPlanner = new PathPlanner(robot);
    pathPlanner->currentWalkCycle_ = walkCycles[0];
    currentWalkCycleIndex = 0;

    GamepadState state;


    SDL_Event e;

    // Start threads
    std::thread padThread(gamepadPollingThread);
    std::thread drawThread(drawingThread);

    while (running) {
        auto loopStart = std::chrono::steady_clock::now();

        // Handle SDL events
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) running = false;
        }

        // Process gamepad events from queue
        {
            std::lock_guard<std::mutex> lock(eventQueueMutex);
            while (!gamepadEventQueue.empty()) {
                const auto& ev = gamepadEventQueue.front();

                if (ev.type == "button" && ev.name == "Y" && ev.action == "pressed") {
                    currentWalkCycleIndex = (currentWalkCycleIndex + 1) % walkCycles.size();
                    pathPlanner->requestWalkCycleSwitch(walkCycles[currentWalkCycleIndex]);
                    std::cout << "Switched to walk cycle index: " << currentWalkCycleIndex << std::endl;
                }

                gamepadEventQueue.pop();
            }
        }

        // Update gamepad state joystick input for path planner
        Vec2 leftJoystickInput(sharedState.leftStick.x, sharedState.leftStick.y);
        if (leftJoystickInput.length() < 0.05f) {
            leftJoystickInput = Vec2(0.f, 0.f);
        }

        // Time PathPlanner update
        auto pathStart = std::chrono::steady_clock::now();
        pathPlanner->update(leftJoystickInput, loopInterval.count() / 1000.0f);
        auto pathEnd = std::chrono::steady_clock::now();
        auto pathDuration = std::chrono::duration_cast<std::chrono::microseconds>(pathEnd - pathStart).count();

        // Time IK update
        auto ikStart = std::chrono::steady_clock::now();
        robot.updateKinematicsAndApply();
        auto ikEnd = std::chrono::steady_clock::now();
        auto ikDuration = std::chrono::duration_cast<std::chrono::microseconds>(ikEnd - ikStart).count();

        // Request drawing
        {
            std::lock_guard<std::mutex> lock(drawMutex);
            drawRequested = true;
        }
        drawCV.notify_one();

        auto loopEnd = std::chrono::steady_clock::now();
        auto loopDuration = std::chrono::duration_cast<std::chrono::microseconds>(loopEnd - loopStart).count();

        std::cout << "Timing (us): PathPlanner=" << pathDuration
                << " IK=" << ikDuration
                << " TotalLoop=" << loopDuration << std::endl;

        // Sleep to maintain loop rate
        std::this_thread::sleep_until(loopStart + loopInterval);
    }

    running = false;

    drawCV.notify_all();
    eventQueueCV.notify_all();

    padThread.join();
    drawThread.join();

    std::cout << "Shutting down robot controller." << std::endl;
    shutdownGraphics();
    std::cout << "Graphics shutdown complete." << std::endl;

    SDL_Quit();
    std::cout << "SDL cleanup complete." << std::endl;

    delete pathPlanner;
    return 0;
}
