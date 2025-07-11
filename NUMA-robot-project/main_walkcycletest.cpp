#include "face_renderer.hpp"
#include "gamepad.hpp"
#include "PathPlanner.hpp"
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
#include <cstdlib>

PathPlanner* pathPlanner = nullptr;
std::vector<WalkCycle*> walkCycles;
size_t currentWalkCycleIndex = 0;

std::queue<GamepadEvent> gamepadEventQueue;
std::mutex eventQueueMutex;
std::condition_variable eventQueueCV;

GamepadController pad;

GamepadState sharedState;
std::mutex stateMutex;

std::atomic<bool> running(true);

std::mutex ikTimingMutex;
uint64_t ikDurationUs = 0;

std::mutex mainTimingMutex;
uint64_t mainLoopDurationUs = 0;
uint64_t pathPlannerDurationUs = 0;

void gamepadPollingThread() {
    while (running) {
        pad.update(sharedState);
        auto events = pad.pollEvents();

        {
            std::lock_guard<std::mutex> lock(eventQueueMutex);
            for (auto& ev : events) {
                gamepadEventQueue.push(ev);
            }
        }
        eventQueueCV.notify_one();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void ikThread(RobotController& robot) {
    const int ikLoopHz = 120;
    const auto ikLoopInterval = std::chrono::milliseconds(1000 / ikLoopHz);

    while (running) {
        auto start = std::chrono::steady_clock::now();

        // Lock joystick state copy for thread safety
        Vec2 joystickCopy;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            joystickCopy = Vec2(sharedState.leftStick.x, sharedState.leftStick.y);
        }
        if (joystickCopy.length() < 0.05f) {
            joystickCopy = Vec2(0.f, 0.f);
        }

        // Time PathPlanner update + IK update combined
        auto pathStart = std::chrono::steady_clock::now();

        pathPlanner->update(joystickCopy, ikLoopInterval.count() / 1000.0f);
        // pathPlanner->printDebugStatus(); // Debug print for foot status

        robot.updateKinematicsAndApply();


        auto pathEnd = std::chrono::steady_clock::now();
        uint64_t durationUs = std::chrono::duration_cast<std::chrono::microseconds>(pathEnd - pathStart).count();

        {
            std::lock_guard<std::mutex> lock(ikTimingMutex);
            ikDurationUs = durationUs;
        }

        // Sleep to maintain loop rate
        std::this_thread::sleep_until(start + ikLoopInterval);
    }
}

int main() {
    std::cout << "Starting robot controller..." << std::endl;

    const int mainLoopHz = 30;
    const auto mainLoopInterval = std::chrono::milliseconds(1000 / mainLoopHz);

    std::cout << "GamepadController created." << std::endl;

    if (!pad.initialize({})) {
        std::cerr << "Warning: Gamepad not detected!\n";
    } else {
        std::cout << "Gamepad initialized successfully.\n";
    }

    if (!initGraphics()) return 1;

    static WalkCycle walkCycle3Set({ {0, 2, 4}, {1, 3, 5} }, 1.f, 1.0f);
    static WalkCycle walkCycle2Set({ {0, 3}, {1, 4}, {2, 5} }, 3.f, 0.5f);
    static WalkCycle walkCycle1Set({ {0}, {1}, {2}, {3}, {4}, {5} }, 6.f, 0.2f);

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

    SDL_Event e;

    std::thread padThread(gamepadPollingThread);
    std::thread ikWorkerThread(ikThread, std::ref(robot));

    while (running) {
        auto loopStart = std::chrono::steady_clock::now();

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) running = false;
        }

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

        // Drawing at ~30 Hz
        auto now = std::chrono::steady_clock::now();
        auto drawStart = now;
        updateAndDrawEyes();
        auto drawEnd = std::chrono::steady_clock::now();
        auto drawDurationUs = std::chrono::duration_cast<std::chrono::microseconds>(drawEnd - drawStart).count();

        //std::cout << "Drawing time (us): " << drawDurationUs << std::endl;

        auto loopEnd = std::chrono::steady_clock::now();
        uint64_t loopDurationUs = std::chrono::duration_cast<std::chrono::microseconds>(loopEnd - loopStart).count();

        {
            std::lock_guard<std::mutex> lock(mainTimingMutex);
            mainLoopDurationUs = loopDurationUs;
        }

        {
            std::lock_guard<std::mutex> lock(mainTimingMutex);
            std::lock_guard<std::mutex> lock2(ikTimingMutex);
            //std::cout << "Timing (us): PathPlanner=" << pathPlannerDurationUs
            //          << " IK=" << ikDurationUs
            //          << " MainLoop=" << mainLoopDurationUs << std::endl;
        }

        std::this_thread::sleep_until(loopStart + mainLoopInterval);
    }

    running = false;

    eventQueueCV.notify_all();

    padThread.join();
    ikWorkerThread.join();

    std::cout << "Shutting down robot controller." << std::endl;
    shutdownGraphics();
    std::cout << "Graphics shutdown complete." << std::endl;

    SDL_Quit();
    std::cout << "SDL cleanup complete." << std::endl;

    delete pathPlanner;
    return 0;
}
