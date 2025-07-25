#include "face_renderer.hpp"
#include "gamepad.hpp"
#include "PathPlanner.hpp"
#include <vector>
#include <string>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
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

extern GamepadController pad;
extern GamepadState sharedState;
GamepadController pad;
GamepadState      sharedState;
std::mutex stateMutex;
std::vector<ComboEvent> comboDefs = {
    {"X", {"Left"}, "X+Left"},
    {"Y", {"Up"}, "Y+Up"},
    {"Y", {"Down"}, "Y+Down"},
    {"Y", {"Left"}, "Y+Left"},
    {"Y", {"Right"}, "Y+Right"}
};

std::atomic<bool> running(true);

std::mutex ikTimingMutex;
uint64_t ikDurationUs = 0;

std::mutex mainTimingMutex;
uint64_t mainLoopDurationUs = 0;
uint64_t pathPlannerDurationUs = 0;

bool showBootPrompt(SDL_Joystick*& joystick) {
    // — 1) Init SDL video, joystick, and TTF —
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return false;
    }
    if (TTF_Init() != 0) {
        std::cerr << "TTF_Init failed: " << TTF_GetError() << "\n";
        SDL_Quit();
        return false;
    }

    // — 2) Create fullscreen window & renderer —
    SDL_DisplayMode dm;
    SDL_GetCurrentDisplayMode(0, &dm);
    SDL_Window*   window   = SDL_CreateWindow(
        "NUMA Boot",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        dm.w, dm.h,
        SDL_WINDOW_FULLSCREEN_DESKTOP
    );
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!window || !renderer) {
        std::cerr << "Failed to create window/renderer: " << SDL_GetError() << "\n";
        TTF_Quit();
        SDL_Quit();
        return false;
    }
    SDL_ShowCursor(SDL_DISABLE);

    // — 3) Load font scaled to screen height (~5%) —
    int fontSize = dm.h / 20;
    TTF_Font* font = TTF_OpenFont(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        fontSize
    );
    if (!font) {
        std::cerr << "Failed to open font: " << TTF_GetError() << "\n";
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    // — 4) Prepare prompt texture —
    const char* msg = "Press Y to boot NUMA hexapod\nPress X to exit";
    SDL_Color white = {255,255,255,255};
    int wrapWidth = dm.w * 8 / 10;
    SDL_Surface* surf = TTF_RenderText_Blended_Wrapped(font, msg, white, wrapWidth);
    SDL_Texture* textTex = SDL_CreateTextureFromSurface(renderer, surf);
    int texW = surf->w, texH = surf->h;
    SDL_FreeSurface(surf);

    // — 5) Setup gamepad combo definitions & state —
    std::vector<ComboEvent> comboDefs = {
        {"X", {"Left"},  "X+Left"},
        {"Y", {"Up"},    "Y+Up"},
        {"Y", {"Down"},  "Y+Down"},
        {"Y", {"Left"},  "Y+Left"},
        {"Y", {"Right"}, "Y+Right"}
    };
    bool padReady = false;

    // — 6) Main prompt loop —
    bool boot = false, done = false;
    while (!done) {
        // 6a) Attempt to initialize your GamepadController
        if (!padReady) {
            if (pad.initialize(comboDefs)) {
                padReady = true;
                std::cout << "Gamepad connected!\n";
            }
        }
        // 6b) Open SDL joystick for raw events (optional)
        if (padReady && SDL_NumJoysticks() > 0 && !joystick) {
            joystick = SDL_JoystickOpen(0);
        }

        // 6c) Poll SDL events
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                boot = false;
                done = true;
            }
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_x) {
                    boot = false; done = true;
                }
                if (e.key.keysym.sym == SDLK_y && padReady) {
                    boot = true; done = true;
                }
            }
            if (e.type == SDL_JOYBUTTONDOWN && padReady) {
                // adjust these indices for your controller
                if (e.jbutton.button == 3) { // Y
                    boot = true; done = true;
                }
                if (e.jbutton.button == 2) { // X
                    boot = false; done = true;
                }
            }
        }

        // 6d) Poll your GamepadController for combo events
        pad.update(sharedState);
        auto evs = pad.pollEvents();
        for (auto& ev : evs) {
            if (ev.type == "click" && ev.name == "X") {
                boot = false; done = true;
            }
            if (ev.type == "click" && ev.name == "Y" && padReady) {
                boot = true;  done = true;
            }
        }

        // 6e) Draw the black background + centered prompt
        SDL_SetRenderDrawColor(renderer, 0,0,0,255);
        SDL_RenderClear(renderer);
        SDL_Rect dst;
        dst.w = texW;
        dst.h = texH;
        dst.x = (dm.w - texW) / 2;
        dst.y = (dm.h - texH) / 2;
        SDL_RenderCopy(renderer, textTex, nullptr, &dst);
        SDL_RenderPresent(renderer);

        SDL_Delay(50);
    }

    // — 7) Cleanup —
    SDL_DestroyTexture(textTex);
    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();

    return boot;
}


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

        // ——— LOOK (right stick) ———
        Joystick lookJoy;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            if (sharedState.lookLocked) {
                lookJoy = sharedState.lockedRightStick;
            } else {
                lookJoy = sharedState.rightStick;
            }
        }

        // Body orientation
        // Set robot heading to max ±30° scaled by left stick X
        robot.Body.targetHeadingDeg = -lookJoy.x_raw * 30.0f;
        robot.Body.targetTiltAzimuthDeg = 90.0f;
        robot.Body.targetTiltPolarDeg   = -lookJoy.y_raw * 30.0f;

        // Set face pan to ±60° scaled by right stick X
        robot.Face.pan.value = robot.Face.pan.restDeg - lookJoy.x_raw * 70.0f;
        // Set face tilt to ±20° scaled by right stick Y (invert Y if natural)
        robot.Face.tilt.value = robot.Face.tilt.restDeg + lookJoy.y_raw * 30.0f;

        // ——— MOVEMENT (left stick) ———
        Joystick moveJoy;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            if (sharedState.movementLocked) {
                moveJoy = sharedState.lockedLeftStick;
            } else {
                moveJoy = sharedState.leftStick;
            }
        }
        
        Vec2 joystickCopy{ moveJoy.x, moveJoy.y };
        
        if (joystickCopy.length() < 0.05f)
            joystickCopy = {0.f, 0.f};


        // -- Steering of movement (Differential LT and RT) --
        float lt, rt;
        {
            std::lock_guard<std::mutex> lk(stateMutex);
            lt = sharedState.triggers.left;  // assume 0…1
            rt = sharedState.triggers.right;  // assume 0…1
        }
        // differential: RT=1, LT=0 → +1 turn right; RT=0, LT=1 → ‑1 turn left
        float turnInput = lt - rt;
        // clamp in case
        turnInput = std::max(-1.0f, std::min(1.0f, turnInput));

        // Time PathPlanner update + IK update combined
        auto pathStart = std::chrono::steady_clock::now();

        pathPlanner->update(joystickCopy, turnInput, ikLoopInterval.count() / 1000.0f);
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

    if (!pad.initialize(comboDefs)) {
        std::cerr << "Warning: Gamepad not detected!\n";
    } else {
        std::cout << "Gamepad initialized successfully.\n";
    }

    SDL_Joystick* joystick = nullptr;
    bool boot = showBootPrompt(joystick);


    if (!boot) {
        std::cout << "User exited before boot.\n";
        return 0;
    }
    // At this point you have a window‐based prompt, user pressed Y,
    // and SDL is still initialized for video/joystick.
    
    {   // 1) Drain raw SDL events
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            // discard
        }
    }

    // 2) Purge GamepadController’s queued clicks
    pad.update(sharedState);
    pad.pollEvents();  // drop them on the floor

    // 3) Empty your main queue of any boot‐phase events
    {
        std::lock_guard<std::mutex> lk(eventQueueMutex);
        while (!gamepadEventQueue.empty()) {
            gamepadEventQueue.pop();
        }
    }
    
    if (!initGraphics()) return 1;

    static WalkCycle walkCycle3Set({ {0, 2, 4}, {1, 3, 5} }, 1.f, 1.0f, 1.0f);
    static WalkCycle walkCycle2Set({ {0, 3}, {1, 4}, {2, 5} }, 3.f, 1.0f, 0.5f);
    static WalkCycle walkCycle1Set({ {0}, {1}, {2}, {3}, {4}, {5} }, 6.f, 1.0f, 0.2f);

    walkCycles = { &walkCycle3Set, &walkCycle2Set, &walkCycle1Set };

    RobotController robot(6);
    if (robot.initialize() != 0) {
        std::cerr << "Failed to initialize robot controller." << std::endl;
        return 1;
    }

    robot.Body.position = { 0.f, 0.f, robot.Body.originStartingHeight - 3.0f};
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

                // Return to zero on X Click
                if (ev.type == "clikc" && ev.name == "X"){
                    pathPlanner->requestReturnToZero(true);
                }

                // Combo X + Left Walkcycle mode cycler
                if (ev.type == "combo" && ev.name == "X+Left") {
                    currentWalkCycleIndex = (currentWalkCycleIndex + 1) % walkCycles.size();
                    pathPlanner->requestWalkCycleSwitch(walkCycles[currentWalkCycleIndex]);
                    std::cout << "Switched to walk cycle index: " << currentWalkCycleIndex << std::endl;
                }

                // Toggle light on/off with Y click
                if (ev.type == "click" && ev.name == "Y") {
                    if (robot.Face.lightMode == LightMode::Off) {
                        robot.Face.lightMode = LightMode::Steady;
                        std::cout << "[Light] Turned ON (Steady mode)\n";
                    } else {
                        robot.Face.lightMode = LightMode::Off;
                        std::cout << "[Light] Turned OFF\n";
                    }
                }

                // Combo: Y + Up → increase brightness
                if (ev.type == "combo" && ev.name == "Y+Up") {
                    robot.Face.lightBrightness += 0.2f;
                    if (robot.Face.lightBrightness > 1.0f) robot.Face.lightBrightness = 0.0f;
                    std::cout << "[Light] Brightness increased to "
                            << static_cast<int>(robot.Face.lightBrightness * 100) << "%\n";
                }

                // Combo: Y + Down → decrease brightness
                if (ev.type == "combo" && ev.name == "Y+Down") {
                    robot.Face.lightBrightness -= 0.2f;
                    if (robot.Face.lightBrightness < 0.0f) robot.Face.lightBrightness = 1.0f;
                    std::cout << "[Light] Brightness decreased to "
                            << static_cast<int>(robot.Face.lightBrightness * 100) << "%\n";
                }

                // Combo: Y + Left → toggle strobe
                if (ev.type == "combo" && ev.name == "Y+Left") {
                    if (robot.Face.lightMode == LightMode::Steady) {
                        robot.Face.lightMode = LightMode::Strobe;
                        std::cout << "[Light] Switched to STROBE mode\n";
                    } else if (robot.Face.lightMode == LightMode::Strobe) {
                        robot.Face.lightMode = LightMode::Steady;
                        std::cout << "[Light] Switched to STEADY mode\n";
                    }
                }

                // Combo: Y + Right → reset
                if (ev.type == "combo" && ev.name == "Y+Right") {
                    robot.Face.lightBrightness = 0.0f;
                    robot.Face.lightMode = LightMode::Steady;
                    std::cout << "[Light] Reset brightness to 0%, mode to STEADY\n";
                }

                // Arrow key clicks handle stance opperations
                if (ev.type == "combo" && ev.name == "A+Up") {
                    // TODO: handle Up arrow click
                    if( robot.Body.targetPosition.z < 45)
                        robot.Body.targetPosition.z += 3.0f;
                }

                if (ev.type == "combo" && ev.name == "A+Down") {
                    // TODO: handle Down arrow click
                    if( robot.Body.targetPosition.z > -robot.Body.originStartingHeight)
                        robot.Body.targetPosition.z -= 3.0f;
                }

                if (ev.type == "combo" && ev.name == "A+Right") {
                    // TODO: handle Left arrow click
                    pathPlanner->stepAreaPlacementDistance += 4.0f;
                }

                if (ev.type == "combo" && ev.name == "A+Left") {
                    // TODO: handle Right arrow click
                    pathPlanner->stepAreaPlacementDistance -= 4.0f;
                }

                // ── Movement lock toggle ──
                if (ev.type == "click" && ev.name == "LB") {
                    sharedState.movementLocked = !sharedState.movementLocked;
                    if (sharedState.movementLocked) {
                        // snapshot the current left stick
                        sharedState.lockedLeftStick = sharedState.leftStick;
                        std::cout << "[Lock] Movement locked at ("
                                << sharedState.lockedLeftStick.x << ", "
                                << sharedState.lockedLeftStick.y << ")\n";
                    } else {
                        std::cout << "[Lock] Movement unlocked\n";
                    }
                }

                // ── Look lock toggle ──
                if (ev.type == "click" && ev.name == "RB") {
                    sharedState.lookLocked = !sharedState.lookLocked;
                    if (sharedState.lookLocked) {
                        // snapshot the current right stick
                        sharedState.lockedRightStick = sharedState.rightStick;
                        std::cout << "[Lock] Look locked at ("
                                << sharedState.lockedRightStick.x_raw << ", "
                                << sharedState.lockedRightStick.y_raw << ")\n";
                    } else {
                        std::cout << "[Lock] Look unlocked\n";
                    }
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
