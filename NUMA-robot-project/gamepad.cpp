
#include "gamepad.hpp"
#include <libevdev/libevdev.h>
#include <thread>
#include <chrono>
#include <vector>
#include <map>
#include <unordered_map>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cmath>
#include <dirent.h>
#include <cstring>
#include <iostream>
#include <algorithm>

static constexpr float EPSILON = 0.01f;             
static constexpr int DEBOUNCE_MS = 75;              // Button debounce ms
static constexpr float JOYSTICK_DEADZONE = 0.02f;   // 2% deadzone


GamepadController::GamepadController() {
    fd = -1;
    scanDevices();
}

GamepadController::~GamepadController() {
    if (fd != -1) {
        close(fd);
    }
}

bool GamepadController::isConnected() const {
    return fd != -1;
}

bool contains_case_insensitive(const std::string& haystack, const std::string& needle) {
    std::string haystack_lower = haystack;
    std::string needle_lower = needle;
    std::transform(haystack_lower.begin(), haystack_lower.end(), haystack_lower.begin(), ::tolower);
    std::transform(needle_lower.begin(), needle_lower.end(), needle_lower.begin(), ::tolower);
    return haystack_lower.find(needle_lower) != std::string::npos;
}

void GamepadController::scanDevices() {
    DIR* dir = opendir("/dev/input");
    if (!dir) return;

    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (strncmp(ent->d_name, "event", 5) == 0) {
            std::string path = "/dev/input/" + std::string(ent->d_name);
            int fd_test = open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd_test < 0) continue;

            struct libevdev *dev = nullptr;
            int rc = libevdev_new_from_fd(fd_test, &dev);
            if (rc < 0) {
                close(fd_test);
                continue;
            }

            const char* dev_name = libevdev_get_name(dev);
            if (dev_name) {
                std::string name_str(dev_name);

                if (contains_case_insensitive(name_str, "mouse") ||
                    contains_case_insensitive(name_str, "keyboard")) {
                    libevdev_free(dev);
                    close(fd_test);
                    continue;
                }

                if (contains_case_insensitive(name_str, "8BitDo")) {
                    fd = fd_test;
                    devicePath = path;
                    libevdev_free(dev);
                    std::cout << "Using gamepad device: " << devicePath << std::endl;
                    break;
                }
            }

            libevdev_free(dev);
            close(fd_test);
        }
    }
    closedir(dir);

    buttonMap[304] = "A";
    buttonMap[305] = "B";
    buttonMap[307] = "Y";
    buttonMap[308] = "X";
    buttonMap[310] = "LB";
    buttonMap[311] = "RB";
    buttonMap[314] = "Minus";
    buttonMap[315] = "Plus";
    buttonMap[316] = "Star";
    buttonMap[317] = "L3";
    buttonMap[318] = "R3";
    buttonMap[9991] = "L4";
    buttonMap[9992] = "R4";
}

bool GamepadController::initialize(const std::vector<ComboEvent>& combos) {
    scanDevices();
    if (fd == -1) {
        std::cerr << "No compatible gamepad found.";
        return false;
    }

    std::cout << "Opened device: " << devicePath << std::endl;

    for (const auto& combo : combos) {
        addCombo(combo.holdButton, combo.sequence, combo.name);
    }

    std::cout << "Press any button to continue...";
    input_event ev;

    while (true) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n == sizeof(ev)) {
            if (ev.type == EV_KEY && ev.value == 1) {
                break;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    return true;
}

void GamepadController::update(GamepadState &state) {
    static auto lastReconnectAttempt = std::chrono::steady_clock::now();
    static std::unordered_map<std::string, std::chrono::steady_clock::time_point> lastEventTimes;

    if (fd == -1) {
        auto now = std::chrono::steady_clock::now();
        if (now - lastReconnectAttempt > std::chrono::milliseconds(500)) {
            lastReconnectAttempt = now;
            scanDevices();
            if (fd != -1) {
                std::cout << "✅ Controller reconnected: " << devicePath << std::endl;
            }
        }
        return;
    }

    input_event ev;
    ssize_t readSize;

    while ((readSize = read(fd, &ev, sizeof(ev))) > 0) {
        if (ev.type == EV_KEY) {
            auto it = buttonMap.find(ev.code);
            if (it != buttonMap.end()) {
                std::string name = it->second;
                int val = ev.value;
                int prevVal = buttonStates[name];
                auto now = std::chrono::steady_clock::now();
                auto &lastTime = lastEventTimes[name];
                if (val != prevVal && (now - lastTime) >= std::chrono::milliseconds(DEBOUNCE_MS)) {
                    buttonStates[name] = val;
                    lastTime = now;
                    if (val == 1) {
                        bool triggered = false;
                        for (const auto& [held, state] : buttonStates) {
                            if (state == 1 && held != name) {
                                std::string combo = held + "+" + name;
                                pendingEvents.push_back({"combo", combo});
                                triggered = true;
                            }
                        }
                        if (!triggered)
                            pendingEvents.push_back({"button", name, "pressed"});
                        recentInputs.push_back(name);
                        if (recentInputs.size() > 10) recentInputs.erase(recentInputs.begin());
                    } else if (val == 0) {
                        pendingEvents.push_back({"button", name, "released"});
                    }
                }
            }
        } else {
            handleEvent(ev);
        }
    }

    if (readSize < 0 && (errno == ENODEV || errno == EIO)) {
        std::cerr << "⚠️ Controller disconnected.";
        close(fd);
        fd = -1;
        return;
    }

    updateJoysticks();
    detectCombos();

    state.buttonEvents.clear();
    for (const auto& [name, val] : buttonStates) {
        if (val == 1) state.buttonEvents[name] = ButtonEvent::Pressed;
        else if (val == 0) state.buttonEvents[name] = ButtonEvent::Released;
    }
    state.leftStick = leftStick;
    state.rightStick = rightStick;
    state.triggers = triggers;
}

void GamepadController::printJoystickAndTriggerChanges() {
    // Calculate magnitude and angle for left stick
    leftStick.magnitude = std::sqrt(leftStick.x * leftStick.x + leftStick.y * leftStick.y);
    leftStick.angle = std::atan2(leftStick.y, leftStick.x);

    // Same for right stick
    rightStick.magnitude = std::sqrt(rightStick.x * rightStick.x + rightStick.y * rightStick.y);
    rightStick.angle = std::atan2(rightStick.y, rightStick.x);

    // Only print if there are meaningful changes (to avoid spam)
    bool leftChanged =
        std::fabs(leftStick.x - prevLeftStick.x) > 0.01f ||
        std::fabs(leftStick.y - prevLeftStick.y) > 0.01f;
    bool rightChanged =
        std::fabs(rightStick.x - prevRightStick.x) > 0.01f ||
        std::fabs(rightStick.y - prevRightStick.y) > 0.01f;
    bool triggersChanged =
        std::fabs(triggers.left - prevTriggers.left) > 0.01f ||
        std::fabs(triggers.right - prevTriggers.right) > 0.01f;

    if (leftChanged || rightChanged || triggersChanged) {
        printf("\n-- Joystick & Trigger State --\n");

        printf("Left Stick  - X: %+ .6f | Y: %+ .6f | Mag: %.6f | Angle: %.2f°\n",
            leftStick.x, leftStick.y, leftStick.magnitude, leftStick.angle * (180.0f / M_PI));

        printf("Right Stick - X: %+ .6f | Y: %+ .6f | Mag: %.6f | Angle: %.2f°\n",
            rightStick.x, rightStick.y, rightStick.magnitude, rightStick.angle * (180.0f / M_PI));

        printf("Triggers    - LT: %.3f | RT: %.3f\n",
            triggers.left, triggers.right);

        printf("-------------------------------\n");
    }

    // Update previous state
    prevLeftStick = leftStick;
    prevRightStick = rightStick;
    prevTriggers = triggers;
}


std::vector<GamepadEvent> GamepadController::pollEvents() {
    auto events = pendingEvents;
    pendingEvents.clear();
    return events;
}

void GamepadController::addCombo(const std::string& hold, const std::vector<std::string>& sequence, const std::string& name) {
    activeCombos[name] = {hold};
    activeCombos[name].insert(activeCombos[name].end(), sequence.begin(), sequence.end());
}

void GamepadController::detectCombos() {
    for (const auto& pair : activeCombos) {
        const auto& name = pair.first;
        const auto& seq = pair.second;
        if (!buttonStates[seq[0]]) continue;
        if (recentInputs.size() < seq.size() - 1) continue;
        bool match = true;
        for (size_t i = 1; i < seq.size(); ++i) {
            if (recentInputs[recentInputs.size() - seq.size() + i] != seq[i]) {
                match = false;
                break;
            }
        }
        if (match) {
            pendingEvents.push_back({"combo", name});
            recentInputs.clear();
        }
    }
}

void GamepadController::simulateButton(const std::string& name, const std::chrono::steady_clock::time_point& now,
    std::unordered_map<std::string, std::chrono::steady_clock::time_point>& lastEventTimes) {
    if (buttonStates[name] != 1 && (now - lastEventTimes[name]) >= std::chrono::milliseconds(DEBOUNCE_MS)) {
        buttonStates[name] = 1;
        lastEventTimes[name] = now;
        pendingEvents.push_back({"button", name, "pressed"});
        recentInputs.push_back(name);
        if (recentInputs.size() > 10) recentInputs.erase(recentInputs.begin());
    }
}

void GamepadController::simulateRelease(const std::string& name, const std::chrono::steady_clock::time_point& now,
    std::unordered_map<std::string, std::chrono::steady_clock::time_point>& lastEventTimes) {
    if (buttonStates[name] == 1 && (now - lastEventTimes[name]) >= std::chrono::milliseconds(DEBOUNCE_MS)) {
        buttonStates[name] = 0;
        lastEventTimes[name] = now;
        pendingEvents.push_back({"button", name, "released"});
    }
}


void GamepadController::handleEvent(const input_event& ev) {
    static std::unordered_map<std::string, std::chrono::steady_clock::time_point> lastEventTimes;

    const auto now = std::chrono::steady_clock::now();

    if (ev.type == EV_ABS && ev.code < ABS_CNT) {
        absState[ev.code] = ev.value;
    }


    if (ev.type == EV_ABS) {
        if (ev.code == ABS_HAT0X) {
            if (ev.value == -1) {
                simulateButton("Left", now, lastEventTimes);
                simulateRelease("Right", now, lastEventTimes);
            } else if (ev.value == 1) {
                simulateButton("Right", now, lastEventTimes);
                simulateRelease("Left", now, lastEventTimes);
            } else {
                simulateRelease("Left", now, lastEventTimes);
                simulateRelease("Right", now, lastEventTimes);
            }
        } else if (ev.code == ABS_HAT0Y) {
            if (ev.value == -1) {
                simulateButton("Up", now, lastEventTimes);
                simulateRelease("Down", now, lastEventTimes);
            } else if (ev.value == 1) {
                simulateButton("Down", now, lastEventTimes);
                simulateRelease("Up", now, lastEventTimes);
            } else {
                simulateRelease("Up", now, lastEventTimes);
                simulateRelease("Down", now, lastEventTimes);
            }
        }
    }

    if (ev.type == EV_ABS) {
        switch (ev.code) {
            case 0: leftStick.x_raw = ev.value / 32767.0f; break;
            case 1: leftStick.y_raw = ev.value / 32767.0f; break;
            case 2: triggers.left = ev.value / 255.0f; break;
            case 3: rightStick.x_raw = ev.value / 32767.0f; break;
            case 4: rightStick.y_raw = ev.value / 32767.0f; break;
            case 5: triggers.right = ev.value / 255.0f; break;
        }
    }
}

void GamepadController::updateJoysticks() {
    auto normalizeAxis = [](int16_t value) -> float {
        return static_cast<float>(value) / 32768.0f;
    };

    // Normalize and store raw stick input
    leftStick.x_raw = normalizeAxis(absState[ABS_X]);
    leftStick.y_raw = normalizeAxis(absState[ABS_Y]);
    rightStick.x_raw = normalizeAxis(absState[ABS_RX]);
    rightStick.y_raw = normalizeAxis(absState[ABS_RY]);

    // Convert square to circular mapping
    squareToCircle(leftStick.x_raw, leftStick.y_raw, leftStick.x, leftStick.y);
    squareToCircle(rightStick.x_raw, rightStick.y_raw, rightStick.x, rightStick.y);

    // Calculate magnitude
    leftStick.magnitude = std::sqrt(leftStick.x * leftStick.x + leftStick.y * leftStick.y);
    rightStick.magnitude = std::sqrt(rightStick.x * rightStick.x + rightStick.y * rightStick.y);

    // Apply deadzone by zeroing out small values
    if (leftStick.magnitude < JOYSTICK_DEADZONE) {
        leftStick.x = leftStick.y = leftStick.magnitude = 0.0f;
    }

    if (rightStick.magnitude < JOYSTICK_DEADZONE) {
        rightStick.x = rightStick.y = rightStick.magnitude = 0.0f;
    }

    // Calculate angles (if needed)
    leftStick.angle = (leftStick.magnitude > 0.0f) ? std::atan2(leftStick.y, leftStick.x) : 0.0f;
    rightStick.angle = (rightStick.magnitude > 0.0f) ? std::atan2(rightStick.y, rightStick.x) : 0.0f;
}


void GamepadController::squareToCircle(float x_in, float y_in, float &x_out, float &y_out) {
    float x = std::max(-1.0f, std::min(1.0f, x_in));
    float y = std::max(-1.0f, std::min(1.0f, y_in));

    x_out = x * std::sqrt(1.0f - (y * y) / 2.0f);
    y_out = y * std::sqrt(1.0f - (x * x) / 2.0f);
}
