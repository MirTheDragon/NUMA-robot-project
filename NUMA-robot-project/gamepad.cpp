#include "gamepad.hpp"
#include <libevdev/libevdev.h>
#include <thread>
#include <chrono>
#include <vector>   // for std::vector
#include <map>      // for std::map
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cmath>
#include <dirent.h>
#include <cstring>
#include <iostream>
#include <algorithm>

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

                // Skip devices with "mouse" or "keyboard" in their name
                if (contains_case_insensitive(name_str, "mouse") ||
                    contains_case_insensitive(name_str, "keyboard")) {
                    libevdev_free(dev);
                    close(fd_test);
                    continue;
                }

                // Accept devices with "8BitDo" in name (or change to your exact controller name)
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

    // Basic button map for example
    buttonMap[304] = "A";
    buttonMap[305] = "B";
    buttonMap[307] = "X";
    buttonMap[308] = "Y";
    buttonMap[310] = "LB";
    buttonMap[311] = "RB";
    buttonMap[314] = "Minus";
    buttonMap[315] = "Plus";
    buttonMap[317] = "Square";
    // buttonMap[544] = "Up";
    // buttonMap[545] = "Down";
    // buttonMap[546] = "Left";
    // buttonMap[547] = "Right";
}

bool GamepadController::initialize(const std::vector<ComboEvent>& combos) {
    scanDevices();
    if (fd == -1) {
        std::cerr << "No compatible gamepad found.\n";
        return false;
    }

    std::cout << "Opened device: " << devicePath << std::endl;


    // Load combos if needed
    for (const auto& combo : combos) {
        addCombo(combo.holdButton, combo.sequence, combo.name);
    }


    std::cout << "Press any button to continue...\n";

    input_event ev;

    while (true) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n == sizeof(ev)) {
            std::cout << "Event type: " << (int)ev.type
                      << ", code: " << (int)ev.code
                      << ", value: " << ev.value << std::endl;

            if (ev.type == EV_KEY && ev.value == 1) {
                std::cout << "Button press detected, continuing...\n";
                break;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    return true;
}


void GamepadController::update(GamepadState &state) {
    if (fd == -1) return;

    input_event ev;
    while (read(fd, &ev, sizeof(ev)) > 0) {
        if (ev.type == EV_ABS && (ev.code == 16 || ev.code == 17)) {
            // Handle D-pad axes manually and push events for presses/releases
            if (ev.code == 16) { // ABS_HAT0X (Left/Right)
                if (ev.value == -1) {
                    if (buttonStates["Left"] == 0)
                        pendingEvents.push_back({"button", "Left", "pressed"});
                    buttonStates["Left"] = 1;

                    if (buttonStates["Right"] == 1)
                        pendingEvents.push_back({"button", "Right", "released"});
                    buttonStates["Right"] = 0;

                    recentInputs.push_back("Left");
                } else if (ev.value == 1) {
                    if (buttonStates["Right"] == 0)
                        pendingEvents.push_back({"button", "Right", "pressed"});
                    buttonStates["Right"] = 1;

                    if (buttonStates["Left"] == 1)
                        pendingEvents.push_back({"button", "Left", "released"});
                    buttonStates["Left"] = 0;

                    recentInputs.push_back("Right");
                } else { // Neutral, release both if pressed
                    if (buttonStates["Left"] == 1)
                        pendingEvents.push_back({"button", "Left", "released"});
                    if (buttonStates["Right"] == 1)
                        pendingEvents.push_back({"button", "Right", "released"});
                    buttonStates["Left"] = 0;
                    buttonStates["Right"] = 0;
                }
            } else if (ev.code == 17) { // ABS_HAT0Y (Up/Down)
                if (ev.value == -1) {
                    if (buttonStates["Up"] == 0)
                        pendingEvents.push_back({"button", "Up", "pressed"});
                    buttonStates["Up"] = 1;

                    if (buttonStates["Down"] == 1)
                        pendingEvents.push_back({"button", "Down", "released"});
                    buttonStates["Down"] = 0;

                    recentInputs.push_back("Up");
                } else if (ev.value == 1) {
                    if (buttonStates["Down"] == 0)
                        pendingEvents.push_back({"button", "Down", "pressed"});
                    buttonStates["Down"] = 1;

                    if (buttonStates["Up"] == 1)
                        pendingEvents.push_back({"button", "Up", "released"});
                    buttonStates["Up"] = 0;

                    recentInputs.push_back("Down");
                } else { // Neutral, release both if pressed
                    if (buttonStates["Up"] == 1)
                        pendingEvents.push_back({"button", "Up", "released"});
                    if (buttonStates["Down"] == 1)
                        pendingEvents.push_back({"button", "Down", "released"});
                    buttonStates["Up"] = 0;
                    buttonStates["Down"] = 0;
                }
            }
        } else if (ev.type == EV_KEY) {
            // For key events, update states and push press/release events
            auto it = buttonMap.find(ev.code);
            if (it != buttonMap.end()) {
                std::string buttonName = it->second;
                int val = ev.value;
                int oldVal = buttonStates[buttonName];
                buttonStates[buttonName] = val;

                if (val == 1 && oldVal != 1) {
                    pendingEvents.push_back({"button", buttonName, "pressed"});
                    recentInputs.push_back(buttonName);
                    if (recentInputs.size() > 10) recentInputs.erase(recentInputs.begin());
                } else if (val == 0 && oldVal != 0) {
                    pendingEvents.push_back({"button", buttonName, "released"});
                }
            }
        } else {
            // Other event types (e.g., joystick axes)
            handleEvent(ev);
        }
    }

    updateJoysticks();
    detectCombos();

    // Update the external state with current button states, sticks, triggers
    state.buttonEvents.clear();
    for (const auto& [buttonName, val] : buttonStates) {
        if (val == 1)
            state.buttonEvents[buttonName] = ButtonEvent::Pressed;
        else if (val == 0)
            state.buttonEvents[buttonName] = ButtonEvent::Released;
    }
    state.leftStick = leftStick;
    state.rightStick = rightStick;
    state.triggers = triggers;
}





void GamepadController::handleEvent(const input_event& ev) {
    if (ev.type == EV_KEY) {
        auto it = buttonMap.find(ev.code);
        if (it != buttonMap.end()) {
            buttonStates[it->second] = ev.value;
            if (ev.value == 1) {
                recentInputs.push_back(it->second);
                if (recentInputs.size() > 10) recentInputs.erase(recentInputs.begin());
            }
        }
    } else if (ev.type == EV_ABS) {
        switch (ev.code) {
            case 0: leftStick.x = ev.value / 32767.0f; break;
            case 1: leftStick.y = ev.value / 32767.0f; break;
            case 3: rightStick.x = ev.value / 32767.0f; break;
            case 4: rightStick.y = ev.value / 32767.0f; break;
            case 2: triggers.left = ev.value / 255.0f; break;
            case 5: triggers.right = ev.value / 255.0f; break;

            case 16: // ABS_HAT0X: Left(-1), Neutral(0), Right(1)
                if (ev.value == -1) {
                    buttonStates["Left"] = 1;
                    buttonStates["Right"] = 0;
                    recentInputs.push_back("Left");
                } else if (ev.value == 1) {
                    buttonStates["Right"] = 1;
                    buttonStates["Left"] = 0;
                    recentInputs.push_back("Right");
                } else {
                    buttonStates["Left"] = 0;
                    buttonStates["Right"] = 0;
                }
                break;

            case 17: // ABS_HAT0Y: Up(-1), Neutral(0), Down(1)
                if (ev.value == -1) {
                    buttonStates["Up"] = 1;
                    buttonStates["Down"] = 0;
                    recentInputs.push_back("Up");
                } else if (ev.value == 1) {
                    buttonStates["Down"] = 1;
                    buttonStates["Up"] = 0;
                    recentInputs.push_back("Down");
                } else {
                    buttonStates["Up"] = 0;
                    buttonStates["Down"] = 0;
                }
                break;
        }
    }
}

void GamepadController::updateJoysticks() {
    leftStick.magnitude = std::sqrt(leftStick.x * leftStick.x + leftStick.y * leftStick.y);
    leftStick.angle = std::atan2(leftStick.y, leftStick.x);

    rightStick.magnitude = std::sqrt(rightStick.x * rightStick.x + rightStick.y * rightStick.y);
    rightStick.angle = std::atan2(rightStick.y, rightStick.x);
}

void GamepadController::addCombo(const std::string& holdButton, const std::vector<std::string>& sequence, const std::string& eventName) {
    std::string key = holdButton + ":";
    for (const auto& s : sequence) key += s + ",";
    activeCombos[eventName] = {holdButton};
    activeCombos[eventName].insert(activeCombos[eventName].end(), sequence.begin(), sequence.end());
}

void GamepadController::detectCombos() {
    for (const auto& pair : activeCombos) {
        const std::string& name = pair.first;
        const auto& sequence = pair.second;

        if (!buttonStates[sequence[0]]) continue; // Holding condition not met

        if (recentInputs.size() < sequence.size() - 1) continue;
        bool match = true;
        for (size_t i = 1; i < sequence.size(); ++i) {
            if (recentInputs[recentInputs.size() - sequence.size() + i] != sequence[i]) {
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

bool GamepadController::isButtonPressed(const std::string& name) const {
    auto it = buttonStates.find(name);
    return it != buttonStates.end() && it->second;
}

bool GamepadController::pollCombo(std::string& outComboName) {
    auto events = pollEvents();
    for (const auto& e : events) {
        if (e.type == "combo") {
            outComboName = e.name;
            return true;
        }
    }
    return false;
}

std::vector<GamepadEvent> GamepadController::pollEvents() {
    auto out = pendingEvents;
    pendingEvents.clear();
    return out;
}