#pragma once

#include <string>
#include <vector>
#include <map>
#include <linux/input.h>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <chrono>
#include <deque>


struct TimedEvent {
    std::string name;
    std::string type;  // "pressed" or "released"
    std::chrono::steady_clock::time_point time;
};

struct Joystick {
    float x_raw = 0;
    float y_raw = 0;
    float x = 0;
    float y = 0;
    float magnitude = 0;
    float angle = 0;
};

struct Triggers {
    float left = 0;
    float right = 0;
};

enum class ButtonEvent {
    None,
    Pressed,
    Released
};

struct ComboEvent {
    std::string holdButton;
    std::vector<std::string> sequence;
    std::string name;
};

struct GamepadEvent {
    std::string type;   // e.g. "button", "combo"
    std::string name;   // e.g. "A", "Left", "Sleep"
    std::string action; // e.g. "pressed", "released" (for buttons)
};

struct GamepadState {
    std::map<std::string, ButtonEvent> buttonEvents;
    Joystick leftStick;
    Joystick rightStick;
    Triggers triggers;

    // ── LOCKING MEMBERS ──
    bool movementLocked      = false;
    bool lookLocked          = false;
    Joystick lockedLeftStick;   // to store frozen movement
    Joystick lockedRightStick;  // to store frozen look
};

class GamepadController {
public:
    GamepadController();
    ~GamepadController();

    bool initialize(const std::vector<ComboEvent>& combos = {});
    bool isConnected() const;
    void update(GamepadState& state);
    void addCombo(const std::string& holdButton, const std::vector<std::string>& sequence, const std::string& eventName);
    bool isButtonPressed(const std::string& name) const;
    std::vector<GamepadEvent> pollEvents();
    
    void printJoystickAndTriggerChanges();

    std::unordered_map<std::string, std::function<void()>> comboCallbacks;

    void registerComboCallback(const std::string& name, std::function<void()> callback);

private:
    int fd = -1;
    std::string devicePath;

    std::map<int, std::string> buttonMap;
    std::map<std::string, bool> buttonStates;
    std::map<std::string, bool> previousButtonStates;

    std::deque<TimedEvent> eventHistory;


    int absState[ABS_CNT] = {0};
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> pressStartTime;
    std::unordered_map<std::string, bool> invalidClick;



    // For tracking previous joystick/trigger values to detect change
    Joystick prevLeftStick;
    Joystick prevRightStick;
    Triggers prevTriggers;

    Joystick leftStick;
    Joystick rightStick;
    Triggers triggers;

    std::vector<ComboEvent> comboEvents;
    std::vector<std::string> recentInputs;
    std::map<std::string, std::vector<std::string>> activeCombos;
    std::vector<GamepadEvent> pendingEvents;

    void scanDevices();
    void simulateButton(const std::string& name, const std::chrono::steady_clock::time_point& now,
    std::unordered_map<std::string, std::chrono::steady_clock::time_point>& lastEventTimes);
    void simulateRelease(const std::string& name, const std::chrono::steady_clock::time_point& now,
    std::unordered_map<std::string, std::chrono::steady_clock::time_point>& lastEventTimes);
    void handleEvent(const input_event& ev);
    void updateJoysticks();
    static void squareToCircle(float x_in, float y_in, float &x_out, float &y_out);

    void detectCombosAndClicks();
};
