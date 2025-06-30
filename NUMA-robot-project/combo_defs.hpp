#pragma once
#include <string>
#include <vector>
#include <map>

struct Combo {
    std::string name;
    std::string modifier;         // Button that must be held (e.g., "A")
    std::vector<std::string> sequence;  // Sequence of buttons (e.g., {"UP", "UP", "LEFT"})
};

inline const std::vector<Combo> predefinedCombos = {
    {"dash_left",  "A", {"UP", "UP", "LEFT"}},
    {"power_shot", "B", {"DOWN", "RIGHT"}},
    {"step_right", "X", {"RIGHT", "RIGHT"}},
};
