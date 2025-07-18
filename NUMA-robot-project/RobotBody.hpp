#pragma once
#include <vector>
#include <array>
#include "ThreePointLeg.hpp"
#include <algorithm>
#include "chrono"


enum class LightMode {
    Off,
    Steady,
    Strobe
};

struct FaceServo {
    float value = 0.f;
    float minDeg;
    float maxDeg;
    float restDeg;
    float retractDeg;
    bool isRetracted = false;

    float getClampedPosition() const {
        return isRetracted ? retractDeg : std::clamp(value, minDeg, maxDeg);
    }

    void resetToRest() {
        value = restDeg;
        isRetracted = false;
    }

    void retract() {
        isRetracted = true;
    }
};

struct FaceState {
    FaceServo pan = { 0.f, -70.f, 70.f, 0.f, 0.f };
    FaceServo tilt = { 40.f, 20.f, 60.f, 40.f, -60.f };

    LightMode lightMode = LightMode::Off;
    float lightBrightness = 1.0f;   // base brightness (0–1)
    float strobePWM = 1.0f;         // modulation for strobe "on" state (0–1)

    // internal state
    bool strobeLightOn = false;
    std::chrono::steady_clock::time_point lastToggle = std::chrono::steady_clock::now();

    bool isRetracted() const {
        return tilt.isRetracted;  // defines retract state for face and light logic
    }

    bool lightShouldBeOn() const {
        return isRetracted() && lightMode != LightMode::Off && lightBrightness > 0.f;
    }

        // Final computed output brightness (0–1)
    float getLightOutput() const {
        if (!lightShouldBeOn()) return 0.0f;

        switch (lightMode) {
            case LightMode::Steady:
                return lightBrightness;
            case LightMode::Strobe:
                return strobeLightOn ? (lightBrightness * strobePWM) : 0.0f;
            default:
                return 0.0f;
        }
    }

    void updateStrobeState() {
        if (lightMode != LightMode::Strobe || !isRetracted()) return;

        const int strobeRateHz = 12;
        const int intervalMs = 1000 / (strobeRateHz * 2); // toggle half-cycle

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastToggle).count();

        if (elapsed > intervalMs) {
            strobeLightOn = !strobeLightOn;
            lastToggle = now;
        }
    }

};


class RobotBody {
public:
            
    // Hardcoded collision geometry parameters, all in cm now
    float originStartingHeight = 12.2f;      // 12.2 cm above ground (was 0.122 m)
    float bodyBottomFromOrigin = -11.6f;     // bottom plate is 11.6 cm below origin (was -0.116 m)
    float bottomPlateRadius = 14.6f;         // 14.6 cm radius bottom plate (was 0.146 m)
    float legPlaneFromOrigin = -6.74f;     // 6.74 cm from origin to hip plane (was 0.0674 m)


    Vec3 position = {0.f, 0.f, originStartingHeight};
    float tiltAzimuthDeg = 0.f;
    float tiltPolarDeg = 0.f;
    float headingDeg = 0.f;

    Vec3 worldToBodyCoords(const Vec3& worldPos) const;
    Vec3 worldToLegPlaneCoords(const Vec3& footWorld) const;

    RobotBody() = default;
};
