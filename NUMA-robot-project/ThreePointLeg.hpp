#pragma once
#include <array>
#include <cmath>

constexpr float PI = 3.14159265358979323846f;

struct Vec3 {
    float x;
    float y;
    float z;

    Vec3() : x(0.f), y(0.f), z(0.f) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

class ThreePointLeg {
public:
    // Constructor takes leg angle in degrees but converts to radians internally
    explicit ThreePointLeg(float legAngleDeg);
    float legAngleRad_;          // Leg angle from front (Y axis) in radians

    // Converts foot position from body coords to leg coords for IK
    Vec3 bodyToLegCoords();

    // Public foot target in body coordinates (set this before calling solveIK)
    Vec3 footTarget = {0.f, 0.f, 0.f};
    Vec3 footInLegTarget = {0.f, 0.f, 0.f}; // Foot target in leg coordinates (after IK)


    // Raw IK solved joint angles (radians)
    float rawHipHorizontalRad = 0.f;
    float rawHipVerticalRad = 0.f;
    float rawKneeRad = 0.f;

    // Final servo command angles (radians), after applying mechanical offsets
    float servoHipHorizontalRad = 0.f;
    float servoHipVerticalRad = 0.f;
    float servoKneeRad = 0.f;

    // Servo zero position mechanical offsets (radians)
    float hipHorizontalServoOffsetRad_ = (0.0f) * PI / 180.0f;      // hip horizontal servo offset: 0° (no offset)
    float hipVerticalServoOffsetRad_ = (25.0f) * PI / 180.0f;       // hip vertical servo offset: +25°
    float kneeServoOffsetRad_ = (97.0f) * PI / 180.0f;             // knee servo offset: 97°

    // Servo gear ratios (input angle / output angle)
    float hipHorizontalGearRatio = 270.f / 120.f;  // 2.25
    float hipVerticalGearRatio = 1.f;               // 1:1
    float kneeGearRatio = 270.f / 140.f;             // ~1.93


    // Solve IK for current footTarget and update joint angles
    void solveIK();

    // Helper conversions
    inline static float deg2rad(float deg) {
        return deg * PI / 180.0f;
    }

    inline static float rad2deg(float rad) {
        return rad * 180.0f / PI;
    }

    // Rotate vector v around Z by angle radians (note: angle in radians)
    inline void rotateZ(Vec3& v, float angleRad) {
        float cosA = std::cos(angleRad);
        float sinA = std::sin(angleRad);
        float xNew = v.x * cosA - v.y * sinA;
        float yNew = v.x * sinA + v.y * cosA;
        v.x = xNew;
        v.y = yNew;
    }

    // Rotate vector v around Y by angle radians (note: angle in radians)
    inline void rotateY(Vec3& v, float angleRad) {
        float c = std::cos(angleRad);
        float s = std::sin(angleRad);
        float x = v.x, z = v.z;
        v.x = c * x + s * z;
        v.z = -s * x + c * z;
    }

private:

    // Physical dimensions (cm)
    static constexpr float hipRadius_ = 17.49f;   // Fixed hip radius
    static constexpr float hipOffsetX_ = 3.55f;   // Offset from hip horizontal to vertical joint (joint length)
    static constexpr float femurLength_ = 15.0f;  // Length of upper leg (femur)
    static constexpr float tibiaLength_ = 17.0f;  // Length of lower leg (tibia / shin)

    // Physical/mechanical joint limits (radians)
    static constexpr float kneeMinAngleRad_ = 0.0f;
    static constexpr float kneeMaxAngleRad_ = 0.0f;

    static constexpr float hipVerticalMinAngleRad_ = (-90.0f) * PI / 180.0f;
    static constexpr float hipVerticalMaxAngleRad_ = (90.0f) * PI / 180.0f;
};
