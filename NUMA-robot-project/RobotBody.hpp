#pragma once
#include <vector>
#include <array>
#include "ThreePointLeg.hpp"


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
