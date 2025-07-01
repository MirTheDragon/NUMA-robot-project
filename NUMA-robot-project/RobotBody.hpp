#pragma once
#include <vector>
#include <array>
#include "ThreePointLeg.hpp"

using Vec3 = std::array<float, 3>;

class RobotBody {
public:
    struct BodyPose {
        
        // New hardcoded collision geometry parameters
        float originStartingHeight = 0.122f; // origin is 12.2 cm above ground
        float bodyBottomFromOrigin = -0.116f;   // bottom plate is 11.6cm below origin
        float bottomPlateRadius = 0.146f;      // 14.6 cm radius bottom plate
        float legHipPlaneFromOrigin = 0.0674f; // 67.4 mm from origin to hip plane


        Vec3 position = {0.f, 0.f, originStartingHeight};
        float tiltAzimuthDeg = 0.f;
        float tiltPolarDeg = 0.f;
        float headingDeg = 0.f;


        BodyPose() = default;
        BodyPose(const Vec3& pos, float tiltAz, float tiltPol, float heading)
            : position(pos), tiltAzimuthDeg(tiltAz), tiltPolarDeg(tiltPol), headingDeg(heading) {}

        Vec3 worldToBodyCoords(const Vec3& worldPos) const;
    };

    BodyPose pose;
    std::vector<ThreePointLeg> legs;

    RobotBody() = default;

    void updateLegs();

    void addLeg(const ThreePointLeg& leg);
};
