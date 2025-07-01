#include "ThreePointLeg.hpp"
#include <algorithm>  // for std::clamp
#include <cmath>

ThreePointLeg::ThreePointLeg(float legAngleDeg)
    : legAngleRad_(deg2rad(legAngleDeg)) {}

// Main transform: body coords -> leg coords
Vec3 ThreePointLeg::bodyToLegCoords(const Vec3& footBodyPos) const {
    Vec3 v = footBodyPos;

    // 1) Rotate around Z by (legAngleRad - pi/2)
    rotateZ(v, legAngleRad_ - PI/2);

    // 2) Subtract hip horizontal offset along X axis (hip radius)
    v.x -= hipRadiusMm_;

    // 3) Rotate around Y by -45 degrees (-pi/4 radians) (servo tilt downward)
    rotateY(v, -PI/4);

    return v;
}

void ThreePointLeg::solveIK() {
    // Step 1: Calculate hip horizontal angle (radians)
    hipHorizontalAngleRad = std::atan2(footTarget.y, footTarget.x);

    // Cache rotation angle in radians for efficiency
    const float negHipHorizontalRad = -hipHorizontalAngleRad;

    // Step 2: Rotate foot target around Z by -hipHorizontalAngleRad (radians)
    Vec3 footInLegPlane = footTarget;
    rotateZ(footInLegPlane, negHipHorizontalRad);

    // Step 3: Translate along X axis by hipOffsetXMm_
    footInLegPlane.x -= hipOffsetXMm_;

    // Step 4: Compute distance d in XZ plane
    float d = std::sqrt(footInLegPlane.x * footInLegPlane.x + footInLegPlane.z * footInLegPlane.z);

    float femur = femurLengthMm_;
    float tibia = tibiaLengthMm_;

    // Step 5: Compute knee angle using law of cosines
    float cosKnee = (femur * femur + tibia * tibia - d * d) / (2.0f * femur * tibia);
    cosKnee = std::clamp(cosKnee, -1.0f, 1.0f);
    kneeAngleRad = std::acos(cosKnee);

    // Step 6: Compute beta angle at hip vertical servo
    float cosBeta = (femur * femur + d * d - tibia * tibia) / (2.0f * femur * d);
    cosBeta = std::clamp(cosBeta, -1.0f, 1.0f);
    float beta = std::acos(cosBeta);

    // Step 7: Angle gamma from X axis to foot vector in XZ plane
    float gamma = std::atan2(footInLegPlane.z, footInLegPlane.x);

    // Step 8: Hip vertical servo angle (radians)
    hipVerticalAngleRad = gamma - beta;

    // Offsets to servo zero positions to be applied later
}
