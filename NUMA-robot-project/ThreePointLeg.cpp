#include "ThreePointLeg.hpp"
#include <algorithm>  // for std::clamp
#include <cmath>

ThreePointLeg::ThreePointLeg(float legAngleDeg)
    : legAngleRad_(deg2rad(legAngleDeg)) {}

// Main transform: body coords -> leg coords
Vec3 ThreePointLeg::bodyToLegCoords(const Vec3& footBodyPos) const {
    Vec3 v = footBodyPos;

    // 1) Rotate around Z by (-legAngleRad)
    rotateZ(v, -legAngleRad_);

    // 2) Subtract hip horizontal offset along X axis (hip radius)
    v.x -= hipRadiusMm_;

    // 3) Rotate around Y by -45 degrees (-pi/4 radians) (servo tilt downward)
    rotateY(v, -PI/4);

    return v;
}


void ThreePointLeg::solveIK() {
    // --- Step 1: Calculate hip horizontal angle (radians)
    // Angle between leg projection on XY plane and X axis,
    // positive CCW, negative CW (atan2 returns CCW positive)
    rawHipHorizontalRad = std::atan2(footTarget.y, footTarget.x);

    // --- Step 2: Rotate foot target into the hip vertical plane by undoing horizontal rotation
    Vec3 footInLegPlane = footTarget;
    rotateZ(footInLegPlane, -rawHipHorizontalRad * 180.0f / PI);  // rotateZ expects degrees

    // --- Step 3: Translate along X axis to hip vertical servo origin
    footInLegPlane.x -= hipOffsetXMm_;

    // --- Step 4: Compute distance d in XZ plane (distance from vertical servo to foot)
    float d = std::sqrt(footInLegPlane.x * footInLegPlane.x + footInLegPlane.z * footInLegPlane.z);

    // --- Step 5: Compute knee angle using law of cosines
    float cosKnee = (femurLengthMm_ * femurLengthMm_ + tibiaLengthMm_ * tibiaLengthMm_ - d * d) / 
                    (2.0f * femurLengthMm_ * tibiaLengthMm_);
    cosKnee = std::clamp(cosKnee, -1.0f, 1.0f);
    rawKneeRad = std::acos(cosKnee);

    // --- Step 6: Compute angle beta (angle between femur and line from hip vertical servo to foot)
    float cosBeta = (femurLengthMm_ * femurLengthMm_ + d * d - tibiaLengthMm_ * tibiaLengthMm_) / 
                    (2.0f * femurLengthMm_ * d);
    cosBeta = std::clamp(cosBeta, -1.0f, 1.0f);
    float beta = std::acos(cosBeta);

    // --- Step 7: Compute angle gamma (angle between X axis and foot vector in XZ plane)
    float gamma = std::atan2(footInLegPlane.z, footInLegPlane.x);

    // --- Step 8: Compute raw hip vertical servo angle (radians)
    // This is the angle the hip vertical servo needs to move from horizontal to reach foot position
    rawHipVerticalRad = gamma - beta;

    // --- Step 9: Apply servo mounting offsets and direction conventions
    // Note:
    // - Horizontal servo rotation: servo turns clockwise with increasing PWM,
    //   so invert rawHipHorizontalRad for servo command.
    // - Hip vertical servo: adjust for mechanical offset and direction inversion.
    // - Knee servo: offset for servo zero and direction.

    servoHipHorizontalRad = 0 - (rawHipHorizontalRad - hipHorizontalServoOffsetRad_) * hipHorizontalGearRatio;  // invert direction
    servoHipVerticalRad = 0 - (rawHipVerticalRad - hipVerticalServoOffsetRad_) * hipVerticalGearRatio;  // invert and offset
    servoKneeRad = (rawKneeRad - kneeServoOffsetRad_) * kneeGearRatio;  // apply offset, no direction inversion assumed
}

