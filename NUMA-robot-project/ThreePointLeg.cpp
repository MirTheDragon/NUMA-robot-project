#include "ThreePointLeg.hpp"
#include <algorithm>  // for std::clamp
#include <cstdio>  // for printf
#include <cmath>

ThreePointLeg::ThreePointLeg(float legAngleDeg)
    : legAngleRad_(deg2rad(legAngleDeg)) {}

// Main transform: body coords -> leg coords
Vec3 ThreePointLeg::bodyToLegCoords() {
    Vec3 v = footTarget;

    // 1) Rotate around Z by (-legAngleRad)
    rotateZ(v, -legAngleRad_);

    // 2) Subtract hip horizontal offset along X axis (hip radius)
    v.x -= hipRadius_;

    // 3) Rotate around Y by -45 degrees (-pi/4 radians) (servo tilt downward)
    rotateY(v, -PI/4);

    footInLegTarget = v;
    return footInLegTarget;  // Return the transformed foot position in leg coordinates
}


void ThreePointLeg::solveIK() {

    bodyToLegCoords();  // Convert foot target to leg coordinates first


    // --- Step 1: Calculate hip horizontal angle (radians)
    // Angle between leg projection on XY plane and X axis,
    // positive CCW, negative CW (atan2 returns CCW positive)
    rawHipHorizontalRad = std::atan2(footInLegTarget.y, footInLegTarget.x);

    // --- Step 2: Rotate foot target into the hip vertical plane by undoing horizontal rotation

    Vec3 footInLegPlane = footInLegTarget;
    rotateZ(footInLegPlane, -rawHipHorizontalRad);  // rotateZ expects degrees

    // --- Step 3: Translate along X axis to hip vertical servo origin
    footInLegPlane.x -= hipOffsetX_;

    // --- Step 4: Compute distance d in XZ plane (distance from vertical servo to foot)
    float d = std::sqrt(footInLegPlane.x * footInLegPlane.x + footInLegPlane.z * footInLegPlane.z);

    // --- Step 5: Compute knee angle using law of cosines
    float cosKnee = (femurLength_ * femurLength_ + tibiaLength_ * tibiaLength_ - d * d) / 
                    (2.0f * femurLength_ * tibiaLength_);
    cosKnee = std::clamp(cosKnee, -1.0f, 1.0f);
    rawKneeRad = std::acos(cosKnee);

    // --- Step 6: Compute angle beta (angle between femur and line from hip vertical servo to foot)
    float cosBeta = (femurLength_ * femurLength_ + d * d - tibiaLength_ * tibiaLength_) / 
                    (2.0f * femurLength_ * d);
    cosBeta = std::clamp(cosBeta, -1.0f, 1.0f);
    float beta = std::acos(cosBeta);

    // --- Step 7: Compute angle gamma (angle between X axis and foot vector in XZ plane)
    float gamma = std::atan2(footInLegPlane.z, footInLegPlane.x);

    // --- Step 8: Compute raw hip vertical servo angle (radians)
    // This is the angle the hip vertical servo needs to move from horizontal to reach foot position
    rawHipVerticalRad = gamma + beta;

    /*
    // Debug printouts (angles converted to degrees)
    constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
    printf("=== IK Debug ===\n");
    printf("hipOffsetX = %.3f cm\n", hipOffsetX_);
    printf("d = %.3f cm\n", d);
    printf("beta = %.3f rad (%.2f deg)\n", beta, beta * RAD_TO_DEG);
    printf("gamma = %.3f rad (%.2f deg)\n", gamma, gamma * RAD_TO_DEG);
    printf("rawHipHorizontalRad = %.3f rad (%.2f deg)\n", rawHipHorizontalRad, rawHipHorizontalRad * RAD_TO_DEG);
    printf("rawHipVerticalRad = %.3f rad (%.2f deg)\n", rawHipVerticalRad, rawHipVerticalRad * RAD_TO_DEG);
    printf("rawKneeRad = %.3f rad (%.2f deg)\n", rawKneeRad, rawKneeRad * RAD_TO_DEG);
    */


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

