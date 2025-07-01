#include "RobotBody.hpp"
#include <cmath>

// Helper math functions (rotate matrices) copied or included from earlier

constexpr float deg2rad(float deg) {
    return deg * 3.14159265358979323846f / 180.f;
}

void rotateX(float angleRad, float out[3][3]);
void rotateY(float angleRad, float out[3][3]);
void rotateZ(float angleRad, float out[3][3]);
void mat3_mul(const float a[3][3], const float b[3][3], float out[3][3]);
std::array<float,3> mat3_mul_vec3(const float mat[3][3], const std::array<float,3>& v);
void mat3_transpose(const float src[3][3], float dst[3][3]);

// --- BodyPose method implementation ---
Vec3 worldToBodyCoords(
    const Vec3& footWorld,
    const Vec3& bodyPosition,
    float tiltAzimuthDeg,
    float tiltPolarDeg,
    float headingDeg)
{
    // 1) Translate to body-relative
    Vec3 p = {footWorld[0] - bodyPosition[0],
              footWorld[1] - bodyPosition[1],
              footWorld[2] - bodyPosition[2]};

    // 2) Undo heading (rotate -heading around Z)
    rotateZ(p, -headingDeg);

    // 3) Undo tilt azimuth (rotate -tiltAzimuth around Z)
    rotateZ(p, -tiltAzimuthDeg);

    // 4) Undo tilt polar (rotate -tiltPolar around Y)
    rotateY(p, -tiltPolarDeg);

    // 5) Reapply tilt azimuth (rotate +tiltAzimuth around Z)
    rotateZ(p, tiltAzimuthDeg);

    return p;
}
// --- RobotBody methods ---

void RobotBody::updateLegs() {
    for (auto& leg : legs) {
        leg.updateLegCoords(pose);
        leg.solveIK();
    }
}

void RobotBody::addLeg(const ThreePointLeg& leg) {
    legs.push_back(leg);
}

// --- (You will need to implement or link the rotation helper functions here) ---
