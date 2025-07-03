#include "RobotBody.hpp"
#include <cmath>

// Rotation helper functions updated for Vec3 with x,y,z members

constexpr float deg2rad(float deg) {
    return deg * 3.14159265358979323846f / 180.f;
}

void rotateX(float angleDeg, Vec3& v) {
    float rad = deg2rad(angleDeg);
    float c = std::cos(rad);
    float s = std::sin(rad);
    float y = v.y;
    float z = v.z;
    v.y = c * y - s * z;
    v.z = s * y + c * z;
}

void rotateY(float angleDeg, Vec3& v) {
    float rad = deg2rad(angleDeg);
    float c = std::cos(rad);
    float s = std::sin(rad);
    float x = v.x;
    float z = v.z;
    v.x = c * x + s * z;
    v.z = -s * x + c * z;
}

void rotateZ(float angleDeg, Vec3& v) {
    float rad = deg2rad(angleDeg);
    float c = std::cos(rad);
    float s = std::sin(rad);
    float x = v.x;
    float y = v.y;
    v.x = c * x - s * y;
    v.y = s * x + c * y;
}

// Convert world coordinates to body coordinates (using BodyPose)
Vec3 RobotBody::worldToBodyCoords(const Vec3& footWorld) const
{
    Vec3 p = {
        footWorld.x - position.x,
        footWorld.y - position.y,
        footWorld.z - position.z
    };

    // Undo heading (rotate -heading around Z)
    rotateZ(-headingDeg, p);

    // Undo tilt azimuth (rotate -tiltAzimuth around Z)
    rotateZ(-tiltAzimuthDeg, p);

    // Undo tilt polar (rotate -tiltPolar around Y)
    rotateY(-tiltPolarDeg, p);

    // Reapply tilt azimuth (rotate +tiltAzimuth around Z)
    rotateZ(tiltAzimuthDeg, p);

    return p;
}

Vec3 RobotBody::worldToLegPlaneCoords(const Vec3& footWorld) const {
    // Convert world position to body coordinates first
    Vec3 bodyCoords = worldToBodyCoords(footWorld);

    // Subtract the leg hip plane offset on the z axis (vertical)
    bodyCoords.z -= legPlaneFromOrigin;  // legHipPlaneFromOrigin is in cm

    return bodyCoords;
}

