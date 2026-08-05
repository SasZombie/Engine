#include "Math.hpp"

sas::math::Mat4 sas::math::lookAt(const Vec3 &position, const Vec3 &target, const Vec3 &worldUp) noexcept
{
    Vec3 Z = (position - target).normalized();
    Vec3 X = (worldUp * Z).normalized();
    Vec3 Y = Z * X;

    return Mat4(
        X.x,  X.y,  X.z,  dotProduct(-1 * X, position),
        Y.x,  Y.y,  Y.z,  dotProduct(-1 * Y, position),
        Z.x,  Z.y,  Z.z,  dotProduct(-1 * Z, position),
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

sas::math::Mat4 sas::math::perspective(float fovY, float aspect, float zNear, float zFar) noexcept
{
    const float tanHalfFovY = std::tan(fovY / 2.0f);

    sas::math::Mat4 result(0.0f); 

    result(0, 0) = 1.0f / (aspect * tanHalfFovY);

    result(1, 1) = 1.0f / tanHalfFovY;

    result(2, 2) = zFar / (zNear - zFar); // = -1.001001
    result(3, 2) = -1.0f;

    result(2, 3) = -(zFar * zNear) / (zFar - zNear); // = +0.1001001 (FIXED SIGN)
    result(3, 3) = 0.0f;

    return result;
}

float sas::math::degToRad(float deg) noexcept
{
    return deg * M_PI / 180;
}

float sas::math::radToDeg(float rad) noexcept
{
    return rad * 180 / M_PI;
}