#include "Math.hpp"

sas::math::Mat4 sas::math::lookAt(const Vec3 &position, const Vec3 &target, const Vec3 &worldUp) noexcept
{
    Vec3 Z = (position - target).normalized();
    Vec3 X = (worldUp * Z).normalized();
    Vec3 Y = Z * X;

    return Mat4(
        X.x, X.y, X.z, dotProduct(-1 * X, position),
        Y.x, Y.y, Y.z, dotProduct(-1 * Y, position),
        Z.x, Z.y, Z.z, dotProduct(-1 * Z, position),
        0.0f, 0.0f, 0.0f, 1.0f);
}

sas::math::Mat4 sas::math::perspective(float fovY, float aspect, float zNear, float zFar) noexcept
{
    const float tanHalfFovY = std::tan(fovY / 2.0f);

    sas::math::Mat4 result(0.0f);

    result(0, 0) = 1.0f / (aspect * tanHalfFovY);

    result(1, 1) = 1.0f / tanHalfFovY;

    result(2, 2) = zFar / (zNear - zFar);
    result(3, 2) = -1.0f;

    result(2, 3) = -(zFar * zNear) / (zFar - zNear);
    result(3, 3) = 0.0f;

    return result;
}

sas::math::Mat4 sas::math::translate(const Mat4 &model, const Vec3 &position) noexcept
{
    Mat4 result{model};

    const float x = position.data[0];
    const float y = position.data[1];
    const float z = position.data[2];

    result[12] = model[0] * x + model[4] * y + model[8] * z + model[12];
    result[13] = model[1] * x + model[5] * y + model[9] * z + model[13];
    result[14] = model[2] * x + model[6] * y + model[10] * z + model[14];
    result[15] = model[3] * x + model[7] * y + model[11] * z + model[15];

    return result;
}

sas::math::Mat4 sas::math::rotate(const Mat4 &m, float angleDeg, const Vec3 &v) noexcept
{
    const float angleRad = degToRad(angleDeg);
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);

    const Vec3 a = v.normalized();
    const float k = 1.f - c;

    const float r00 = c + a.x * a.x * k;
    const float r01 = a.x * a.y * k - s * a.z;
    const float r02 = a.x * a.z * k + s * a.y;

    const float r10 = a.y * a.x * k + s * a.z;
    const float r11 = c + a.y * a.y * k;
    const float r12 = a.y * a.z * k - s * a.x;

    const float r20 = a.z * a.x * k - s * a.y;
    const float r21 = a.z * a.y * k + s * a.x;
    const float r22 = c + a.z * a.z * k;

    Mat4 Result;

    Result[0] = m[0] * r00 + m[4] * r10 + m[8] * r20;
    Result[1] = m[1] * r00 + m[5] * r10 + m[9] * r20;
    Result[2] = m[2] * r00 + m[6] * r10 + m[10] * r20;
    Result[3] = m[3] * r00 + m[7] * r10 + m[11] * r20;

    Result[4] = m[0] * r01 + m[4] * r11 + m[8] * r21;
    Result[5] = m[1] * r01 + m[5] * r11 + m[9] * r21;
    Result[6] = m[2] * r01 + m[6] * r11 + m[10] * r21;
    Result[7] = m[3] * r01 + m[7] * r11 + m[11] * r21;

    Result[8] = m[0] * r02 + m[4] * r12 + m[8] * r22;
    Result[9] = m[1] * r02 + m[5] * r12 + m[9] * r22;
    Result[10] = m[2] * r02 + m[6] * r12 + m[10] * r22;
    Result[11] = m[3] * r02 + m[7] * r12 + m[11] * r22;

    Result[12] = m[12];
    Result[13] = m[13];
    Result[14] = m[14];
    Result[15] = m[15];

    return Result;
}

sas::math::Mat4 sas::math::scale(const Mat4 &m, const Vec3 &v) noexcept
{
    Mat4 Result;

    Result[0] = m[0] * v.x;
    Result[1] = m[1] * v.x;
    Result[2] = m[2] * v.x;
    Result[3] = m[3] * v.x;

    Result[4] = m[4] * v.y;
    Result[5] = m[5] * v.y;
    Result[6] = m[6] * v.y;
    Result[7] = m[7] * v.y;

    Result[8] = m[8] * v.z;
    Result[9] = m[9] * v.z;
    Result[10] = m[10] * v.z;
    Result[11] = m[11] * v.z;

    Result[12] = m[12];
    Result[13] = m[13];
    Result[14] = m[14];
    Result[15] = m[15];

    return Result;
}

float sas::math::degToRad(float deg) noexcept
{
    return deg * M_PI / 180;
}

float sas::math::radToDeg(float rad) noexcept
{
    return rad * 180 / M_PI;
}