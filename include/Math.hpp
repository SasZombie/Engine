#pragma once

#include <cmath>

namespace sas
{
    namespace math
    {
        struct alignas(16) Vec2
        {
            static constexpr int ElementCount = 2;
            union
            {
                struct
                {
                    float x, y;
                };

                float data[2];
            };

            constexpr Vec2() : x(0), y(0) {}
            constexpr explicit Vec2(float val) : x(val), y(val) {}
            constexpr Vec2(float nx, float ny) : x(nx), y(ny) {}

            constexpr Vec2 operator+(const Vec2 &other) const
            {
                return {x + other.x, y + other.y};
            }
            constexpr Vec2 operator-(const Vec2 &other) const
            {
                return {x - other.x, y - other.y};
            }

            constexpr Vec2 operator*(float scalar) const
            {
                return {x * scalar, y * scalar};
            }

            constexpr Vec2 operator/(float scalar) const
            {
                return {x / scalar, y / scalar};
            }

            constexpr float length() const
            {
                return std::sqrt(lengthSq());
            }

            constexpr float lengthSq() const
            {
                return x * x + y * y;
            }

            constexpr Vec2 normalized() const
            {
                float lenSq = lengthSq();
                if (lenSq < 1e-8f)
                    return {0, 0};
                float invLen = 1.f / std::sqrt(lenSq);

                return *this * invLen;
            }
        };

        static_assert(sizeof(Vec2) == 16);

        struct alignas(16) Vec3
        {
            union
            {
                struct
                {
                    float x, y, z;
                };

                float data[3];
            };
            static constexpr int ElementCount = 3;

            constexpr Vec3() : x(0), y(0), z(0) {}
            constexpr explicit Vec3(float val) : x(val), y(val), z(val) {}
            constexpr Vec3(float nx, float ny, float nz) : x(nx), y(ny), z(nz) {}

            // data(0, 1, 2) is proven to be one next to the other
            constexpr Vec3 operator+(const Vec3 &other) const
            {
                Vec3 res;

                res.data[0] = data[0] + other.data[0];
                res.data[1] = data[1] + other.data[1];
                res.data[2] = data[2] + other.data[2];

                return res;
            }
            constexpr Vec3 operator-(const Vec3 &other) const
            {
                Vec3 res;

                res.data[0] = data[0] - other.data[0];
                res.data[1] = data[1] - other.data[1];
                res.data[2] = data[2] - other.data[2];

                return res;
            }

            constexpr Vec3 operator*(float scalar) const
            {
                Vec3 res;

                res.data[0] = data[0] * scalar;
                res.data[1] = data[1] * scalar;
                res.data[2] = data[2] * scalar;

                return res;
            }

            constexpr Vec3 operator/(float scalar) const
            {
                Vec3 res;

                res.data[0] = data[0] / scalar;
                res.data[1] = data[1] / scalar;
                res.data[2] = data[2] / scalar;

                return res;
            }

            constexpr float length() const
            {
                return std::sqrt(lengthSq());
            }

            constexpr float lengthSq() const
            {
                return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
            }

            constexpr Vec3 normalized() const
            {
                float lenSq = lengthSq();
                if (lenSq < 1e-8f)
                    return {0, 0, 0};
                float invLen = 1.f / std::sqrt(lenSq);

                return *this * invLen;
            }
        };

        static_assert(sizeof(Vec3) == 16);

        struct alignas(16) Vec4
        {
            static constexpr int ElementCount = 4;

            union
            {
                struct
                {
                    float x, y, z, w;
                };

                float data[4];
            };

            constexpr Vec4() : x(0), y(0), z(0), w(0) {}
            constexpr explicit Vec4(float val) : x(val), y(val), z(val), w(val) {}

            constexpr Vec4(float nx, float ny, float nz, float nw) : x(nx), y(ny), z(nz), w(nw) {}

            constexpr Vec4 operator+(const Vec4 &other) const
            {
                Vec4 res;

                res.data[0] = data[0] + other.data[0];
                res.data[1] = data[1] + other.data[1];
                res.data[2] = data[2] + other.data[2];
                res.data[3] = data[3] + other.data[3];

                return res;
            }
            constexpr Vec4 operator-(const Vec4 &other) const
            {
                Vec4 res;

                res.data[0] = data[0] - other.data[0];
                res.data[1] = data[1] - other.data[1];
                res.data[2] = data[2] - other.data[2];
                res.data[3] = data[3] - other.data[3];

                return res;
            }

            constexpr Vec4 operator*(float scalar) const
            {
                Vec4 res;

                res.data[0] = data[0] * scalar;
                res.data[1] = data[1] * scalar;
                res.data[2] = data[2] * scalar;
                res.data[3] = data[3] * scalar;

                return res;
            }

            constexpr Vec4 operator/(float scalar) const
            {
                Vec4 res;

                res.data[0] = data[0] / scalar;
                res.data[1] = data[1] / scalar;
                res.data[2] = data[2] / scalar;
                res.data[3] = data[3] / scalar;

                return res;
            }

            constexpr float length() const
            {
                return std::sqrt(lengthSq());
            }

            constexpr float lengthSq() const
            {
                return data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
            }

            constexpr Vec4 normalized() const
            {
                float lenSq = lengthSq();
                if (lenSq < 1e-8f)
                    return {0, 0, 0, 0};
                float invLen = 1.f / std::sqrt(lenSq);

                return *this * invLen;
            }
        };

        static_assert(sizeof(Vec4) == 16);


        // Vec3 crossProduct(const Vec3 &v1, const Vec3 &v2) noexcept
        // {
        //     return Vec3(
        //         v1.y * v2.z - v1.z * v2.y,
        //         v1.z * v2.x - v1.x * v2.z,
        //         v1.x * v2.y - v1.y * v2.x);
        // }

        // Limits to my custom vectors
        template <typename T>
        concept customVector = std::same_as<T, Vec2> || std::same_as<T, Vec3> || std::same_as<T, Vec4>;

        template <customVector T>
        float dotProduct(const T &vec1, const T &vec2) noexcept
        {
            float result = 0.0f;
            for (int i = 0; i < T::ElementCount; ++i)
            {
                result += vec1.data[i] * vec2.data[i];
            }
            return result;
        }

        // template <customVector T>
        // float distance(const T &vec1, const T &vec2) noexcept
        // {
        //     float result = (vec2.x - vec1.x) * (vec2.x - vec1.x) + (vec2.y - vec1.y) * (vec2.y - vec1.y);
        //     if constexpr (requires { vec1.z; })
        //     {
        //         result = result + (vec2.z - vec1.z) * (vec2.z - vec1.z);

        //         if constexpr (requires { vec1.w; })
        //         {
        //             result = result + (vec2.w - vec1.w) * (vec2.w - vec1.w);
        //         }
        //     }

        //     return std::sqrt(result);
        // }

        // template <customVector T>
        // T projection(const T &vec1, const T &vec2) noexcept
        // {
        //     float scalar = dotProduct(vec1, vec2) / dotProduct(vec2, vec2);

        //     return vec2 * scalar;
        // }

        // template <customVector T>
        // T lerp(const T &vec1, const T &vec2, float t) noexcept
        // {
        //     return vec1 * (1 - t) + vec2 * t;
        // }

        // template <customVector T>
        // T min(const T &vec1, const T &vec2) noexcept
        // {
        //     T result;
        //     result.x = std::min(vec1.x, vec2.x);
        //     result.y = std::min(vec1.y, vec2.y);

        //     if constexpr (requires { vec1.z; })
        //     {
        //         result.z = std::min(vec1.z, vec2.z);

        //         if constexpr (requires { vec1.w; })
        //         {
        //             result.w = std::min(vec1.w, vec2.w);
        //         }
        //     }

        //     return result;
        // }

        // template <customVector T>
        // T max(const T &vec1, const T &vec2) noexcept
        // {
        //     T result;
        //     result.x = std::max(vec1.x, vec2.x);
        //     result.y = std::max(vec1.y, vec2.y);

        //     if constexpr (requires { vec1.z; })
        //     {
        //         result.z = std::max(vec1.z, vec2.z);

        //         if constexpr (requires { vec1.w; })
        //         {
        //             result.w = std::max(vec1.w, vec2.w);
        //         }
        //     }

        //     return result;
        // }

        // template <customVector T>
        // T clamp(const T &vec1, const T &min, const T &max) noexcept
        // {
        //     T result;
        //     result.x = std::clamp(vec1.x, min.x, max.x);
        //     result.y = std::clamp(vec1.y, min.y, max.y);

        //     if constexpr (requires { vec1.z; })
        //     {
        //         result.z = std::clamp(vec1.z, min.y, max.y);

        //         if constexpr (requires { vec1.w; })
        //         {
        //             result.w = std::clamp(vec1.w, min.y, max.y);
        //         }
        //     }

        //     return result;
        // }

        // struct Mat3
        // {
        //     float data[3][3]{};

        //     Mat3 identity() noexcept
        //     {
        //         Mat3 result;

        //         result.data[0][0] = 1;
        //         result.data[1][1] = 1;
        //         result.data[2][2] = 1;

        //         return result;
        //     }

        //     // Not doing all that
        //     Mat3 operator*(const Mat3 &other) const noexcept
        //     {
        //     }

        //     Mat3 transpose() noexcept
        //     {
        //         Mat3 result;
        //         for (int i = 0; i < 3; ++i)
        //         {
        //             for (int j = 0; j < 3; ++j)
        //             {
        //                 result.data[i][j] = data[j][i];
        //             }
        //         }

        //         return result;
        //     }

        //     // Diabolical, calculate adjugate + inverse + check det != 0
        //     Mat3 inverse() noexcept
        //     {
        //     }
        // };

        // struct Mat4
        // {
        //     float data[4][4]{};

        //     Mat4 identity() noexcept
        //     {
        //         Mat4 result;

        //         result.data[0][0] = 1;
        //         result.data[1][1] = 1;
        //         result.data[2][2] = 1;
        //         result.data[3][3] = 1;

        //         return result;
        //     }

        //     Mat4 operator*(const Mat4 &other) const noexcept
        //     {
        //     }

        //     Mat4 transpose() noexcept
        //     {
        //         Mat4 result;
        //         for (int i = 0; i < 4; ++i)
        //         {
        //             for (int j = 0; j < 4; ++j)
        //             {
        //                 result.data[i][j] = data[j][i];
        //             }
        //         }

        //         return result;
        //     }

        //     Mat4 inverse() noexcept
        //     {
        //     }
        // };

    } // namespace math

} // namespace sas