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

            constexpr Vec2() noexcept : x(0), y(0) {}
            constexpr explicit Vec2(float val) noexcept : x(val), y(val) {}
            constexpr Vec2(float nx, float ny) noexcept : x(nx), y(ny) {}

            constexpr Vec2 operator+(const Vec2 &other) const noexcept
            {
                return {x + other.x, y + other.y};
            }
            constexpr Vec2 operator-(const Vec2 &other) const noexcept
            {
                return {x - other.x, y - other.y};
            }

            constexpr Vec2 operator*(float scalar) const noexcept
            {
                return {x * scalar, y * scalar};
            }

            // CrossProduct
            constexpr float operator*(const Vec2 &other) const noexcept
            {
                return {data[0] * other.data[1] - data[1] * other.data[0]};
            }

            constexpr Vec2 operator/(float scalar) const noexcept
            {
                return {x / scalar, y / scalar};
            }

            constexpr float length() const noexcept
            {
                return std::sqrt(lengthSq());
            }

            constexpr float lengthSq() const noexcept
            {
                return x * x + y * y;
            }

            constexpr Vec2 normalized() const noexcept
            {
                float lenSq = lengthSq();
                if (lenSq < 1e-8f)
                    return {0, 0};
                float invLen = 1.f / std::sqrt(lenSq);

                return *this * invLen;
            }

            Vec2 operator-() const
            {
                return {-x, -y};
            }

            // So cringe ngl
            friend constexpr Vec2 operator*(float scalar, const Vec2 &v) noexcept
            {
                return v * scalar;
            }

            friend constexpr Vec2 operator/(float scalar, const Vec2 &v) noexcept
            {
                return v / scalar;
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

            constexpr Vec3() noexcept : x(0), y(0), z(0) {}
            constexpr explicit Vec3(float val) noexcept : x(val), y(val), z(val) {}
            constexpr Vec3(float nx, float ny, float nz) noexcept : x(nx), y(ny), z(nz) {}

            // data(0, 1, 2) is proven to be one next to the other
            constexpr Vec3 operator+(const Vec3 &other) const noexcept
            {
                Vec3 res;

                res.data[0] = data[0] + other.data[0];
                res.data[1] = data[1] + other.data[1];
                res.data[2] = data[2] + other.data[2];

                return res;
            }
            constexpr Vec3 operator-(const Vec3 &other) const noexcept
            {
                Vec3 res;

                res.data[0] = data[0] - other.data[0];
                res.data[1] = data[1] - other.data[1];
                res.data[2] = data[2] - other.data[2];

                return res;
            }

            // Dot
            constexpr Vec3 operator*(float scalar) const noexcept
            {
                Vec3 res;

                res.data[0] = data[0] * scalar;
                res.data[1] = data[1] * scalar;
                res.data[2] = data[2] * scalar;

                return res;
            }

            constexpr Vec3 operator/(float scalar) const noexcept
            {
                Vec3 res;

                res.data[0] = data[0] / scalar;
                res.data[1] = data[1] / scalar;
                res.data[2] = data[2] / scalar;

                return res;
            }

            constexpr float length() const noexcept
            {
                return std::sqrt(lengthSq());
            }

            constexpr float lengthSq() const noexcept
            {
                return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
            }

            constexpr Vec3 normalized() const noexcept
            {
                float lenSq = lengthSq();
                if (lenSq < 1e-8f)
                    return {0, 0, 0};
                float invLen = 1.f / std::sqrt(lenSq);

                return *this * invLen;
            }

            Vec3 operator-() const
            {
                return {-x, -y, -z};
            }

            // Cross
            constexpr Vec3 operator*(const Vec3 &other) const noexcept
            {
                return {
                    data[1] * other.data[2] - data[2] * other.data[1],
                    data[2] * other.data[0] - data[0] * other.data[2],
                    data[0] * other.data[1] - data[1] * other.data[0]};
            }

            friend constexpr Vec3 operator*(float scalar, const Vec3 &v) noexcept
            {
                return v * scalar;
            }

            friend constexpr Vec3 operator/(float scalar, const Vec3 &v) noexcept
            {
                return v / scalar;
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

            constexpr Vec4() noexcept : x(0), y(0), z(0), w(0) {}
            constexpr explicit Vec4(float val) noexcept : x(val), y(val), z(val), w(val) {}

            constexpr Vec4(float nx, float ny, float nz, float nw) noexcept : x(nx), y(ny), z(nz), w(nw) {}

            constexpr Vec4 operator+(const Vec4 &other) const noexcept
            {
                Vec4 res;

                res.data[0] = data[0] + other.data[0];
                res.data[1] = data[1] + other.data[1];
                res.data[2] = data[2] + other.data[2];
                res.data[3] = data[3] + other.data[3];

                return res;
            }
            constexpr Vec4 operator-(const Vec4 &other) const noexcept
            {
                Vec4 res;

                res.data[0] = data[0] - other.data[0];
                res.data[1] = data[1] - other.data[1];
                res.data[2] = data[2] - other.data[2];
                res.data[3] = data[3] - other.data[3];

                return res;
            }

            constexpr Vec4 operator*(float scalar) const noexcept
            {
                Vec4 res;

                res.data[0] = data[0] * scalar;
                res.data[1] = data[1] * scalar;
                res.data[2] = data[2] * scalar;
                res.data[3] = data[3] * scalar;

                return res;
            }

            constexpr Vec3 operator*(const Vec3 &other) const noexcept
            {

                Vec3 temp{data[0], data[1], data[2]};

                return temp * other;
            }

            constexpr Vec4 operator/(float scalar) const noexcept
            {
                Vec4 res;

                res.data[0] = data[0] / scalar;
                res.data[1] = data[1] / scalar;
                res.data[2] = data[2] / scalar;
                res.data[3] = data[3] / scalar;

                return res;
            }

            constexpr float length() const noexcept
            {
                return std::sqrt(lengthSq());
            }

            constexpr float lengthSq() const noexcept
            {
                return data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
            }

            constexpr Vec4 normalized() const noexcept
            {
                float lenSq = lengthSq();
                if (lenSq < 1e-8f)
                    return {0, 0, 0, 0};
                float invLen = 1.f / std::sqrt(lenSq);

                return *this * invLen;
            }

            Vec4 operator-() const
            {
                return {-x, -y, -z, -w};
            }

            friend constexpr Vec4 operator*(float scalar, const Vec4 &v) noexcept
            {
                return v * scalar;
            }

            friend constexpr Vec4 operator/(float scalar, const Vec4 &v) noexcept
            {
                return v / scalar;
            }
        };

        static_assert(sizeof(Vec4) == 16);

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

        struct Mat2
        {
        private:
            float data[4];

            size_t getIndex(size_t row, size_t col) const noexcept
            {
                return col * Rows + row;
            }

        public:
            static constexpr unsigned Rows = 2;
            static constexpr unsigned Cols = 2;
            static constexpr unsigned ElementCount = Rows * Cols;

            constexpr Mat2() noexcept
                : data{1.f, 0.f,
                       0.f, 1.f} {}

            constexpr explicit Mat2(float val) noexcept
            {
                for (unsigned i = 0; i < ElementCount; ++i)
                {
                    data[i] = val;
                }
            }

            constexpr Mat2(float m00, float m01,
                           float m10, float m11) noexcept
                : data{m00, m10, m01, m11} {}

            constexpr float &operator[](size_t index) noexcept
            {
                return data[index];
            }
            constexpr const float &operator[](size_t index) const noexcept
            {
                return data[index];
            }

            constexpr float &operator()(size_t row, size_t col) noexcept
            {
                return data[getIndex(row, col)];
            }
            constexpr const float &operator()(size_t row, size_t col) const noexcept
            {
                return data[getIndex(row, col)];
            }

            // Pointer access for passing directly to graphics APIs (OpenGL/Vulkan)
            constexpr float *data_ptr() noexcept
            {
                return data;
            }
            constexpr const float *data_ptr() const noexcept
            {
                return data;
            }

            constexpr Mat2 operator*(const Mat2 &other) const noexcept
            {
                Mat2 result;

                result[0] = data[0] * other[0] + data[2] * other[1];
                result[1] = data[1] * other[0] + data[3] * other[1];

                result[2] = data[0] * other[2] + data[2] * other[3];
                result[3] = data[1] * other[2] + data[3] * other[3];

                return result;
            }

            constexpr Mat2 operator*(float scalar) const noexcept
            {
                Mat2 temp;
                temp[0] = data[0] * scalar;
                temp[1] = data[1] * scalar;
                temp[2] = data[2] * scalar;
                temp[3] = data[3] * scalar;

                return temp;
            }
        };

        struct Mat3
        {
        private:
            float data[9];

            size_t getIndex(size_t row, size_t col) const noexcept
            {
                return col * Rows + row;
            }

        public:
            static constexpr unsigned Rows = 3;
            static constexpr unsigned Cols = 3;
            static constexpr unsigned ElementCount = Rows * Cols;

            constexpr Mat3() noexcept
                : data{1.f, 0.f, 0.f,
                       0.f, 1.f, 0.f,
                       0.f, 0.f, 1.f} {}

            constexpr explicit Mat3(float val) noexcept
            {
                for (unsigned i = 0; i < ElementCount; ++i)
                {
                    data[i] = val;
                }
            }

            constexpr Mat3(float m00, float m01, float m02,
                           float m10, float m11, float m12,
                           float m20, float m21, float m22) noexcept
                : data{m00, m10, m20, m01, m11, m21, m02, m12, m22} {}

            constexpr explicit Mat3(const Mat2 &m2) noexcept
                : data{
                      m2(0, 0), m2(1, 0), 0.0f, // Column 0
                      m2(0, 1), m2(1, 1), 0.0f, // Column 1
                      0.0f, 0.0f, 1.0f}
            {
            }
            constexpr float &operator[](size_t index) noexcept
            {
                return data[index];
            }
            constexpr const float &operator[](size_t index) const noexcept
            {
                return data[index];
            }

            constexpr float &operator()(size_t row, size_t col) noexcept
            {
                return data[getIndex(row, col)];
            }
            constexpr const float &operator()(size_t row, size_t col) const noexcept
            {
                return data[getIndex(row, col)];
            }

            constexpr float *data_ptr() noexcept
            {
                return data;
            }
            constexpr const float *data_ptr() const noexcept
            {
                return data;
            }

            constexpr Mat3 operator*(const Mat3 &other) const noexcept
            {
                Mat3 result;

                result[0] = data[0] * other[0] + data[3] * other[1] + data[6] * other[2];
                result[1] = data[1] * other[0] + data[4] * other[1] + data[7] * other[2];
                result[2] = data[2] * other[0] + data[5] * other[1] + data[8] * other[2];

                result[3] = data[0] * other[3] + data[3] * other[4] + data[6] * other[5];
                result[4] = data[1] * other[3] + data[4] * other[4] + data[7] * other[5];
                result[5] = data[2] * other[3] + data[5] * other[4] + data[8] * other[5];

                result[6] = data[0] * other[6] + data[3] * other[7] + data[6] * other[8];
                result[7] = data[1] * other[6] + data[4] * other[7] + data[7] * other[8];
                result[8] = data[2] * other[6] + data[5] * other[7] + data[8] * other[8];

                return result;
            }

            constexpr Mat3 operator*(float scalar) const noexcept
            {
                Mat3 temp;
                for (unsigned i = 0; i < ElementCount; ++i)
                {
                    temp[i] = data[i] * scalar;
                }

                return temp;
            }
        };

        struct Mat4
        {
        private:
            float data[16];

            size_t getIndex(size_t row, size_t col) const noexcept
            {
                return col * Rows + row;
            }

        public:
            static constexpr unsigned Rows = 4;
            static constexpr unsigned Cols = 4;
            static constexpr unsigned ElementCount = Rows * Cols;

            constexpr Mat4() noexcept
                : data{1.f, 0.f, 0.f, 0.f,
                       0.f, 1.f, 0.f, 0.f,
                       0.f, 0.f, 1.f, 0.f,
                       0.f, 0.f, 0.f, 1.f} {}

            constexpr explicit Mat4(float val) noexcept
            {
                for (unsigned i = 0; i < ElementCount; ++i)
                {
                    data[i] = val;
                }
            }

            constexpr Mat4(float m00, float m01, float m02, float m03,
                           float m10, float m11, float m12, float m13,
                           float m20, float m21, float m22, float m23,
                           float m30, float m31, float m32, float m33) noexcept
                : data{m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33} {}

            constexpr explicit Mat4(const Mat2 &m2) noexcept
                : data{
                      m2(0, 0), m2(1, 0), 0.0f, 0.0f,
                      m2(0, 1), m2(1, 1), 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f}
            {
            }

            constexpr explicit Mat4(const Mat3 &m3) noexcept
                : data{
                      m3(0, 0), m3(1, 0), m3(2, 0), 0.0f,
                      m3(0, 1), m3(1, 1), m3(2, 1), 0.0f,
                      m3(0, 2), m3(1, 2), m3(2, 2), 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f}
            {
            }
            constexpr float &operator[](size_t index) noexcept
            {
                return data[index];
            }
            constexpr const float &operator[](size_t index) const noexcept
            {
                return data[index];
            }

            constexpr float &operator()(size_t row, size_t col) noexcept
            {
                return data[getIndex(row, col)];
            }
            constexpr const float &operator()(size_t row, size_t col) const noexcept
            {
                return data[getIndex(row, col)];
            }

            constexpr float *data_ptr() noexcept
            {
                return data;
            }
            constexpr const float *data_ptr() const noexcept
            {
                return data;
            }

            constexpr Mat4 operator*(const Mat4 &other) const noexcept
            {
                Mat4 result;

                result[0] = data[0] * other[0] + data[4] * other[1] + data[8] * other[2] + data[12] * other[3];
                result[1] = data[1] * other[0] + data[5] * other[1] + data[9] * other[2] + data[13] * other[3];
                result[2] = data[2] * other[0] + data[6] * other[1] + data[10] * other[2] + data[14] * other[3];
                result[3] = data[3] * other[0] + data[7] * other[1] + data[11] * other[2] + data[15] * other[3];

                result[4] = data[0] * other[4] + data[4] * other[5] + data[8] * other[6] + data[12] * other[7];
                result[5] = data[1] * other[4] + data[5] * other[5] + data[9] * other[6] + data[13] * other[7];
                result[6] = data[2] * other[4] + data[6] * other[5] + data[10] * other[6] + data[14] * other[7];
                result[7] = data[3] * other[4] + data[7] * other[5] + data[11] * other[6] + data[15] * other[7];

                result[8] = data[0] * other[8] + data[4] * other[9] + data[8] * other[10] + data[12] * other[11];
                result[9] = data[1] * other[8] + data[5] * other[9] + data[9] * other[10] + data[13] * other[11];
                result[10] = data[2] * other[8] + data[6] * other[9] + data[10] * other[10] + data[14] * other[11];
                result[11] = data[3] * other[8] + data[7] * other[9] + data[11] * other[10] + data[15] * other[11];

                result[12] = data[0] * other[12] + data[4] * other[13] + data[8] * other[14] + data[12] * other[15];
                result[13] = data[1] * other[12] + data[5] * other[13] + data[9] * other[14] + data[13] * other[15];
                result[14] = data[2] * other[12] + data[6] * other[13] + data[10] * other[14] + data[14] * other[15];
                result[15] = data[3] * other[12] + data[7] * other[13] + data[11] * other[14] + data[15] * other[15];

                return result;
            }

            constexpr Mat4 operator*(float scalar) const noexcept
            {
                Mat4 temp;
                for (unsigned i = 0; i < ElementCount; ++i)
                {
                    temp[i] = data[i] * scalar;
                }

                return temp;
            }
        };

        Mat4 lookAt(const Vec3 &position, const Vec3 &target, const Vec3 &worldUp) noexcept;
        Mat4 perspective(float fovY, float aspect, float zNear, float zFar) noexcept;

        Mat4 translate(const Mat4& model, const Vec3& position) noexcept;
        Mat4 rotate(const Mat4& model, float angleDeg, const Vec3& degrees) noexcept;
        Mat4 scale(const Mat4& model, const Vec3& scale) noexcept;

        float degToRad(float deg) noexcept;
        float radToDeg(float rad) noexcept;

    } // namespace math

} // namespace sas