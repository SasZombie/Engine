#include <gtest/gtest.h>
#include "Math.hpp"

using namespace sas::math;

template <typename MatType, size_t Size>
void ExpectMatrixEq(const MatType &mat, const float (&expected)[Size])
{
    const float *data = mat.data_ptr();
    for (size_t i = 0; i < Size; ++i)
    {
        EXPECT_FLOAT_EQ(data[i], expected[i]) << "Mismatch at array index " << i;
    }
}

TEST(MathTests, Mat2_DefaultConstructor_IsIdentity)
{
    Mat2 m;
    // Column-major layout for 2x2 Identity: [1, 0, 0, 1]
    const float expected[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    ExpectMatrixEq(m, expected);
}

TEST(MathTests, Mat2_ScalarConstructor_FillsAllElements)
{
    Mat2 m(5.0f);
    const float expected[4] = {5.0f, 5.0f, 5.0f, 5.0f};
    ExpectMatrixEq(m, expected);
}

TEST(MathTests, Mat2_ExplicitConstructor_StoresColumnMajor)
{
    Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);

    const float expected[4] = {1.0f, 3.0f, 2.0f, 4.0f};
    ExpectMatrixEq(m, expected);
}

TEST(MathTests, Mat2_ScalarMultiplication)
{
    Mat2 m(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 result = m * 2.0f;

    const float expected[4] = {2.0f, 6.0f, 4.0f, 8.0f};
    ExpectMatrixEq(result, expected);
}

TEST(MathTests, Mat2_MatrixMultiplication)
{

    Mat2 a(1.0f, 2.0f, 3.0f, 4.0f);
    Mat2 b(2.0f, 0.0f, 1.0f, 2.0f);

    Mat2 result = a * b;

    const float expected[4] = {4.0f, 10.0f, 4.0f, 8.0f};
    ExpectMatrixEq(result, expected);
}

TEST(MathTests, Mat3_DefaultConstructor_IsIdentity)
{
    Mat3 m;
    const float expected[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f};
    ExpectMatrixEq(m, expected);
}

TEST(MathTests, Mat3_From_Mat2_Conversion)
{
    Mat2 m2(1.0f, 2.0f, 3.0f, 4.0f);
    Mat3 m3(m2);

    const float expected[9] = {
        1.0f, 3.0f, 0.0f,
        2.0f, 4.0f, 0.0f,
        0.0f, 0.0f, 1.0f};
    ExpectMatrixEq(m3, expected);
}

TEST(MathTests, Mat3_MatrixMultiplication)
{
    Mat3 a;
    Mat3 b(1.0f, 2.0f, 3.0f,
           4.0f, 5.0f, 6.0f,
           7.0f, 8.0f, 9.0f);

    Mat3 result = a * b;

    const float expected[9] = {
        1.0f, 4.0f, 7.0f,
        2.0f, 5.0f, 8.0f,
        3.0f, 6.0f, 9.0f};
    ExpectMatrixEq(result, expected);
}

TEST(MathTests, Mat4_DefaultConstructor_IsIdentity)
{
    Mat4 m;
    const float expected[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f};
    ExpectMatrixEq(m, expected);
}

TEST(MathTests, Mat4_From_Mat3_Conversion)
{

    Mat3 m3(1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f);
    Mat4 m4(m3);

    const float expected[16] = {
        1.0f, 4.0f, 7.0f, 0.0f,
        2.0f, 5.0f, 8.0f, 0.0f,
        3.0f, 6.0f, 9.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f};
    ExpectMatrixEq(m4, expected);
}

TEST(MathTests, Mat4_MatrixMultiplication_IdentityProperty)
{
    Mat4 identity;
    Mat4 m(1.0f, 2.0f, 3.0f, 4.0f,
           5.0f, 6.0f, 7.0f, 8.0f,
           9.0f, 10.0f, 11.0f, 12.0f,
           13.0f, 14.0f, 15.0f, 16.0f);

    Mat4 result = m * identity;

    const float expected[16] = {
        1.0f, 5.0f, 9.0f, 13.0f,
        2.0f, 6.0f, 10.0f, 14.0f,
        3.0f, 7.0f, 11.0f, 15.0f,
        4.0f, 8.0f, 12.0f, 16.0f};
    ExpectMatrixEq(result, expected);
}
TEST(MathTests, Mat2_VectorMultiplication)
{
    Mat2 m(1.0f, 2.0f,
           3.0f, 4.0f);

    Vec2 v(5.0f, 6.0f);

    Vec2 result = m * v;


    EXPECT_FLOAT_EQ(result.x, 17.0f);
    EXPECT_FLOAT_EQ(result.y, 39.0f);
}

TEST(MathTests, Mat2_MatrixMultiplication_IdentityProperty)
{
    Mat2 identity;
    Mat2 m(1.0f, 2.0f,
           3.0f, 4.0f);

    Mat2 result = m * identity;

    const float expected[4] = {
        1.0f, 3.0f,
        2.0f, 4.0f};

    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_FLOAT_EQ(result[i], expected[i]);
    }
}


TEST(MathTests, Mat3_VectorMultiplication)
{
    Mat3 m(1.0f, 2.0f, 3.0f,
           4.0f, 5.0f, 6.0f,
           7.0f, 8.0f, 9.0f);

    Vec3 v(2.0f, 3.0f, 4.0f);

    Vec3 result = m * v;

    EXPECT_FLOAT_EQ(result.x, 20.0f);
    EXPECT_FLOAT_EQ(result.y, 47.0f);
    EXPECT_FLOAT_EQ(result.z, 74.0f);
}

TEST(MathTests, Mat4_VectorMultiplication)
{
    Mat4 m(1.0f, 2.0f, 3.0f, 4.0f,
           5.0f, 6.0f, 7.0f, 8.0f,
           9.0f, 10.0f, 11.0f, 12.0f,
           13.0f, 14.0f, 15.0f, 16.0f);

    Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);

    Vec4 result = m * v;
    EXPECT_FLOAT_EQ(result.x, 30.0f);
    EXPECT_FLOAT_EQ(result.y, 70.0f);
    EXPECT_FLOAT_EQ(result.z, 110.0f);
    EXPECT_FLOAT_EQ(result.w, 150.0f);
}