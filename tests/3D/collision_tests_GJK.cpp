#include <gtest/gtest.h>
#include "Fixture.hpp"

[[nodiscard]] static bool GJK(const sas::Body3D &obj, const sas::Body3D &other) noexcept
{
    using namespace sas;
    // 1 direction vector
    math::Vec3 d = obj.transform.position - other.transform.position;

    if (d == math::Vec3{0})
    {
        d = math::Vec3{1, 0, 0};
    }

    // 2 find Sc(d) = Sa(d) - Sb(-d)
    // Sa(d) furthest point from d
    // Sb(d) furthest point from d in opp direction
    // Both have analytical formulas ---> No vertex loops needed
    // If more shpaes are added, this should be a function pointer instead of this

    auto signOf = [](float f) noexcept
    {
        if (f < 0)
        {
            return -1;
        }

        return 1;
    };

    auto computerSD = [signOf](const Body3D &body, const math::Vec3 &objD) noexcept -> math::Vec3
    {
        switch (body.shape.type)
        {
        case ShapeType3D::Orthotope:
        {
            const auto &scaledHalfsObj = body.shape.halfSize * body.transform.scale;
            return {
                signOf(objD.x) * scaledHalfsObj.x,
                signOf(objD.y) * scaledHalfsObj.y,
                signOf(objD.z) * scaledHalfsObj.z,
            };
        }
        case ShapeType3D::Cilinder:
        {
            math::Vec2 scaledCil;
            // X and Z's scale must be equal, otherwise this is not a cylinder
            scaledCil.x = body.shape.radHeight.x * body.transform.scale.x;
            scaledCil.y = body.shape.radHeight.y * body.transform.scale.y;

            const float y = signOf(objD.y) * scaledCil.y;
            const float lenxz = std::sqrt(objD.x * objD.x + objD.z * objD.z);
            if (lenxz > 0)
            {
                return {
                    scaledCil.x * (objD.x / lenxz),
                    y,
                    scaledCil.x * (objD.z / lenxz)};
            }

            return {0, y, 0};
        }
        default:
            BaseLogger::error(std::string("Passed to GJK a shape that is not known") + enumToString(body.shape.type));
            std::unreachable();
        }
    };

    const auto supportD = [&obj, &other, computerSD, signOf](const math::Vec3 &currentD) noexcept
    {
        const auto &localObjD = obj.transform.quaterion.conj().norm() * currentD;
        const auto &localOtherD = other.transform.quaterion.conj().norm() * currentD;
        const math::Vec3 Sda = obj.transform.quaterion.norm() * computerSD(obj, localObjD) + obj.transform.position;
        const math::Vec3 Sdb = other.transform.quaterion.norm() * computerSD(other, -localOtherD) + other.transform.position;
        const auto &Scd = Sda - Sdb;

        return Scd;
    };

    math::Vec3 simplex[4];
    size_t size = 1;

    const auto doSimplex = [&]()
    {
        switch (size)
        {
        case 2:
        {
            const auto &ab = simplex[0] - simplex[1];
            const auto &ao = -simplex[1];

            auto n = ab * ao;

            if (n == math::Vec3{})
            {
                n = {1, 0, 0};
            }
            const auto &perp = n * ab;

            d = perp;

            return false;
        }

        case 3:
        {
            const auto &ab = simplex[1] - simplex[2];
            const auto &ac = simplex[0] - simplex[2];
            const auto &ao = -simplex[2];

            const auto &normal = ab * ac;
            auto perp = ab * normal;

            // AB
            if (math::dotProduct(perp, ao) > 0)
            {
                // B --> C
                simplex[0] = simplex[1];
                // A --> B
                simplex[1] = simplex[2];
                --size;
                d = perp;
                return false;
            }

            perp = ac * normal;
            // AC
            if (math::dotProduct(perp, ao) > 0)
            {
                simplex[1] = simplex[2];
                --size;
                d = perp;
                return false;
            }

            if (math::dotProduct(normal, ao) > 0)
            {
                d = normal;
            }
            else
            {
                std::swap(simplex[0], simplex[1]);
                d = -normal;
            }
            return false;
        }

        case 4:
        {
            using namespace math;
            const auto &A = simplex[3];
            const auto &B = simplex[2];
            const auto &C = simplex[1];
            const auto &D = simplex[0];

            const auto &ao = -A;
            const auto &ab = B - A;
            const auto &ac = C - A;
            const auto &ad = D - A;

            auto nABC = ab * ac;
            if (dotProduct(nABC, ad) > 0)
            {
                nABC = -nABC;
            }

            auto nACD = ac * ad;
            if (dotProduct(nACD, ab) > 0)
            {
                nACD = -nACD;
            }

            auto nADB = ad * ab;
            if (dotProduct(nADB, ac) > 0)
            {
                nADB = -nADB;
            }

            if (dotProduct(nABC, ao) > 0)
            {
                simplex[0] = C;
                simplex[1] = B;
                simplex[2] = A;
                --size;
                d = nABC;
                return false;
            }

            if (dotProduct(nACD, ao) > 0)
            {
                simplex[0] = D;
                simplex[1] = C;
                simplex[2] = A;
                --size;
                d = nACD;
                return false;
            }

            if (dotProduct(nADB, ao) > 0)
            {
                simplex[0] = B;
                simplex[1] = D;
                simplex[2] = A;

                --size;
                d = nADB;
                return false;
            }

            return true;
        }

        default:
            BaseLogger::error("Do simplex has a weird value: " + std::to_string(size));
            std::unreachable();
            return false;
        }
    };

    simplex[0] = supportD(d);
    d = -simplex[0];

    constexpr size_t maxIterrations = 30;
    size_t counter = 0;
    while (counter < maxIterrations)
    {
        const auto &p = supportD(d);
        if (math::dotProduct(p, d) <= 0)
        {
            return false;
        }

        simplex[size] = p;
        ++size;
        if (doSimplex())
        {
            return true;
        }
        ++counter;
    }

    return false;
};

TEST_F(FixtureTest3D, BoxCilinder)
{
    sas::Transform3D t1;
    t1.position = {2, 0, 0};

    sas::Transform3D t2;
    t2.position = {2, 0, 0};

    sas::Kinematics3D k1;
    k1.velocity = {0, 0, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 0, 0};



    sas::BodyHandle bh1 = AddBox(t1, k1);
    
    bh1->shape.halfSize = {1, 1, 1};

    sas::BodyHandle bh2 = AddCilinder(t2, k2);
    // bh2->shape.radHeight = {0.4, 1};
    // bh2->transform.quaterion = {0, 0, 0.7071068, 0.7071068};

    EXPECT_TRUE(GJK(*bh1.get(), *bh2.get()));
}