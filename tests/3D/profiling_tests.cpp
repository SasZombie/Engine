#include <gtest/gtest.h>
#include <chrono>

#include "Fixture.hpp"

TEST_F(FixtureTest3D, Profile_CollisionSolverHotspot)
{
    constexpr float dt = 1.f / 60.f;
    constexpr size_t gridDim = 10; 
    constexpr float spacing = 2.05f;

    for (size_t x = 0; x < gridDim; ++x)
    {
        for (size_t y = 0; y < gridDim; ++y)
        {
            for (size_t z = 0; z < gridDim; ++z)
            {
                sas::Transform3D t;
                t.position = { x * spacing, 5.0f + y * spacing, z * spacing };
                
                size_t type = (x + y + z) % 3;
                if (type == 0)      AddBox(t, {});
                else if (type == 1) AddSphere(t, {});
                else                AddCilinder(t, {});
            }
        }
    }

    addWalls();

    for (int i = 0; i < 120; ++i)
    {
        world->step(dt);
    }

    constexpr int profileFrames = 1000;
    
    const auto start = std::chrono::steady_clock::now();
    for (int f = 0; f < profileFrames; ++f)
    {
        world->step(dt);
    }
    const auto end = std::chrono::steady_clock::now();

    const double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "Avg frame time: " << (elapsedMs / profileFrames) << " ms\n";
}