#include <gtest/gtest.h>
#include <chrono>

#include "Fixture.hpp"

TEST_F(FixtureTest3D, Performance_Spere3D)
{
    constexpr float dt = 1.f / 60.f;
    constexpr double targetFrameBudgetMs = 16.66; 
    constexpr size_t batchSize = 20;

    size_t totalObjects = 0;

    while (true)
    {
        for (size_t i = 0; i < batchSize; ++i)
        {
            sas::Transform3D t1;
            float x = 50.f + static_cast<float>((totalObjects + i) % 20) * 35.f;
            float y = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            float z = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            t1.position = {x, y, z};
            
            AddSphere(t1, {});
        }
        totalObjects += batchSize;

        for (int w = 0; w < 3; ++w) {
            world->step(dt);
        }

        const auto start = std::chrono::steady_clock::now();
        constexpr int sampleFrames = 30;
        for (int f = 0; f < sampleFrames; ++f)
        {
            world->step(dt);
        }
        const auto end = std::chrono::steady_clock::now();

        const double totalUs = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        const double avgMs = (totalUs / 1000.0) / sampleFrames;

        if (avgMs >= targetFrameBudgetMs)
        {
            std::cout << "Max objects at 60 FPS: " << (totalObjects - batchSize) << "\n";
            break;
        }

        if (totalObjects >= 5000) break;
    }
}

TEST_F(FixtureTest3D, Performance_Box3D)
{
    constexpr float dt = 1.f / 60.f;
    constexpr double targetFrameBudgetMs = 16.66; 
    constexpr size_t batchSize = 20;

    size_t totalObjects = 0;

    while (true)
    {
        for (size_t i = 0; i < batchSize; ++i)
        {
            sas::Transform3D t1;
            float x = 50.f + static_cast<float>((totalObjects + i) % 20) * 35.f;
            float y = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            float z = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            t1.position = {x, y, z};
            
            AddBox(t1, {});
        }
        totalObjects += batchSize;

        for (int w = 0; w < 3; ++w) {
            world->step(dt);
        }

        const auto start = std::chrono::steady_clock::now();
        constexpr int sampleFrames = 30;
        for (int f = 0; f < sampleFrames; ++f)
        {
            world->step(dt);
        }
        const auto end = std::chrono::steady_clock::now();

        const double totalUs = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        const double avgMs = (totalUs / 1000.0) / sampleFrames;


        if (avgMs >= targetFrameBudgetMs)
        {
            std::cout << "Max objects at 60 FPS: " << (totalObjects - batchSize) << "\n";
            break;
        }

        if (totalObjects >= 5000) break;
    }
}

TEST_F(FixtureTest3D, Performance_CilD)
{
    constexpr float dt = 1.f / 60.f;
    constexpr double targetFrameBudgetMs = 16.66; 
    constexpr size_t batchSize = 20;

    size_t totalObjects = 0;

    while (true)
    {
        for (size_t i = 0; i < batchSize; ++i)
        {
            sas::Transform3D t1;
            float x = 50.f + static_cast<float>((totalObjects + i) % 20) * 35.f;
            float y = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            float z = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            t1.position = {x, y, z};
            
            AddCilinder(t1, {});
        }
        totalObjects += batchSize;

        for (int w = 0; w < 3; ++w) {
            world->step(dt);
        }

        const auto start = std::chrono::steady_clock::now();
        constexpr int sampleFrames = 30;
        for (int f = 0; f < sampleFrames; ++f)
        {
            world->step(dt);
        }
        const auto end = std::chrono::steady_clock::now();

        const double totalUs = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        const double avgMs = (totalUs / 1000.0) / sampleFrames;


        if (avgMs >= targetFrameBudgetMs)
        {
            std::cout << "Max objects at 60 FPS: " << (totalObjects - batchSize) << "\n";
            break;
        }

        if (totalObjects >= 5000) break;
    }
}

TEST_F(FixtureTest3D, Performance_Mixed3D)
{
    constexpr float dt = 1.f / 60.f;
    constexpr double targetFrameBudgetMs = 16.66; 
    constexpr size_t batchSize = 20;

    size_t totalObjects = 0;

    while (true)
    {
        for (size_t i = 0; i < batchSize; ++i)
        {
            sas::Transform3D t1; 
            size_t idx = totalObjects + i;
            
            // Distribute objects in a 3D grid (10x10xN)
            float x = 50.f + static_cast<float>(idx % 10) * 35.f;
            float y = 50.f + static_cast<float>((idx / 10) % 10) * 35.f;
            float z = 50.f + static_cast<float>(idx / 100) * 35.f;
            
            t1.position = {x, y, z};
            
            // Cycle evenly between Box, Sphere, and Cylinder
            if (idx % 3 == 0)
            {
                AddBox(t1, {});
            }
            else if (idx % 3 == 1)
            {
                AddSphere(t1, {});
            }
            else
            {
                AddCilinder(t1, {});
            }
        }
        totalObjects += batchSize;

        for (int w = 0; w < 3; ++w) {
            world->step(dt);
        }

        const auto start = std::chrono::steady_clock::now();
        constexpr int sampleFrames = 30;
        for (int f = 0; f < sampleFrames; ++f)
        {
            world->step(dt);
        }
        const auto end = std::chrono::steady_clock::now();

        const double totalUs = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
        const double avgMs = (totalUs / 1000.0) / sampleFrames;

        if (avgMs >= targetFrameBudgetMs)
        {
            std::cout << "Max objects at 60 FPS: " << (totalObjects - batchSize) << "\n";
            break;
        }

        if (totalObjects >= 5000) break;
    }
}

// 4820
// 4820