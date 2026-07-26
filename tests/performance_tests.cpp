#include <gtest/gtest.h>
#include <chrono>

#include "Fixture.hpp"

TEST_F(FixtureTest, Performance_Circle)
{
    constexpr float dt = 1.f / 60.f;
    constexpr double targetFrameBudgetMs = 16.66; 
    constexpr size_t batchSize = 20;

    size_t totalObjects = 0;

    while (true)
    {
        for (size_t i = 0; i < batchSize; ++i)
        {
            sas::Transform t1;
            float x = 50.f + static_cast<float>((totalObjects + i) % 20) * 35.f;
            float y = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            t1.position = {x, y};
            
            AddCircle(t1, {});
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

TEST_F(FixtureTest, Performance_Box)
{
    constexpr float dt = 1.f / 60.f;
    constexpr double targetFrameBudgetMs = 16.66; 
    constexpr size_t batchSize = 20;

    size_t totalObjects = 0;

    while (true)
    {
        for (size_t i = 0; i < batchSize; ++i)
        {
            sas::Transform t1;
            float x = 50.f + static_cast<float>((totalObjects + i) % 20) * 35.f;
            float y = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            t1.position = {x, y};
            
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

TEST_F(FixtureTest, Performance_Mixed)
{
    constexpr float dt = 1.f / 60.f;
    constexpr double targetFrameBudgetMs = 16.66; 
    constexpr size_t batchSize = 20;

    size_t totalObjects = 0;

    while (true)
    {
        for (size_t i = 0; i < batchSize; ++i)
        {
            sas::Transform t1;
            float x = 50.f + static_cast<float>((totalObjects + i) % 20) * 35.f;
            float y = 50.f + static_cast<float>((totalObjects + i) / 20) * 35.f;
            t1.position = {x, y};
            
            if(i % 2 == 0)
            {
                AddBox(t1, {});
            }else
            {
                AddCircle(t1, {});
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

// 4820
// 4820