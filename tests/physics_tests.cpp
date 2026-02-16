#include <gtest/gtest.h>
#include "PhysicsWorld.hpp"

static constexpr float WIDTH = 800, HEIGHT = 450;

TEST(PhysicsTests, CircleIsBounded)
{
    sas::PhysicsWorld world({0, 0, 800, 450});
    world.settings.gravity = 500.0f;
    std::vector<sas::Body> circles;

    sas::Transform t1;
    t1.position = {470, 50};

    sas::Kinematics k1;
    k1.inverseMass = 1.0f;
    k1.velocity = {0, 0};
    k1.restituition = 0.5f;
    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t1, &k1);

    sas::Transform t2;
    t1.position = {-50, 50};

    sas::Kinematics k2;
    k1.inverseMass = 1.0f;
    k1.velocity = {0, 0};
    k1.restituition = 0.5f;

    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t2, &k2);

    sas::Transform t3;
    t1.position = {1000, 50};

    sas::Kinematics k3;
    k1.inverseMass = 1.0f;
    k1.velocity = {0, 0};
    k1.restituition = 0.5f;
    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t3, &k3);


    float dt = 0.01f;
    world.Step(circles, dt);

    EXPECT_LE(circles[0].transform.position.y, HEIGHT);
    EXPECT_LE(circles[1].transform.position.x, WIDTH);
    EXPECT_GE(circles[2].transform.position.x, 0);
}

TEST(PhysicsTests, CircleStopsOnFloor)
{
    sas::PhysicsWorld world({0, 0, WIDTH, HEIGHT});
    world.settings.gravity = 500.0f;
    std::vector<sas::Body> circles;

    sas::Transform t;
    t.position = {400, 440};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 100};
    k.restituition = 0.5f;

    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t, &k);

    float dt = 0.1f;
    world.Step(circles, dt);

    EXPECT_LT(circles[0].kinematics->velocity.y, 0);
    EXPECT_EQ(circles[0].transform.position.y, HEIGHT - circles[0].shape.radius);
}

TEST(PhysicsTests, CircleBouncesOnFloor)
{
    sas::PhysicsWorld world({0, 0, WIDTH, HEIGHT});
    world.settings.gravity = 500.0f;
    std::vector<sas::Body> circles;

    sas::Transform t;
    t.position = {400, 440};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 100};
    k.restituition = 0.5f;

    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t, &k);

    float dt = 0.1f;
    world.Step(circles, dt);

    EXPECT_LT(circles[0].kinematics->velocity.y, 0);

    world.Step(circles, dt);

    EXPECT_LE(circles[0].transform.position.y, HEIGHT - circles[0].shape.radius);
}

TEST(PhysicsTests, EnergyLossOnHighDrop)
{
    sas::PhysicsWorld world({0, 0, 800, 450});
    world.settings.gravity = 500.0f;

    sas::Transform t;
    t.position = {400, 50};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.5f;

    std::vector<sas::Body> circles;
    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t, &k);

    float dt = 0.01f;
    bool hasBounced = false;
    float peakHeightAfterBounce = 450.0f;

    for (int i = 0; i < 200; ++i)
    {
        float lastVelY = circles[0].kinematics->velocity.y;
        world.Step(circles, dt);
        float currentVelY = circles[0].kinematics->velocity.y;

        if (lastVelY > 0 && currentVelY < 0)
        {
            hasBounced = true;
        }

        if (hasBounced && currentVelY <= 0)
        {
            peakHeightAfterBounce = std::min(peakHeightAfterBounce, circles[0].transform.position.y);
        }
    }

    ASSERT_TRUE(hasBounced) << "The ball never hit the floor!";
    EXPECT_GT(peakHeightAfterBounce, 100.0f);
}

TEST(PhysicsTests, CircleBouncesOnWall)
{
    sas::PhysicsWorld world({0, 0, 800, 450});
    world.settings.gravity = 500.0f;

    sas::Transform t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.5f;

    std::vector<sas::Body> circles;
    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t, &k);

    float dt = 0.01f;
    float currentVelX;
    for (int i = 0; i < 200; ++i)
    {
        world.Step(circles, dt);
        currentVelX = circles[0].kinematics->velocity.x;
    }

    EXPECT_LE(currentVelX, 0);
}

TEST(PhysicsTests, CircleIsInnelastic)
{
    sas::PhysicsWorld world({0, 0, 800, 450});
    world.settings.gravity = 500.0f;

    sas::Transform t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.0f;

    std::vector<sas::Body> circles;
    circles.emplace_back(sas::Shape{sas::ShapeType::Circle, 10.f}, t, &k);

    float dt = 0.01f;
    for (int i = 0; i < 20; ++i)
    {
        world.Step(circles, dt);
    }

    EXPECT_EQ(circles[0].kinematics->velocity.x, 0);
}