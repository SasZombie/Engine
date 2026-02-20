#include <gtest/gtest.h>
#include "Fixture.hpp"

// static constexpr float WIDTH = 800, HEIGHT = 450;

TEST_F(FixtureTest, CircleIsBounded)
{
    sas::Transform t1;
    t1.position = {470, 50};

    AddCircle(t1, {});

    sas::Transform t2;
    t2.position = {-50, 50};
    AddCircle(t2, {});

    sas::Transform t3;
    t3.position = {1000, 50};

    AddCircle(t3, {});

    sas::Transform t4;
    t4.position = {-100, 50};

    AddCircle(t4, {});

    float dt = 0.1f;

    world->Step(dt);

    EXPECT_LE(world->bodies[0].transform.position.y, HEIGHT);
    EXPECT_LE(world->bodies[1].transform.position.x, WIDTH);
    EXPECT_GE(world->bodies[2].transform.position.x, 0);
    EXPECT_GE(world->bodies[3].transform.position.y, 0);
}

TEST_F(FixtureTest, CircleStopsOnFloor)
{
    sas::Transform t;
    t.position = {400, 440};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 100};
    k.restituition = 0.5f;

    AddCircle(t, k);

    float dt = 0.1f;
    world->Step(dt);

    EXPECT_LT(world->bodies[0].kinematics.velocity.y, 0);
    EXPECT_EQ(world->bodies[0].transform.position.y, HEIGHT - world->bodies[0].shape.radius);
}

TEST_F(FixtureTest, CircleBouncesOnFloor)
{

    sas::Transform t;
    t.position = {400, 440};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 100};
    k.restituition = 0.5f;

    AddCircle(t, k);

    float dt = 0.1f;
    world->Step(dt);

    EXPECT_LT(world->bodies[0].kinematics.velocity.y, 0);

    world->Step(dt);

    EXPECT_LE(world->bodies[0].transform.position.y, HEIGHT - world->bodies[0].shape.radius);
}

TEST_F(FixtureTest, EnergyLossOnHighDrop)
{
    sas::Transform t;
    t.position = {400, 50};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.5f;

    AddCircle(t, k);

    float dt = 0.01f;
    bool hasBounced = false;
    float peakHeightAfterBounce = 450.0f;

    for (int i = 0; i < 200; ++i)
    {
        float lastVelY = world->bodies[0].kinematics.velocity.y;
        world->Step(dt);
        float currentVelY = world->bodies[0].kinematics.velocity.y;

        if (lastVelY > 0 && currentVelY < 0)
        {
            hasBounced = true;
        }

        if (hasBounced && currentVelY <= 0)
        {
            peakHeightAfterBounce = std::min(peakHeightAfterBounce, world->bodies[0].transform.position.y);
        }
    }

    ASSERT_TRUE(hasBounced) << "The ball never hit the floor!";
    EXPECT_GT(peakHeightAfterBounce, 100.0f);
}

TEST_F(FixtureTest, CircleBouncesOnWall)
{
    sas::Transform t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.5f;

    AddCircle(t, k);

    float dt = 0.01f;
    float currentVelX;
    for (int i = 0; i < 200; ++i)
    {
        world->Step(dt);
        currentVelX = world->bodies[0].kinematics.velocity.x;
    }

    EXPECT_LE(currentVelX, 0);
}

TEST_F(FixtureTest, CircleIsInnelastic)
{
    sas::Transform t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.0f;

    AddCircle(t, k);

    float dt = 0.01f;
    for (int i = 0; i < 20; ++i)
    {
        world->Step(dt);
    }

    EXPECT_EQ(world->bodies[0].kinematics.velocity.x, 0);
}

TEST_F(FixtureTest, CircleIsPerfectEllastic)
{
    world->settings.gravity = 500.0f;
    world->settings.dragCoeff = 0.f;
    world->settings.groundFriction = 1.f;
    world->settings.wallFriction = 1.f;

    sas::Transform t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {400.f, 0};
    k.restituition = 1.0f;

    AddCircle(t, k);

    float dt = 0.01f;
    for (int i = 0; i < 10; ++i)
    {
        world->Step(dt);
    }

    EXPECT_NEAR(std::abs(world->bodies[0].kinematics.velocity.x), 400, 50);

}