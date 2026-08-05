#include <gtest/gtest.h>
#include "Fixture.hpp"

// static constexpr float WIDTH = 800, HEIGHT = 450;

TEST_F(FixtureTest, CircleIsBounded)
{
    sas::Transform2D t1;
    t1.position = {470, 50};

    AddCircle(t1, {});

    sas::Transform2D t2;
    t2.position = {-50, 50};
    AddCircle(t2, {});

    sas::Transform2D t3;
    t3.position = {1000, 50};

    AddCircle(t3, {});

    sas::Transform2D t4;
    t4.position = {-100, 50};

    AddCircle(t4, {});

    float dt = 1.f / 60.f;

    world->step(dt);

    EXPECT_LE(world->bodies[0].transform.position.y, HEIGHT);
    EXPECT_LE(world->bodies[1].transform.position.x, WIDTH);
    EXPECT_GE(world->bodies[2].transform.position.x, 0);
    EXPECT_GE(world->bodies[3].transform.position.y, 0);
}

TEST_F(FixtureTest, CircleStopsOnFloor)
{
    sas::Transform2D t;
    t.position = {400, 440};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 100};
    k.restituition = 0.f;

    const auto& handle = AddCircle(t, k);

    float dt = 1.0f / 60.0f;

    walls[0]->kinematics.restituition = 0;
    walls[1]->kinematics.restituition = 0;
    walls[2]->kinematics.restituition = 0;
    walls[3]->kinematics.restituition = 0;
    
    for (int i = 0; i < 30; ++i) 
    {
        world->step(dt);
    }
    float expectedY = HEIGHT - handle.get()->shape.radius; 
    
    EXPECT_NEAR(handle.get()->transform.position.y, expectedY, 1.f);
    EXPECT_NEAR(world->bodies[0].kinematics.velocity.y, 0.0f, 1.0f);
}

TEST_F(FixtureTest, CircleBouncesOnFloor)
{

    sas::Transform2D t;
    t.position = {400, 440};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 100};
    k.restituition = 0.5f;

    const auto& body = AddCircle(t, k);

    float dt = 1.f / 60.f;
    world->step(dt);

    EXPECT_LT(body.get()->kinematics.velocity.y, 0);

    world->step(dt);

    EXPECT_LE(body.get()->transform.position.y, HEIGHT - body.get()->shape.radius);
}

TEST_F(FixtureTest, EnergyLossOnHighDrop)
{
    sas::Transform2D t;
    t.position = {400, 50};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.5f;

    const auto& body = AddCircle(t, k);

    float dt = 1.f / 60.f;
    bool hasBounced = false;
    float peakHeightAfterBounce = 450.0f;

    for (int i = 0; i < 200; ++i)
    {
        float lastVelY = body.get()->kinematics.velocity.y;
        world->step(dt);
        float currentVelY = body.get()->kinematics.velocity.y;

        if (lastVelY > 0 && currentVelY < 0)
        {
            hasBounced = true;
        }

        if (hasBounced && currentVelY <= 0)
        {
            peakHeightAfterBounce = std::min(peakHeightAfterBounce, body.get()->transform.position.y);
        }
    }

    ASSERT_TRUE(hasBounced) << "The ball never hit the floor!";
    EXPECT_GT(peakHeightAfterBounce, 100.0f);
}

TEST_F(FixtureTest, CircleBouncesOnWall)
{
    sas::Transform2D t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.5f;

    AddCircle(t, k);

    float dt = 1.f / 60.f;
    float currentVelX;
    for (int i = 0; i < 200; ++i)
    {
        world->step(dt);
        currentVelX = world->bodies[0].kinematics.velocity.x;
    }

    EXPECT_LE(currentVelX, 0);
}

TEST_F(FixtureTest, CircleIsInnelastic)
{
    sas::Transform2D t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {0, 0};
    k.restituition = 0.0f;

    AddCircle(t, k);

    float dt = 1.f / 60.f;
    for (int i = 0; i < 20; ++i)
    {
        world->step(dt);
    }

    EXPECT_EQ(world->bodies[0].kinematics.velocity.x, 0);
}

TEST_F(FixtureTest, CircleIsPerfectEllastic)
{
    world->settings.gravity = 500.0f;
    world->settings.dragCoeff = 0.f;
    world->settings.groundFriction = 1.f;
    world->settings.wallFriction = 1.f;

    sas::Transform2D t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {400.f, 0};
    k.restituition = 1.0f;

    const auto& body = AddCircle(t, k);

    float dt = 1.f / 60.f;
    for (int i = 0; i < 10; ++i)
    {
        world->step(dt);
    }

    EXPECT_NEAR(std::abs(body.get()->kinematics.velocity.x), 400, 50);

}

TEST_F(FixtureTest, ResetDoesNotCrashOrLeak)
{
    sas::Transform2D t;
    t.position = {750, 225};

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.velocity = {400.f, 0};
    k.restituition = 1.0f;

    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);

    world->step(0.1f);

    world->clear();

    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);
    AddCircle(t, k);

    world->step(0.1f);

    EXPECT_NO_FATAL_FAILURE();

}