#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest, CircleCollide)
{
    sas::Transform t;
    t.position = {0, 0};

    sas::Kinematics k;
    k.velocity = {1, 1};

    const auto& bh1 = AddCircle(t, k);
    const auto& bh2 = AddCircle(t, k);

    world->AddToCollisionPool(world->bodies[0]);
    world->AddToCollisionPool(world->bodies[1]);
    world->Step(0.01f);

    ASSERT_TRUE(bh1.IsColliding());
    ASSERT_TRUE(bh2.IsColliding());
}

TEST_F(FixtureTest, CirclesDontCollide)
{
    sas::Transform t;
    t.position = {0, 0};

    sas::Kinematics k;
    k.velocity = {1, 1};

    const auto& bh1 = AddCircle({{0, 0}, {0, 0}, {0, 0}}, k);
    const auto& bh2 = AddCircle({{100, 20}, {0, 0}, {0, 0}}, k);

    world->AddToCollisionPool(world->bodies[0]);
    world->AddToCollisionPool(world->bodies[1]);
    world->Step(0.01f);


    ASSERT_FALSE(bh1.IsColliding());
    ASSERT_FALSE(bh2.IsColliding());
}

TEST_F(FixtureTest, CirclesColideAfterMoving)
{
    sas::Kinematics k;
    k.velocity = {1, 1};

    sas::Kinematics k2;
    k2.velocity = {-100, 0};

    const auto& bh1 = AddCircle({{0, 0}, {0, 0}, {0, 0}}, k);
    const auto& bh2 = AddCircle({{50, 0}, {0, 0}, {0, 0}}, k2);

    world->AddToCollisionPool(world->bodies[0]);
    world->AddToCollisionPool(world->bodies[1]);

    bool bothColide = false;

    for (float i = 0; i < 10; i = i + 0.1)
    {
        world->Step(0.01f);
        if (bh1.IsColliding() && bh2.IsColliding())
        {
            bothColide = true;
        }
    }

    ASSERT_TRUE(bothColide);
}

TEST_F(FixtureTest, CirclesDontColideAfterMoving)
{
    sas::Kinematics k;
    k.velocity = {-10, 0};

    sas::Kinematics k2;
    k2.velocity = {10, 0};

    const auto& bh1 = AddCircle({{0, 0}, {0, 0}, {0, 0}}, k);
    const auto& bh2 = AddCircle({{50, 0}, {0, 0}, {0, 0}}, k2);

    world->AddToCollisionPool(world->bodies[0]);
    world->AddToCollisionPool(world->bodies[1]);

    bool bothColide = false;

    for (float i = 0; i < 10; i = i + 0.1)
    {
        world->Step(0.01f);
        if (bh1.IsColliding() && bh2.IsColliding())
        {
            bothColide = true;
        }
    }

    ASSERT_FALSE(bothColide);
}