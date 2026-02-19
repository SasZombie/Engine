#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest, CircleCollide)
{
    sas::Transform t;
    t.position = {0, 0};

    sas::Kinematics k;
    k.velocity = {1, 1};

    AddCircle(t, k);
    AddCircle(t, k);

    world->addToCollisionPool(world->bodies[0]);
    world->addToCollisionPool(world->bodies[1]);
    world->Step(0.01f);

    ASSERT_TRUE(world->bodies[0].isColliding);
    ASSERT_TRUE(world->bodies[1].isColliding);
}

TEST_F(FixtureTest, CirclesDontCollide)
{
    sas::Transform t;
    t.position = {0, 0};

    sas::Kinematics k;
    k.velocity = {1, 1};

    AddCircle({{0, 0}, {0, 0}, {0, 0}}, k);
    AddCircle({{100, 20}, {0, 0}, {0, 0}}, k);

    world->addToCollisionPool(world->bodies[0]);
    world->addToCollisionPool(world->bodies[1]);
    world->Step(0.01f);

    ASSERT_FALSE(world->bodies[0].isColliding);
    ASSERT_FALSE(world->bodies[1].isColliding);
}

TEST_F(FixtureTest, CirclesColideAfterMoving)
{
    sas::Kinematics k;
    k.velocity = {1, 1};

    sas::Kinematics k2;
    k2.velocity = {-100, 0};

    AddCircle({{0, 0}, {0, 0}, {0, 0}}, k);
    AddCircle({{50, 0}, {0, 0}, {0, 0}}, k2);

    world->addToCollisionPool(world->bodies[0]);
    world->addToCollisionPool(world->bodies[1]);

    bool bothColide = false;

    for (float i = 0; i < 10; i = i + 0.1)
    {
        world->Step(0.01f);
        if (world->bodies[0].isColliding && world->bodies[1].isColliding)
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

    AddCircle({{0, 0}, {0, 0}, {0, 0}}, k);
    AddCircle({{50, 0}, {0, 0}, {0, 0}}, k2);

    world->addToCollisionPool(world->bodies[0]);
    world->addToCollisionPool(world->bodies[1]);

    bool bothColide = false;

    for (float i = 0; i < 10; i = i + 0.1)
    {
        world->Step(0.01f);
        if (world->bodies[0].isColliding && world->bodies[1].isColliding)
        {
            bothColide = true;
        }
    }

    ASSERT_FALSE(bothColide);
}