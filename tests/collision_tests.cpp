#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest, CircleCollide)
{
    AddCircle({0, 0}, {1, 1}, 0.f);
    AddCircle({0, 0}, {1, 1}, 0.f);

    world->addToCollisionPool(bodies[0]);
    world->addToCollisionPool(bodies[1]);
    world->Step(bodies, 0.01f);

    ASSERT_TRUE(bodies[0].isColliding);
    ASSERT_TRUE(bodies[1].isColliding);
}


TEST_F(FixtureTest, CirclesDontCollide)
{
    AddCircle({0, 0}, {1, 1}, 0.f);
    AddCircle({100, 20}, {1, 1}, 0.f);

    world->addToCollisionPool(bodies[0]);
    world->addToCollisionPool(bodies[1]);
    world->Step(bodies, 0.01f);

    ASSERT_FALSE(bodies[0].isColliding);
    ASSERT_FALSE(bodies[1].isColliding);
}

TEST_F(FixtureTest, CirclesColideAfterMoving)
{
    AddCircle({0, 0}, {100, 0}, 0.f);
    AddCircle({50, 0}, {-100, 0}, 0.f);

    world->addToCollisionPool(bodies[0]);
    world->addToCollisionPool(bodies[1]);

    bool bothColide = false;

    for(float i = 0; i < 10; i = i + 0.1)
    {
        world->Step(bodies, 0.01f);
        if(bodies[0].isColliding && bodies[1].isColliding)
        {
            bothColide = true;
        } 
    }

    ASSERT_TRUE(bothColide);
}


TEST_F(FixtureTest, CirclesDontColideAfterMoving)
{
    AddCircle({0, 0}, {-10, 0}, 0.f);
    AddCircle({50, 0}, {10, 0}, 0.f);

    world->addToCollisionPool(bodies[0]);
    world->addToCollisionPool(bodies[1]);

    bool bothColide = false;

    for(float i = 0; i < 10; i = i + 0.1)
    {
        world->Step(bodies, 0.01f);
        if(bodies[0].isColliding && bodies[1].isColliding)
        {
            bothColide = true;
        } 
    }

    ASSERT_FALSE(bothColide);;
}