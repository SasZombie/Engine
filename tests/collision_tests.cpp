#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest, CircleCollide)
{
    sas::Transform t1;
    t1.position = {400, 400};

    sas::Transform t2;
    t2.position = {410, 400};

    sas::Kinematics k1;
    k1.velocity = {10, 0};

    sas::Kinematics k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddCircle(t1, k1);
    sas::BodyHandle bh2 = AddCircle(t2, k2);

    bh1.SetCollisionOn();
    bh2.SetCollisionOn();
    world->Step(0.01f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.IsColliding());
    EXPECT_TRUE(bh2.IsColliding());
}

TEST_F(FixtureTest, CircleDontCollide)
{
    sas::Transform t1;
    t1.position = {400, 400};

    sas::Transform t2;
    t2.position = {450, 400};

    sas::Kinematics k1;
    k1.velocity = {10, 0};

    sas::Kinematics k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddCircle(t1, k1);
    sas::BodyHandle bh2 = AddCircle(t2, k2);

    bh1.SetCollisionOn();
    bh2.SetCollisionOn();
    world->Step(0.01f);

    EXPECT_FALSE(bh1.IsColliding());
    EXPECT_FALSE(bh2.IsColliding());
}

TEST_F(FixtureTest, CircleCollideAfterMoving)
{
    sas::Transform t1;
    t1.position = {310, 200};

    sas::Transform t2;
    t2.position = {510, 200};

    sas::Kinematics k1;
    k1.velocity = {100, 0};

    sas::Kinematics k2;
    k2.velocity = {-100, 0};

    sas::BodyHandle bh1 = AddCircle(t1, k1);
    sas::BodyHandle bh2 = AddCircle(t2, k2);

    bh1.SetCollisionOn();
    bh2.SetCollisionOn();

    bool bothColide = false;
    for(int i = 0; i < 1500; ++i)
    {
        world->Step(0.016f);
        if(bh1.IsColliding() && bh2.IsColliding())
        {
            bothColide = true;
            break;
        }
    }

    EXPECT_TRUE(bothColide) << "Failed to collide! Final distance: " 
                            << std::abs(bh1->transform.position.x - bh2->transform.position.x);
}


TEST_F(FixtureTest, CirclesDontCollideAfterMoving)
{
    sas::Transform t1;
    t1.position = {310, 200};

    sas::Transform t2;
    t2.position = {510, 200};

    sas::Kinematics k1;
    k1.velocity = {-10, 0};

    sas::Kinematics k2;
    k2.velocity = {10, 0};

    sas::BodyHandle bh1 = AddCircle(t1, k1);
    sas::BodyHandle bh2 = AddCircle(t2, k2);

    bh1.SetCollisionOn();
    bh2.SetCollisionOn();

    bool bothColide = false;

    for(int i = 0; i < 1500; ++i)
    {
        world->Step(1.f);
        if(bh1.IsColliding() && bh2.IsColliding())
        {
            bothColide = true;
        }
    }

    EXPECT_FALSE(bothColide);
}
