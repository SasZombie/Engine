#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest3D, CircleCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {1, 0, 0};

    sas::Kinematics3D k2;
    k2.velocity = {-1, 0, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    world->step(1.f / 60.f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CircleDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {0, 0, 0};

    sas::Kinematics3D k1;
    k1.velocity = {0, 0, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 0, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CircleCollideAfterMoving3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    k1.inverseMass = 0.2f;
    k2.inverseMass = 0.2f;

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    bool bothColide = false;
    for (int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if (bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
            break;
        }
    }

    EXPECT_TRUE(bothColide) << "Failed to collide! Final distance: "
                            << std::abs(bh1->transform.position.x - bh2->transform.position.x);
}

TEST_F(FixtureTest3D, CirclesDontCollideAfterMoving3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, 1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, -1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    bool bothColide = false;

    for (int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if (bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
        }
    }

    EXPECT_FALSE(bothColide);
}

TEST_F(FixtureTest3D, CirclesWithoutCollisionsDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1.setCollisionOff();
    bh2.setCollisionOff();

    bool bothColide = false;

    for (int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if (bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
        }
    }

    EXPECT_FALSE(bothColide);
}

TEST_F(FixtureTest3D, CircleDifferentLayersDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer1, sas::Flags::Mask1);

    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest3D, CircleLayerTwoCollides3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);

    world->step(1.f / 60.f);

    EXPECT_TRUE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCircleCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    world->step(1.f / 60.f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCircleDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 8, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, 1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, -1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCircleCollideAfterMoving3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    k1.inverseMass = 0.2f;
    k2.inverseMass = 0.2f;

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    bool bothColide = false;
    for (int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if (bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
            break;
        }
    }

    EXPECT_TRUE(bothColide) << "Failed to collide! Final distance: "
                            << std::abs(bh1->transform.position.x - bh2->transform.position.x);
}

TEST_F(FixtureTest3D, ScaledCirclesDontCollideAfterMoving3D)
{
    walls.clear();
    world->clear();
    sas::Transform3D t1;
    t1.position = {5, 8, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, 1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, -1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    bool bothColide = false;

    for (int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if (bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
        }
    }

    EXPECT_FALSE(bothColide);
}

TEST_F(FixtureTest3D, ScaledCirclesWithoutCollisionsDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollisionOff();
    bh2.setCollisionOff();

    bool bothColide = false;

    for (int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if (bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
        }
    }

    EXPECT_FALSE(bothColide);
}

TEST_F(FixtureTest3D, ScaledCircleDifferentLayersDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer1, sas::Flags::Mask1);

    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCircleLayerTwoCollides3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddSphere(t1, k1);
    sas::BodyHandle bh2 = AddSphere(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);

    world->step(1.f / 60.f);

    EXPECT_TRUE(bh1.isColliding() && bh2.isColliding());
}
