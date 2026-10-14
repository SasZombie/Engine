#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest3D, CilCil3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {1, 0, 0};

    sas::Kinematics3D k2;
    k2.velocity = {-1, 0, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    world->step(1.f / 60.f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CilDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {0, 0, 0};

    sas::Kinematics3D k1;
    k1.velocity = {0, 0, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 0, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CilCollideAfterMoving3D)
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

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

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

TEST_F(FixtureTest3D, CilsDontCollideAfterMoving3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, 1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, -1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

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

TEST_F(FixtureTest3D, CilsWithoutCollisionsDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

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

TEST_F(FixtureTest3D, CilDifferentLayersDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer1, sas::Flags::Mask1);

    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest3D, CilLayerTwoCollides3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);

    world->step(1.f / 60.f);

    EXPECT_TRUE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCilCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 1, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    world->step(1.f / 60.f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCilDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 8, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, 1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, -1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCilCollideAfterMoving3D)
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

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

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

TEST_F(FixtureTest3D, ScaledCilsDontCollideAfterMoving3D)
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

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

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

TEST_F(FixtureTest3D, ScaledCilsWithoutCollisionsDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 0, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

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

TEST_F(FixtureTest3D, ScaledCilDifferentLayersDontCollide3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer1, sas::Flags::Mask1);

    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest3D, ScaledCilLayerTwoCollides3D)
{
    sas::Transform3D t1;
    t1.position = {5, 5, 5};

    sas::Transform3D t2;
    t2.position = {5, 5, 5};

    sas::Kinematics3D k1;
    k1.velocity = {0, -1, 0};

    sas::Kinematics3D k2;
    k2.velocity = {0, 1, 0};

    sas::BodyHandle bh1 = AddCilinder(t1, k1);
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1->transform.scale = {2, 2, 2};
    bh2->transform.scale = {2, 2, 2};

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);

    world->step(1.f / 60.f);

    EXPECT_TRUE(bh1.isColliding() && bh2.isColliding());
}
