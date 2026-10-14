#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest3D, CircleBoxCollide3D)
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
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    world->step(1.f / 60.f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CircleBoxDontCollide3D)
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
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CircleBoxCollideAfterMoving3D)
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
    sas::BodyHandle bh2 = AddBox(t2, k2);

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

TEST_F(FixtureTest3D, CircleBoxsDontCollideAfterMoving3D)
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
    sas::BodyHandle bh2 = AddBox(t2, k2);

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

TEST_F(FixtureTest3D, CircleCilCollide3D)
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
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    world->step(1.f / 60.f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CircleCilDontCollide3D)
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
    sas::BodyHandle bh2 = AddCilinder(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest3D, CircleCilCollideAfterMoving3D)
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

TEST_F(FixtureTest3D, CircleCilDontCollideAfterMoving3D)
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