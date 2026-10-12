#include <gtest/gtest.h>
#include "Fixture.hpp"

TEST_F(FixtureTest2D, BoxCollide2D)
{
    sas::Transform2D t1;
    t1.position = {400, 400};

    sas::Transform2D t2;
    t2.position = {410, 400};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    
    world->step(1.f / 60.f);

    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
    ASSERT_EQ(world->contacts.size(), 1);
}

TEST_F(FixtureTest2D, BoxDontCollide2D)
{
    sas::Transform2D t1;
    t1.position = {400, 400};

    sas::Transform2D t2;
    t2.position = {450, 400};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest2D, BoxCollideAfterMoving2D)
{
    sas::Transform2D t1;
    t1.position = {310, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {100, 0};
    k1.inverseMass = 0.2f;

    sas::Kinematics2D k2;
    k2.velocity = {-100, 0};
    k2.inverseMass = 0.2f;


    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    bool bothColide = false;
    for(int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if(bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
            break;
        }
    }

    EXPECT_TRUE(bothColide) << "Failed to collide! Final distance: " 
                            << std::abs(bh1->transform.position.x - bh2->transform.position.x);
}


TEST_F(FixtureTest2D, BoxsDontCollideAfterMoving2D)
{
    sas::Transform2D t1;
    t1.position = {310, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {-10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    bool bothColide = false;

    for(int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if(bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
        }
    }

    EXPECT_FALSE(bothColide);
}


TEST_F(FixtureTest2D, BoxsWithoutCollisionsDontCollide2D)
{
    sas::Transform2D t1;
    t1.position = {310, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollisionOff();
    bh2.setCollisionOff();

    bool bothColide = false;

    for(int i = 0; i < 1500; ++i)
    {
        world->step(1.f / 60.f);
        if(bh1.isColliding() && bh2.isColliding())
        {
            bothColide = true;
        }
    }

    EXPECT_FALSE(bothColide);
}

TEST_F(FixtureTest2D, DifferentLayersDontCollideBoxes2D)
{
    sas::Transform2D t1;
    t1.position = {510, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer1, sas::Flags::Mask1);

    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest2D, LayerTwoCollidesBoxes2D)
{
    sas::Transform2D t1;
    t1.position = {510, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);

    world->step(1.f / 60.f);

    EXPECT_TRUE(bh1.isColliding() && bh2.isColliding());
}


TEST_F(FixtureTest2D, ScaledBoxCollide2D)
{
    sas::Transform2D t1;
    t1.position = {400, 400};

    sas::Transform2D t2;
    t2.position = {410, 400};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1->transform.scale = {2, 2};
    bh2->transform.scale = {2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();

    world->step(1.f / 60.f);

    ASSERT_EQ(world->contacts.size(), 1);
    EXPECT_TRUE(bh1.isColliding());
    EXPECT_TRUE(bh2.isColliding());
}

TEST_F(FixtureTest2D, ScaledBoxDontCollide2D)
{
    sas::Transform2D t1;
    t1.position = {200, 400};

    sas::Transform2D t2;
    t2.position = {450, 400};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1->transform.scale = {2, 2};
    bh2->transform.scale = {2, 2};

    bh1.setCollisionOn();
    bh2.setCollisionOn();
    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding());
    EXPECT_FALSE(bh2.isColliding());
}

TEST_F(FixtureTest2D, ScaledBoxCollideAfterMoving2D)
{
    sas::Transform2D t1;
    t1.position = {310, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {100, 0};
    k1.inverseMass = 0.2f;

    sas::Kinematics2D k2;
    k2.velocity = {-100, 0};
    k2.inverseMass = 0.2f;

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1->transform.scale = {2, 2};
    bh2->transform.scale = {2, 2};

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

TEST_F(FixtureTest2D, ScaledBoxsDontCollideAfterMoving2D)
{
    sas::Transform2D t1;
    t1.position = {310, 200};

    sas::Transform2D t2;
    t2.position = {610, 200};

    sas::Kinematics2D k1;
    k1.velocity = {-10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1->transform.scale = {2, 2};
    bh2->transform.scale = {2, 2};

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

TEST_F(FixtureTest2D, ScaledBoxsWithoutCollisionsDontCollide2D)
{
    sas::Transform2D t1;
    t1.position = {310, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1->transform.scale = {2, 2};
    bh2->transform.scale = {2, 2};

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

TEST_F(FixtureTest2D, ScaledBoxDifferentLayersDontCollide2D)
{
    sas::Transform2D t1;
    t1.position = {510, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1->transform.scale = {2, 2};
    bh2->transform.scale = {2, 2};

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer1, sas::Flags::Mask1);

    world->step(1.f / 60.f);

    EXPECT_FALSE(bh1.isColliding() && bh2.isColliding());
}

TEST_F(FixtureTest2D, ScaledBoxLayerTwoCollides2D)
{
    sas::Transform2D t1;
    t1.position = {510, 200};

    sas::Transform2D t2;
    t2.position = {510, 200};

    sas::Kinematics2D k1;
    k1.velocity = {10, 0};

    sas::Kinematics2D k2;
    k2.velocity = {-10, 0};

    sas::BodyHandle bh1 = AddBox(t1, k1);
    sas::BodyHandle bh2 = AddBox(t2, k2);

    bh1->transform.scale = {2, 2};
    bh2->transform.scale = {2, 2};

    bh1.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);
    bh2.setCollision(sas::Flags::Layer2, sas::Flags::Mask2);

    world->step(1.f / 60.f);

    EXPECT_TRUE(bh1.isColliding() && bh2.isColliding());
}
