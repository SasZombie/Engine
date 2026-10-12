#include <gtest/gtest.h>
#include "PhysicsWorld.hpp"

class FixtureTest2D : public ::testing::Test
{
protected:
    void SetUp() override
    {
        sas::Kinematics2D bouncyKinematics;
        bouncyKinematics.restitution = 1.f;

        // Pointer so it resets each test
        world = std::make_unique<sas::PhysicsWorld<sas::Traits2D>>();
        world->settings.gravity = {0.f, 500.f, 0.f};
        world->settings.dragCoeff = 0.47f;

        walls.push_back(world->createBody(sas::Shape2D::MakeBox(WIDTH, 1), sas::Transform2D{{WIDTH / 2, 0}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
        walls.push_back(world->createBody(sas::Shape2D::MakeBox(WIDTH, 1), sas::Transform2D{{WIDTH / 2, HEIGHT}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
        walls.push_back(world->createBody(sas::Shape2D::MakeBox(1, HEIGHT), sas::Transform2D{{0, HEIGHT / 2}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
        walls.push_back(world->createBody(sas::Shape2D::MakeBox(1, HEIGHT), sas::Transform2D{{WIDTH, HEIGHT / 2}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
    }

    void TearDown() override
    {
        world->clear();
    }

    sas::BodyHandle<sas::Traits2D> AddCircle(sas::Transform2D trans, sas::Kinematics2D kin)
    {
        sas::BodyHandle bh = world->createBody(sas::Shape2D{sas::ShapeType2D::Circle, 10.f}, trans, kin);
        return bh;
    }

    sas::BodyHandle<sas::Traits2D> AddBox(sas::Transform2D trans, sas::Kinematics2D kin)
    {
        sas::BodyHandle bh = world->createBody(sas::Shape2D::MakeBox(20.f, 20.f), trans, kin);
        return bh;
    }

    std::vector<sas::BodyHandle<sas::Traits2D>> walls;

    std::unique_ptr<sas::PhysicsWorld<sas::Traits2D>> world;

    const float WIDTH = 800;
    const float HEIGHT = 450;
};

class FixtureTest3D : public ::testing::Test
{
protected:
    void SetUp() override
    {
        sas::Kinematics3D bouncyKinematics;
        bouncyKinematics.restitution = 1.f;

        // Pointer so it resets each test
        world = std::make_unique<sas::PhysicsWorld<sas::Traits3D>>();
        world->settings.gravity = {0.f, 0.f, 0.f};
        world->settings.dragCoeff = 0.47f;

        sas::Transform3D t;
        t.position = {0, 20, 0};

        sas::Kinematics3D k;
        k.pushForce = 10;

        sas::BodyHandle3D RoofBH = world->createBody(sas::Shape3D::MakeOrthotope(20, 0.1, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
        t.position = {0, 0, 0};
        sas::BodyHandle3D FloorBH = world->createBody(sas::Shape3D::MakeOrthotope(20, 0.1, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
        t.position = {10, 10, 0};
        sas::BodyHandle3D Wall1BH = world->createBody(sas::Shape3D::MakeOrthotope(0.1, 20, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
        t.position = {-10, 10, 0};
        sas::BodyHandle3D Wall2BH = world->createBody(sas::Shape3D::MakeOrthotope(0.1, 20, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
        t.position = {0, 10, 10};
        sas::BodyHandle3D Wall3BH = world->createBody(sas::Shape3D::MakeOrthotope(20, 20, 0.1), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
        t.position = {0, 10, -10};
        sas::BodyHandle3D Wall4BH = world->createBody(sas::Shape3D::MakeOrthotope(20, 20, 0.1), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);

        walls.push_back(RoofBH);
        walls.push_back(FloorBH);
        walls.push_back(Wall1BH);
        walls.push_back(Wall2BH);
        walls.push_back(Wall3BH);
        walls.push_back(Wall4BH);
    }

    void TearDown() override
    {
        world->clear();
    }

    sas::BodyHandle3D AddSphere(sas::Transform3D trans, sas::Kinematics3D kin)
    {
        sas::BodyHandle bh = world->createBody(sas::Shape3D{sas::ShapeType3D::Sphere, 2.f}, trans, kin);
        return bh;
    }

    sas::BodyHandle3D AddBox(sas::Transform3D trans, sas::Kinematics3D kin)
    {
        sas::BodyHandle bh = world->createBody(sas::Shape3D::MakeOrthotope(2.f, 2.f, 2.f), trans, kin);
        return bh;
    }

    sas::BodyHandle3D AddCilinder(sas::Transform3D trans, sas::Kinematics3D kin)
    {
        sas::BodyHandle bh = world->createBody(sas::Shape3D::MakeCilinder(2.f, 2.f), trans, kin);
        return bh;
    }

    std::vector<sas::BodyHandle<sas::Traits3D>> walls;

    std::unique_ptr<sas::PhysicsWorld<sas::Traits3D>> world;

    const float WIDTH = 800;
    const float HEIGHT = 450;
};