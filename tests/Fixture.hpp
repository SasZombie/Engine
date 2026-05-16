#include <gtest/gtest.h>
#include "PhysicsWorld.hpp"

class FixtureTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        sas::Kinematics bouncyKinematics;
        bouncyKinematics.restituition = 1.f;
        
        // Pointer so it resets each test
        world = std::make_unique<sas::PhysicsWorld>();
        world->settings.gravity = 500.0f;
        world->settings.dragCoeff = 0.47f;

        walls.push_back(world->createBody(sas::Shape::MakeBox(WIDTH, 1), sas::Transform{{WIDTH / 2, 0}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
        walls.push_back(world->createBody(sas::Shape::MakeBox(WIDTH, 1), sas::Transform{{WIDTH / 2, HEIGHT}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
        walls.push_back(world->createBody(sas::Shape::MakeBox(1, HEIGHT), sas::Transform{{0, HEIGHT / 2}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
        walls.push_back(world->createBody(sas::Shape::MakeBox(1, HEIGHT), sas::Transform{{WIDTH, HEIGHT / 2}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic));
    }

    void TearDown() override
    {
        world->clear();
    }

    sas::BodyHandle AddCircle(sas::Transform trans, sas::Kinematics kin)
    {
        sas::BodyHandle bh = world->createBody(sas::Shape{sas::ShapeType::Circle, 10.f}, trans, kin);
        return bh;
    }

    sas::BodyHandle AddBox(sas::Transform trans, sas::Kinematics kin)
    {
        sas::BodyHandle bh = world->createBody(sas::Shape::MakeBox(20.f, 20.f), trans, kin);
        return bh;
    }

    std::vector<sas::BodyHandle> walls;

    std::unique_ptr<sas::PhysicsWorld> world;

    const float WIDTH = 800;
    const float HEIGHT = 450;
};