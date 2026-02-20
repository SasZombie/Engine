#include <gtest/gtest.h>
#include "PhysicsWorld.hpp"

class FixtureTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Pointer so it resets each test
        world = std::make_unique<sas::PhysicsWorld>(sas::Rectangle{0, 0, 800, 450});
        world->settings.gravity = 500.0f;
    }

    void TearDown() override
    {
        world->Clear();
    }

    sas::BodyHandle AddCircle(sas::Transform trans, sas::Kinematics kin)
    {

        sas::BodyHandle bh = world->CreateBody(sas::Shape{sas::ShapeType::Circle, 10.f}, trans);
        kin.inverseMass = 0.2f;

        bh->kinematics = kin;

        return bh;
    }

    std::unique_ptr<sas::PhysicsWorld> world;

    const float WIDTH = 800;
    const float HEIGHT = 450;
};