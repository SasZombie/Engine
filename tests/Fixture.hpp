#include <gtest/gtest.h>
#include "PhysicsWorld.hpp"

class FixtureTest : public ::testing::Test {
protected:

    void SetUp() override {
        world = std::make_unique<sas::PhysicsWorld>(sas::Rectangle{0, 0, 800, 450});
        world->settings.gravity = 500.0f;
    }

    void TearDown() override {
        bodies.clear();
    }

    void AddCircle(sas::math::Vec2 pos, sas::math::Vec2 vel, float restitution = 1.0f) {
        sas::Transform t;
        t.position = pos;

        sas::Kinematics k; 
        k.velocity = vel;
        k.restituition = restitution;
        k.inverseMass = 1.0f;
        
        bodies.emplace_back(t, k, sas::Shape{sas::ShapeType::Circle, 10.f}, bodies.size());
    }

    std::unique_ptr<sas::PhysicsWorld> world;
    std::vector<sas::Body> bodies;

    const float WIDTH = 800;
    const float HEIGHT = 450;
};