#include <gtest/gtest.h>
#include "PhysicsWorld.hpp"

static constexpr float WIDTH = 800, HEIGHT = 450;

TEST(CollisionTests, CircleCollide)
{
    sas::PhysicsWorld world({0, 0, 800, 450});
    world.settings.gravity = 500.0f;
    std::vector<sas::Body> circles;

    sas::Transform t1;
    t1.position = {470, 50};

    sas::Kinematics k1;
    k1.inverseMass = 1.0f;
    k1.velocity = {0, 0};
    k1.restituition = 0.5f;
    circles.emplace_back(0, sas::Shape{sas::ShapeType::Circle, 10.f}, t1, k1);

    sas::Transform t2;
    t1.position = {-50, 50};

    sas::Kinematics k2;
    k1.inverseMass = 1.0f;
    k1.velocity = {0, 0};
    k1.restituition = 0.5f;

    circles.emplace_back(1, sas::Shape{sas::ShapeType::Circle, 10.f}, t2, k2);

    sas::Transform t3;
    t1.position = {1000, 50};

    sas::Kinematics k3;
    k1.inverseMass = 1.0f;
    k1.velocity = {0, 0};
    k1.restituition = 0.5f;
    circles.emplace_back(2, sas::Shape{sas::ShapeType::Circle, 10.f}, t3, k3);

    float dt = 0.01f;
    world.Step(circles, dt);

    EXPECT_LE(circles[0].transform.position.y, HEIGHT);
    EXPECT_LE(circles[1].transform.position.x, WIDTH);
    EXPECT_GE(circles[2].transform.position.x, 0);
}