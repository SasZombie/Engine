#include <chrono>
#include <random>
#include "PhysicsWorld.hpp"

std::vector<sas::BodyHandle<sas::Traits3D>> walls;

sas::PhysicsWorld<sas::Traits3D> world;

static void addWalls() noexcept
{
    sas::Transform3D t;
    t.position = {0, 20, 0};

    sas::Kinematics3D k;
    k.pushForce = 10;

    sas::BodyHandle3D RoofBH = world.createBody(sas::Shape3D::MakeOrthotope(20, 0.1, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    t.position = {0, 0, 0};
    sas::BodyHandle3D FloorBH = world.createBody(sas::Shape3D::MakeOrthotope(20, 0.1, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    t.position = {10, 10, 0};
    sas::BodyHandle3D Wall1BH = world.createBody(sas::Shape3D::MakeOrthotope(0.1, 20, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    t.position = {-10, 10, 0};
    sas::BodyHandle3D Wall2BH = world.createBody(sas::Shape3D::MakeOrthotope(0.1, 20, 20), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    t.position = {0, 10, 10};
    sas::BodyHandle3D Wall3BH = world.createBody(sas::Shape3D::MakeOrthotope(20, 20, 0.1), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    t.position = {0, 10, -10};
    sas::BodyHandle3D Wall4BH = world.createBody(sas::Shape3D::MakeOrthotope(20, 20, 0.1), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);

    walls.push_back(RoofBH);
    walls.push_back(FloorBH);
    walls.push_back(Wall1BH);
    walls.push_back(Wall2BH);
    walls.push_back(Wall3BH);
    walls.push_back(Wall4BH);
}


static sas::BodyHandle3D AddSphere(sas::Transform3D trans, sas::Kinematics3D kin)
{
    sas::BodyHandle bh = world.createBody(sas::Shape3D{sas::ShapeType3D::Sphere, 2.f}, trans, kin);
    return bh;
}

static sas::BodyHandle3D AddBox(sas::Transform3D trans, sas::Kinematics3D kin)
{
    sas::BodyHandle bh = world.createBody(sas::Shape3D::MakeOrthotope(2.f, 2.f, 2.f), trans, kin);
    return bh;
}

static sas::BodyHandle3D AddCilinder(sas::Transform3D trans, sas::Kinematics3D kin)
{
    sas::BodyHandle bh = world.createBody(sas::Shape3D::MakeCilinder(2.f, 2.f), trans, kin);
    return bh;
}

int main()
{
    constexpr float dt = 1.f / 60.f;

    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> distX(-8.5f, 8.5f);
    std::uniform_real_distribution<float> distY(1.5f, 18.5f);
    std::uniform_real_distribution<float> distZ(-8.5f, 8.5f);

    sas::Kinematics3D boucyKinematics;
    boucyKinematics.pushForce = 10;
    boucyKinematics.restitution = 1;
    boucyKinematics.pushForce = 1;
    boucyKinematics.inverseMass = 0.2;


    for (int i = 0; i < 500; ++i)
    {
        sas::Transform3D t;
        t.position = {distX(rng), distY(rng), distZ(rng)};
        AddSphere(t, boucyKinematics);
    }

    for (int i = 0; i < 1000; ++i)
    {
        sas::Transform3D t;
        t.position = {distX(rng), distY(rng), distZ(rng)};
        AddBox(t, boucyKinematics);
    }

    for (int i = 0; i < 2500; ++i)
    {
        sas::Transform3D t;
        t.position = {distX(rng), distY(rng), distZ(rng)};

        AddCilinder(t, boucyKinematics);
    }

    addWalls();

    for (int i = 0; i < 120; ++i)
    {
        world.step(dt);
    }

    constexpr int profileFrames = 1000;

    const auto start = std::chrono::steady_clock::now();
    for (int f = 0; f < profileFrames; ++f)
    {
        world.step(dt);
    }
    const auto end = std::chrono::steady_clock::now();

    const double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "Avg frame time: " << (elapsedMs / profileFrames) << " ms\n";
}