#include "Fixture.hpp"

TEST_F(FixtureTest, BoxFlatDrop_NoRotation)
{
    sas::Transform t;
    t.position = {WIDTH / 2, 300};
    t.rotation = 0.0f;

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.restituition = 0.5f;

    const auto &handle = AddBox(t, k);

    float dt = 1.0f / 60.0f;

    for (int i = 0; i < 60; ++i)
    {
        world->step(dt);
    }

    EXPECT_FLOAT_EQ(handle.get()->kinematics.angularVelocity, 0.0f);
    EXPECT_FLOAT_EQ(handle.get()->transform.rotation, 0.0f);
}

TEST_F(FixtureTest, BoxTiltedDrop_InducesRotation)
{
    sas::Transform t;
    t.position = {WIDTH / 2, 400};
    t.rotation = 0.2f;

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.restituition = 0.5f;

    const auto &handle = AddBox(t, k);

    float dt = 1.0f / 60.0f;
    float maxAngularVelocityObserved = 0.0f;

    for (int i = 0; i < 35; ++i)
    {
        world->step(dt);
        
        float currentSpin = std::abs(handle.get()->kinematics.angularVelocity);
        if (currentSpin > maxAngularVelocityObserved)
        {
            maxAngularVelocityObserved = currentSpin;
        }
    }

    EXPECT_GT(maxAngularVelocityObserved, 0.1f) 
        << "Box failed to spin. Peak spin observed was only: " << maxAngularVelocityObserved;
}

TEST_F(FixtureTest, BoxTiltedDrop_SettlesFlat)
{
    sas::Transform t;
    t.position = {WIDTH / 2, 350};
    t.rotation = 0.2f;

    sas::Kinematics k;
    k.inverseMass = 1.0f;
    k.restituition = 0.0f;

    const auto &handle = AddBox(t, k);

    float dt = 1.0f / 60.0f;

    walls[0]->kinematics.restituition = 0.0f;
    walls[1]->kinematics.restituition = 0.0f;
    walls[2]->kinematics.restituition = 0.0f;
    walls[3]->kinematics.restituition = 0.0f;

    for (int i = 0; i < 380; ++i)
    {
        world->step(dt);
    }
    float rot = std::abs(handle.get()->transform.rotation);

    const float PI_OVER_2 = 1.57079632679f;
    float remainder = std::fmod(rot, PI_OVER_2);

    float distanceToFlat = std::min(remainder, PI_OVER_2 - remainder);


    EXPECT_NEAR(distanceToFlat, 0.0f, 0.05f);
    EXPECT_NEAR(handle.get()->kinematics.angularVelocity, 0.0f, 0.05f);
}