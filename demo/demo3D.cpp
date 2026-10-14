#include <raylib.h>
#include <rlgl.h>
#include <vector>
#include <iostream>

#include "PhysicsWorld.hpp"

extern "C" const char *__lsan_default_suppressions();

struct Entity
{
    sas::BodyHandle<sas::Traits3D> bodyHandle;

    Color c;

    sas::ShapeType3D type;
};
static Vector3 toRaylibVector(const sas::math::Vec3 &v) noexcept
{
    return {
        v.data[0],
        v.data[1],
        v.data[2]};
}

int main()
{
    constexpr float SCREEN_WIDTH = 800, SCREEN_HEIGHT = 450;
    // constexpr float circleRad = 25.f, e = 1.f;

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Engine");
    SetTargetFPS(60);

    auto lambda = [](const sas::AABB3D &aabb, bool isLeaf)
    {
        // float width = aabb.maxX - aabb.minX;
        // float height = aabb.maxY - aabb.minY;
        // float length = aabb.maxZ - aabb.minZ;

        BoundingBox box = {
            .min = (Vector3){aabb.minX, aabb.minY, aabb.minZ},
            .max = (Vector3){aabb.maxX, aabb.maxY, aabb.maxZ}};

        DrawBoundingBox(box, isLeaf ? PURPLE : YELLOW);
    };

    Camera3D camera;
    camera.position = (Vector3){12.5105f, 13.5f, 26.869f};
    camera.target = (Vector3){0.0f, 2.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float dt = 0;
    bool drawHitbox = false;
    DisableCursor();

    // ==== Entitites ====
    sas::PhysicsWorld<sas::Traits3D> world;
    sas::PhysicsSettings &settings = world.settings;
    settings.dragCoeff = 0.47f;
    settings.gravity = {0, -3, 0};

    std::vector<Entity> entities;
    std::vector<Entity> arenaWalls;

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

    arenaWalls.push_back({RoofBH, LIME, sas::ShapeType3D::Orthotope});
    arenaWalls.push_back({FloorBH, LIME, sas::ShapeType3D::Orthotope});
    arenaWalls.push_back({Wall1BH, RED, sas::ShapeType3D::Orthotope});
    arenaWalls.push_back({Wall2BH, RED, sas::ShapeType3D::Orthotope});
    arenaWalls.push_back({Wall3BH, RED, sas::ShapeType3D::Orthotope});
    arenaWalls.push_back({Wall4BH, RED, sas::ShapeType3D::Orthotope});

    t.position = {0, 15, 0};

    // Bouncy :D
    k.restitution = 1;
    k.pushForce = 1;
    k.inverseMass = 0.2;

    // sas::BodyHandle3D Sphere = world.createBody(sas::Shape3D::MakeSphere(1), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);
    // entities.push_back({Sphere, PURPLE, sas::ShapeType3D::Sphere});

    // Sphere->kinematics.angularVelocity = {1, 1, 1};

    t.position = {0, 3, 0};
    sas::BodyHandle3D Cube = world.createBody(sas::Shape3D::MakeOrthotope(3, 3, 3), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    entities.push_back({Cube, PURPLE, sas::ShapeType3D::Orthotope});
    t.position = {0, 0, 0};
    sas::BodyHandle3D Cube3 = world.createBody(sas::Shape3D::MakeOrthotope(3, 3, 3), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    entities.push_back({Cube3, PURPLE, sas::ShapeType3D::Orthotope});
    // Cube->transform.scale = {3, 3, 3};

    // Cube->kinematics.angularVelocity = {1, 0, 0};

    t.position = {0, 15, 0};
    sas::BodyHandle3D Cilinder = world.createBody(sas::Shape3D::MakeCilinder(1, 2), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);
    entities.push_back({Cilinder, PURPLE, sas::ShapeType3D::Cilinder});

    t.position = {15, 15, 0};
    sas::BodyHandle3D Cilinder2 = world.createBody(sas::Shape3D::MakeCilinder(1, 2), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);
    entities.push_back({Cilinder2, PURPLE, sas::ShapeType3D::Cilinder});
    Cilinder2->kinematics.angularVelocity = {1, 1, 1};

    t.position = {4, 15, 4};
    sas::BodyHandle3D Cilinder3 = world.createBody(sas::Shape3D::MakeCilinder(1, 2), t, k, sas::Flags::Active | sas::Flags::RigidBodyDynamic);
    entities.push_back({Cilinder3, PURPLE, sas::ShapeType3D::Cilinder});
    // Cilinder2->kinematics.angularVelocity = {1, 1, 1};

    t.position = {10, 15, 0};
    sas::BodyHandle3D Cube2 = world.createBody(sas::Shape3D::MakeOrthotope(1, 3, 1), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    entities.push_back({Cube2, PURPLE, sas::ShapeType3D::Orthotope});

    // Cilinder->kinematics.angularVelocity = {1, 1, 1};

    // t.position = {0, 3, 0};
    // t.position = {0, 15, 0};
    // sas::BodyHandle3D Spehere = world.createBody(sas::Shape3D::MakeSphere(1), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);
    // entities.push_back({Spehere, PURPLE, sas::ShapeType3D::Sphere});

    // ColSphere->kinematics.angularVelocity = {1, 1, 1};

    while (!WindowShouldClose())
    {
        dt = GetFrameTime();
        UpdateCamera(&camera, CAMERA_FIRST_PERSON);

        if (IsKeyPressed(KEY_LEFT_ALT))
        {
            if (IsCursorHidden())
                EnableCursor();
            else
                DisableCursor();
        }

        if (IsKeyPressed(KEY_P))
        {
            std::cout << camera.position.x << ' ' << camera.position.y << ' ' << camera.position.z << '\n';
        }

        if (IsKeyDown(KEY_SPACE))
        {
            camera.position.y += 0.1;
        }

        if (IsKeyDown(KEY_LEFT_CONTROL))
        {
            camera.position.y -= 0.1;
        }

        if (IsKeyPressed(KEY_R))
        {
            world.clear();
            entities.clear();
        }

        if (IsKeyPressed(KEY_Q))
        {
            settings.gravity.y += 50.f;
        }

        if (IsKeyPressed(KEY_E))
        {
            settings.gravity.y -= 50.f;
        }

        if (IsKeyPressed(KEY_L))
        {
            drawHitbox = !drawHitbox;
        }

        float playerSpeed = 3.f;
        sas::math::Vec3 newVelocity = sas::math::Vec3{0};

        if (IsKeyDown(KEY_LEFT))
        {
            newVelocity.x = -playerSpeed;
        }
        if (IsKeyDown(KEY_RIGHT))
        {
            newVelocity.x = playerSpeed;
        }
        if (IsKeyDown(KEY_UP))
        {
            newVelocity.y = -playerSpeed;
        }
        if (IsKeyDown(KEY_DOWN))
        {
            newVelocity.y = playerSpeed;
        }

        // Set the velocity directly!
        Cilinder->kinematics.velocity = newVelocity;

        world.step(dt);

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode3D(camera);

        DrawGrid(20, 10.0f);

        if (drawHitbox)
        {
            world.drawDebug(lambda);
        }
        // MARK: Arena

        for (size_t i = 0; i < arenaWalls.size(); ++i)
        {
            const auto &entity = arenaWalls[i];
            const auto &handle = entity.bodyHandle.get();

            float width = handle->shape.halfSize.x * 2 * handle->transform.scale.x;
            float height = handle->shape.halfSize.y * 2 * handle->transform.scale.y;
            float length = handle->shape.halfSize.z * 2 * handle->transform.scale.z;
            if (i < 2)
            {
                DrawCube(toRaylibVector(handle->transform.position), width, height, length, entity.c);
            }
            else
            {
                DrawCubeWires(toRaylibVector(handle->transform.position), width, height, length, entity.c);
            }
        }

        for (const auto &entity : entities)
        {
            const auto &handle = entity.bodyHandle.get();
            // std::cout << handle->transform.quaterion.x << ' ' << handle->transform.quaterion.y << ' ' << handle->transform.quaterion.z << ' ' << handle->transform.quaterion.w << '\n';
            switch (entity.type)
            {
            case sas::ShapeType3D::Sphere:
            {
                const auto &pos = handle->transform.position;
                const auto &scale = handle->transform.scale;

                float radius = handle->shape.radius;

                rlPushMatrix();

                rlTranslatef(pos.x, pos.y, pos.z);

                rlMultMatrixf(reinterpret_cast<const float *>(&handle->transform.rotMatrix));

                rlScalef(scale.x, scale.y, scale.z);
                DrawSphere({0.0f, 0.0f, 0.0f}, radius, entity.c);

                DrawSphereWires({0.0f, 0.0f, 0.0f}, radius, 16, 16, BLACK);

                rlPopMatrix();
                break;
            }

            case sas::ShapeType3D::Orthotope:
            {

                const auto &pos = handle->transform.position;
                const auto &scale = handle->transform.scale;

                rlPushMatrix();

                rlTranslatef(pos.x, pos.y, pos.z);
                rlMultMatrixf(reinterpret_cast<const float *>(&handle->transform.rotMatrix));
                rlScalef(scale.x, scale.y, scale.z);

                DrawCube({0.0f, 0.0f, 0.0f}, handle->shape.halfSize.x * 2, handle->shape.halfSize.y * 2, handle->shape.halfSize.z * 2, entity.c);
                DrawCubeWires({0.0f, 0.0f, 0.0f}, handle->shape.halfSize.x * 2, handle->shape.halfSize.y * 2, handle->shape.halfSize.z * 2, BLACK);

                rlPopMatrix();
                break;
            }

            case sas::ShapeType3D::Cilinder:
            {
                // const auto &pos = handle->transform.position;
                // const auto &scale = handle->transform.scale;

                // float radius = handle->shape.radHeight.x * scale.x;
                // float halfHeight = handle->shape.radHeight.y * scale.y;

                // rlPushMatrix();
                // rlTranslatef(pos.x, pos.y, pos.z);
                // rlMultMatrixf(reinterpret_cast<const float *>(&handle->transform.rotMatrix));

                // // Draw a box that perfectly wraps your physics cylinder bounds
                // DrawCubeWires({0, 0, 0}, radius * 2, halfHeight * 2, radius * 2, RED);

                // rlPopMatrix();
                // break;

                const auto &pos = handle->transform.position;
                const auto &scale = handle->transform.scale;

                const float radius = handle->shape.radHeight.x;
                const float halfHeight = handle->shape.radHeight.y;
                const float fullHeight = halfHeight * 2.0f; // Total height

                const int slices = 16;

                // Shift the base DOWN by half the height so drawing UP by fullHeight centers it
                const Vector3 basePos = {0.0f, -halfHeight, 0.0f};

                rlPushMatrix();

                rlTranslatef(pos.x, pos.y, pos.z);
                rlMultMatrixf(reinterpret_cast<const float *>(&handle->transform.rotMatrix));
                rlScalef(scale.x, scale.y, scale.z);

                // Draw using the shifted base and full height
                DrawCylinder(basePos, radius, radius, fullHeight, slices, entity.c);
                DrawCylinderWires(basePos, radius, radius, fullHeight, slices, BLACK);

                rlPopMatrix();
                break;
            }
            }
        }

        EndMode3D();

        if (Cilinder.isColliding())
        {
            std::cout << "Collision Detected\n";
        }

        if (Cilinder3.isColliding())
        {
            std::cout << "Collision Detected\n";

            std::cout << Cilinder3.getCollisions()[0].depth << Cilinder3.getCollisions()[0].normal.x << ' ' << Cilinder3.getCollisions()[0].normal.y << '\n';
        }

        const std::string msg1("Gravity = " + std::to_string(settings.gravity.y));
        const std::string msg2("DragCoeff = " + std::to_string(settings.dragCoeff));
        const std::string msg3("Objects = " + std::to_string(entities.size()));

        DrawText(msg1.c_str(), 0, 0, 30, RED);
        DrawText(msg2.c_str(), 0, 30, 30, RED);
        DrawText(msg3.c_str(), 0, 60, 30, RED);
        DrawText(std::to_string(dt).c_str(), 650, 40, 30, RED);

        EndDrawing();
    }

    CloseWindow();
}

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || defined(__SANITIZE_ADDRESS__)
extern "C" const char *__lsan_default_suppressions()
{
    return "leak:libnvidia-glcore.so\n"
           "leak:libGLX_nvidia.so\n"
           "leak:libnvidia-glsi.so\n";
}
#endif
#endif