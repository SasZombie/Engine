#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include <vector>
#include <iostream>

#include "PhysicsWorld.hpp"

extern "C" const char *__lsan_default_suppressions();

struct Entity
{
    sas::BodyHandle<sas::Traits3D> bodyHandle;

    Color c;
};
static Vector3 toRaylibVector(const sas::math::Vec3 &v) noexcept
{
    return {
        v.data[0],
        v.data[1],
        v.data[2]};
}

static void setUpBox(sas::PhysicsWorld3D &world, std::vector<Entity> &objects) noexcept
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

    objects.push_back({RoofBH, LIME});
    objects.push_back({FloorBH, LIME});
    objects.push_back({Wall1BH, RED});
    objects.push_back({Wall2BH, RED});
    objects.push_back({Wall3BH, RED});
    objects.push_back({Wall4BH, RED});
}

static void setUpModeOne(sas::PhysicsWorld3D &world, std::vector<Entity> &objects) noexcept
{
    sas::Transform3D t;
    t.position = {0.5105f, 13.5f, 29.869f};

    sas::Kinematics3D k;
    k.angularVelocity = {1, 1, 1};

    sas::BodyHandle3D demoSphere = world.createBody(sas::Shape3D::MakeSphere(0.5), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);
    objects.push_back({demoSphere, PURPLE});

    t.position = {0.5105f, 13.5f, 27.869f};

    sas::BodyHandle3D demoCube = world.createBody(sas::Shape3D::MakeOrthotope(1, 1, 1), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);
    objects.push_back({demoCube, YELLOW});

    t.position = {0.5105f, 13.5f, 25.869f};
    sas::BodyHandle3D demoCil = world.createBody(sas::Shape3D::MakeCilinder(0.5, 1), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);
    objects.push_back({demoCil, CLITERAL(Color){0, 255, 255, 255}});
}

static void drawEntity(const Entity &entity) noexcept
{
    const auto &handle = entity.bodyHandle.get();
    switch (handle->shape.type)
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
        const auto &pos = handle->transform.position;
        const auto &scale = handle->transform.scale;

        const float radius = handle->shape.radHeight.x;
        const float halfHeight = handle->shape.radHeight.y;
        const float fullHeight = halfHeight * 2.f;

        const int slices = 16;

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
int main()
{
    constexpr float SCREEN_WIDTH = 800, SCREEN_HEIGHT = 450;

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Engine");
    SetTargetFPS(60);

    auto lambda = [](const sas::AABB3D &aabb, bool isLeaf)
    {
        BoundingBox box = {
            .min = (Vector3){aabb.minX, aabb.minY, aabb.minZ},
            .max = (Vector3){aabb.maxX, aabb.maxY, aabb.maxZ}};

        DrawBoundingBox(box, isLeaf ? PURPLE : YELLOW);
    };

    Camera3D camera;
    camera.position = (Vector3){5.20857, 13.5, 27.9685f};
    camera.target = (Vector3){-23.862, 10.6888, 23.1718};
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
    std::vector<Entity> demoEntities;
    std::vector<Entity> arenaWalls;
    setUpBox(world, arenaWalls);
    setUpModeOne(world, demoEntities);

    sas::Kinematics3D boucyKinematics;
    boucyKinematics.pushForce = 10;

    // Bouncy :D
    boucyKinematics.restitution = 1;
    boucyKinematics.pushForce = 1;
    boucyKinematics.inverseMass = 0.2;

    sas::ShapeType3D currentType = sas::ShapeType3D::Sphere;

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
            std::cout << camera.target.x << ' ' << camera.target.y << ' ' << camera.target.z << '\n';
        }

        if (IsKeyPressed(KEY_ONE))
        {
            currentType = sas::ShapeType3D::Sphere;
        }
        if (IsKeyPressed(KEY_TWO))
        {
            currentType = sas::ShapeType3D::Orthotope;
        }
        if (IsKeyPressed(KEY_THREE))
        {
            currentType = sas::ShapeType3D::Cilinder;
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
            demoEntities.clear();
            arenaWalls.clear();

            setUpBox(world, arenaWalls);
            setUpModeOne(world, demoEntities);
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

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
        {
            Vector3 forward = {
                camera.target.x - camera.position.x,
                camera.target.y - camera.position.y,
                camera.target.z - camera.position.z};
            forward = Vector3Normalize(forward);

            constexpr float spawnDistance = 10.0f;

            sas::Transform3D worldPos;
            worldPos.position = {
                camera.position.x + forward.x * spawnDistance,
                camera.position.y + forward.y * spawnDistance, 
                camera.position.z + forward.z * spawnDistance};

            switch (currentType)
            {
            case sas::ShapeType3D::Sphere:
            {
                sas::BodyHandle3D newSphere = world.createBody(
                    sas::Shape3D::MakeSphere(0.5f),
                    worldPos,
                    boucyKinematics,
                    sas::Flags::Active | sas::Flags::RigidBodyDynamic);
                entities.push_back({newSphere, PURPLE});
                break;
            }

            case sas::ShapeType3D::Cilinder:
            {
                sas::BodyHandle3D newCilinder = world.createBody(
                    sas::Shape3D::MakeCilinder(0.5f, 1.0f),
                    worldPos,
                    boucyKinematics,
                    sas::Flags::Active | sas::Flags::RigidBodyDynamic);
                entities.push_back({newCilinder, CLITERAL(Color){0, 255, 255, 255}});
                break;
            }

            case sas::ShapeType3D::Orthotope:
            {
                sas::BodyHandle3D newOrthope = world.createBody(
                    sas::Shape3D::MakeOrthotope(1.0f, 1.0f, 1.0f),
                    worldPos,
                    boucyKinematics,
                    sas::Flags::Active | sas::Flags::RigidBodyDynamic);
                entities.push_back({newOrthope, YELLOW});
                break;
            }
            }
        }

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

        for (const auto &entity : demoEntities)
        {
            drawEntity(entity);
        }

        for (const auto &entity : entities)
        {
            drawEntity(entity);
        }

        EndMode3D();

        for (size_t i = 0; i < demoEntities.size(); ++i)
        {
            const auto &handle = demoEntities[i].bodyHandle.get();

            Vector3 topAnchor = {
                handle->transform.position.x,
                handle->transform.position.y + 0.8f,
                handle->transform.position.z};

            Vector3 toTarget = Vector3Subtract(topAnchor, camera.position);

            Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));

            if (Vector3DotProduct(forward, toTarget) > 0.0f)
            {
                Vector2 screenPos = GetWorldToScreen(topAnchor, camera);

                if (screenPos.x >= 0 && screenPos.x <= GetScreenWidth() &&
                    screenPos.y >= 0 && screenPos.y <= GetScreenHeight())
                {
                    const char *label = "";
                    switch (i)
                    {
                    case 0:
                        label = "Press [1]";
                        break;
                    case 1:
                        label = "Press [2]";
                        break;
                    case 2:
                        label = "Press [3]";
                        break;
                    default:
                        break;
                    }

                    constexpr int fontSize = 20;
                    int textWidth = MeasureText(label, fontSize);

                    int drawX = static_cast<int>(screenPos.x) - textWidth / 2;
                    int drawY = static_cast<int>(screenPos.y);

                    DrawText(label, drawX + 1, drawY + 1, fontSize, BLACK);
                    DrawText(label, drawX, drawY, fontSize, RAYWHITE);
                }
            }
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