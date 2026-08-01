#include <raylib.h>
#include <vector>
#include <iostream>

#include "PhysicsWorld.hpp"

extern "C" const char *__lsan_default_suppressions();

struct Entity
{
    sas::BodyHandle bodyHandle;

    Color c;

    sas::ShapeType type;
};

int main()
{
    constexpr float SCREEN_WIDTH = 800, SCREEN_HEIGHT = 450;
    constexpr float circleRad = 25.f, e = 1.f;

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Engine");
    SetTargetFPS(60);

    sas::PhysicsWorld world;
    sas::PhysicsSettings &settings = world.settings;
    settings.dragCoeff = 0.47f;
    Entity *currentBody = nullptr;

    std::vector<Entity> entities;

    sas::Transform t;
    t.position = {470, 230};
    t.scale = {1, 1};
    float rot = 30.f * DEG2RAD;
    t.rotation = rot;

    sas::Kinematics k;
    k.inverseMass = 0.2f;
    k.restituition = e;
    k.velocity.x = 0;

    sas::BodyHandle firstBH = world.createBody(sas::Shape::MakeBox(200, 16), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);

    t.position = {300, 180};
    t.rotation = 0;

    sas::BodyHandle seccondBh = world.createBody(sas::Shape::MakeBox(200, 16), t, k, sas::Flags::Active | sas::Flags::RigidBodyStatic);

    t.position = {500, 180};
    t.rotation = 0;

    sas::BodyHandle thirdBh = world.createBody(sas::Shape::MakeBox(16, 16), t, k, sas::Flags::Active | sas::Flags::RigidBodyKinematic);

    t.position = {500, 100};
    sas::BodyHandle gravityBh = world.createBody(sas::Shape::MakeBox(100, 100), t, k, sas::Flags::Active | sas::Flags::Trigger);

    Entity firstEntity{firstBH, MAROON, sas::ShapeType::Box};
    Entity seccondEntity{seccondBh, MAROON, sas::ShapeType::Box};
    Entity thirdEntity{thirdBh, MAROON, sas::ShapeType::Box};

    Entity forthEntity{gravityBh, GREEN, sas::ShapeType::Box};

    entities.push_back(firstEntity);
    entities.push_back(seccondEntity);
    entities.push_back(thirdEntity);
    entities.push_back(forthEntity);
    entities[0].bodyHandle->kinematics = k;

    auto lambda = [](const sas::AABB &b, bool isLeaf)
    {
        float width = b.maxX - b.minX;
        float height = b.maxY - b.minY;
        DrawRectangleLines(b.minX, b.minY, width, height, isLeaf ? GREEN : YELLOW);
    };

    float dt = 0;
    bool drawHitbox = false;
    bool collision = false;
    bool shapeType = false;

    sas::ShapeType st;

    sas::Kinematics bouncyKinematics;
    bouncyKinematics.restituition = 0.2f;

    sas::BodyHandle topWall = world.createBody(sas::Shape::MakeBox(SCREEN_WIDTH, 1), sas::Transform{{SCREEN_WIDTH / 2, 0}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    sas::BodyHandle bottomWall = world.createBody(sas::Shape::MakeBox(SCREEN_WIDTH, 1), sas::Transform{{SCREEN_WIDTH / 2, SCREEN_HEIGHT - 1}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    sas::BodyHandle leftWall = world.createBody(sas::Shape::MakeBox(1, SCREEN_HEIGHT), sas::Transform{{1, SCREEN_HEIGHT / 2}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic);
    sas::BodyHandle rightWall = world.createBody(sas::Shape::MakeBox(1, SCREEN_HEIGHT), sas::Transform{{SCREEN_WIDTH, SCREEN_HEIGHT / 2}}, bouncyKinematics, sas::Flags::Active | sas::Flags::RigidBodyStatic);

    entities.push_back({topWall, WHITE, sas::ShapeType::Box});
    entities.push_back({bottomWall, WHITE, sas::ShapeType::Box});
    entities.push_back({rightWall, WHITE, sas::ShapeType::Box});
    entities.push_back({leftWall, WHITE, sas::ShapeType::Box});

    while (!WindowShouldClose())
    {
        dt = GetFrameTime();

        if (forthEntity.bodyHandle.isColliding())
        {
            std::cout << "Collision\n";
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            const auto &[x, y] = GetMousePosition();

            if (currentBody)
            {
                currentBody->bodyHandle->transform.position = {x, y};
            }
            else
            {
                sas::Transform t1;
                t1.position = {x, y};
                t1.rotation = 0.f;
                t1.scale = sas::math::Vec2{1.f};

                sas::Kinematics kin;
                kin.inverseMass = 0.01f;
                kin.restituition = 0;

                sas::BodyHandle bh = world.createBody(shapeType ? sas::Shape::MakeCircle(25) : sas::Shape::MakeBox(50, 50), t1, kin);

                Entity temp{bh, MAROON, shapeType ? sas::ShapeType::Circle : sas::ShapeType::Box};

                if (collision)
                {
                    temp.bodyHandle.setCollisionOff();
                }
                entities.push_back(temp);
                currentBody = &entities.back();
            }
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {

            const auto &[x, y] = GetMouseDelta();
            currentBody->bodyHandle->kinematics.velocity = {x * 10, y * 10};

            currentBody = nullptr;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT))
        {
            sas::Kinematics kin;
            kin.inverseMass = 0.5f;
            kin.restituition = 1.f;

            const auto &[x, y] = GetMouseDelta();
            kin.velocity = {x * 10, y * 10};
            currentBody->bodyHandle->kinematics = kin;

            currentBody = nullptr;
        }

        if (IsKeyPressed(KEY_N))
        {
            const auto &[x, y] = GetMousePosition();

            size_t ind = 0;
            for (size_t i = 0; i < entities.size(); ++i)
            {
                auto &entity = entities[i];
                if (CheckCollisionPointCircle({x, y}, {entity.bodyHandle->transform.position.x, entity.bodyHandle->transform.position.y}, entity.bodyHandle->shape.radius))
                {

                    ind = i;
                    break;
                }
            }

            world.removeBody(entities[ind].bodyHandle->bodyID);

            entities[ind] = entities.back();
            entities.pop_back();
        }

        if (IsKeyPressed(KEY_S))
        {
            shapeType = !shapeType;
        }
        if (IsKeyPressed(KEY_R))
        {
            world.clear();
            entities.clear();
        }

        if (IsKeyPressed(KEY_Q))
        {
            settings.gravity += 50.f;
        }

        if (IsKeyPressed(KEY_E))
        {
            settings.gravity -= 50.f;
        }

        if (IsKeyPressed(KEY_A))
        {
            settings.dragCoeff += 0.1;
        }

        if (IsKeyPressed(KEY_D))
        {
            if (settings.dragCoeff <= 0)
            {
                settings.dragCoeff = 0;
            }
            else
            {
                settings.dragCoeff -= 0.1;
            }
        }

        if (IsKeyPressed(KEY_L))
        {
            drawHitbox = !drawHitbox;
        }

        if (IsKeyPressed(KEY_K))
        {
            collision = !collision;
        }

        if (IsKeyPressed(KEY_DELETE))
        {
            if (entities.size() > 6)
            {
                world.removeBody(entities.back().bodyHandle);
                entities.pop_back();
            }
        }

        float playerSpeed = 300.0f;
        sas::math::Vec2 newVelocity = {0.0f, 0.f};

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
        thirdBh->kinematics.velocity = newVelocity;

        world.step(dt);

        BeginDrawing();
        ClearBackground(BLACK);

        if (drawHitbox)
        {
            world.drawDebug(lambda);
        }

        for (auto &entity : entities)
        {
            const auto &handle = entity.bodyHandle.get();
            if (entity.type == sas::ShapeType::Circle)
            {
                DrawCircle(handle->transform.position.x, handle->transform.position.y, handle->shape.radius * handle->transform.scale.x, entity.c);
            }
            else if (entity.type == sas::ShapeType::Box)
            {
                float width = handle->shape.halfSize.x * 2 * handle->transform.scale.x;
                float height = handle->shape.halfSize.y * 2 * handle->transform.scale.y;

                Rectangle rect = {
                    handle->transform.position.x,
                    handle->transform.position.y,
                    width,
                    height};

                Vector2 origin = {width / 2, height / 2};
                DrawRectanglePro(rect, origin, handle->transform.rotation * RAD2DEG, entity.c);
            }
        }

        const std::string msg1("Gravity = " + std::to_string(settings.gravity));
        const std::string msg2("DragCoeff = " + std::to_string(settings.dragCoeff));
        const std::string msg3("Objects = " + std::to_string(entities.size()));

        DrawText(msg1.c_str(), 0, 0, 30, RED);
        DrawText(msg2.c_str(), 0, 30, 30, RED);
        DrawText(msg3.c_str(), 0, 60, 30, RED);
        DrawText("Shape ", 500, 0, 30, RED);
        DrawText((shapeType ? "Circle" : "Square"), 620, 0, 30, RED);
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