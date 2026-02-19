#include <raylib.h>
#include <vector>
#include <iostream>

#include "PhysicsWorld.hpp"

extern "C" const char *__lsan_default_suppressions();

struct Entity
{
    sas::Transform bodyTransform;

    sas::BodyHandle bodyHandle;

    Color c;
};

int main()
{
    constexpr float SCREEN_WIDTH = 800, SCREEN_HEIGHT = 450;
    constexpr float circleRad = 25.f, e = 0.5f;

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Engine");
    SetTargetFPS(60);

    std::vector<Entity> entities;

    sas::Transform t;
    t.position = {400, 225};
    t.scale = {1, 1};

    sas::Kinematics k;
    k.inverseMass = 0.2f;
    k.restituition = e;
    k.velocity.x = 500;

    sas::PhysicsWorld world({0, 0, SCREEN_WIDTH, SCREEN_HEIGHT});
    sas::PhysicsSettings &settings = world.settings;

    sas::Body *currentBody = nullptr;

    sas::BodyHandle firstBH = world.CreateBody({sas::ShapeType::Circle, circleRad}, t);
    Entity firstEntity{{}, firstBH, MAROON};
    entities.push_back(firstEntity);

    entities[0].bodyHandle->kinematics = k;

    auto lambda = [](const sas::AABB &b, bool isLeaf)
    {
        float width = b.maxX - b.minX;
        float height = b.maxY - b.minY;
        DrawRectangleLines(b.minX, b.minY, width, height, isLeaf ? GREEN : YELLOW); };

    float dt = 0;
    bool drawHitbox = false;

    while (!WindowShouldClose())
    {
        dt = GetFrameTime();

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            const auto &[x, y] = GetMousePosition();

            if (currentBody)
            {
                currentBody->transform.position = {x, y};
            }
            else
            {
                sas::Transform t1;
                t1.position = {x, y};
                t1.scale = sas::math::Vec2{1};

                sas::BodyHandle bh = world.CreateBody({sas::ShapeType::Circle, circleRad}, t1);

                Entity temp{{}, bh, MAROON};
                entities.push_back(temp);
                currentBody = bh.get();
            }
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            sas::Kinematics kin;
            kin.inverseMass = 0.2f;
            kin.restituition = e;

            const auto &[x, y] = GetMouseDelta();
            kin.velocity = {x * 10, y * 10};
            currentBody->kinematics = kin;

            currentBody = nullptr;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT))
        {
            sas::Kinematics kin;
            kin.inverseMass = 0.2f;
            kin.restituition = 1;

            const auto &[x, y] = GetMouseDelta();
            kin.velocity = {x * 10, y * 10};
            currentBody->kinematics = kin;

            currentBody = nullptr;
        }

        if (IsKeyPressed(KEY_R))
        {
            world.Clear();
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
            drawHitbox = false;
        }

        if (IsKeyPressed(KEY_DELETE))
        {
            if (!entities.empty())
            {
                world.RemoveBody(entities.back().bodyHandle);
                entities.pop_back();
            }
        }

        world.Step(dt);

        BeginDrawing();
        ClearBackground(BLACK);

        if (drawHitbox)
        {

            world.DrawDebug(lambda);
        }

        for (auto &entity : entities)
        {
            const auto &circle = entity.bodyHandle.get();
            DrawCircle(circle->transform.position.x, circle->transform.position.y, circle->shape.radius, entity.c);
        }

        const std::string msg1("Gravity = " + std::to_string(settings.gravity));
        const std::string msg2("DragCoeff = " + std::to_string(settings.dragCoeff));
        const std::string msg3("Objects = " + std::to_string(entities.size()));

        DrawText(msg1.c_str(), 0, 0, 30, RED);
        DrawText(msg2.c_str(), 0, 30, 30, RED);
        DrawText(msg3.c_str(), 0, 60, 30, RED);

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