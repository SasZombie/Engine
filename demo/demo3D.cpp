#include <raylib.h>
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

int main()
{
    constexpr float SCREEN_WIDTH = 800, SCREEN_HEIGHT = 450;
    constexpr float circleRad = 25.f, e = 1.f;

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Engine");
    SetTargetFPS(60);

    sas::PhysicsWorld<sas::Traits3D> world;
    sas::PhysicsSettings &settings = world.settings;
    settings.dragCoeff = 0.47f;

    auto lambda = [](const sas::AABB2D &b, bool isLeaf)
    {
        float width = b.maxX - b.minX;
        float height = b.maxY - b.minY;
        DrawRectangleLines(b.minX, b.minY, width, height, isLeaf ? GREEN : YELLOW);
    };

    std::vector<Entity> entities;

    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 2.0f, 4.0f};
    camera.target = (Vector3){0.0f, 2.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float dt = 0;
    float drawHitbox;
    DisableCursor();

    while (!WindowShouldClose())
    {
        dt = GetFrameTime();

        if (IsKeyPressed(KEY_LEFT_ALT))
        {
            if (IsCursorHidden())
                EnableCursor();
            else
                DisableCursor();
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

        world.step(dt);

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode3D(camera);

        DrawGrid(20, 1.0f);

        // if (drawHitbox)
        // {
        //     world.drawDebug(lambda);
        // }

        for (auto &entity : entities)
        {
            const auto &handle = entity.bodyHandle.get();

            // if (entity.type == sas::ShapeType2D::Circle)
            // {
            //     DrawCircle(handle->transform.position.x, handle->transform.position.y, handle->shape.radius * handle->transform.scale.x, entity.c);
            // }
            // else if (entity.type == sas::ShapeType2D::Box)
            // {
            //     float width = handle->shape.halfSize.x * 2 * handle->transform.scale.x;
            //     float height = handle->shape.halfSize.y * 2 * handle->transform.scale.y;

            //     Rectangle rect = {
            //         handle->transform.position.x,
            //         handle->transform.position.y,
            //         width,
            //         height};

            //     Vector2 origin = {width / 2, height / 2};
            //     DrawRectanglePro(rect, origin, handle->transform.rotation * RAD2DEG, entity.c);
            // }
        }

        const std::string msg1("Gravity = " + std::to_string(settings.gravity));
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