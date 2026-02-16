#include <raylib.h>
#include <vector>
#include <iostream>

#include "PhysicsWorld.hpp"

extern "C" const char *__lsan_default_suppressions();

int main()
{
    constexpr float SCREEN_WIDTH = 800, SCREEN_HEIGHT = 450;
    constexpr float circleRad = 25.f, e = 0.5f;

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Engine");
    SetTargetFPS(60);

    std::vector<sas::Body> circles;

    sas::Transform t;
    t.position = {400, 225};
    t.scale = {1, 1};


    sas::Kinematics k;
    k.inverseMass = 0.2f;
    k.restituition = e;
    k.velocity.x = 500;

    sas::Body defaultCircle{{sas::ShapeType::Circle, circleRad}, t, &k};

    circles.emplace_back(defaultCircle);

    float dt = 0;

    sas::PhysicsWorld world({0, 0, SCREEN_WIDTH, SCREEN_HEIGHT});
    sas::PhysicsSettings &settings = world.settings;

    while (!WindowShouldClose())
    {
        dt = GetFrameTime();

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            const auto &[x, y] = GetMousePosition();
            sas::Transform t1;
            t1.position = {x, y};
            t1.scale = {1};
            // Leaks!!!
            sas::Kinematics *kin = new sas::Kinematics;
            kin->inverseMass = 0.2f;
            kin->restituition = e;

            circles.emplace_back(sas::Shape{sas::ShapeType::Circle, circleRad}, t1, kin);
        }

        if (IsKeyPressed(KEY_R))
        {
            circles.clear();
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

        world.Step(circles, dt);

        BeginDrawing();
        ClearBackground(BLACK);

        for (auto &circle : circles)
        {
            DrawCircle(circle.transform.position.x, circle.transform.position.y, circle.shape.radius, MAROON);
        }
        const std::string msg1("Gravity = " + std::to_string(settings.gravity));
        const std::string msg2("DragCoeff = " + std::to_string(settings.dragCoeff));

        DrawText(msg1.c_str(), 0, 0, 30, RED);
        DrawText(msg2.c_str(), 0, 30, 30, RED);

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