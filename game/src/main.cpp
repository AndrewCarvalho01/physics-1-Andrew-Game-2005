#include "raylib.h"
#include <math.h>
#include <stdio.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#define PI 3.14159265358979323846f

int main(void)
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "raylib - Angry Bird Launch - horizontal red line");
    SetTargetFPS(60);

    // The launch parameters
    Vector2 launchPos = { 100, screenHeight - 100 };
    float launchAngleDeg = 45.0f;
    float launchSpeed = 200.0f;

    while (!WindowShouldClose())
    {
        // GUI sliders (Had expected expression errors, until I used Rectangle{...}
        GuiSliderBar( Rectangle { 60, 20, 200, 20 }, "Launch X", NULL, & launchPos.x, 0, screenWidth);
        GuiSliderBar( Rectangle { 60, 50, 200, 20 }, "Launch Y", NULL, & launchPos.y, 0, screenHeight);
        GuiSliderBar( Rectangle { 60, 80, 200, 20 }, "Angle", NULL, & launchAngleDeg, 0, 180);
        GuiSliderBar( Rectangle { 60, 110, 200, 20 }, "Speed", NULL, & launchSpeed, 0, 400);

        // Computes the velocity vector
        float angleRad = launchAngleDeg * (PI / 180.0f);
        Vector2 velocity = {
            cosf(angleRad) * launchSpeed,
            -sinf(angleRad) * launchSpeed
        };

        // endpoint for the velocity vector 
        Vector2 end = {
            launchPos.x + velocity.x * 0.3f,
            launchPos.y + velocity.y * 0.3f
        };

        BeginDrawing();
        ClearBackground(BLACK);

        // I did this to show the mark launch position or just coordinate 0,0
        DrawCircleV(launchPos, 5, YELLOW);

        // draw velocity vector
        DrawLineV(launchPos, end, RED);

        // draw text of current values
        DrawText(TextFormat("X: %.0f", launchPos.x), 280, 20, 20, WHITE);
        DrawText(TextFormat("Y: %.0f", launchPos.y), 280, 50, 20, WHITE);
        DrawText(TextFormat("Angle: %.1f deg", launchAngleDeg), 280, 80, 20, WHITE);
        DrawText(TextFormat("Speed: %.1f", launchSpeed), 280, 110, 20, WHITE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}