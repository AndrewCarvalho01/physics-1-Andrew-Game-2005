#include "raylib.h"

int main(void)
{
    // Initialization
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "raylib - horizontal red line");

    SetTargetFPS(60);

    // Main game loop
    while (!WindowShouldClose())
    {
        BeginDrawing();

        ClearBackground(BLACK);

        // Draw a horizontal red line across the screen at the middle of the window
        DrawLine(50, screenHeight - 50, 300, 250, RED);



        EndDrawing();
    }

    CloseWindow();

    return 0;
}
