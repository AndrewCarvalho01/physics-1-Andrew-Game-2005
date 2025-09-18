#include "raylib.h"
#include <math.h>  
#include <stdio.h>  

int main(void)
{
    // Initialization
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "raylib - horizontal red line");
    SetTargetFPS(60);

    // line start and end points
    Vector2 start = { 50, screenHeight - 50 };
    Vector2 end = { 300, 250 };

    while (!WindowShouldClose())
    {
        // Calculate angle
        float dx = end.x - start.x;
        float dy = end.y - start.y;
        float angleRad = atan2f(dy, dx);          // angle in radians
        float angleDeg = angleRad * (180.0f / PI); // converting to degrees

        BeginDrawing();
        ClearBackground(BLACK);

        // This Draws the line
        DrawLineV(start, end, RED);

        // This draws the box for the angle text
        int boxWidth = 190;
        int boxHeight = 40;
        int boxX = 10;
        int boxY = 10;

        DrawRectangle(boxX, boxY, boxWidth, boxHeight, DARKGRAY);
        DrawRectangleLines(boxX, boxY, boxWidth, boxHeight, RAYWHITE); // border

        // Draws the angle text then I put them in the box
        char angleText[64];
        _snprintf_s(angleText, sizeof(angleText), _TRUNCATE, "Angle: %.2f deg", angleDeg);
        DrawText(angleText, boxX + 10, boxY + 10, 20, RAYWHITE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
