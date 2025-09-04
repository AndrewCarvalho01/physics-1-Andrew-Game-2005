/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"

const unsigned int TARGET_FPS = 50; //frames per seconds
float dt = 1; //seconds/frames
float time = 0;
float x = 500;
float y = 500;
float freequency = 5;
float amplitude = 100;


void update()
{
	dt = 1.0f / TARGET_FPS;
	time += dt;

    x = x + (-cos(time * freequency )) * freequency + amplitude * dt;
	y = y + (-sin(time * freequency )) * freequency + amplitude * dt;

}


void draw()
{
	BeginDrawing();
	ClearBackground(RED);
	DrawText("Hello Andrew Carvalho 101549315!", 10,  GetScreenHeight() - 30 , 20, LIGHTGRAY);

	GuiSliderBar(Rectangle{ 60, 10, 1000, 10 }, "Time", TextFormat("%.2f", time), &time, 0, 240);
	DrawText("T: %.1f", GetScreenWidth() - 40, 10, 30, LIGHTGRAY);
	DrawCircle(x, y, 50, BLUE);
	EndDrawing();
}

int main()
{
    InitWindow(InitialWidth, InitialHeight, "Andrew Carvalho 101549315 GAME2005");
    SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose())
	{
		update();
		draw();
	}

    CloseWindow();
    return 0;
}
