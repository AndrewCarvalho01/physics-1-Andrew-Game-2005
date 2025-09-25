#include "raylib.h"
#include <math.h>
#include <stdio.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#define PI 3.14159265358979323846f


class PhysicsBody {
public:
    Vector2 position;
    Vector2 velocity;
    float drag;
    float mass;
    bool active; // to indicate if the body is active or not

    // Constructor to initialize the physics body
    PhysicsBody(Vector2 pos, Vector2 vel, float m, float d)
        : position(pos), velocity(vel), mass(m), drag(d), active(true) {
    }

};

class PhysicsSIM {
public: float deltaTime; // ---------
      float time;        //         | -------- applies to all physics bodies
      Vector2 gravity;   // --------- 

      // Constructor to initialize the values
      PhysicsSIM(float dt = 0.0f, float t = 0.0f, Vector2 grav = { 0, 98.0f })
      {
          deltaTime = dt;
          time = t;
          gravity = grav;
      }

      void Update(PhysicsBody& body, float dt)
      {
          deltaTime = dt; // updates deltaTime each frame

          // apply gravity to velocity
          body.velocity.x += gravity.x * dt;
          body.velocity.y += gravity.y * dt;
          // apply drag to velocity
          body.position.x += body.velocity.x * dt;
          body.position.y += body.velocity.y * dt;

          time += dt;
      }
};

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

    // Create a PhysicsSIM object
    PhysicsSIM simulation;  // uses default values

    PhysicsBody launchedBall({ 0, 0 }, { 0, 0 }, 1.0f, 0.0f);  // starts inactive
    bool ballActive = false;  // track if ball is in flight

    while (!WindowShouldClose())
    {
        // GUI sliders (Had expected expression errors, until I used Rectangle{...}
        GuiSliderBar(Rectangle{ 60, 20, 200, 20 }, "Launch X", NULL, &launchPos.x, 0, screenWidth);
        GuiSliderBar(Rectangle{ 60, 50, 200, 20 }, "Launch Y", NULL, &launchPos.y, 0, screenHeight);
        GuiSliderBar(Rectangle{ 60, 80, 200, 20 }, "Angle", NULL, &launchAngleDeg, 0, 180);
        GuiSliderBar(Rectangle{ 60, 110, 200, 20 }, "Speed", NULL, &launchSpeed, 0, 400);

        // GUI sliders for gravity and magnitude
        GuiSliderBar(Rectangle{ 520, 20, 200, 20 }, "Gravity X", NULL, &simulation.gravity.x, -200, 200);
        GuiSliderBar(Rectangle{ 520, 50, 200, 20 }, "Gravity Y", NULL, &simulation.gravity.y, -200, 200);

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

        float frameTime = GetFrameTime();

        if (ballActive) {
            simulation.Update(launchedBall, frameTime);
        }

        if (IsKeyPressed(KEY_SPACE)) //shoots ball with space bar
        {

            launchedBall.position = launchPos;
            launchedBall.velocity = velocity;  // Uses the velo that I already calcualted from Lab 1
            ballActive = true;
        }
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


        // This code makes the ball that is launched
        if (ballActive) 
        {
            DrawCircleV(launchedBall.position, 8, ORANGE);
        }

        EndDrawing();

    }

    CloseWindow();
    return 0;
}