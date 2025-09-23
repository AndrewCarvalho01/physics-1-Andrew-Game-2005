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
		  deltaTime = dt; // update deltaTime each frame

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

    // TEST: This test is just to prove I can create multiple PhysicsBody objects to prove the framework works
    PhysicsBody ball1({ 100, 300 }, { 50, -100 }, 1.0f, 0.1f);
    PhysicsBody ball2({ 200, 250 }, { -30, -80 }, 2.0f, 0.2f);
    PhysicsBody ball3({ 150, 200 }, { 0, -120 }, 1.5f, 0.0f);

    // TEST: Create a PhysicsSIM object
    PhysicsSIM simulation;  // uses default values
    printf("=== PhysicsSIM Test ===\n");
    printf("Delta Time: %.3f\n", simulation.deltaTime);
    printf("Total Time: %.3f\n", simulation.time);
    printf("Gravity: (%.1f, %.1f)\n", simulation.gravity.x, simulation.gravity.y);
    printf("====================\n");

    printf("=== Multiple PhysicsBody Test ===\n");
    printf("Ball 1: pos(%.1f,%.1f) vel(%.1f,%.1f) mass:%.1f\n",
        ball1.position.x, ball1.position.y, ball1.velocity.x, ball1.velocity.y, ball1.mass);
    printf("Ball 2: pos(%.1f,%.1f) vel(%.1f,%.1f) mass:%.1f\n",
        ball2.position.x, ball2.position.y, ball2.velocity.x, ball2.velocity.y, ball2.mass);
    printf("Ball 3: pos(%.1f,%.1f) vel(%.1f,%.1f) mass:%.1f\n",
        ball3.position.x, ball3.position.y, ball3.velocity.x, ball3.velocity.y, ball3.mass);

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
        simulation.Update(ball1, frameTime);
        simulation.Update(ball2, frameTime);
        simulation.Update(ball3, frameTime);


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

        // Draws the multiple test balls
        DrawCircleV(ball1.position, 6, RED);
        DrawCircleV(ball2.position, 6, GREEN);
        DrawCircleV(ball3.position, 6, BLUE);

        EndDrawing();


    }


    CloseWindow();
    return 0;
}