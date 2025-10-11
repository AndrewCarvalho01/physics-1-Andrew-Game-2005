#include "raylib.h"
#include <math.h>
#include <stdio.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#define PI 3.14159265358979323846f
#include <vector>


class PhysicsBody {
public:
    Vector2 position;
    Vector2 velocity;
    float drag;
    float mass;
    bool active; // to indicate if the body is active or not
    float radius; // radius for drawing the body

    // Constructor to initialize the physics body
    PhysicsBody(Vector2 pos, Vector2 vel, float m, float d, float r)
        : position(pos), velocity(vel), mass(m), drag(d), radius(r), active(true) {
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

      bool CheckSphereCollision(const PhysicsBody& bodyA, const PhysicsBody& bodyB) {
          float dx = bodyA.position.x - bodyB.position.x;
          float dy = bodyA.position.y - bodyB.position.y;
          float distance = sqrtf(dx * dx + dy * dy);

          float combinedRad = bodyA.radius + bodyB.radius;
          return distance <= combinedRad;
      }

      bool CheckLineCollision(const PhysicsBody& bodyWithLine, const PhysicsBody& targetBody) {
          Vector2 lineStart = { bodyWithLine.position.x + bodyWithLine.radius, bodyWithLine.position.y };
          Vector2 lineEnd = { bodyWithLine.position.x + bodyWithLine.radius * 8, bodyWithLine.position.y };

          float dx = targetBody.position.x - lineStart.x;
          float dy = targetBody.position.y - lineStart.y;
          float lineLength = lineEnd.x - lineStart.x;

          float t = (dx) / lineLength;
          if (t < 0) t = 0;
          if (t > 1) t = 1;

          float closestX = lineStart.x + t * lineLength;
          float closestY = lineStart.y;

          float distX = targetBody.position.x - closestX;
          float distY = targetBody.position.y - closestY;
          float distance = sqrtf(distX * distX + distY * distY);

          return distance <= targetBody.radius;
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

    std::vector<PhysicsBody> launchedBalls;  // let's me shoot multiple balls without losing the previous ones

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

        for (auto& ball : launchedBalls)
        {
            simulation.Update(ball, frameTime);
        }

        if (IsKeyPressed(KEY_SPACE)) //shoots ball with space bar
        {
            PhysicsBody newBall(launchPos, velocity, 1.0f, 0.1f, 8.0f);
            launchedBalls.push_back(newBall); // add the new ball to the vector
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

        for (size_t i = 0; i < launchedBalls.size(); ++i) {
            bool isColliding = false;

            for (size_t j = 0; j < launchedBalls.size(); ++j) {
                if (i != j) {
                    if (simulation.CheckSphereCollision(launchedBalls[i], launchedBalls[j]) ||
                        simulation.CheckLineCollision(launchedBalls[i], launchedBalls[j])) {
                        isColliding = true;
                        break;
                    }
                }
            }

            Vector2 lineStart = { launchedBalls[i].position.x + launchedBalls[i].radius, launchedBalls[i].position.y };
            Vector2 lineEnd = { launchedBalls[i].position.x + launchedBalls[i].radius * 8, launchedBalls[i].position.y };
            DrawLineV(lineStart, lineEnd, PURPLE);

            Color ballColor = isColliding ? RED : GREEN;
            DrawCircleV(launchedBalls[i].position, launchedBalls[i].radius, ballColor);
        }

        EndDrawing();

    }

    CloseWindow();
    return 0;
}