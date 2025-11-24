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
    float mu; // coefficient of kinetic friction
    Color color; // color of the sphere
    // Constructor to initialize the physics body
    PhysicsBody(Vector2 pos, Vector2 vel, float m, float d, float r, float friction = 0.0f, Color c = GREEN)
        : position(pos), velocity(vel), mass(m), drag(d), radius(r), mu(friction), color(c), active(true) {
    }
};

class HalfSpace {
public:
    Vector2 point; // A point on the line
    Vector2 normal; // Direction perpendicular to the line (pointing "out")
    HalfSpace(Vector2 p, Vector2 n) : point(p), normal(n) {}
};

class PhysicsSIM {
public:
    float deltaTime; // ---------
    float time; //              -------- applies to all physics bodies
    Vector2 gravity; // ---------
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
    bool CheckSphereHalfSpaceCollision(const PhysicsBody& sphere, const HalfSpace& halfSpace) {
        float dx = sphere.position.x - halfSpace.point.x;
        float dy = sphere.position.y - halfSpace.point.y;
        float distance = dx * halfSpace.normal.x + dy * halfSpace.normal.y;
        return distance <= sphere.radius;
    }
};

int main(void)
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "raylib - Angry Bird Launch");
    SetTargetFPS(60);

    Vector2 launchPos = { 100, screenHeight - 100 };
    float launchAngleDeg = 45.0f;
    float launchSpeed = 200.0f;

    PhysicsSIM simulation;
    std::vector<PhysicsBody> launchedBalls;
    float planeX = screenWidth / 2;
    float planeY = screenHeight / 2;
    float planeAngleDeg = 30.0f; // Start with a nice slope!

    Vector2 staticBallPos = { planeX, planeY - 20 };
    PhysicsBody staticBall(staticBallPos, { 0,0 }, 1.0f, 0.0f, 10.0f);

    Vector2 fbdCenter = { 650, 170 };
    float fbdRadius = 30.0f;

    while (!WindowShouldClose())
    {
        GuiSliderBar(Rectangle{ 60, 20, 200, 20 }, "Launch X", NULL, &launchPos.x, 0, screenWidth);
        GuiSliderBar(Rectangle{ 60, 50, 200, 20 }, "Launch Y", NULL, &launchPos.y, 0, screenHeight);
        GuiSliderBar(Rectangle{ 60, 80, 200, 20 }, "Angle", NULL, &launchAngleDeg, 0, 180);
        GuiSliderBar(Rectangle{ 60, 110, 200, 20 }, "Speed", NULL, &launchSpeed, 0, 400);
        GuiSliderBar(Rectangle{ 520, 20, 200, 20 }, "Gravity X", NULL, &simulation.gravity.x, -200, 200);
        GuiSliderBar(Rectangle{ 520, 50, 200, 20 }, "Gravity Y", NULL, &simulation.gravity.y, -200, 200);
        GuiSliderBar(Rectangle{ 60, 140, 200, 20 }, "Plane X", NULL, &planeX, 0, screenWidth);
        GuiSliderBar(Rectangle{ 60, 170, 200, 20 }, "Plane Y", NULL, &planeY, 0, screenHeight);
        GuiSliderBar(Rectangle{ 60, 200, 200, 20 }, "Plane Angle", NULL, &planeAngleDeg, 0, 360);

        float angleRad = launchAngleDeg * (PI / 180.0f);
        Vector2 velocity = {
            cosf(angleRad) * launchSpeed,
            -sinf(angleRad) * launchSpeed
        };
        Vector2 end = {
            launchPos.x + velocity.x * 0.3f,
            launchPos.y + velocity.y * 0.3f
        };

        float planeAngleRad = planeAngleDeg * (PI / 180.0f);
        Vector2 planeNormal = {
            cosf(planeAngleRad),
            sinf(planeAngleRad)
        };
        HalfSpace adjustablePlane({ planeX, planeY }, planeNormal);

        staticBall.position.x = planeX - planeNormal.x * staticBall.radius;
        staticBall.position.y = planeY - planeNormal.y * staticBall.radius;

        Vector2 gravityForce = { simulation.gravity.x * staticBall.mass,
                                simulation.gravity.y * staticBall.mass };
        float gravityAlongNormal = gravityForce.x * planeNormal.x + gravityForce.y * planeNormal.y;
        Vector2 normalForce = { -planeNormal.x * gravityAlongNormal,
                               -planeNormal.y * gravityAlongNormal };
        Vector2 frictionForce = { gravityForce.x - normalForce.x,
                                 gravityForce.y - normalForce.y };

        float frameTime = GetFrameTime();
        for (auto& ball : launchedBalls)
        {
            simulation.Update(ball, frameTime);
        }

        if (IsKeyPressed(KEY_ONE)) {
            PhysicsBody newBall(launchPos, velocity, 2.0f, 0.1f, 8.0f, 0.1f, RED);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_TWO)) {
            PhysicsBody newBall(launchPos, velocity, 8.0f, 0.1f, 8.0f, 0.1f, BLUE);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_THREE)) {
            PhysicsBody newBall(launchPos, velocity, 2.0f, 0.1f, 8.0f, 0.8f, GREEN);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_FOUR)) {
            PhysicsBody newBall(launchPos, velocity, 8.0f, 0.1f, 8.0f, 0.8f, YELLOW);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_SPACE))
        {
            PhysicsBody newBall(launchPos, velocity, 1.0f, 0.1f, 8.0f, 0.0f, GREEN);
            launchedBalls.push_back(newBall);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        DrawCircleV(launchPos, 5, YELLOW);
        DrawLineV(launchPos, end, RED);

        DrawText(TextFormat("X: %.0f", launchPos.x), 280, 20, 20, WHITE);
        DrawText(TextFormat("Y: %.0f", launchPos.y), 280, 50, 20, WHITE);
        DrawText(TextFormat("Angle: %.1f deg", launchAngleDeg), 280, 80, 20, WHITE);
        DrawText(TextFormat("Speed: %.1f", launchSpeed), 280, 110, 20, WHITE);

        float lineLength = 400;
        Vector2 perpendicular = { -planeNormal.y, planeNormal.x };
        Vector2 lineStart = { planeX - perpendicular.x * lineLength, planeY - perpendicular.y * lineLength };
        Vector2 lineEnd = { planeX + perpendicular.x * lineLength, planeY + perpendicular.y * lineLength };
        DrawLineV(lineStart, lineEnd, WHITE);
        Vector2 normalEnd = { planeX + planeNormal.x * 50, planeY + planeNormal.y * 50 };
        DrawLineV({ planeX, planeY }, normalEnd, ORANGE);
        DrawCircleV({ planeX, planeY }, 5, ORANGE);

        DrawCircleV(staticBall.position, staticBall.radius, BLUE);
        DrawCircleV(fbdCenter, fbdRadius, WHITE);
        DrawCircleV(fbdCenter, fbdRadius - 2, BLACK);
        DrawCircleV(fbdCenter, 3, WHITE);

        float forceScale = 0.3f;
        Vector2 gravityEnd = { fbdCenter.x + gravityForce.x * forceScale,
                              fbdCenter.y + gravityForce.y * forceScale };
        DrawLineV(fbdCenter, gravityEnd, RED);
        DrawCircleV(gravityEnd, 2, RED);
        Vector2 normalFEnd = { fbdCenter.x + normalForce.x * forceScale,
                             fbdCenter.y + normalForce.y * forceScale };
        DrawLineV(fbdCenter, normalFEnd, GREEN);
        DrawCircleV(normalFEnd, 2, GREEN);
        Vector2 frictionEnd = { fbdCenter.x + frictionForce.x * forceScale,
                               fbdCenter.y + frictionForce.y * forceScale };
        DrawLineV(fbdCenter, frictionEnd, YELLOW);
        DrawCircleV(frictionEnd, 2, YELLOW);

        DrawText("FBD", fbdCenter.x - 15, fbdCenter.y - fbdRadius - 20, 16, WHITE);
        DrawText("G", gravityEnd.x + 5, gravityEnd.y, 10, RED);
        DrawText("N", normalFEnd.x + 5, normalFEnd.y, 10, GREEN);
        DrawText("F", frictionEnd.x + 5, frictionEnd.y, 10, YELLOW);
        DrawText(TextFormat("Net: (%.1f, %.1f)",
            gravityForce.x + normalForce.x + frictionForce.x,
            gravityForce.y + normalForce.y + frictionForce.y),
            fbdCenter.x - 60, fbdCenter.y + fbdRadius + 10, 14, WHITE);

        for (size_t i = 0; i < launchedBalls.size(); ++i) {
            bool isColliding = false;
            float slopeSpeed = 0.0f;      
            bool onSlope = false;         

            for (size_t j = 0; j < launchedBalls.size(); ++j) {
                if (i != j) {
                    if (simulation.CheckSphereCollision(launchedBalls[i], launchedBalls[j])) {

                        float dx = launchedBalls[i].position.x - launchedBalls[j].position.x;
                        float dy = launchedBalls[i].position.y - launchedBalls[j].position.y;
                        float distance = sqrtf(dx * dx + dy * dy);
                        float overlap = (launchedBalls[i].radius + launchedBalls[j].radius) - distance;
                        float nx = dx / distance;
                        float ny = dy / distance;

                        launchedBalls[i].position.x += nx * overlap * 0.5f;
                        launchedBalls[i].position.y += ny * overlap * 0.5f;
                        launchedBalls[j].position.x -= nx * overlap * 0.5f;
                        launchedBalls[j].position.y -= ny * overlap * 0.5f;
                        isColliding = true;
                        break;
                    }
                    if (simulation.CheckLineCollision(launchedBalls[i], launchedBalls[j])) {
                        isColliding = true;
                        break;
                    }
                }
            }

            // === FRICTION ON SLOPE ===
            if (!isColliding) {
                if (simulation.CheckSphereHalfSpaceCollision(launchedBalls[i], adjustablePlane)) {
                    float dx = launchedBalls[i].position.x - adjustablePlane.point.x;
                    float dy = launchedBalls[i].position.y - adjustablePlane.point.y;
                    float distance = dx * adjustablePlane.normal.x + dy * adjustablePlane.normal.y;

                    if (distance < launchedBalls[i].radius) {
                        float penetration = launchedBalls[i].radius - distance;
                        launchedBalls[i].position.x += adjustablePlane.normal.x * penetration;
                        launchedBalls[i].position.y += adjustablePlane.normal.y * penetration;
                    }

                    Vector2 gForce = { simulation.gravity.x * launchedBalls[i].mass,
                                       simulation.gravity.y * launchedBalls[i].mass };

                    float N_mag = gForce.x * adjustablePlane.normal.x + gForce.y * adjustablePlane.normal.y;
                    Vector2 normalForce = { -adjustablePlane.normal.x * N_mag,
                                            -adjustablePlane.normal.y * N_mag };

                    Vector2 tangent = { -adjustablePlane.normal.y, adjustablePlane.normal.x };
                    float v_tan = launchedBalls[i].velocity.x * tangent.x + launchedBalls[i].velocity.y * tangent.y;
                    onSlope = true;                     
                    slopeSpeed = fabsf(v_tan);          

                    float frictionMax = launchedBalls[i].mu * fabsf(N_mag);
                    Vector2 frictionForce = { 0 };

                    if (fabsf(v_tan) > 0.01f) {
                        Vector2 fDir = (v_tan > 0) ? Vector2{ -tangent.x, -tangent.y } : tangent;
                        frictionForce = Vector2{ fDir.x * frictionMax, fDir.y * frictionMax };
                    }

                    Vector2 netForce = {
                        gForce.x + normalForce.x + frictionForce.x,
                        gForce.y + normalForce.y + frictionForce.y
                    };

                    Vector2 accel = {
                        netForce.x / launchedBalls[i].mass,
                        netForce.y / launchedBalls[i].mass
                    };

                    launchedBalls[i].velocity.x += accel.x * frameTime;
                    launchedBalls[i].velocity.y += accel.y * frameTime;

                    if (fabsf(v_tan) < 10.0f) {
                        launchedBalls[i].velocity.x *= 0.98f;
                        launchedBalls[i].velocity.y *= 0.98f;
                    }

                    isColliding = true;
                }
            }

            Vector2 lineStart = { launchedBalls[i].position.x + launchedBalls[i].radius, launchedBalls[i].position.y };
            Vector2 lineEnd = { launchedBalls[i].position.x + launchedBalls[i].radius * 8, launchedBalls[i].position.y };
            DrawLineV(lineStart, lineEnd, PURPLE);

            Color ballColor = isColliding ? RED : launchedBalls[i].color;
            DrawCircleV(launchedBalls[i].position, launchedBalls[i].radius, ballColor);

            // friction feedback 
            if (onSlope)
            {
                int textX = (int)launchedBalls[i].position.x - 40;
                int textY = (int)launchedBalls[i].position.y + launchedBalls[i].radius + 6;

                DrawText(TextFormat("mu=%.2f", launchedBalls[i].mu), textX, textY, 14, WHITE);
                DrawText(TextFormat("v=%.1f", slopeSpeed), textX, textY + 16, 14,
                    slopeSpeed > 30.0f ? YELLOW : LIGHTGRAY);
            }
        }

       
        EndDrawing();
    }
    CloseWindow();
    return 0;
}