#include "raylib.h"
#include <math.h>
#include <stdio.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#define PI 3.14159265358979323846f
#include <vector>

// PhysicsBody class represents a dynamic object in the simulation
class PhysicsBody {
public:
    Vector2 position;     // Current position in 2D space (x, y)
    Vector2 velocity;     // Current velocity (speed and direction)
    float drag;           // Air resistance (not currently used in calculations)
    float mass;           // Mass of the object (affects force calculations)
    bool active;          // Flag to indicate if the body should be simulated
    float radius;         // Size of the sphere for collision and rendering
    float mu;             // Coefficient of kinetic friction (0 = no friction, 1 = high friction)
    Color color;          // Visual color of the sphere

    // Constructor to initialize the physics body with default values
    PhysicsBody(Vector2 pos, Vector2 vel, float m, float d, float r, float friction = 0.0f, Color c = GREEN)
        : position(pos), velocity(vel), mass(m), drag(d), radius(r), mu(friction), color(c), active(true) {
    }
};

// HalfSpace class represents an infinite plane (like a ground or slope)
class HalfSpace {
public:
    Vector2 point;   // A reference point that lies on the plane
    Vector2 normal;  // Direction perpendicular to the plane (points "outward" from solid side)

    HalfSpace(Vector2 p, Vector2 n) : point(p), normal(n) {}
};

// PhysicsSIM class handles all physics calculations and collision detection
class PhysicsSIM {
public:
    float deltaTime;    // Time elapsed since last update (seconds)
    float time;         // Total simulation time elapsed
    Vector2 gravity;    // Gravity force applied to all bodies (pixels/second²)

    // Constructor with default gravity pointing downward
    PhysicsSIM(float dt = 0.0f, float t = 0.0f, Vector2 grav = { 0, 98.0f })
    {
        deltaTime = dt;
        time = t;
        gravity = grav;
    }

    // Updates a physics body's position and velocity based on gravity
    void Update(PhysicsBody& body, float dt)
    {
        deltaTime = dt; // Store the current frame's time step

        // Apply gravity acceleration to velocity (v = v + a*t)
        body.velocity.x += gravity.x * dt;
        body.velocity.y += gravity.y * dt;

        // Update position based on velocity (p = p + v*t)
        body.position.x += body.velocity.x * dt;
        body.position.y += body.velocity.y * dt;

        time += dt; // Increment total simulation time
    }

    // Checks if two spheres are colliding (distance between centers < sum of radii)
    bool CheckSphereCollision(const PhysicsBody& bodyA, const PhysicsBody& bodyB) {
        // Calculate distance between sphere centers
        float dx = bodyA.position.x - bodyB.position.x;
        float dy = bodyA.position.y - bodyB.position.y;
        float distance = sqrtf(dx * dx + dy * dy);

        // Combined radius is the minimum distance before collision
        float combinedRad = bodyA.radius + bodyB.radius;
        return distance <= combinedRad;
    }

    // Checks if a sphere collides with a horizontal line extending from another sphere
    bool CheckLineCollision(const PhysicsBody& bodyWithLine, const PhysicsBody& targetBody) {
        // Define a horizontal line extending to the right of bodyWithLine
        Vector2 lineStart = { bodyWithLine.position.x + bodyWithLine.radius, bodyWithLine.position.y };
        Vector2 lineEnd = { bodyWithLine.position.x + bodyWithLine.radius * 8, bodyWithLine.position.y };

        // Calculate closest point on line to target sphere
        float dx = targetBody.position.x - lineStart.x;
        float dy = targetBody.position.y - lineStart.y;
        float lineLength = lineEnd.x - lineStart.x;

        // t represents position along the line (0 = start, 1 = end)
        float t = (dx) / lineLength;

        // Clamp t to line segment bounds
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        // Find the closest point on the line
        float closestX = lineStart.x + t * lineLength;
        float closestY = lineStart.y;

        // Calculate distance from target sphere to closest point
        float distX = targetBody.position.x - closestX;
        float distY = targetBody.position.y - closestY;
        float distance = sqrtf(distX * distX + distY * distY);

        return distance <= targetBody.radius;
    }

    // Checks if a sphere collides with a plane (HalfSpace)
    bool CheckSphereHalfSpaceCollision(const PhysicsBody& sphere, const HalfSpace& halfSpace) {
        // Vector from plane point to sphere center
        float dx = sphere.position.x - halfSpace.point.x;
        float dy = sphere.position.y - halfSpace.point.y;

        // Distance from sphere center to plane (dot product with normal)
        // Positive = sphere is on the "outside" of the plane
        float distance = dx * halfSpace.normal.x + dy * halfSpace.normal.y;

        // Collision occurs when distance is less than sphere radius
        return distance <= sphere.radius;
    }

	// Resolves an elastic collision between two spheres, updating their velocities
    void ResolveElasticCollision(PhysicsBody& bodyA, PhysicsBody& bodyB) {
		float dx = bodyB.position.x - bodyA.position.x; // Difference in positions
		float dy = bodyB.position.y - bodyA.position.y; // Difference in positions

		float distance = sqrtf(dx * dx + dy * dy);
		if (distance == 0.0f) return; // Prevent division by zero

		// Normal vector
		float nx = dx / distance;
		float ny = dy / distance;
		// Relative velocity
		float rvx = bodyB.velocity.x - bodyA.velocity.x;
		float rvy = bodyB.velocity.y - bodyA.velocity.y;
		// Velocity along normal
		float velAlongNormal = rvx * nx + rvy * ny;
		if (velAlongNormal > 0) return; // They are moving apart
		// Calculate restitution (elasticity)
		float e = 1.0f; // Perfectly elastic
		// Calculate impulse scalar
		float j = -(1 + e) * velAlongNormal;
		j /= (1 / bodyA.mass) + (1 / bodyB.mass);
		// Apply impulse to both bodies
		bodyA.velocity.x -= (j / bodyA.mass) * nx;
		bodyA.velocity.y -= (j / bodyA.mass) * ny;
		bodyB.velocity.x += (j / bodyB.mass) * nx;
		bodyB.velocity.y += (j / bodyB.mass) * ny;

	}
	// Resolves collision between a sphere and a plane, updating the sphere's velocity
	void ResolvePlaneCollision(PhysicsBody& body, const HalfSpace& halfSpace) { // Reflect velocity off the plane
		float dx = body.position.x - halfSpace.point.x; // Vector from plane point to sphere center
		float dy = body.position.y - halfSpace.point.y ;
		float distance = dx * halfSpace.normal.x + dy * halfSpace.normal.y; // Distance from sphere center to plane
        if (distance < body.radius) {
            // Reflect velocity
			float velAlongNormal = body.velocity.x * halfSpace.normal.x + body.velocity.y * halfSpace.normal.y; // Velocity component along normal
			if (velAlongNormal < 0) { // Only reflect if moving into the plane
                body.velocity.x -= (1 + 1.0f) * velAlongNormal * halfSpace.normal.x; // e=1 for elastic
				body.velocity.y -= (1 + 1.0f) * velAlongNormal * halfSpace.normal.y; 
            }
		}
	}
};

int main(void)
{
    // Initialize window
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "raylib - Angry Bird Launch");
    SetTargetFPS(60); // Target 60 frames per second

    // Launch parameters for projectiles
    Vector2 launchPos = { 100, screenHeight - 100 };  // Starting position
    float launchAngleDeg = 45.0f;                      // Launch angle in degrees
    float launchSpeed = 200.0f;                        // Initial speed

    // Create physics simulation instance
    PhysicsSIM simulation;

    // Vector to store all launched projectile balls
    std::vector<PhysicsBody> launchedBalls;

    // Adjustable inclined plane parameters
    float planeX = screenWidth / 2;
    float planeY = screenHeight / 2;
    float planeAngleDeg = 30.0f; // Angle of the slope

    // Static ball that sits on the plane (for demonstration)
    Vector2 staticBallPos = { planeX, planeY - 20 };
    PhysicsBody staticBall(staticBallPos, { 0,0 }, 1.0f, 0.0f, 10.0f);

    // Free Body Diagram (FBD) display position and size
    Vector2 fbdCenter = { 650, 170 };
    float fbdRadius = 30.0f;

    // Main game loop
    while (!WindowShouldClose())
    {
        // === GUI SLIDERS ===
        // Left side: Launch controls
        GuiSliderBar(Rectangle{ 60, 20, 200, 20 }, "Launch X", NULL, &launchPos.x, 0, screenWidth);
        GuiSliderBar(Rectangle{ 60, 50, 200, 20 }, "Launch Y", NULL, &launchPos.y, 0, screenHeight);
        GuiSliderBar(Rectangle{ 60, 80, 200, 20 }, "Angle", NULL, &launchAngleDeg, 0, 180);
        GuiSliderBar(Rectangle{ 60, 110, 200, 20 }, "Speed", NULL, &launchSpeed, 0, 400);

        // Right side: Gravity controls
        GuiSliderBar(Rectangle{ 520, 20, 200, 20 }, "Gravity X", NULL, &simulation.gravity.x, -200, 200);
        GuiSliderBar(Rectangle{ 520, 50, 200, 20 }, "Gravity Y", NULL, &simulation.gravity.y, -200, 200);

        // Plane controls
        GuiSliderBar(Rectangle{ 60, 140, 200, 20 }, "Plane X", NULL, &planeX, 0, screenWidth);
        GuiSliderBar(Rectangle{ 60, 170, 200, 20 }, "Plane Y", NULL, &planeY, 0, screenHeight);
        GuiSliderBar(Rectangle{ 60, 200, 200, 20 }, "Plane Angle", NULL, &planeAngleDeg, 0, 360);

        // === LAUNCH VELOCITY CALCULATION ===
        // Convert angle to radians and calculate velocity components
        float angleRad = launchAngleDeg * (PI / 180.0f);
        Vector2 velocity = {
            cosf(angleRad) * launchSpeed,   // Horizontal component
            -sinf(angleRad) * launchSpeed   // Vertical component (negative = upward)
        };

        // Calculate end point for launch direction arrow (scaled for visibility)
        Vector2 end = {
            launchPos.x + velocity.x * 0.3f,
            launchPos.y + velocity.y * 0.3f
        };

        // === PLANE SETUP ===
        // Calculate plane normal vector (perpendicular to the plane surface)
        float planeAngleRad = planeAngleDeg * (PI / 180.0f);
        Vector2 planeNormal = {
            cosf(planeAngleRad),  // X component of normal
            sinf(planeAngleRad)   // Y component of normal
        };
        HalfSpace adjustablePlane({ planeX, planeY }, planeNormal);

        // === STATIC BALL PHYSICS (for Free Body Diagram) ===
        // Position static ball on the plane surface
        staticBall.position.x = planeX - planeNormal.x * staticBall.radius;
        staticBall.position.y = planeY - planeNormal.y * staticBall.radius;

        // Calculate forces on static ball
        Vector2 gravityForce = { simulation.gravity.x * staticBall.mass,
                                simulation.gravity.y * staticBall.mass };

        // Normal force: component of gravity perpendicular to plane
        float gravityAlongNormal = gravityForce.x * planeNormal.x + gravityForce.y * planeNormal.y;
        Vector2 normalForce = { -planeNormal.x * gravityAlongNormal,
                               -planeNormal.y * gravityAlongNormal };

        // Friction force: component parallel to plane
        Vector2 frictionForce = { gravityForce.x - normalForce.x,
                                 gravityForce.y - normalForce.y };

        // === UPDATE ALL LAUNCHED BALLS ===
        float frameTime = GetFrameTime(); // Time since last frame
        for (auto& ball : launchedBalls)
        {
            simulation.Update(ball, frameTime); // Apply gravity and update position
        }

        // === KEYBOARD INPUT TO LAUNCH BALLS ===
        // Different keys launch balls with different properties
        if (IsKeyPressed(KEY_ONE)) {
            // Red ball: low mass, low friction
            PhysicsBody newBall(launchPos, velocity, 2.0f, 0.1f, 8.0f, 0.1f, RED);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_TWO)) {
            // Blue ball: high mass, low friction
            PhysicsBody newBall(launchPos, velocity, 8.0f, 0.1f, 8.0f, 0.1f, BLUE);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_THREE)) {
            // Green ball: low mass, high friction
            PhysicsBody newBall(launchPos, velocity, 2.0f, 0.1f, 8.0f, 0.8f, GREEN);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_FOUR)) {
            // Yellow ball: high mass, high friction
            PhysicsBody newBall(launchPos, velocity, 8.0f, 0.1f, 8.0f, 0.8f, YELLOW);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_FIVE)) {
			launchedBalls.clear(); // Remove all existing balls
			PhysicsBody newBall({ 400, 100 }, { 0, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, PURPLE);
			launchedBalls.push_back(newBall);
		}
        if (IsKeyPressed(KEY_SIX)) {
			launchedBalls.clear(); // Remove all existing balls
			PhysicsBody ball1({ 400, 100 }, { 100, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, ORANGE);
			launchedBalls.push_back(ball1);
			PhysicsBody ball2({ 450, 100 }, { 0, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, SKYBLUE);
			launchedBalls.push_back(ball2);
		}
        if (IsKeyPressed(KEY_SPACE))
        {
            // Green ball: default properties
            PhysicsBody newBall(launchPos, velocity, 1.0f, 0.1f, 8.0f, 0.0f, GREEN);
            launchedBalls.push_back(newBall);
        }

        // === RENDERING ===
        BeginDrawing();
        ClearBackground(BLACK);

        // Draw launch position and direction arrow
        DrawCircleV(launchPos, 5, YELLOW);
        DrawLineV(launchPos, end, RED);

        // Display launch parameters as text
        DrawText(TextFormat("X: %.0f", launchPos.x), 280, 20, 20, WHITE);
        DrawText(TextFormat("Y: %.0f", launchPos.y), 280, 50, 20, WHITE);
        DrawText(TextFormat("Angle: %.1f deg", launchAngleDeg), 280, 80, 20, WHITE);
        DrawText(TextFormat("Speed: %.1f", launchSpeed), 280, 110, 20, WHITE);

        // === DRAW THE INCLINED PLANE ===
        float lineLength = 400;
        // Calculate perpendicular direction to draw plane line
        Vector2 perpendicular = { -planeNormal.y, planeNormal.x };
        Vector2 lineStart = { planeX - perpendicular.x * lineLength, planeY - perpendicular.y * lineLength };
        Vector2 lineEnd = { planeX + perpendicular.x * lineLength, planeY + perpendicular.y * lineLength };
        DrawLineV(lineStart, lineEnd, WHITE); // Draw the plane surface

        // Draw normal vector (shows which side is "solid")
        Vector2 normalEnd = { planeX + planeNormal.x * 50, planeY + planeNormal.y * 50 };
        DrawLineV({ planeX, planeY }, normalEnd, ORANGE);
        DrawCircleV({ planeX, planeY }, 5, ORANGE);

        // === DRAW FREE BODY DIAGRAM ===
        DrawCircleV(staticBall.position, staticBall.radius, BLUE); // Static ball on plane

        // Draw FBD circle
        DrawCircleV(fbdCenter, fbdRadius, WHITE);
        DrawCircleV(fbdCenter, fbdRadius - 2, BLACK);
        DrawCircleV(fbdCenter, 3, WHITE); // Center dot

        // Scale factor to make forces visible
        float forceScale = 0.3f;

        // Draw gravity force (red arrow)
        Vector2 gravityEnd = { fbdCenter.x + gravityForce.x * forceScale,
                              fbdCenter.y + gravityForce.y * forceScale };
        DrawLineV(fbdCenter, gravityEnd, RED);
        DrawCircleV(gravityEnd, 2, RED);

        // Draw normal force (green arrow)
        Vector2 normalFEnd = { fbdCenter.x + normalForce.x * forceScale,
                             fbdCenter.y + normalForce.y * forceScale };
        DrawLineV(fbdCenter, normalFEnd, GREEN);
        DrawCircleV(normalFEnd, 2, GREEN);

        // Draw friction force (yellow arrow)
        Vector2 frictionEnd = { fbdCenter.x + frictionForce.x * forceScale,
                               fbdCenter.y + frictionForce.y * forceScale };
        DrawLineV(fbdCenter, frictionEnd, YELLOW);
        DrawCircleV(frictionEnd, 2, YELLOW);

        // Label the FBD forces
        DrawText("FBD", fbdCenter.x - 15, fbdCenter.y - fbdRadius - 20, 16, WHITE);
        DrawText("G", gravityEnd.x + 5, gravityEnd.y, 10, RED);        // Gravity
        DrawText("N", normalFEnd.x + 5, normalFEnd.y, 10, GREEN);      // Normal
        DrawText("F", frictionEnd.x + 5, frictionEnd.y, 10, YELLOW);   // Friction

        // Display net force
        DrawText(TextFormat("Net: (%.1f, %.1f)",
            gravityForce.x + normalForce.x + frictionForce.x,
            gravityForce.y + normalForce.y + frictionForce.y),
            fbdCenter.x - 60, fbdCenter.y + fbdRadius + 10, 14, WHITE);

        // === COLLISION DETECTION AND RESPONSE FOR EACH BALL ===
        for (size_t i = 0; i < launchedBalls.size(); ++i) {
            bool isColliding = false;  // Track if this ball is touching anything
            float slopeSpeed = 0.0f;   // Speed along the slope surface
            bool onSlope = false;      // Is this ball on the slope?

            // Check sphere-to-sphere collisions
            for (size_t j = 0; j < launchedBalls.size(); ++j) {
                if (i != j) {
                    if (simulation.CheckSphereCollision(launchedBalls[i], launchedBalls[j])) {
                        // Calculate collision normal
                        float dx = launchedBalls[i].position.x - launchedBalls[j].position.x;
                        float dy = launchedBalls[i].position.y - launchedBalls[j].position.y;
                        float distance = sqrtf(dx * dx + dy * dy);

                        // Calculate overlap and resolve interpenetration
                        float overlap = (launchedBalls[i].radius + launchedBalls[j].radius) - distance;
                        float nx = dx / distance; // Normalized collision vector
                        float ny = dy / distance;

                        // Push balls apart equally
                        launchedBalls[i].position.x += nx * overlap * 0.5f;
                        launchedBalls[i].position.y += ny * overlap * 0.5f;
                        launchedBalls[j].position.x -= nx * overlap * 0.5f;
                        launchedBalls[j].position.y -= ny * overlap * 0.5f;

						// Resolve collision velocities
						simulation.ResolveElasticCollision(launchedBalls[i], launchedBalls[j]);

                        isColliding = true;
                        break;
                    }
                    // Check line collision (horizontal line from each ball)
                    if (simulation.CheckLineCollision(launchedBalls[i], launchedBalls[j])) {
                        isColliding = true;
                        break;
                    }


                }
            }

            // === FRICTION ON SLOPE ===
            if (!isColliding) {
                if (simulation.CheckSphereHalfSpaceCollision(launchedBalls[i], adjustablePlane)) {
                    // Calculate distance from ball to plane
                    float dx = launchedBalls[i].position.x - adjustablePlane.point.x;
                    float dy = launchedBalls[i].position.y - adjustablePlane.point.y;
                    float distance = dx * adjustablePlane.normal.x + dy * adjustablePlane.normal.y;

                    // If ball has penetrated the plane, push it out
                    if (distance < launchedBalls[i].radius) {
                        float penetration = launchedBalls[i].radius - distance;
                        launchedBalls[i].position.x += adjustablePlane.normal.x * penetration;
                        launchedBalls[i].position.y += adjustablePlane.normal.y * penetration;
                    }

                    // Resolve collision velocities
					simulation.ResolvePlaneCollision(launchedBalls[i], adjustablePlane); // Reflect velocity

                    // Calculate forces on the ball
                    Vector2 gForce = { simulation.gravity.x * launchedBalls[i].mass,
                                       simulation.gravity.y * launchedBalls[i].mass };

                    // Normal force magnitude (projection of gravity onto normal)
                    float N_mag = gForce.x * adjustablePlane.normal.x + gForce.y * adjustablePlane.normal.y;
                    Vector2 normalForce = { -adjustablePlane.normal.x * N_mag,
                                            -adjustablePlane.normal.y * N_mag };

                    // Tangent direction (along the slope)
                    Vector2 tangent = { -adjustablePlane.normal.y, adjustablePlane.normal.x };

                    // Velocity component along the slope
                    float v_tan = launchedBalls[i].velocity.x * tangent.x + launchedBalls[i].velocity.y * tangent.y;
                    onSlope = true;
                    slopeSpeed = fabsf(v_tan); // Store speed for display

                    // Calculate kinetic friction force
                    float frictionMax = launchedBalls[i].mu * fabsf(N_mag);
                    Vector2 frictionForce = { 0 };

                    // Apply friction opposite to motion direction
                    if (fabsf(v_tan) > 0.01f) {
                        // Determine friction direction (opposite to velocity)
                        Vector2 fDir = (v_tan > 0) ? Vector2{ -tangent.x, -tangent.y } : tangent;
                        frictionForce = Vector2{ fDir.x * frictionMax, fDir.y * frictionMax };
                    }

                    // Calculate net force (gravity + normal + friction)
                    Vector2 netForce = {
                        gForce.x + normalForce.x + frictionForce.x,
                        gForce.y + normalForce.y + frictionForce.y
                    };

                    // Calculate acceleration (F = ma, so a = F/m)
                    Vector2 accel = {
                        netForce.x / launchedBalls[i].mass,
                        netForce.y / launchedBalls[i].mass
                    };

                    // Update velocity based on acceleration
                    launchedBalls[i].velocity.x += accel.x * frameTime;
                    launchedBalls[i].velocity.y += accel.y * frameTime;

                    // Apply damping when moving slowly (simulates coming to rest)
                    if (fabsf(v_tan) < 10.0f) {
                        launchedBalls[i].velocity.x *= 0.98f;
                        launchedBalls[i].velocity.y *= 0.98f;
                    }

                    isColliding = true;
                }

                

                
                
            }

            // Draw horizontal line extending from each ball (collision visualization)
            Vector2 lineStart = { launchedBalls[i].position.x + launchedBalls[i].radius, launchedBalls[i].position.y };
            Vector2 lineEnd = { launchedBalls[i].position.x + launchedBalls[i].radius * 8, launchedBalls[i].position.y };
            DrawLineV(lineStart, lineEnd, PURPLE);

            // Draw ball (red if colliding, otherwise original color)
            Color ballColor = isColliding ? RED : launchedBalls[i].color;
            DrawCircleV(launchedBalls[i].position, launchedBalls[i].radius, ballColor);

            // Display friction coefficient and speed when on slope
            if (onSlope)
            {
                int textX = (int)launchedBalls[i].position.x - 40;
                int textY = (int)launchedBalls[i].position.y + launchedBalls[i].radius + 6;

                DrawText(TextFormat("mu=%.2f", launchedBalls[i].mu), textX, textY, 14, WHITE);

                // Color speed text based on magnitude
                DrawText(TextFormat("v=%.1f", slopeSpeed), textX, textY + 16, 14,
                    slopeSpeed > 30.0f ? YELLOW : LIGHTGRAY);
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}