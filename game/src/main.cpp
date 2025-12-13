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

// AABB class represents an Axis-Aligned Bounding Box (rectangle)
class AABB {
public:
    Vector2 position;     // Center position of the box
    Vector2 velocity;     // Current velocity
    float width;          // Width of the box
    float height;         // Height of the box
    float mass;           // Mass of the box
    bool isFixed;         // If true, box doesn't move (infinite mass)
    Color color;          // Visual color

    AABB(Vector2 pos, float w, float h, float m, bool fixed = false, Color c = BROWN)
        : position(pos), velocity({ 0, 0 }), width(w), height(h), mass(m), isFixed(fixed), color(c) {
	} // Constructor

    // Get min/max coordinates
	float GetMinX() const { return position.x - width / 2; } // Minimum X coordinate
	float GetMaxX() const { return position.x + width / 2; } // Maximum X coordinate
	float GetMinY() const { return position.y - height / 2; } // Minimum Y coordinate
	float GetMaxY() const { return position.y + height / 2; } // Maximum Y coordinate
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

    // Updates an AABB's position and velocity based on gravity
    void UpdateAABB(AABB& box, float dt)
    {
        if (box.isFixed) return; // Fixed boxes don't move

        // Apply gravity acceleration to velocity
        box.velocity.x += gravity.x * dt;
        box.velocity.y += gravity.y * dt;

        // Update position based on velocity
        box.position.x += box.velocity.x * dt;
        box.position.y += box.velocity.y * dt;
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

    // Check if a sphere collides with an AABB
    bool CheckSphereAABBCollision(const PhysicsBody& sphere, const AABB& box) {
        // Find the closest point on the AABB to the sphere center
		float closestX = fmaxf(box.GetMinX(), fminf(sphere.position.x, box.GetMaxX())); // Clamp sphere x to box bounds
		float closestY = fmaxf(box.GetMinY(), fminf(sphere.position.y, box.GetMaxY())); // Clamp sphere y to box bounds

        // Calculate distance from sphere center to closest point
		float dx = sphere.position.x - closestX; // Difference in x
		float dy = sphere.position.y - closestY; // Difference in y
		float distanceSquared = dx * dx + dy * dy; // Squared distance

		return distanceSquared < (sphere.radius * sphere.radius); // Collision if within radius squared
    }

    // Resolve collision between sphere and AABB
    void ResolveSphereAABBCollision(PhysicsBody& sphere, AABB& box) {
        // Find the closest point on the AABB to the sphere center
		float closestX = fmaxf(box.GetMinX(), fminf(sphere.position.x, box.GetMaxX())); // Clamp sphere x to box bounds
		float closestY = fmaxf(box.GetMinY(), fminf(sphere.position.y, box.GetMaxY())); // Clamp sphere y to box bounds

        // Calculate collision normal
		float dx = sphere.position.x - closestX; // Difference in x
		float dy = sphere.position.y - closestY; // Difference in y
		float distance = sqrtf(dx * dx + dy * dy); // Distance from sphere center to closest point

        if (distance == 0.0f) {
            // Sphere center is inside AABB, push it out
			dx = sphere.position.x - box.position.x; // Difference in x
			dy = sphere.position.y - box.position.y; // Difference in y
			distance = sqrtf(dx * dx + dy * dy); // Distance
			if (distance == 0.0f) return; // Prevent division by zero
        }

        // Normalize the collision normal
		float nx = dx / distance; // Normal x component
		float ny = dy / distance; // Normal y component

        // Calculate penetration depth
		float penetration = sphere.radius - distance; // How much the sphere is penetrating the box
         
        // Separate the objects
        if (box.isFixed) {
            // Only move the sphere
			sphere.position.x += nx * penetration; // Push sphere out along normal
			sphere.position.y += ny * penetration; // Push sphere out along normal
        }
        else {
            // Move both based on mass ratio
			float totalMass = sphere.mass + box.mass; // Total mass
			float sphereRatio = box.mass / totalMass; // Sphere movement ratio
			float boxRatio = sphere.mass / totalMass; // Box movement ratio

			sphere.position.x += nx * penetration * sphereRatio; // Push sphere out
			sphere.position.y += ny * penetration * sphereRatio; // Push sphere out
			box.position.x -= nx * penetration * boxRatio; // Push box out
			box.position.y -= ny * penetration * boxRatio; // Push box out
        }

        // Calculate relative velocity
		float rvx = sphere.velocity.x - box.velocity.x; // Relative velocity x
		float rvy = sphere.velocity.y - box.velocity.y; // Relative velocity y

        // Velocity along normal
		float velAlongNormal = rvx * nx + rvy * ny; // Dot product

        // Don't resolve if velocities are separating
        if (velAlongNormal > 0) return;

        // Calculate restitution (bounciness)
        float e = 0.6f; // Coefficient of restitution

        // Calculate impulse scalar
        float j = -(1 + e) * velAlongNormal;
        if (box.isFixed) {
            j /= (1 / sphere.mass);
        }
        else {
            j /= (1 / sphere.mass) + (1 / box.mass);
        }

        // Apply impulse
        sphere.velocity.x -= (j / sphere.mass) * nx;
        sphere.velocity.y -= (j / sphere.mass) * ny;

        if (!box.isFixed) {
            box.velocity.x += (j / box.mass) * nx;
            box.velocity.y += (j / box.mass) * ny;
        }
    }

    // Check AABB-AABB collision
    bool CheckAABBCollision(const AABB& boxA, const AABB& boxB) {
		return (boxA.GetMinX() <= boxB.GetMaxX() && // Check overlap on X axis
			boxA.GetMaxX() >= boxB.GetMinX() && // Check overlap on X axis
			boxA.GetMinY() <= boxB.GetMaxY() && // Check overlap on Y axis
			boxA.GetMaxY() >= boxB.GetMinY()); // Check overlap on Y axis
    }

    // Resolve AABB-AABB collision
    void ResolveAABBCollision(AABB& boxA, AABB& boxB) {
        if (boxA.isFixed && boxB.isFixed) return;

        // Calculate overlap on each axis
        float overlapX = fminf(boxA.GetMaxX() - boxB.GetMinX(), boxB.GetMaxX() - boxA.GetMinX());
        float overlapY = fminf(boxA.GetMaxY() - boxB.GetMinY(), boxB.GetMaxY() - boxA.GetMinY());

        // Find the axis of least penetration
        Vector2 normal;
        float penetration;

        if (overlapX < overlapY) {
            penetration = overlapX;
            normal.x = (boxA.position.x < boxB.position.x) ? -1.0f : 1.0f;
            normal.y = 0;
        }
        else {
            penetration = overlapY;
            normal.x = 0;
            normal.y = (boxA.position.y < boxB.position.y) ? -1.0f : 1.0f;
        }

        // Separate the boxes
        if (boxA.isFixed) {
            boxB.position.x -= normal.x * penetration;
            boxB.position.y -= normal.y * penetration;
        }
        else if (boxB.isFixed) {
            boxA.position.x += normal.x * penetration;
            boxA.position.y += normal.y * penetration;
        }
        else {
            float totalMass = boxA.mass + boxB.mass;
            float ratioA = boxB.mass / totalMass;
            float ratioB = boxA.mass / totalMass;

            boxA.position.x += normal.x * penetration * ratioA;
            boxA.position.y += normal.y * penetration * ratioA;
            boxB.position.x -= normal.x * penetration * ratioB;
            boxB.position.y -= normal.y * penetration * ratioB;
        }

        // Calculate relative velocity
        float rvx = boxA.velocity.x - boxB.velocity.x;
        float rvy = boxA.velocity.y - boxB.velocity.y;

        // Velocity along normal
        float velAlongNormal = rvx * normal.x + rvy * normal.y;

        if (velAlongNormal > 0) return;

        // Calculate restitution
        float e = 0.3f;

        // Calculate impulse scalar
        float j = -(1 + e) * velAlongNormal;
        if (boxA.isFixed) {
            j /= (1 / boxB.mass);
            boxB.velocity.x -= (j / boxB.mass) * normal.x;
            boxB.velocity.y -= (j / boxB.mass) * normal.y;
        }
        else if (boxB.isFixed) {
            j /= (1 / boxA.mass);
            boxA.velocity.x += (j / boxA.mass) * normal.x;
            boxA.velocity.y += (j / boxA.mass) * normal.y;
        }
        else {
            j /= (1 / boxA.mass) + (1 / boxB.mass);
            boxA.velocity.x += (j / boxA.mass) * normal.x;
            boxA.velocity.y += (j / boxA.mass) * normal.y;
            boxB.velocity.x -= (j / boxB.mass) * normal.x;
            boxB.velocity.y -= (j / boxB.mass) * normal.y;
        }
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
    void ResolvePlaneCollision(PhysicsBody& body, const HalfSpace& halfSpace) {
        // Calculate velocity component along normal
        float velAlongNormal = body.velocity.x * halfSpace.normal.x + body.velocity.y * halfSpace.normal.y;

        // Only reflect if moving into the plane
        if (velAlongNormal < 0) {
            float e = 1.0f; // Coefficient of restitution (1.0 = perfectly elastic)
            // Reflect and scale by restitution
            body.velocity.x -= (1 + e) * velAlongNormal * halfSpace.normal.x;
            body.velocity.y -= (1 + e) * velAlongNormal * halfSpace.normal.y;
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

    // Vector to store all AABBs
    std::vector<AABB> aabbBoxes;

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

    int currentScenario = 0; // 0 = free play, 5 = bouncy ball, 6 = pool, 7 = cannon, 8 = tower
    float ball1Mass = 2.0f;
    float ball2Mass = 2.0f;
    float ball1VelX = 0.0f;
    float ball1VelY = 0.0f;
    float ball2VelX = 0.0f;
    float ball2VelY = 0.0f;

    // Main game loop
    while (!WindowShouldClose())
    {
        // === GUI SLIDERS ===
        if (currentScenario == 0) {

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
        }
        else
        {
            // SCENARIO MODE - Ball parameter sliders
            DrawText("SCENARIO MODE", 60, 20, 20, YELLOW);
            if (currentScenario == 5) {
                DrawText("Scenario 5: Bouncy Ball", 60, 45, 16, WHITE);
                GuiSliderBar(Rectangle{ 60, 70, 200, 20 }, "Ball Mass", NULL, &ball1Mass, 0.5f, 10.0f);
            }
            else if (currentScenario == 6) {
                DrawText("Scenario 6: Pool Table", 60, 45, 16, WHITE);
                GuiSliderBar(Rectangle{ 60, 70, 200, 20 }, "Ball 1 Mass", NULL, &ball1Mass, 0.5f, 10.0f);
                GuiSliderBar(Rectangle{ 60, 95, 200, 20 }, "Ball 1 Vel X", NULL, &ball1VelX, 0, 300);
                GuiSliderBar(Rectangle{ 60, 120, 200, 20 }, "Ball 2 Mass", NULL, &ball2Mass, 0.5f, 10.0f);
            }
            else if (currentScenario == 7) {
                DrawText("Scenario 7: Galilean Cannon", 60, 45, 16, WHITE);
                GuiSliderBar(Rectangle{ 60, 70, 200, 20 }, "Bottom Mass", NULL, &ball1Mass, 0.5f, 10.0f);
                GuiSliderBar(Rectangle{ 60, 95, 200, 20 }, "Top Mass", NULL, &ball2Mass, 0.5f, 10.0f);
            }
            else if (currentScenario == 8) {
                DrawText("Scenario 8: AABB Tower", 60, 45, 16, WHITE);
                DrawText("Press SPACE to launch!", 60, 70, 14, LIGHTGRAY);
                GuiSliderBar(Rectangle{ 60, 95, 200, 20 }, "Launch Angle", NULL, &launchAngleDeg, 0, 90);
                GuiSliderBar(Rectangle{ 60, 120, 200, 20 }, "Launch Speed", NULL, &launchSpeed, 100, 500);
            }

            // Reset button for scenarios
            if (GuiButton(Rectangle{ 60, 160, 200, 30 }, "Reset Scenario")) {
                // Re-trigger the current scenario
                if (currentScenario == 5) {
                    launchedBalls.clear();
                    planeAngleDeg = 270.0f;
                    planeY = screenHeight - 50;
                    planeX = screenWidth / 2;
                    simulation.gravity = { 0, 98.0f };
                    PhysicsBody newBall({ 400, 100 }, { 0, 0 }, ball1Mass, 0.1f, 8.0f, 0.0f, PURPLE);
                    launchedBalls.push_back(newBall);
                }
                else if (currentScenario == 6) {
                    launchedBalls.clear();
                    planeAngleDeg = 270.0f;
                    planeY = 250;
                    planeX = screenWidth / 2;
                    simulation.gravity = { 0, 98.0f };
                    PhysicsBody ball1({ 200, 240 }, { ball1VelX, 0 }, ball1Mass, 0.1f, 8.0f, 0.0f, ORANGE);
                    launchedBalls.push_back(ball1);
                    PhysicsBody ball2({ 500, 240 }, { 0, 0 }, ball2Mass, 0.1f, 8.0f, 0.0f, SKYBLUE);
                    launchedBalls.push_back(ball2);
                }
                else if (currentScenario == 7) {
                    launchedBalls.clear();
                    planeAngleDeg = 270.0f;
                    planeY = screenHeight - 50;
                    planeX = screenWidth / 2;
                    simulation.gravity = { 0, 98.0f };
                    PhysicsBody ball1({ 400, 200 }, { 0, 0 }, ball1Mass, 0.1f, 8.0f, 0.0f, DARKGREEN);
                    PhysicsBody ball2({ 400, 184 }, { 0, 0 }, ball2Mass, 0.1f, 8.0f, 0.0f, BROWN);
                    launchedBalls.push_back(ball1);
                    launchedBalls.push_back(ball2);
                }
                else if (currentScenario == 8) {
                    // Reset tower scenario
                    launchedBalls.clear();
                    aabbBoxes.clear();

                    // Set up ground
                    planeAngleDeg = 270.0f;
                    planeY = screenHeight - 30;
                    planeX = screenWidth / 2;
                    simulation.gravity = { 0, 98.0f };

                    // Reset launch position for tower scenario
                    launchPos = { 100, screenHeight - 100 };
                    launchAngleDeg = 30.0f;
                    launchSpeed = 300.0f;

                    // Build tower of AABBs
                    float boxWidth = 40;
                    float boxHeight = 30;
                    float towerX = 600;
                    float groundY = screenHeight - 30;

                    // Bottom box (fixed)
                    AABB bottomBox({ towerX, groundY - boxHeight / 2 }, boxWidth, boxHeight, 5.0f, true, DARKBROWN);
                    aabbBoxes.push_back(bottomBox);

                    // Stack 5 movable boxes on top
                    for (int i = 1; i <= 5; i++) {
                        AABB box({ towerX, groundY - boxHeight / 2 - boxHeight * i },
                            boxWidth, boxHeight, 2.0f, false, BROWN);
                        aabbBoxes.push_back(box);
                    }
                }
            }

            // Exit scenario button
            if (GuiButton(Rectangle{ 60, 200, 200, 30 }, "Exit to Free Play")) {
                currentScenario = 0;
                launchedBalls.clear();
                aabbBoxes.clear();
            }

        }
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

        // === UPDATE ALL AABBs ===
        for (auto& box : aabbBoxes)
        {
			simulation.UpdateAABB(box, frameTime); // Apply gravity and update position
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
            currentScenario = 5;
            launchedBalls.clear(); // Remove all existing balls
            aabbBoxes.clear();

            // Set up plane: horizontal at bottom of screen
            planeAngleDeg = 270.0f;
            planeY = screenHeight - 50;
            planeX = screenWidth / 2;

            // Reset gravity to normal
            simulation.gravity = { 0, 98.0f };

            // Scenario 1: Bouncy ball - single ball drops and bounces
            PhysicsBody newBall({ 400, 100 }, { 0, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, PURPLE);
            launchedBalls.push_back(newBall);
        }
        if (IsKeyPressed(KEY_SIX)) {
            currentScenario = 6;
            launchedBalls.clear(); // Remove all existing balls
            aabbBoxes.clear();

            // Set up plane: horizontal in middle of screen
            planeAngleDeg = 270.0f;
            planeY = 250;
            planeX = screenWidth / 2;

            // Reset gravity to normal
            simulation.gravity = { 0, 98.0f };
            // Scenario 2: Pool table - moving ball hits stationary ball
            PhysicsBody ball1({ 300, 100 }, { 150, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, ORANGE);
            launchedBalls.push_back(ball1);
            PhysicsBody ball2({ 500, 100 }, { 0, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, SKYBLUE);
            launchedBalls.push_back(ball2);
        }
        if (IsKeyPressed(KEY_SEVEN)) {
            currentScenario = 7;
            launchedBalls.clear(); // Remove all existing balls
            aabbBoxes.clear();

            // Set up plane: horizontal at bottom of screen
            planeAngleDeg = 270.0f;
            planeY = screenHeight - 50;
            planeX = screenWidth / 2;

            // Reset gravity to normal
            simulation.gravity = { 0, 98.0f };

            // Scenario 3: Galilean Cannon - stacked balls fall and collide
            PhysicsBody ball1({ 400, 300 }, { 0, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, DARKGREEN);
            PhysicsBody ball2({ 400, 284 }, { 0, 0 }, 2.0f, 0.1f, 8.0f, 0.0f, BROWN);
            launchedBalls.push_back(ball1);
            launchedBalls.push_back(ball2);
        }
        if (IsKeyPressed(KEY_EIGHT)) {
            currentScenario = 8;
            launchedBalls.clear(); // Remove all existing balls
            aabbBoxes.clear();

            // Set up ground
            planeAngleDeg = 270.0f;
            planeY = screenHeight - 30;
            planeX = screenWidth / 2;

            // Reset gravity to normal
            simulation.gravity = { 0, 98.0f };

            // Set launch position for tower scenario
            launchPos = { 100, screenHeight - 100 };
            launchAngleDeg = 30.0f;
            launchSpeed = 300.0f;

            // Build tower of AABBs
            float boxWidth = 40;
            float boxHeight = 30;
            float towerX = 600;
            float groundY = screenHeight - 30;

            // Bottom box (fixed in place)
            AABB bottomBox({ towerX, groundY - boxHeight / 2 }, boxWidth, boxHeight, 5.0f, true, DARKBROWN);
            aabbBoxes.push_back(bottomBox);

            // Stack 5 movable boxes on top
            for (int i = 1; i <= 5; i++) {
                AABB box({ towerX, groundY - boxHeight / 2 - boxHeight * i },
                    boxWidth, boxHeight, 2.0f, false, BROWN);
                aabbBoxes.push_back(box);
            }
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
        if (currentScenario != 8) {
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
        }

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

                        // Apply elastic collision response to update velocities
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

            // Check sphere-AABB collisions
            for (size_t j = 0; j < aabbBoxes.size(); ++j) {
                if (simulation.CheckSphereAABBCollision(launchedBalls[i], aabbBoxes[j])) {
                    simulation.ResolveSphereAABBCollision(launchedBalls[i], aabbBoxes[j]);
                    isColliding = true;
                }
            }

            // === PLANE COLLISION AND FRICTION ===

            {
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

                    // Apply elastic bounce (works for all balls)
                    simulation.ResolvePlaneCollision(launchedBalls[i], adjustablePlane);

                    // Only apply friction forces if ball has friction coefficient > 0
                    if (launchedBalls[i].mu > 0.0f) {
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
                    }

                    isColliding = true;
                }
            }

            // Draw horizontal line extending from each ball (collision visualization)
            if (currentScenario != 8) {
                Vector2 lineStart = { launchedBalls[i].position.x + launchedBalls[i].radius, launchedBalls[i].position.y };
                Vector2 lineEnd = { launchedBalls[i].position.x + launchedBalls[i].radius * 8, launchedBalls[i].position.y };
                DrawLineV(lineStart, lineEnd, PURPLE);
            }

            // Draw ball (red if colliding, otherwise original color)
            Color ballColor = isColliding ? RED : launchedBalls[i].color;
            DrawCircleV(launchedBalls[i].position, launchedBalls[i].radius, ballColor);

            // Display friction coefficient and speed when on slope
            if (onSlope && currentScenario != 8)
            {
                int textX = (int)launchedBalls[i].position.x - 40;
                int textY = (int)launchedBalls[i].position.y + launchedBalls[i].radius + 6;

                DrawText(TextFormat("mu=%.2f", launchedBalls[i].mu), textX, textY, 14, WHITE);

                // Color speed text based on magnitude
                DrawText(TextFormat("v=%.1f", slopeSpeed), textX, textY + 16, 14,
                    slopeSpeed > 30.0f ? YELLOW : LIGHTGRAY);
            }
        }

        // === AABB-AABB COLLISION DETECTION ===
        for (size_t i = 0; i < aabbBoxes.size(); ++i) {
            for (size_t j = i + 1; j < aabbBoxes.size(); ++j) {
                if (simulation.CheckAABBCollision(aabbBoxes[i], aabbBoxes[j])) {
                    simulation.ResolveAABBCollision(aabbBoxes[i], aabbBoxes[j]);
                }
            }
        }

        // === DRAW ALL AABBs ===
        for (const auto& box : aabbBoxes) {
            Rectangle rect = {
                box.GetMinX(),
                box.GetMinY(),
                box.width,
                box.height
            };
            DrawRectangleRec(rect, box.color);
            DrawRectangleLinesEx(rect, 2, box.isFixed ? RED : DARKGRAY);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}