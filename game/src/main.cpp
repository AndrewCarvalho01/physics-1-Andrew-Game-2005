#include "raylib.h"
#include <math.h>
#include <stdio.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#define PI 3.14159265358979323846f
#include <vector>

// Enum for bird types
enum BirdType {
	BIRD_CIRCLE, // Default bird type
	BIRD_SQUARE, // For block-like birds
	BIRD_RECTANGLE // For wide rectangle blocks (roof)
};

// PhysicsBody class represents a dynamic object in the simulation
class PhysicsBody {
public:
    Vector2 position; // Center position
    Vector2 velocity; // Velocity vector
    float drag; // Drag coefficient
    float mass; // Mass
    bool active; // Is the body active in simulation
    float radius; // For circle: radius; For square: half-size; For rectangle: half-width
    float mu; // Friction coefficient
    Color color; // Color for rendering

    BirdType birdType;  // Track what type of bird this is
    bool activePhysics; // Only true if physics should be applied
    bool isPig = false;       // Identify if this body is a pig
    float toughness = 0.0f;   // Threshold momentum to survive
    bool destroyed = false;   // Marks pig as dead

	PhysicsBody(Vector2 pos, Vector2 vel, float m, float d, float r, float friction = 0.0f, Color c = GREEN, BirdType type = BIRD_CIRCLE) // Constructor
		: position(pos), velocity(vel), mass(m), drag(d), radius(r), mu(friction), color(c), active(true), activePhysics(true), birdType(type) { // Initialize all properties
        if (type == BIRD_SQUARE || type == BIRD_RECTANGLE) activePhysics = false; // Fort blocks start inactive
    }
};

// HalfSpace class represents an infinite plane
class HalfSpace {
public:
    Vector2 point; // A point on the plane
    Vector2 normal; // Normal vector of the plane
    HalfSpace(Vector2 p, Vector2 n) : point(p), normal(n) {} // Constructor
};

// PhysicsSIM class handles all physics calculations
class PhysicsSIM {
public:
    float deltaTime; // Time step
    float time; // Total simulation time
    Vector2 gravity; // Gravity vector

    PhysicsSIM(float dt = 0.0f, float t = 0.0f, Vector2 grav = { 0, 98.0f }) // Gravity pointing down
    {
		deltaTime = dt; // Initialize time step
		time = t; // Initialize total time 
		gravity = grav; // Initialize gravity
    }

    void Update(PhysicsBody& body, float dt)
    {
		deltaTime = dt;  // Update time step
		body.velocity.x += gravity.x * dt; // Apply gravity
		body.velocity.y += gravity.y * dt; // Apply gravity
		body.position.x += body.velocity.x * dt; // Update position
		body.position.y += body.velocity.y * dt; // Update position
		time += dt; // Update total time
    }

    bool CheckSphereCollision(const PhysicsBody& bodyA, const PhysicsBody& bodyB) {
		float dx = bodyA.position.x - bodyB.position.x; // Delta x
		float dy = bodyA.position.y - bodyB.position.y; // Delta y
		float distance = sqrtf(dx * dx + dy * dy); // Distance between centers
		float combinedRad = bodyA.radius + bodyB.radius; // Sum of radii
		return distance <= combinedRad; // Check collision
    }

    // Check AABB (square) collision
    bool CheckAABBCollision(const PhysicsBody& bodyA, const PhysicsBody& bodyB) {
		float halfSizeA = bodyA.radius; // Half-size of body A
		float halfSizeB = bodyB.radius; // Half-size of body B

		bool xOverlap = fabsf(bodyA.position.x - bodyB.position.x) < (halfSizeA + halfSizeB); // Check x overlap
		bool yOverlap = fabsf(bodyA.position.y - bodyB.position.y) < (halfSizeA + halfSizeB); // Check y overlap

		return xOverlap && yOverlap; // Return collision result
    }

    // Check rectangle collision (wide rectangles)
    bool CheckRectangleCollision(const PhysicsBody& bodyA, const PhysicsBody& bodyB) {
		float halfWidthA = bodyA.radius; // Half-width of body A
		float halfHeightA = (bodyA.birdType == BIRD_RECTANGLE) ? bodyA.radius / 4.0f : bodyA.radius; // Half-height of body A

		float halfWidthB = bodyB.radius; // Half-width of body B
		float halfHeightB = (bodyB.birdType == BIRD_RECTANGLE) ? bodyB.radius / 4.0f : bodyB.radius; // Half-height of body B

		bool xOverlap = fabsf(bodyA.position.x - bodyB.position.x) < (halfWidthA + halfWidthB); // Check x overlap
		bool yOverlap = fabsf(bodyA.position.y - bodyB.position.y) < (halfHeightA + halfHeightB); // Check y overlap

		return xOverlap && yOverlap; // Return collision result
    }

    // Check collision between any two bodies (handles different shapes)
    bool CheckCollision(const PhysicsBody& bodyA, const PhysicsBody& bodyB) {
        // Handle rectangle collisions
        if (bodyA.birdType == BIRD_RECTANGLE || bodyB.birdType == BIRD_RECTANGLE) {
			return CheckRectangleCollision(bodyA, bodyB); // Use rectangle collision check
        }

        if (bodyA.birdType == BIRD_CIRCLE && bodyB.birdType == BIRD_CIRCLE) {
			return CheckSphereCollision(bodyA, bodyB); // Circle vs Circle
        }
        else if (bodyA.birdType == BIRD_SQUARE && bodyB.birdType == BIRD_SQUARE) {
			return CheckAABBCollision(bodyA, bodyB); // Square vs Square
        }
        else {
            // Circle vs Square - simplified check
			return CheckSphereCollision(bodyA, bodyB); // Use sphere collision for simplicity
        }
    }

    bool CheckSphereHalfSpaceCollision(const PhysicsBody& sphere, const HalfSpace& halfSpace) {
		float dx = sphere.position.x - halfSpace.point.x; // Delta x
		float dy = sphere.position.y - halfSpace.point.y; // Delta y
		float distance = dx * halfSpace.normal.x + dy * halfSpace.normal.y; // Distance from plane

        // Use smaller radius for rectangle height
		float effectiveRadius = (sphere.birdType == BIRD_RECTANGLE) ? sphere.radius / 4.0f : sphere.radius; // Effective radius

		return distance <= effectiveRadius; // Check collision
    }

    void ResolveElasticCollision(PhysicsBody& bodyA, PhysicsBody& bodyB) { 
		float dx = bodyB.position.x - bodyA.position.x; // Delta x
		float dy = bodyB.position.y - bodyA.position.y; // Delta y
		float distance = sqrtf(dx * dx + dy * dy); // Distance between centers
		if (distance == 0.0f) return; // Prevent division by zero

		float nx = dx / distance; // Normal x
		float ny = dy / distance; // Normal y
		float rvx = bodyB.velocity.x - bodyA.velocity.x; // Relative velocity x
		float rvy = bodyB.velocity.y - bodyA.velocity.y; // Relative velocity y
		float velAlongNormal = rvx * nx + rvy * ny; // Velocity along normal
		if (velAlongNormal > 0) return; // Bodies are separating

		float e = 0.8f; // Coefficient of restitution
		float j = -(1 + e) * velAlongNormal; // Impulse scalar
		j /= (1 / bodyA.mass) + (1 / bodyB.mass); // Divide by mass sum

		bodyA.velocity.x -= (j / bodyA.mass) * nx; // Update velocity A
		bodyA.velocity.y -= (j / bodyA.mass) * ny; // Update velocity A
		bodyB.velocity.x += (j / bodyB.mass) * nx; // Update velocity B
		bodyB.velocity.y += (j / bodyB.mass) * ny; // Update velocity B
    }

    void ResolvePlaneCollision(PhysicsBody& body, const HalfSpace& halfSpace) {
		float velAlongNormal = body.velocity.x * halfSpace.normal.x + body.velocity.y * halfSpace.normal.y; // Velocity along normal
		if (velAlongNormal < 0) // Only resolve if moving towards plane
        {
            float e = 0.6f;
			body.velocity.x -= (1 + e) * velAlongNormal * halfSpace.normal.x; // Reflect velocity 
			body.velocity.y -= (1 + e) * velAlongNormal * halfSpace.normal.y; // Reflect velocity
        }
    }
};

int main(void) // Entry point
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Angry Birds - Final Project");
    SetTargetFPS(60);
    bool fortTriggered = false;

    // === SLINGSHOT SETUP ===
	Vector2 slingshotBase = { 100, screenHeight - 100 }; // Base position
	float maxPullDistance = 100.0f; // Max pull distance
	float launchForceMultiplier = 3.0f; // Launch force multiplier

	bool isDragging = false; // Is the slingshot being dragged
	Vector2 dragPosition = slingshotBase; // Current drag position

    // === BIRD SELECTION ===
	BirdType selectedBirdType = BIRD_CIRCLE; // Default bird type

    // Bird properties
    float circleBirdMass = 2.0f;
    float circleBirdRadius = 10.0f;
    float circleBirdFriction = 0.3f;
    Color circleBirdColor = RED;

    float squareBirdMass = 8.0f;
    float squareBirdRadius = 12.0f;
    float squareBirdFriction = 0.5f;
    Color squareBirdColor = BLUE;

    // Physics simulation
    PhysicsSIM simulation;
	std::vector<PhysicsBody> launchedBirds; // Launched birds
	std::vector<PhysicsBody> fortBlocks; // Fort blocks
	std::vector<PhysicsBody> piggies; // Piggies

    // === CREATE FORT STRUCTURE (PROPER SQUARE + ROOF) ===
	float fortX = 520.0f; // Center X position of the fort
	float fortY = screenHeight - 50; // Base Y position of the fort
	float blockSize = 15.0f; // Size of each block

    // --- LEFT WALL: 3 blocks ---
    fortBlocks.push_back(PhysicsBody({ fortX - 70, fortY - blockSize }, { 0,0 }, 3, 0, blockSize, 2, BROWN, BIRD_SQUARE));
    fortBlocks.back().activePhysics = false;

    fortBlocks.push_back(PhysicsBody({ fortX - 70, fortY - 3 * blockSize }, { 0,0 }, 3, 0, blockSize, 2, BROWN, BIRD_SQUARE));
    fortBlocks.back().activePhysics = false;

    fortBlocks.push_back(PhysicsBody({ fortX - 70, fortY - 5 * blockSize }, { 0,0 }, 3, 0, blockSize, 2, BROWN, BIRD_SQUARE));
    fortBlocks.back().activePhysics = false;

    // --- RIGHT WALL: 3 blocks ---
    fortBlocks.push_back(PhysicsBody({ fortX + 45, fortY - blockSize }, { 0,0 }, 3, 0, blockSize, 2, BROWN, BIRD_SQUARE));
    fortBlocks.back().activePhysics = false;
    fortBlocks.push_back(PhysicsBody({ fortX + 45, fortY - 3 * blockSize }, { 0,0 }, 3, 0, blockSize, 2, BROWN, BIRD_SQUARE));
    fortBlocks.back().activePhysics = false;
    fortBlocks.push_back(PhysicsBody({ fortX + 45, fortY - 5 * blockSize }, { 0,0 }, 3, 0, blockSize, 2, BROWN, BIRD_SQUARE));
    fortBlocks.back().activePhysics = false;

    // --- ROOF: long rectangle on top ---
    fortBlocks.push_back(PhysicsBody({ fortX - 10, fortY - 10 * blockSize + 45 }, { 0,0 }, 5, 0, blockSize * 4, 2, DARKBROWN, BIRD_RECTANGLE));
    fortBlocks.back().activePhysics = false;

    // === PIGGIES ===
	float pigRadius = 12.0f; // Pig radius
	float pigYInside = fortY - pigRadius; // Y position for pigs inside the fort
	float pigXOffset = 20.0f; // X offset from center for pigs

    // --- Piggies inside the fort ---
    piggies.push_back(PhysicsBody({ fortX - pigXOffset, pigYInside }, { 0,0 }, 2.5f, 0, pigRadius, 0, GREEN, BIRD_CIRCLE));
	piggies.back().activePhysics = false; // Disable physics initially
	piggies.back().isPig = true; // Mark as pig
	piggies.back().toughness = 150.0f; // Set toughness

    piggies.push_back(PhysicsBody({ fortX + pigXOffset, pigYInside },{ 0,0 }, 2.5f, 0, pigRadius, 0, LIME, BIRD_CIRCLE));
    piggies.back().activePhysics = false;
    piggies.back().isPig = true;
    piggies.back().toughness = 150.0f;

    // --- Piggy on top of the roof ---
    float roofTopY = fortY - 14 * blockSize - 2;
piggies.push_back(PhysicsBody({ fortX - 10, roofTopY - pigRadius + 92.0f },{ 0,0 }, 2.5f, 0, pigRadius, 0, DARKGREEN, BIRD_CIRCLE));
piggies.back().activePhysics = false; // Disable physics initially
piggies.back().isPig = true; // Mark as pig
piggies.back().toughness = 600.0f; // Higher toughness for roof pig

    // Ground plane
    float planeY = screenHeight - 50;
    float planeX = screenWidth / 2;
	Vector2 planeNormal = { 0, -1 }; // Upward normal
	HalfSpace ground({ planeX, planeY }, planeNormal); // Create ground plane

    while (!WindowShouldClose())
    {
        float frameTime = GetFrameTime(); // Get frame time
        Vector2 mousePos = GetMousePosition(); // Get mouse position

        // === SLINGSHOT INTERACTION ===
        float distToBase = sqrtf(powf(mousePos.x - slingshotBase.x, 2) + powf(mousePos.y - slingshotBase.y, 2)); // Distance to slingshot base

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && distToBase < 30.0f) {
            isDragging = true; // Start dragging
        }

        if (isDragging) {
            Vector2 pullVector = { mousePos.x - slingshotBase.x, mousePos.y - slingshotBase.y }; // Pull vector
            float pullDistance = sqrtf(pullVector.x * pullVector.x + pullVector.y * pullVector.y); // Pull distance

            if (pullDistance > maxPullDistance) { // Limit pull distance
                pullVector.x = (pullVector.x / pullDistance) * maxPullDistance; // Clamp X
                pullVector.y = (pullVector.y / pullDistance) * maxPullDistance; // Clamp Y
            }

            dragPosition.x = slingshotBase.x + pullVector.x; // Update drag position
            dragPosition.y = slingshotBase.y + pullVector.y; // Update drag position
        }

        // Release to launch
        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON) && isDragging) { // On release
            isDragging = false; // Stop dragging

            Vector2 pullVector = { slingshotBase.x - dragPosition.x, slingshotBase.y - dragPosition.y }; // Calculate pull vector
            float pullDistance = sqrtf(pullVector.x * pullVector.x + pullVector.y * pullVector.y); // Pull distance

            Vector2 launchVelocity = { // Calculate launch velocity
                (pullVector.x / pullDistance) * pullDistance * launchForceMultiplier, // Launch velocity X
                (pullVector.y / pullDistance) * pullDistance * launchForceMultiplier // Launch velocity Y
            };

            if (selectedBirdType == BIRD_CIRCLE) { // Create bird based on selection
                PhysicsBody newBird(slingshotBase, launchVelocity, circleBirdMass, 0.1f,
                    circleBirdRadius, circleBirdFriction, circleBirdColor, BIRD_CIRCLE); // Circle bird
                launchedBirds.push_back(newBird); // Add to launched birds
            }
            else {
                PhysicsBody newBird(slingshotBase, launchVelocity, squareBirdMass, 0.1f, // Square bird
                    squareBirdRadius, squareBirdFriction, squareBirdColor, BIRD_SQUARE); // Create square bird
                launchedBirds.push_back(newBird); // Add to launched birds
            }

            dragPosition = slingshotBase; // Reset drag position
        }

        // === BIRD SELECTION INPUT ===
        if (IsKeyPressed(KEY_ONE)) { selectedBirdType = BIRD_CIRCLE; } // Select circle bird
        if (IsKeyPressed(KEY_TWO)) { selectedBirdType = BIRD_SQUARE; } // Select square bird

        // === UPDATE PHYSICS ===

        // Update all birds
        for (auto& bird : launchedBirds) {
            simulation.Update(bird, frameTime); // Update bird position and velocity

            if (simulation.CheckSphereHalfSpaceCollision(bird, ground)) { // Check ground collision
                float dx = bird.position.x - ground.point.x; // Delta x
                float dy = bird.position.y - ground.point.y; // Delta y
                float distance = dx * ground.normal.x + dy * ground.normal.y; // Distance from plane    

                if (distance < bird.radius) { // If colliding
                    float penetration = bird.radius - distance; // Penetration depth
                    bird.position.x += ground.normal.x * penetration; // Correct position X
                    bird.position.y += ground.normal.y * penetration; // Correct position Y
                }

                simulation.ResolvePlaneCollision(bird, ground); // Resolve collision
            }
        }

        // Update all fort blocks
        for (auto& block : fortBlocks) { // Iterate through blocks
            if (!block.activePhysics) continue; // Skip inactive blocks
            simulation.Update(block, frameTime); // Update block position and velocity

            if (simulation.CheckSphereHalfSpaceCollision(block, ground)) { // Check ground collision
                float dx = block.position.x - ground.point.x; // Delta x
                float dy = block.position.y - ground.point.y; // Delta y
                float distance = dx * ground.normal.x + dy * ground.normal.y; // Distance from plane

                float effectiveRadius = (block.birdType == BIRD_RECTANGLE) ? block.radius / 4.0f : block.radius; // Effective radius

                if (distance < effectiveRadius) { // If colliding
                    float penetration = effectiveRadius - distance; // Penetration depth
                    block.position.x += ground.normal.x * penetration; // Correct position X
                    block.position.y += ground.normal.y * penetration; // Correct position Y
                }

                simulation.ResolvePlaneCollision(block, ground); // Resolve collision

                if (block.mu > 0.0f) {
                    Vector2 gForce = { simulation.gravity.x * block.mass, simulation.gravity.y * block.mass }; // Gravitational force
                    float N_mag = gForce.x * ground.normal.x + gForce.y * ground.normal.y; // Normal force magnitude
                    Vector2 tangent = { -ground.normal.y, ground.normal.x }; // Tangent vector
                    float v_tan = block.velocity.x * tangent.x + block.velocity.y * tangent.y; // Tangential velocity

                    float frictionMax = block.mu * fabsf(N_mag); // Maximum friction force
                    Vector2 frictionForce = { 0 }; // Initialize friction force

                    if (fabsf(v_tan) > 0.01f) { // Apply friction
                        Vector2 fDir = (v_tan > 0) ? Vector2{ -tangent.x, -tangent.y } : tangent; // Friction direction
                        frictionForce = Vector2{ fDir.x * frictionMax, fDir.y * frictionMax }; // Calculate friction force

                        Vector2 accel = {
                            frictionForce.x / block.mass, // Acceleration X
                            frictionForce.y / block.mass // Acceleration Y
                        };

                        block.velocity.x += accel.x * frameTime; // Update velocity X
                        block.velocity.y += accel.y * frameTime; // Update velocity Y
                    }

                    if (fabsf(v_tan) < 5.0f) { // Apply damping
                        block.velocity.x *= 0.95f; // Dampen velocity X
                        block.velocity.y *= 0.95f; // Dampen velocity Y
                    }
                }
            }
        }

        // Update all piggies
        for (auto& piggy : piggies) {

            // Skip piggies that haven't been "activated" yet (roof pig, etc.)
            if (!piggy.activePhysics) continue;

            // Apply gravity and integrate velocity → position
            simulation.Update(piggy, frameTime);

            // Check collision between this piggy (circle) and the ground plane
            if (simulation.CheckSphereHalfSpaceCollision(piggy, ground)) {

                // --- Calculate penetration depth into the ground ---
                // dx/dy = vector from ground reference point to pig
                float dx = piggy.position.x - ground.point.x;
                float dy = piggy.position.y - ground.point.y;

                // Project that vector onto the ground normal
                float distance = dx * ground.normal.x + dy * ground.normal.y;

                // If the pig’s center is *closer to the plane* than its radius,
                // it means the pig is clipping into the ground.
                if (distance < piggy.radius) {
                    float penetration = piggy.radius - distance;

                    // Push pig OUT of the ground along the plane normal
                    piggy.position.x += ground.normal.x * penetration;
                    piggy.position.y += ground.normal.y * penetration;
                }

                // Resolve velocity change from bouncing off the plane (with restitution)
                simulation.ResolvePlaneCollision(piggy, ground);
                // ----------------------------
                //    FRICTION CALCULATIONS
                // ----------------------------

                // Only apply friction if the pig actually has friction enabled
                if (piggy.mu > 0.0f) {

                    // Gravity force: F = m * g
                    Vector2 gForce = { simulation.gravity.x * piggy.mass,
                                       simulation.gravity.y * piggy.mass };

                    // Normal force magnitude = projection of gravity onto the plane normal
                    float N_mag = gForce.x * ground.normal.x + gForce.y * ground.normal.y;

                    // Tangent vector = 90 degrees rotated version of normal
                    Vector2 tangent = { -ground.normal.y, ground.normal.x };

                    // Component of pig velocity along tangent
                    float v_tan = piggy.velocity.x * tangent.x + piggy.velocity.y * tangent.y;

                    // Max friction force = μ * |NormalForce|
                    float frictionMax = piggy.mu * fabsf(N_mag);
                    Vector2 frictionForce = { 0 };

                    // --------- Dynamic Friction (when sliding fast) ---------
                    if (fabsf(v_tan) > 0.01f) {

                        // Friction direction opposes tangent velocity
                        Vector2 fDir = (v_tan > 0)
                            ? Vector2{ -tangent.x, -tangent.y }
                        : tangent;
                        // Apply friction with max magnitude
                        frictionForce = Vector2{
                            fDir.x * frictionMax,
                            fDir.y * frictionMax
                        };
                        // Convert force → acceleration
                        Vector2 accel = {
                            frictionForce.x / piggy.mass,
                            frictionForce.y / piggy.mass
                        };
                        // Apply acceleration for this frame
                        piggy.velocity.x += accel.x * frameTime;
                        piggy.velocity.y += accel.y * frameTime;
                    }
                    // --------- Static Friction / Damping (when almost stopped) ---------
                    // If pig is barely sliding, slowly damp remaining motion
                    if (fabsf(v_tan) < 5.0f) {
                        piggy.velocity.x *= 0.95f;
                        piggy.velocity.y *= 0.95f;
                    }
                }
            }
        }


        // === COLLISION DETECTION & RESPONSE ===

        // Bird-to-bird collisions
        for (size_t i = 0; i < launchedBirds.size(); ++i) { // Iterate through birds
            for (size_t j = i + 1; j < launchedBirds.size(); ++j) { // Check against other birds
                if (simulation.CheckCollision(launchedBirds[i], launchedBirds[j])) { // Check collision
                    simulation.ResolveElasticCollision(launchedBirds[i], launchedBirds[j]); // Resolve collision
                }
            }
        }

        // Bird-to-block collisions
        for (auto& bird : launchedBirds) { // Iterate through birds
            for (auto& block : fortBlocks) { // Check against fort blocks
                if (simulation.CheckCollision(bird, block)) { // Check collision
                    fortTriggered = true; // Activate fort
                    simulation.ResolveElasticCollision(bird, block); // Resolve collision
                }
            }
        }

        // Activate fort and roof pigs if triggered
        if (fortTriggered) { // If fort is triggered
            for (auto& block : fortBlocks) // Activate all blocks
                block.activePhysics = true; // Enable physics

            for (auto& piggy : piggies) { // Activate roof pig
                if (piggy.position.y < fortY - 5 * blockSize) // Roof pig check
                    piggy.activePhysics = true; // Enable physics
            }
        }

        // Bird-to-piggy collisions
        for (auto& bird : launchedBirds) { // Iterate through birds
            for (auto& piggy : piggies) { // Check against piggies
                if (simulation.CheckCollision(bird, piggy)) { // Check collision
                    simulation.ResolveElasticCollision(bird, piggy); // Resolve collision
                    piggy.activePhysics = true; // Activate pig physics

                    if (piggy.isPig && !piggy.destroyed) { // If it's a pig
                        Vector2 relVel = { bird.velocity.x - piggy.velocity.x, bird.velocity.y - piggy.velocity.y }; // Relative velocity
                        float relSpeed = sqrtf(relVel.x * relVel.x + relVel.y * relVel.y); // Relative speed
                        float momentum = bird.mass * relSpeed; // Calculate momentum
                        if (momentum > piggy.toughness) piggy.destroyed = true; // Check toughness
                    }
                }
            }
        }

        // ================================
// Block-to-block collisions
// ================================
        for (size_t i = 0; i < fortBlocks.size(); ++i) {
            for (size_t j = i + 1; j < fortBlocks.size(); ++j) {

                // Check if two fort blocks overlap
                if (simulation.CheckCollision(fortBlocks[i], fortBlocks[j])) {

                    // If they collide, apply physics so they bounce apart
                    simulation.ResolveElasticCollision(fortBlocks[i], fortBlocks[j]);
                }
            }
        }

        // ================================
        // Block-to-piggy collisions
        // ================================
        for (auto& block : fortBlocks) {
            for (auto& piggy : piggies) {

                // Check if a block hits a piggy
                if (simulation.CheckCollision(block, piggy)) {

                    // Apply elastic collision response (bounce)
                    simulation.ResolveElasticCollision(block, piggy);

                    // Enable physics on the piggy so it can fall/move
                    piggy.activePhysics = true;

                    // If this is a real pig (not just a decoration chunk)
                    // check if the block hit it hard enough to kill it
                    if (piggy.isPig && !piggy.destroyed) {

                        // Calculate relative velocity between block and pig
                        Vector2 relVel = { block.velocity.x - piggy.velocity.x,
                                           block.velocity.y - piggy.velocity.y };

                        float relSpeed = sqrtf(relVel.x * relVel.x +
                            relVel.y * relVel.y);

                        // Momentum used as a "damage" value
                        float momentum = block.mass * relSpeed;

                        // If momentum exceeds toughness, pig dies
                        if (momentum > piggy.toughness)
                            piggy.destroyed = true;
                    }
                }
            }
        }

        // ================================
        // Piggy-to-piggy collisions
        // ================================
        for (size_t i = 0; i < piggies.size(); ++i) {
            for (size_t j = i + 1; j < piggies.size(); ++j) {

                // Check if two piggies collide
                if (simulation.CheckCollision(piggies[i], piggies[j])) {

                    // Make them bounce off each other
                    simulation.ResolveElasticCollision(piggies[i], piggies[j]);

                    // For damage calculation, store references
                    PhysicsBody& pigA = piggies[i];
                    PhysicsBody& pigB = piggies[j];

                    // Relative velocity between the pigs
                    Vector2 relVel = {
                        pigB.velocity.x - pigA.velocity.x,
                        pigB.velocity.y - pigA.velocity.y
                    };

                    float relSpeed = sqrtf(relVel.x * relVel.x +
                        relVel.y * relVel.y);

                    // Damage values based on momentum
                    float momentumA = pigB.mass * relSpeed;
                    float momentumB = pigA.mass * relSpeed;

                    // Kill pig A if pig B hit it hard enough
                    if (pigA.isPig && !pigA.destroyed && momentumA > pigA.toughness)
                        pigA.destroyed = true;

                    // Kill pig B if pig A hit it hard enough
                    if (pigB.isPig && !pigB.destroyed && momentumB > pigB.toughness)
                        pigB.destroyed = true;
                }
            }
        }
        // Remove destroyed pigs from array
        piggies.erase(std::remove_if(piggies.begin(), piggies.end(), [](const PhysicsBody& p) { return p.destroyed; }), piggies.end()); // Remove destroyed pigs

        // === RENDERING ===
        BeginDrawing();
        ClearBackground(SKYBLUE);

        DrawRectangle(0, planeY, screenWidth, screenHeight - planeY, DARKGREEN);

        // === DRAW SLINGSHOT ===
        DrawRectangle(slingshotBase.x - 15, slingshotBase.y, 8, 60, DARKBROWN); // Left post
        DrawRectangle(slingshotBase.x + 7, slingshotBase.y, 8, 60, DARKBROWN); // Right post

        if (isDragging) {
            DrawLineEx(Vector2{ slingshotBase.x - 10, slingshotBase.y + 10 }, dragPosition, 3, BROWN); // Left band
            DrawLineEx(Vector2{ slingshotBase.x + 10, slingshotBase.y + 10 }, dragPosition, 3, BROWN); // Right band

            Vector2 previewVel = {
                (slingshotBase.x - dragPosition.x) * launchForceMultiplier,(slingshotBase.y - dragPosition.y) * launchForceMultiplier // Preview launch velocity
            };

            for (int i = 0; i < 20; i++) {
                float t = i * 0.1f; // Time step
                float x = slingshotBase.x + previewVel.x * t; // Predicted X position
                float y = slingshotBase.y + previewVel.y * t + 0.5f * simulation.gravity.y * t * t; // Predicted Y position
                DrawCircle(x, y, 2, YELLOW); // Draw trajectory point
            }

            if (selectedBirdType == BIRD_CIRCLE) { // Draw dragged bird
                DrawCircleV(dragPosition, circleBirdRadius, circleBirdColor); // Circle bird
                DrawCircleLines(dragPosition.x, dragPosition.y, circleBirdRadius, BLACK); // Circle outline
            }
            else { // Square bird
                DrawRectangle(dragPosition.x - squareBirdRadius, dragPosition.y - squareBirdRadius, // Draw square bird
                    squareBirdRadius * 2, squareBirdRadius * 2, squareBirdColor); // Square bird
                DrawRectangleLines(dragPosition.x - squareBirdRadius, dragPosition.y - squareBirdRadius, // Square outline
                    squareBirdRadius * 2, squareBirdRadius * 2, BLACK); // Square outline
            }
        }

        else {
            DrawLineEx(Vector2{ slingshotBase.x - 10, slingshotBase.y + 10 }, slingshotBase, 3, BROWN); // Left band
            DrawLineEx(Vector2{ slingshotBase.x + 10, slingshotBase.y + 10 }, slingshotBase, 3, BROWN); // Right band

            if (selectedBirdType == BIRD_CIRCLE) { // Draw selected bird
                DrawCircleV(slingshotBase, circleBirdRadius, circleBirdColor); // Circle bird
                DrawCircleLines(slingshotBase.x, slingshotBase.y, circleBirdRadius, BLACK); // Circle outline
            }
            else { // Square bird
                DrawRectangle(slingshotBase.x - squareBirdRadius, slingshotBase.y - squareBirdRadius, // Draw square bird
                    squareBirdRadius * 2, squareBirdRadius * 2, squareBirdColor); // Square bird
                DrawRectangleLines(slingshotBase.x - squareBirdRadius, slingshotBase.y - squareBirdRadius, // Square outline
                    squareBirdRadius * 2, squareBirdRadius * 2, BLACK); // Square outline
            }
        }

        // DRAW FORT BLOCKS
        for (const auto& block : fortBlocks) {

            // Check what shape this block uses for drawing
            if (block.birdType == BIRD_RECTANGLE) {

                // This block is a wide rectangle (used for fort beams)
                float halfWidth = block.radius;          // Horizontal half-size
                float halfHeight = block.radius / 4.0f;  // Thin vertical size

                // Draw filled rectangle
                DrawRectangle(
                    block.position.x - halfWidth,
                    block.position.y - halfHeight,
                    halfWidth * 2,
                    halfHeight * 2,
                    block.color
                );

                // Draw outline
                DrawRectangleLines(
                    block.position.x - halfWidth,
                    block.position.y - halfHeight,
                    halfWidth * 2,
                    halfHeight * 2,
                    BLACK
                );
            }
            else {
                // Draw this block as a square
                DrawRectangle(
					block.position.x - block.radius, // Top-left X
					block.position.y - block.radius, // Top-left Y
					block.radius * 2, // Width
					block.radius * 2, // Height
					block.color //  Fill color
                );

                // Outline
				DrawRectangleLines( // Draw outline
					block.position.x - block.radius, // Top-left X
					block.position.y - block.radius, // Top-left Y
					block.radius * 2, // Width
					block.radius * 2, // Height
                    BLACK
                );
            }
        }
        // DRAW PIGGIES

		for (const auto& piggy : piggies) { // Iterate through piggies

            // Draw pig body
            DrawCircleV(piggy.position, piggy.radius, piggy.color);
            DrawCircleLines(piggy.position.x, piggy.position.y, piggy.radius, BLACK);

            // Draw eyes
			DrawCircle(piggy.position.x - 4, piggy.position.y - 2, 2, WHITE); // Left eye white
			DrawCircle(piggy.position.x + 4, piggy.position.y - 2, 2, WHITE); // Right eye white
			DrawCircle(piggy.position.x - 3, piggy.position.y - 2, 1, BLACK); // Left eye pupil
			DrawCircle(piggy.position.x + 5, piggy.position.y - 2, 1, BLACK); // Right eye pupil
             
            // Draw snout
			DrawCircle(piggy.position.x, piggy.position.y + 4, 4, DARKGREEN); // Snout base
			DrawCircle(piggy.position.x - 1, piggy.position.y + 4, 1, BLACK); // Left nostril
			DrawCircle(piggy.position.x + 1, piggy.position.y + 4, 1, BLACK); // Right nostril
        }
        // DRAW LAUNCHED BIRDS
        for (const auto& bird : launchedBirds) {

            // Check bird shape
            if (bird.birdType == BIRD_CIRCLE) {

                // Draw circle bird
                DrawCircleV(bird.position, bird.radius, bird.color);
                DrawCircleLines(bird.position.x, bird.position.y, bird.radius, BLACK);
            }
            else {

                // Draw square bird
                DrawRectangle(
                    bird.position.x - bird.radius,
                    bird.position.y - bird.radius,
                    bird.radius * 2,
                    bird.radius * 2,
                    bird.color
                );

                // Outline
                DrawRectangleLines(
                    bird.position.x - bird.radius,
                    bird.position.y - bird.radius,
                    bird.radius * 2,
                    bird.radius * 2,
                    BLACK
                );
            }
        }

        // UI PANEL (Left)
        DrawRectangle(10, 10, 250, 120, Fade(BLACK, 0.7f)); // Background box
        DrawText("BIRD SELECTION", 20, 20, 16, WHITE);

        // Highlight the selected bird type in yellow
        DrawText("Press 1: Circle Bird (Low Mass)", 20, 45, 14,
            selectedBirdType == BIRD_CIRCLE ? YELLOW : WHITE);

        DrawText("Press 2: Square Bird (High Mass)", 20, 65, 14,
            selectedBirdType == BIRD_SQUARE ? YELLOW : WHITE);

        DrawText("Click & drag slingshot to launch", 20, 95, 12, LIGHTGRAY);

        // Show number of birds fired
        DrawText(TextFormat("Birds launched: %d", launchedBirds.size()),
            20, 110, 12, LIGHTGRAY);

        // UI PANEL (Right)
        // Shows CURRENT BIRD being prepared
        DrawRectangle(screenWidth - 220, 10, 210, 90, Fade(BLACK, 0.7f));
        DrawText("CURRENT BIRD:", screenWidth - 210, 20, 14, WHITE);

        if (selectedBirdType == BIRD_CIRCLE) {

            // Preview a circle bird
            DrawCircle(screenWidth - 170, 50, circleBirdRadius, circleBirdColor);
            DrawText("Circle Bird", screenWidth - 210, 70, 12, WHITE);
            DrawText(TextFormat("Mass: %.1f kg", circleBirdMass),
                screenWidth - 210, 85, 10, LIGHTGRAY);
        }
        else {
            // Preview a square bird
            DrawRectangle(
                screenWidth - 170 - squareBirdRadius / 2,
                50 - squareBirdRadius / 2,
                squareBirdRadius,
                squareBirdRadius,
                squareBirdColor
            );

            DrawText("Square Bird", screenWidth - 210, 70, 12, WHITE);
            DrawText(TextFormat("Mass: %.1f kg", squareBirdMass),
                screenWidth - 210, 85, 10, LIGHTGRAY);
        }

        EndDrawing();  
    }