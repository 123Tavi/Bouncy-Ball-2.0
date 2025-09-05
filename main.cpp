#include "raylib.h"
#include <iostream>
#include <cmath>

int main(void) {

	// Window Settings 
	const int WINDOW_WIDTH = 800;
	const int WINDOW_HEIGHT = 450;
	InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "BALL");
	SetTargetFPS(60);

	// Coordinate line
	Vector2 origin = { (float)WINDOW_WIDTH / 2, (float)WINDOW_HEIGHT / 2 };

	// Ball Settings
	Vector2 ball_position = { (float)WINDOW_WIDTH / 2, (float)WINDOW_HEIGHT / 2 };
	float mass = 20;
	Vector2 initial_height = { (float)WINDOW_WIDTH / 2, (float)WINDOW_HEIGHT / 2 };
	Vector2 final_height = { 0,0 };

	// Physics Settings
	float e = 0.8; // Coefficient of restitution
	float gravity = 9.81f;
	float velocity = 0;
	float lastBounceHeight = 0;
	float secondLastBounceHeight = 0;

	// TODO GET FINAL HEIGHT

	while (!WindowShouldClose()) {

		// Update
		float dt = GetFrameTime();
		velocity += gravity * dt;
		ball_position.y += velocity;

		// Energy
		float energy = 0.5 * (mass * pow(velocity, 2)); // Energy formula
		float PE_initial = mass * gravity * initial_height.y;
		float PE_final = mass * gravity * final_height.y;
		float deltaPE = mass * gravity * (initial_height.y - final_height.y);


		// Bounce 
		if (ball_position.y + mass > WINDOW_HEIGHT){
			ball_position.y = WINDOW_HEIGHT - mass;
			velocity *= -e;

			float currentBounceHeight = (velocity * velocity) / (2 * gravity);
			secondLastBounceHeight = lastBounceHeight;
			lastBounceHeight = currentBounceHeight;

			if (secondLastBounceHeight > 0) {
				float diff = secondLastBounceHeight - lastBounceHeight;
				std::cout << diff << std::endl;
			}
			
			// If the velocity is super small, the ball stops from bouncing 
			if (energy < 0.5) {velocity = 0;}
		}

		// Raylib 
		BeginDrawing();
			ClearBackground(BLACK);
			
			// --- Draw grid every 50 pixels ---
			for (int x = 0; x < WINDOW_WIDTH; x += 50)
			{
				DrawLine(x, 0, x, WINDOW_HEIGHT, LIGHTGRAY);
				DrawText(TextFormat("%d", x), x + 2, 2, 10, RAYWHITE);
			}
			for (int y = 0; y < WINDOW_HEIGHT; y += 50)
			{
				DrawLine(0, y, WINDOW_WIDTH, y, LIGHTGRAY);
				DrawText(TextFormat("%d", y), 2, y + 2, 10, RAYWHITE);
			}

			// Highlight the axes
			DrawLine(0, 0, WINDOW_WIDTH, 0, WHITE);
			DrawLine(0, 0, 0, WINDOW_HEIGHT, WHITE);

			// Draw Ball
			DrawCircleV(ball_position, mass, SKYBLUE); // Circle

			EndDrawing();
	}

	CloseWindow();
	return 0;
}
