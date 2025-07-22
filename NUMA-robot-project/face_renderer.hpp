#pragma once
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

// Initializes SDL window and renderer
bool initGraphics();
void shutdownGraphics();

// Call this every frame to update and render the robot face (eyes)
void updateAndDrawEyes();
bool showBootPrompt(SDL_Joystick*& joystick);

// Gives access to the SDL renderer if needed
SDL_Renderer* getRenderer();
