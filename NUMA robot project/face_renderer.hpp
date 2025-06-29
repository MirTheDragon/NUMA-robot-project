#pragma once
#include <SDL2/SDL.h>

// Initializes SDL window and renderer
bool initGraphics();
void shutdownGraphics();

// Call this every frame to update and render the robot face (eyes)
void updateAndDrawEyes();

// Gives access to the SDL renderer if needed
SDL_Renderer* getRenderer();
