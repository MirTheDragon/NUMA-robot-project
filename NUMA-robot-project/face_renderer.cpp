#include "face_renderer.hpp"
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <iostream>
#include <random>
#include <algorithm>

// === Screen Parameters ===
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 480;

// === Eye Parameters ===
const SDL_Point leftEyeBaseCenter  = {160, 240};
const SDL_Point rightEyeBaseCenter = {640, 240};
const int eyeBaseWidth  = 200;
const int eyeBaseHeight = 300;

// === Movement Limits ===
const int maxGazeOffsetX = 60;
const int maxGazeOffsetY = 40;

const int gazeTransitionFrames   = 10;
const int gazeIntervalMinFrames  = 60;
const int gazeIntervalMaxFrames  = 600;

const int jitterIntervalMinFrames = 30;
const int jitterIntervalMaxFrames = 120;
const int jitterMaxOffset         = 8;

const float pupilGazeFollowRatio = 0.15f;
const float pupilWidthRatio  = 0.8f;
const float pupilHeightRatio = 0.8f;

const int totalBlinkFrames = 20;

const float minDilation       = 0.80f;
const float rareMinDilation   = 0.20f;
const float rareMaxDilation   = 0.40f;  // Maximum size for rare dilation (e.g. not more than 75%)
const float rareMinChance     = 0.20f;

const SDL_Color glowColor     = {64, 200, 255, 5};
const SDL_Color eyeFillColor  = {20, 220, 255, 255};
const SDL_Color pupilColor    = {255, 255, 255, 220};
const int glowPadding = 80;

// === Internal State and Random Setup ===
struct EyeMotion {
    SDL_Point gazeOffset{0, 0};
    SDL_Point gazeTarget{0, 0};
    SDL_Point jitterOffset{0, 0};
    int transitionFrame = 0;
    int totalFrames = gazeTransitionFrames;
    int gazeWaitTimer = 0;
    int jitterTimer = 0;
    float dilationFactor = 1.0f;
};

static EyeMotion eyeMotion;
static SDL_Window* window = nullptr;
static SDL_Renderer* renderer = nullptr;

std::mt19937 rng(std::random_device{}());
std::uniform_int_distribution<int> centerChance(0, 4);
std::uniform_int_distribution<int> gazeWaitDist(gazeIntervalMinFrames, gazeIntervalMaxFrames);
std::uniform_int_distribution<int> jitterWaitDist(jitterIntervalMinFrames, jitterIntervalMaxFrames);
std::uniform_real_distribution<float> dilationDist(rareMinDilation, 1.0f);

float getRandomDilation() {
    float d = dilationDist(rng);
    if (d < minDilation && dilationDist(rng) > rareMinChance) {
        d = minDilation + ((rng() % 10) / 100.0f);
        d = std::min(d, rareMaxDilation);  // Cap rare dilation to max
    }
    return d;
}

int randRange(int minVal, int maxVal) {
    return minVal + (rng() % (maxVal - minVal + 1));
}

SDL_Renderer* getRenderer() {
    return renderer;
}

bool initGraphics() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return false;
    }
    window = SDL_CreateWindow("Robot Face",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_BORDERLESS);
    SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
    if (!window) return false;
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    return renderer != nullptr;
}

void shutdownGraphics() {
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
}

void updateAndDrawEyes() {
    eyeMotion.dilationFactor = std::clamp(eyeMotion.dilationFactor, minDilation, 1.0f);
    int eyeW = static_cast<int>(eyeBaseWidth * eyeMotion.dilationFactor);
    int eyeH = static_cast<int>(eyeBaseHeight * eyeMotion.dilationFactor);
    int radius = std::min(eyeW, eyeH) / 4;

    SDL_Point maxOffset = { maxGazeOffsetX, maxGazeOffsetY };

    if (--eyeMotion.gazeWaitTimer <= 0 && eyeMotion.transitionFrame == 0) {
        eyeMotion.gazeTarget = (centerChance(rng) == 0) ? SDL_Point{0, 0}
            : SDL_Point{ randRange(-maxOffset.x, maxOffset.x), randRange(-maxOffset.y, maxOffset.y) };
        eyeMotion.dilationFactor = getRandomDilation();
        eyeMotion.transitionFrame = eyeMotion.totalFrames;
        eyeMotion.gazeWaitTimer = gazeWaitDist(rng);
    }

    if (--eyeMotion.jitterTimer <= 0) {
        eyeMotion.jitterOffset = {
            randRange(-jitterMaxOffset, jitterMaxOffset),
            randRange(-jitterMaxOffset, jitterMaxOffset)
        };
        eyeMotion.jitterTimer = jitterWaitDist(rng);
    }

    if (eyeMotion.transitionFrame > 0) {
        float t = 1.0f - (float)eyeMotion.transitionFrame / eyeMotion.totalFrames;
        eyeMotion.gazeOffset.x = static_cast<int>(
            eyeMotion.gazeOffset.x * (1 - t) + eyeMotion.gazeTarget.x * t);
        eyeMotion.gazeOffset.y = static_cast<int>(
            eyeMotion.gazeOffset.y * (1 - t) + eyeMotion.gazeTarget.y * t);
        eyeMotion.transitionFrame--;
    }

    SDL_Point totalOffset = {
        eyeMotion.gazeOffset.x + eyeMotion.jitterOffset.x,
        eyeMotion.gazeOffset.y + eyeMotion.jitterOffset.y
    };

    SDL_Point leftCenter  = {leftEyeBaseCenter.x + totalOffset.x, leftEyeBaseCenter.y + totalOffset.y};
    SDL_Point rightCenter = {rightEyeBaseCenter.x + totalOffset.x, rightEyeBaseCenter.y + totalOffset.y};

    static bool blinkingL = false, blinkingR = false;
    static int blinkFrame = 0;
    static std::uniform_int_distribution<int> blinkTimerDist(1, 15);
    static std::uniform_real_distribution<float> doubleBlinkChance(0.0f, 1.0f);
    static int blinkTimer = blinkTimerDist(rng) * 60;

    if (blinkingL || blinkingR) {
        blinkFrame++;
        if (blinkFrame > totalBlinkFrames) {
            blinkingL = blinkingR = false;
            blinkFrame = 0;
        }
    }

    if (--blinkTimer <= 0 && !blinkingL && !blinkingR) {
        blinkingL = blinkingR = true;
        blinkFrame = 0;
        blinkTimer = blinkTimerDist(rng) * 60;
        if (doubleBlinkChance(rng) < 0.1f) blinkTimer /= 2;
    }

    SDL_SetRenderDrawColor(getRenderer(), 0, 0, 0, 255);
    SDL_RenderClear(getRenderer());

    auto drawEye = [&](SDL_Point center, bool isBlinking) {
        int w = eyeW;
        int h = eyeH;
        int r = radius;

        if (isBlinking) {
            float t = static_cast<float>(blinkFrame) / totalBlinkFrames;
            float squish = (t < 0.5f) ? (1.0f - 2 * t) : ((t - 0.5f) * 2);
            h = static_cast<int>(h * squish);
        }

        int x1 = center.x - w / 2;
        int y1 = center.y - h / 2;
        int x2 = center.x + w / 2;
        int y2 = center.y + h / 2;


        // Simulated glow without blend mode: outer layers first, dimmer as they get closer
        for (int i = glowPadding - 1; i >= 0; --i) {
            float normalized = (float)i / glowPadding;

            // Simulated exponential decay in brightness (not alpha)
            float brightness = std::pow(1.0f - normalized, 2.2f);

            Uint8 r = static_cast<Uint8>(glowColor.r * brightness);
            Uint8 g = static_cast<Uint8>(glowColor.g * brightness);
            Uint8 b = static_cast<Uint8>(glowColor.b * brightness);
            Uint8 a = static_cast<Uint8>(glowColor.a);  // Always max (opaque), since we're not blending

            roundedBoxRGBA(getRenderer(),
                x1 - i, y1 - i,
                x2 + i, y2 + i,
                r + i * 2,
                r, g, b, a
            );
        }

        roundedBoxRGBA(getRenderer(), x1, y1, x2, y2, r,
                       eyeFillColor.r, eyeFillColor.g, eyeFillColor.b, eyeFillColor.a);

        int pupilW = static_cast<int>(w * pupilWidthRatio);
        int pupilH = static_cast<int>(h * pupilHeightRatio);
        int pupilX = center.x + static_cast<int>(eyeMotion.gazeOffset.x * pupilGazeFollowRatio);
        int pupilY = center.y + static_cast<int>(eyeMotion.gazeOffset.y * pupilGazeFollowRatio);
        int px1 = pupilX - pupilW / 2;
        int py1 = pupilY - pupilH / 2;
        int px2 = pupilX + pupilW / 2;
        int py2 = pupilY + pupilH / 2;

        roundedBoxRGBA(getRenderer(), px1, py1, px2, py2, std::min(pupilW, pupilH) / 4,
                       pupilColor.r, pupilColor.g, pupilColor.b, pupilColor.a);
    };

    drawEye(leftCenter, blinkingL);
    drawEye(rightCenter, blinkingR);
    SDL_RenderPresent(getRenderer());
}
