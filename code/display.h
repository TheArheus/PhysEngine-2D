#if !defined(DISPLAY_H_)

#include <math.h>
#include <stdbool.h>
#include <SDL2\SDL.h>
#include <vector>
#include "intrinsics.h"

struct texture_t
{
    u32 Width;
    u32 Height;
    u32* Memory;
};

extern SDL_Window*      window;
extern SDL_Renderer*    renderer;
extern SDL_Texture*     texture;
extern texture_t*       ColorBuffer;

#include "hmath.h"
#include "p_particle.h"
#include "world.h"

bool InitWindow();
void RenderColorBuffer();
void ClearColorBuffer(texture_t* Texture, u32);
void DrawPixel(texture_t* Texture, u32, u32, u32);
void DrawGrid(texture_t* Texture, u32);
void DrawLine(texture_t* Texture, v2 Min, v2 Max, u32 Color);
void DrawRect(texture_t* RenderBuffer, v2, v2, u32);
void DrawRotRect(texture_t* RenderBuffer, v2 Origin, v2 XAxis, v2 YAxis, u32 color, texture_t* Texture);
void DrawCircle(v2 P, u32 Width, u32 Height, r32 Radius, u32 Color);
void DrawFilledCircle(v2 P, u32 Width, u32 Height, r32 R, u32 Color);
texture_t CreateRectangleTexture(u32 Width, u32 Height, u32 color);
texture_t CreateGridTexture(u32 Dim);
void DestroyWindow();

#define DISPLAY_H_
#endif
