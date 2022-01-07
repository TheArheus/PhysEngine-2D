#include "display.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

SDL_Window*     window          = NULL;
SDL_Renderer*   renderer        = NULL;
SDL_Texture*    texture         = NULL;
texture_t*      ColorBuffer     = NULL;

bool InitWindow(void)
{
    if(SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        fprintf(stderr, "Error: initializing SDL\n");
        return false;
    }

    SDL_DisplayMode display_mode;
    SDL_GetCurrentDisplayMode(0, &display_mode);

    ColorBuffer = (texture_t*)calloc(1, sizeof(ColorBuffer));

    ColorBuffer->Width  = display_mode.w;
    ColorBuffer->Height = display_mode.h;

    window = SDL_CreateWindow(NULL, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, ColorBuffer->Width, ColorBuffer->Height, SDL_WINDOW_SHOWN);//SDL_WINDOW_BORDERLESS);
    if(!window)
    {
        fprintf(stderr, "Error: creating SDL window");
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, NULL);
    if(!renderer)
    {
        fprintf(stderr, "Error: creating SDL renderer");
        return false;
    }

    //SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN);

    return true;
}

void RenderColorBuffer()
{
    SDL_UpdateTexture(texture, NULL, ColorBuffer->Memory, sizeof(u32)*ColorBuffer->Width);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
}

void ClearColorBuffer(texture_t* Texture, u32 color)
{
    for(u32 Y = 0; Y < Texture->Height; ++Y)
    {
        for (u32 X = 0; X < Texture->Width; ++X)
        {
            Texture->Memory[(Texture->Width*Y) + X] = color;
        }
    }
}

void DrawPixel(texture_t* Texture, u32 X, u32 Y, u32 Color)
{
    if(X > 0 && Y > 0 && X < Texture->Width && Y < Texture->Height)
    {
        Texture->Memory[Texture->Width*Y + X] = Color;
    }
}

void DrawGrid(texture_t* Texture, u32 color)
{
    for(u32 Y = 0; (Y < Texture->Height); ++Y)
    {
        for(u32 X = 0; (X < Texture->Width); ++X)
        {
            if((Y % 12 == 0) || (X % 12 == 0)) DrawPixel(Texture, X, Y, color);
        }
    }
}

void DrawLine(texture_t* Texture, v2 Min, v2 Max, u32 Color)
{
    i32 dX = (Max.x - Min.x);
    i32 dY = (Max.y - Min.y);

    i32 SideLength = (fabsf(dX) >= fabsf(dY)) ? fabsf(dX) : fabsf(dY);

    if (SideLength > 0)
    {
        r32 IncX = (r32)dX / SideLength;
        r32 IncY = (r32)dY / SideLength;

        r32 CurrentX = Min.x;
        r32 CurrentY = Min.y;
        for (i32 PIndex = 0;
            PIndex <= SideLength;
            ++PIndex)
        {
            DrawPixel(Texture, roundf(CurrentX), roundf(CurrentY), Color);

            CurrentX += IncX;
            CurrentY += IncY;
        }
    }
}

v4 Unpack4x8ColorRGBA(u32 Color)
{
    v4 Result = V4i((Color >> 0) & 0xFF, (Color >> 8) & 0xFF, (Color >> 16) & 0xFF, (Color >> 24) & 0xFF);
    return Result;
}

v4 Unpack4x8ColorARGB(u32 Color)
{
    v4 Result = V4i((Color >> 16) & 0xFF, (Color >> 8) & 0xFF, (Color >> 0) & 0xFF, (Color >> 24) & 0xFF);
    return Result;
}


v4 SRGBTo1Linear(v4 A)
{
    v4 Result = {};

    Result.x = Square(A.x / 255.0f);
    Result.y = Square(A.y / 255.0f);
    Result.z = Square(A.z / 255.0f);
    Result.w = A.w / 255.0f;

    return Result;
}

v4 LinearTo255SRGB(v4 A)
{
    v4 Result = {};

    Result.x = sqrtf(A.x) * 255.0f;
    Result.y = sqrtf(A.y) * 255.0f;
    Result.z = sqrtf(A.z) * 255.0f;
    Result.w = A.w * 255.0f;

    return Result;
}

void 
DrawRotRect(texture_t* RenderBuffer, v2 Origin, v2 XAxis, v2 YAxis, u32 color, texture_t* Texture)
{
    i32 MinX = RenderBuffer->Width - 1;
    i32 MinY = RenderBuffer->Height - 1;
    i32 MaxX = 0;
    i32 MaxY = 0;

    v4 UnpackedColor = Unpack4x8ColorARGB(color);
    UnpackedColor.xyz *= UnpackedColor.w;

    //Origin = Origin - V2(XAxis.x/2, YAxis.y/2);
    v2 Ps[] = {Origin, Origin + XAxis, Origin + XAxis + YAxis, Origin + YAxis};
    for(u32 PIndex = 0;
        PIndex < 4;
        ++PIndex)
    {
        v2 P = Ps[PIndex];

        i32 FloorX = (i32)floorf(P.x);
        i32 CeilX =  (i32)ceilf(P.x);
        i32 FloorY = (i32)floorf(P.y);
        i32 CeilY =  (i32)ceilf(P.y);

        if(MinX > FloorX) {MinX = FloorX;}
        if(MaxX < CeilX)  {MaxX = CeilX;}
        if(MinY > FloorY) {MinY = FloorY;}
        if(MaxY < CeilY)  {MaxY = CeilY;}
    }

    if(MinX < 0) MinX = 0;
    if(MinY < 0) MinY = 0;
    if(MaxX > (i32)RenderBuffer->Width) MaxX = RenderBuffer->Width;
    if(MaxY > (i32)RenderBuffer->Height) MaxY = RenderBuffer->Height;

    r32 Det = XAxis.x*YAxis.y - XAxis.y*YAxis.x;
    if(Det == 0.0f){ Det = 1.0f; }

    v2 nXAxis = { YAxis.y/Det, -YAxis.x/Det};
    v2 nYAxis = {-XAxis.y/Det,  XAxis.x/Det};

    u32 Pitch = RenderBuffer->Width * sizeof(u32);
    u8* Row = ((u8*)RenderBuffer->Memory + MinX*sizeof(u32) + MinY*Pitch);
    for(i32 Y = MinY; Y < MaxY; ++Y)
    {
        u32* Pixel = (u32*)Row;
        
        for(i32 X = MinX; X < MaxX; ++X)
        {
            v2 PixelP = V2i(X, Y);
            v2 d = PixelP - Origin;
            
            r32 U = d.x*nXAxis.x + d.y*nXAxis.y;
            r32 V = d.x*nYAxis.x + d.y*nYAxis.y;

#if 0
            // if we want a rotation
            r32 e0 = Inner(d, -Perp(XAxis));
            r32 e1 = Inner(d - XAxis, -Perp(YAxis));
            r32 e2 = Inner(d - YAxis - XAxis, Perp(XAxis));
            r32 e3 = Inner(d - YAxis, Perp(YAxis));

            if((e0 < 0) && 
               (e1 < 0) && 
               (e2 < 0) && 
               (e3 < 0))
#else
            // Optimized for textures
            if((U >= 0) && (U <= 1) && (V >= 0) && (V <= 1))
#endif
            {
                if(Texture)
                {
                    u32 TexturePitch = Texture->Width * sizeof(u32);

                    U = Min(Max(0, U), 1);
                    V = Min(Max(0, V), 1);

                    r32 tX = ((U * (r32)(Texture->Width  - 1)));
                    r32 tY = ((V * (r32)(Texture->Height - 1)));

                    i32 FetchX = (i32)tX;
                    i32 FetchY = (i32)tY;

#if 0
                    // This is for a interpolation of texels
                    r32 fX = tX - (r32)FetchX;
                    r32 fY = tY - (r32)FetchY;
#endif

                    u32 ColorFromMemory = *(u32*)((u8*)Texture->Memory + (u32)FetchY*TexturePitch + (u32)FetchX*sizeof(u32));

                    v4 SrcTexel = Unpack4x8ColorARGB(ColorFromMemory);
                    SrcTexel = SRGBTo1Linear(SrcTexel);

#if 1
                    SrcTexel = Hadamard(SrcTexel, UnpackedColor);
                    SrcTexel.x = Clamp01(SrcTexel.x);
                    SrcTexel.y = Clamp01(SrcTexel.y);
                    SrcTexel.z = Clamp01(SrcTexel.z);

                    v4 DstPixel = Unpack4x8ColorARGB(*Pixel);
                    DstPixel = SRGBTo1Linear(DstPixel);

                    SrcTexel = (1.0f - (SrcTexel.w / 255.0f))*DstPixel + SrcTexel;
#endif
                    SrcTexel = LinearTo255SRGB(SrcTexel);

                    u32 PreNewColor = (((u32)(SrcTexel.w) << 24) | 
                                       ((u32)(SrcTexel.z) <<  0) | 
                                       ((u32)(SrcTexel.y) <<  8) | 
                                       ((u32)(SrcTexel.x) << 16));
                    color = PreNewColor;
                }
                *Pixel = color;
            }
            ++Pixel;
        }
        Row += Pitch;
    }
}

void DrawRect(texture_t* RenderBuffer, v2 Min, v2 Max, u32 color)
{
    u32 MinX = (u32)Min.x;
    u32 MinY = (u32)Min.y;
    u32 MaxX = (u32)Max.x;
    u32 MaxY = (u32)Max.y;

    if(MinX < 0) MinX = 0;
    if(MinY < 0) MinY = 0;
    if(MaxX > RenderBuffer->Width) MaxX = RenderBuffer->Width;
    if(MaxY > RenderBuffer->Height) MaxY = RenderBuffer->Height;

    u32 Pitch = RenderBuffer->Width * sizeof(u32);
    u8* Row = ((u8*)RenderBuffer->Memory + MinX*sizeof(u32) + MinY*Pitch);

    for(u32 Y = MinY; Y < MaxY; ++Y)
    {
        u32* Pixel = (u32*)Row;
        
        for(u32 X = MinX; X < MaxX; ++X)
        {
            *Pixel++ = color;
        }
        Row += Pitch;
    }
}

static void
CirclePoints(texture_t* Texture, v2 C, v2 P, u32 Color)
{
    DrawPixel(Texture, (u32)( P.x + C.x), (u32)( P.y + C.y), Color);
    DrawPixel(Texture, (u32)( P.x + C.x), (u32)(-P.y + C.y), Color);
    DrawPixel(Texture, (u32)(-P.x + C.x), (u32)(-P.y + C.y), Color);
    DrawPixel(Texture, (u32)(-P.x + C.x), (u32)( P.y + C.y), Color);

    DrawPixel(Texture, (u32)( P.y + C.x), (u32)( P.x + C.y), Color);
    DrawPixel(Texture, (u32)( P.y + C.x), (u32)(-P.x + C.y), Color);
    DrawPixel(Texture, (u32)(-P.y + C.x), (u32)(-P.x + C.y), Color);
    DrawPixel(Texture, (u32)(-P.y + C.x), (u32)( P.x + C.y), Color);
}

static void
CircleLinePoints(texture_t* Texture, v2 C, v2 P, u32 Color)
{
    DrawLine(Texture, V2(C.x + P.x, C.y + P.y), V2(C.x - P.x, C.y + P.y), Color);
    DrawLine(Texture, V2(C.x + P.x, C.y - P.y), V2(C.x - P.x, C.y - P.y), Color);
    DrawLine(Texture, V2(C.x + P.y, C.y + P.x), V2(C.x - P.y, C.y + P.x), Color);
    DrawLine(Texture, V2(C.x + P.y, C.y - P.x), V2(C.x - P.y, C.y - P.x), Color);
}

void
DrawCircle(v2 P, u32 Width, u32 Height, r32 Radius, r32 Rotation, u32 Color)
{
    texture_t CircleTexture = {};

    CircleTexture.Width = Width;
    CircleTexture.Height = Height;
    CircleTexture.Memory = (u32*)calloc(Width*Height, sizeof(u32));

    v2 TextureOrigin = V2i((Width / 2) - 1, (Height / 2) - 1);
    v2 TextureOriginN = TextureOrigin * V2i(1/Width, 1/Height);

    i32 X = 0;
    i32 Y = (i32)Radius;
    i32 d = 3 - 2*(i32)Radius;
    CirclePoints(ColorBuffer, P, V2i(X, Y), Color);
    while(X <= Y)
    {
        if(d <= 0)
        {
            d = d + 4*X + 6;
        }
        else
        {
            d = d + 4*X - 4*Y + 10;
            Y--;
        }
        X++;
        CirclePoints(ColorBuffer, P, V2i(X, Y), Color);
    }
    v2 LineMax = V2(Radius, 0.0f);
    LineMax = P + rotate(LineMax, Rotation);
    DrawLine(ColorBuffer, P, LineMax, Color);

    //v2 XAxis = Width*V2(1, 0);
    //v2 YAxis = Height*V2(0, 1);

    // TODO: Make it rotate around TextureOrigin and not around P;
    //XAxis = rotate(XAxis, Rotation);
    //YAxis = rotate(YAxis, Rotation);

    //DrawRotRect(ColorBuffer, P, XAxis, YAxis, Color, &CircleTexture);
}

void
DrawFilledCircle(v2 P, u32 Width, u32 Height, r32 Radius, u32 Color)
{
    texture_t BallTexture = {};

    BallTexture.Width = Width;
    BallTexture.Height = Height;
    BallTexture.Memory = (u32*)calloc(Width*Height, sizeof(u32));

    v2 TextureOrigin = V2i((Width / 2) - 1, (Height / 2) - 1);
    v2 TextureOriginN = TextureOrigin * V2i(1/Width, 1/Height);

    i32 X = 0;
    i32 Y = (i32)Radius;
    i32 d = 3 - 2*(i32)Radius;
    CircleLinePoints(&BallTexture, TextureOrigin, V2i(X, Y), Color);
    while(X <= Y)
    {
        if(d <= 0)
        {
            d = d + 4*X + 6;
        }
        else
        {
            d = d + 4*X - 4*Y + 10;
            Y--;
        }
        X++;
        CircleLinePoints(&BallTexture, TextureOrigin, V2i(X, Y), Color);
    }

    DrawRotRect(ColorBuffer, P, Width*V2(1, 0), Height*V2(0, 1), Color, &BallTexture);
}

void
DrawPolygon(v2 P, std::vector<v2> Vertices, u32 Color)
{
    for(u32 PIndex = 0;
        PIndex < Vertices.size();
        ++PIndex)
    {
        i32 CurrIndex = PIndex;
        i32 NextIndex = (PIndex + 1) % Vertices.size();
        DrawLine(ColorBuffer, Vertices[CurrIndex], Vertices[NextIndex], Color);
    }
}

void DestroyTexture(texture_t* Texture)
{
    free(Texture->Memory);
    free(Texture);
}

void DestroyWindow(void)
{
    DestroyTexture(ColorBuffer);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
