#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
#include "display.h"
#include "p_particle.cpp"

bool is_running;

i32 PreviousFrameTime = 0;
r32 DeltaTime = 0;
r32 TimeForFrame = 0;
r32 dtForFrame = 0;

u32 CubeFOV = 700;

world World;

static void 
setup(void)
{
    ColorBuffer->Memory = (u32*)malloc(sizeof(u32)*ColorBuffer->Width*ColorBuffer->Height);

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, 
                                SDL_TEXTUREACCESS_STREAMING, 
                                ColorBuffer->Width, ColorBuffer->Height);

    TimeForFrame = 1.0f / FPS;
    World.Anchor = V2(ColorBuffer->Width / 2, 30);
    World.SpringTightness = 600;
    World.SpringRestLength = 10;

    World.ParticlesCount = 8;
    for(u32 ParticleIndex = 1;
        ParticleIndex < World.ParticlesCount + 1;
        ++ParticleIndex)
    {
        World.Particles.push_back(new particle(V2(World.Anchor.x, World.Anchor.y + (ParticleIndex * World.SpringRestLength)), 5));
    }

    //World.Particles.push_back(new particle(V2(100, 50), 5));
    
    World.PushForce = V2(0, 0);

    World.Liquid.Min = V2(0, 0.5*ColorBuffer->Height);
    World.Liquid.Max = V2(ColorBuffer->Width, ColorBuffer->Height);
}

static void 
process_input(void)
{
    SDL_Event event;
    SDL_PollEvent(&event);

    switch(event.type)
    {
        case SDL_QUIT:
            is_running = false;
            break;
        case SDL_KEYDOWN:
            if(event.key.keysym.sym == SDLK_ESCAPE) is_running = false;
            if(event.key.keysym.sym == SDLK_UP)    World.PushForce.y = -25;
            if(event.key.keysym.sym == SDLK_DOWN)  World.PushForce.y =  25;
            if(event.key.keysym.sym == SDLK_LEFT)  World.PushForce.x = -25;
            if(event.key.keysym.sym == SDLK_RIGHT) World.PushForce.x =  25;
            break;
        case SDL_KEYUP:
            if(event.key.keysym.sym == SDLK_UP)    World.PushForce.y = 0;
            if(event.key.keysym.sym == SDLK_DOWN)  World.PushForce.y = 0;
            if(event.key.keysym.sym == SDLK_LEFT)  World.PushForce.x = 0;
            if(event.key.keysym.sym == SDLK_RIGHT) World.PushForce.x = 0;
            break;
        case SDL_MOUSEBUTTONDOWN:
            if(event.button.button == SDL_BUTTON_LEFT)
            {
                i32 MouseX, MouseY;
                SDL_GetMouseState(&MouseX, &MouseY);
                particle* Particle = new particle(V2(MouseX, MouseY), 5);
                World.Particles.push_back(Particle);
            }
    }
}

static void 
update(void)
{
    dtForFrame += TimeForFrame;
    int TimeToWait = FRAME_TARGET_TIME - (SDL_GetTicks() + PreviousFrameTime);

    if(TimeToWait > 0 && (TimeToWait <= FRAME_TARGET_TIME))
    {
        SDL_Delay(TimeToWait);
    }

    DeltaTime = (SDL_GetTicks() - PreviousFrameTime) / 1000.0f;
    if (DeltaTime > TimeForFrame) DeltaTime = TimeForFrame;
    PreviousFrameTime = SDL_GetTicks();

    // Particles update

    v2 SpringForce = GenerateSpringForce(*World.Particles[0], World.Anchor, World.SpringRestLength, World.SpringTightness);
    World.Particles[0]->AddForce(SpringForce);
    for(u32 ParticleIndex = 1;
        ParticleIndex < World.ParticlesCount;
        ++ParticleIndex)
    {
        SpringForce = GenerateSpringForce(*World.Particles[ParticleIndex], *World.Particles[ParticleIndex - 1], World.SpringRestLength, World.SpringTightness);
        World.Particles[ParticleIndex]->AddForce(SpringForce);
        World.Particles[ParticleIndex-1]->AddForce(-SpringForce);
    }

    for(particle* Particle : World.Particles)
    {
#if 0
        if(Particle->d.y > World.Liquid.Min.y)
        {
            r32 Value = 0.3f;
            Particle->AddForce(GenerateDragForce(*Particle, Value));
        }
        else
        {
            Particle->AddForce(V2(2.0f, 0.0f));
        }
#else
#endif
        Particle->AddForce(V2(0.0f, 9.8f*Particle->Mass));
        Particle->AddForce(World.PushForce);
        Particle->AddForce(GenerateDragForce(*Particle, 0.002));
        Particle->Integrate(DeltaTime);

        if((Particle->d.x - 3) <= 0)
        {
            v2 Normal = V2(1, 0);
            Particle->dP.x *= -1;
            Particle->d.x = 3;
        }
        else if((Particle->d.x + 3) >= ColorBuffer->Width)
        {
            v2 Normal = V2(-1, 0);
            Particle->dP.x *= -1;
            Particle->d.x = ColorBuffer->Width - 3;
        }
        if((Particle->d.y - 3) <= 0)
        {
            v2 Normal = V2(0, 1);
            Particle->dP.y *= -1;
            Particle->d.y = 3;
        }
        else if((Particle->d.y + 3) >= ColorBuffer->Height)
        {
            v2 Normal = V2(0, -1);
            Particle->dP.y *= -1;
            Particle->d.y = ColorBuffer->Height - 3;
        }
    }
}

static void 
render(void)
{
    ClearColorBuffer(ColorBuffer, 0xFF056263);
    v2 P = {(r32)ColorBuffer->Width/2, (r32)ColorBuffer->Height/2};

    //DrawRect(ColorBuffer, World.Liquid.Min, World.Liquid.Max, 0xFF83D7EE);

    DrawFilledCircle(World.Anchor, 10, 10, 3, 0xFFFFFF00);
    DrawLine(ColorBuffer, World.Anchor + 5, World.Particles[0]->d + 5, 0xFF964B00);
    for(u32 ParticleIndex = 0;
        ParticleIndex < World.ParticlesCount - 1;
        ++ParticleIndex)
    {
        DrawLine(ColorBuffer, World.Particles[ParticleIndex]->d + 5, World.Particles[ParticleIndex + 1]->d + 5, 0xFF964B00);
    }

    for(particle* Particle : World.Particles)
    {
        DrawFilledCircle(Particle->d, 10, 10, 3, 0xFFFFFFFF);
    }

    RenderColorBuffer();

    SDL_RenderPresent(renderer);
}

int main(int argc, char** argv)
{
    is_running = InitWindow();

    setup();

    while(is_running)
    {
        process_input();
        update();
        render();
    }

    DestroyWindow();

    return 0;
}
