#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
#include "display.h"
#include "p_bodies.cpp"

bool is_running;

i32 PreviousFrameTime = 0;
r32 DeltaTime = 0;
r32 TimeForFrame = 0;
r32 dtForFrame = 0;

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
    World.SpringTightness = 3000;
    World.SpringRestLength = 200;
    World.Bodies.push_back(new body(V2(150, 150), 1.0, new shape(50, 50, 23)));
    World.Bodies.push_back(new body(V2(300, 300), 1.0, new shape(50, 50, 23)));
    
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
                World.Bodies.push_back(new body(V2(MouseX, MouseY), 5.0f, new shape(50, 50, 23)));
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

    // Bodies update

    for(body* Body : World.Bodies)
    {
        for(body* BodyCollider : World.Bodies)
        {
            if(Body != BodyCollider)
            {
                contact Contact = {};
                if (IsColliding(Body, BodyCollider, &Contact))
                {
                    Body->IsColliding = true;
                    BodyCollider->IsColliding = true;

                    DrawFilledCircle(Contact.Start - 2, 6, 6, 2, 0xFFFFFF00);
                    DrawFilledCircle(Contact.End - 2, 6, 6, 2, 0xFFFFFF00);
                    DrawLine(ColorBuffer, Contact.End, Contact.Start, 0xFFFFFF00);
                    ResolveCollision(&Contact);
                }
                else
                {
                    Body->IsColliding = false;
                    BodyCollider->IsColliding = false;
                }
            }
        }
        Body->AddForce(V2(0.0f, 9.8f*Body->Mass));
        //Body->AddForce(World.PushForce);
        Body->AddForce(GenerateDragForce(*Body, 0.002));
        //Body->AddTorque(100/PixelsPerMeter);
        Body->Update(DeltaTime);

        switch(Body->Shape->Type)
        {
            case ShapeType_Circle:
            {
                if((Body->d.x) <= 0)
                {
                    v2 Normal = V2(1, 0);
                    Body->dP.x *= -1;
                    Body->d.x = 0;
                }
                else if((Body->d.x + 2.0f*Body->Shape->Radius) >= ColorBuffer->Width)
                {
                    v2 Normal = V2(-1, 0);
                    Body->dP.x *= -1;
                    Body->d.x = ColorBuffer->Width - 2.0f*Body->Shape->Radius;
                }
                if((Body->d.y) <= 0)
                {
                    v2 Normal = V2(0, 1);
                    Body->dP.y *= -1;
                    Body->d.y = 0;
                }
                else if((Body->d.y + 2.0f*Body->Shape->Radius) >= ColorBuffer->Height)
                {
                    v2 Normal = V2(0, -1);
                    Body->dP.y *= -1;
                    Body->d.y = ColorBuffer->Height - 2.0f*Body->Shape->Radius;
                }
            } break;
        }
    }
}

static void 
render(void)
{

    v2 P = {(r32)ColorBuffer->Width/2, (r32)ColorBuffer->Height/2};

    //DrawRect(ColorBuffer, World.Liquid.Min, World.Liquid.Max, 0xFF83D7EE);

    for(body* Body : World.Bodies)
    {
        switch(Body->Shape->Type)
        {
            case ShapeType_Circle:
            {
                u32 Color = Body->IsColliding ? 0xFFFF0000 : 0xFFFFFFFF;
                // NOTE: Warning!!! It is rotating around Body->d!!!!
                // It should rotate around Texture Origin.
                // If I just change the axes with rotating, then it will 
                // not affect the point, it rotating about
                // But still rotating corectly!!! Chenge only how it rotate
                DrawCircle(Body->d, Body->Shape->Width, Body->Shape->Height, Body->Shape->Radius, Body->Rotation, Color);
            } break;
            case ShapeType_Polygon:
            {
                DrawPolygon(Body->d, Body->Shape->WorldVertices, 0xFFFFFFFF);
            } break;
            case ShapeType_Box:
            {
                DrawPolygon(Body->d, Body->Shape->WorldVertices, 0xFFFFFFFF);
            }
        }
    }

    RenderColorBuffer();
    ClearColorBuffer(ColorBuffer, 0xFF056263);

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
