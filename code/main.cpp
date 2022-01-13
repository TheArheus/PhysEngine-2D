#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
#include "display.h"
#include "p_bodies.cpp"
#include "constraint.cpp"
#include "world.cpp"

bool is_running;

i32 PreviousFrameTime = 0;
r32 DeltaTime = 0;
r32 TimeForFrame = 0;
r32 dtForFrame = 0;
b32 CreatingPolygon = false;
std::vector<v2> MousePolygon;

static void 
setup(world* World)
{
    ColorBuffer->Memory = (u32*)malloc(sizeof(u32)*ColorBuffer->Width*ColorBuffer->Height);

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, 
                                SDL_TEXTUREACCESS_STREAMING, 
                                ColorBuffer->Width, ColorBuffer->Height);

    TimeForFrame = 1.0f / FPS;
    World->Anchor = V2(ColorBuffer->Width / 2, 30);
    World->SpringTightness = 3000;
    World->SpringRestLength = 200;

#if 0
    const int BodiesCount = 9;
    for(u32 BodyIndex = 0;
        BodyIndex < BodiesCount;
        ++BodyIndex)
    {
        r32 Mass = (BodyIndex == 0) ? 0.0f : 1.0f;
        body* NewBody = new body(V2(ColorBuffer->Width / 2 - (BodyIndex * 40), 100), Mass, new shape(30, 30));
        World->Bodies.push_back(NewBody);
    }

    for(u32 BodyIndex = 0;
        BodyIndex < BodiesCount - 1;
        ++BodyIndex)
    {
        body* A = World->Bodies[BodyIndex];
        body* B = World->Bodies[BodyIndex + 1];

        constraint* NewConstraint = (constraint*)calloc(sizeof(constraint), 1);
        NewConstraint->JointConstraint(A, B, A->d);
        World->Constraints.push_back(NewConstraint);
    }
#else
    World->Bodies.push_back(new body(V2(ColorBuffer->Width - 100, ColorBuffer->Height - 50), 0.0f, new shape(ColorBuffer->Width - 200, 50)));
    World->Bodies.push_back(new body(V2(ColorBuffer->Width / 2 + 100, ColorBuffer->Height / 2 + 100), 0.0f, new shape(200, 200, 100)));

    //World->Bodies[0]->Rotation = 0.0;
    World->Bodies[0]->Restitution = 0.5;
    //World->Bodies[1]->Rotation = 1.4;
    World->Bodies[1]->Restitution = 0.5;
#endif
    
    World->PushForce = V2(0, 0);

    World->Liquid.Min = V2(0, 0.5*ColorBuffer->Height);
    World->Liquid.Max = V2(ColorBuffer->Width, ColorBuffer->Height);
}

static void 
process_input(world* World)
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
            if(event.key.keysym.sym == SDLK_UP)    World->PushForce.y = -25;
            if(event.key.keysym.sym == SDLK_DOWN)  World->PushForce.y =  25;
            if(event.key.keysym.sym == SDLK_LEFT)  World->PushForce.x = -25;
            if(event.key.keysym.sym == SDLK_RIGHT) World->PushForce.x =  25;
            if(event.key.keysym.sym == SDLK_r)     CreatingPolygon   =  true;
            break;
        case SDL_KEYUP:
            if(event.key.keysym.sym == SDLK_UP)    World->PushForce.y = 0;
            if(event.key.keysym.sym == SDLK_DOWN)  World->PushForce.y = 0;
            if(event.key.keysym.sym == SDLK_LEFT)  World->PushForce.x = 0;
            if(event.key.keysym.sym == SDLK_RIGHT) World->PushForce.x = 0;
            if(event.key.keysym.sym == SDLK_r)     
            {
                CreatingPolygon = false;
                //body* Body = new body(MousePolygon[0], 1.0f, new shape(50, 50, MousePolygon));
                //World->Bodies.push_back(Body);
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
            {
                if(event.button.button == SDL_BUTTON_LEFT)
                {
                    i32 MouseX, MouseY;
                    SDL_GetMouseState(&MouseX, &MouseY);

                    if(CreatingPolygon)
                    {
                        std::vector<v2> Vertices = 
                        {
                            V2(20, 60),
                            V2(-40, 20),
                            V2(-20, -60),
                            V2(20, -60),
                            V2(40, 20)
                        };
                        body* NewBody = new body(V2(MouseX, MouseY), 1.0f, new shape(50, 50, Vertices));
                        NewBody->Restitution = 0.5f;
                        NewBody->Friction = 0.7f;
                        World->Bodies.push_back(NewBody);
                    }
                    else
                    {
                        body* NewBody = new body(V2(MouseX, MouseY), 1.0f, new shape(50, 50, 23));
                        World->Bodies.push_back(NewBody);
                    }
                }
                if(event.button.button == SDL_BUTTON_RIGHT)
                {
                    i32 MouseX, MouseY;
                    SDL_GetMouseState(&MouseX, &MouseY);

                    body* NewBody = new body(V2(MouseX, MouseY), 1.0f, new shape(50, 50));
                    NewBody->Restitution = 0.5f;
                    NewBody->Friction = 0.7f;
                    World->Bodies.push_back(NewBody);
                }
            } break;
        case SDL_MOUSEMOTION:
            {
                i32 MouseX, MouseY;
                SDL_GetMouseState(&MouseX, &MouseY);
                //World->Bodies[2]->d = V2(MouseX, MouseY) - V2(World->Bodies[2]->Shape->Width / 2, World->Bodies[2]->Shape->Height / 2);
            } break;
    }
}

static void 
update(world* World)
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
    World->Update(DeltaTime);
}

static void 
render(world* World)
{

    for(body* Body : World->Bodies)
    {
        u32 Color = Body->IsColliding ? 0xFFFF0000 : 0xFFFFFFFF;
        switch(Body->Shape->Type)
        {
            case ShapeType_Circle:
            {
                // NOTE: Warning!!! It is rotating around Body->d!!!!
                // It should rotate around Texture Origin.
                // If I just change the axes with rotating, then it will 
                // not affect the point, it rotating about
                // But still rotating corectly!!! Chenge only how it rotate
                DrawCircle(Body->d, Body->Shape->Width, Body->Shape->Height, Body->Shape->Radius, Body->Rotation, Color);
            } break;
            case ShapeType_Polygon:
            {
                DrawPolygon(Body->d, Body->Shape->WorldVertices, Color);
            } break;
            case ShapeType_Box:
            {
                DrawPolygon(Body->d, Body->Shape->WorldVertices, Color);
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

    world* World = new world(-9.8f);

    setup(World);

    while(is_running)
    {
        process_input(World);
        update(World);
        render(World);
    }

    DestroyWindow();

    return 0;
}
