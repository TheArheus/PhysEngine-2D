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
b32 CreatingPolygon = false;
std::vector<v2> MousePolygon;

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
    World.Bodies.push_back(new body(V2(ColorBuffer->Width - 100, ColorBuffer->Height - 50), 0.0f, new shape(ColorBuffer->Width - 200, 50)));
    World.Bodies.push_back(new body(V2(ColorBuffer->Width / 2 + 100, ColorBuffer->Height / 2 + 100), 0.0f, new shape(200, 200, 100)));
    //World.Bodies.push_back(new body(V2(ColorBuffer->Width / 2, ColorBuffer->Height / 2), 1.0f, new shape(50, 50, 23)));

    World.Bodies[0]->Rotation = 0.0;
    World.Bodies[0]->Restitution = 0.5;
    World.Bodies[1]->Rotation = 1.4;
    World.Bodies[1]->Restitution = 0.5;
    
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
            if(event.key.keysym.sym == SDLK_r)     CreatingPolygon   =  true;
            break;
        case SDL_KEYUP:
            if(event.key.keysym.sym == SDLK_UP)    World.PushForce.y = 0;
            if(event.key.keysym.sym == SDLK_DOWN)  World.PushForce.y = 0;
            if(event.key.keysym.sym == SDLK_LEFT)  World.PushForce.x = 0;
            if(event.key.keysym.sym == SDLK_RIGHT) World.PushForce.x = 0;
            if(event.key.keysym.sym == SDLK_r)     
            {
                CreatingPolygon = false;
                //body* Body = new body(MousePolygon[0], 1.0f, new shape(50, 50, MousePolygon));
                //World.Bodies.push_back(Body);
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
            {
                if(event.button.button == SDL_BUTTON_LEFT)
                {
                    i32 MouseX, MouseY;
                    SDL_GetMouseState(&MouseX, &MouseY);
#if 0
                    if(CreatingPolygon)
                    {
                        MousePolygon.push_back(V2(MouseX, MouseY));
                    }
                    else
                    {
                        MousePolygon.clear();
                        body* NewBody = new body(V2(MouseX, MouseY), 1.0f, new shape(50, 50, 23));
                        World.Bodies.push_back(NewBody);
                    }
#endif
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
                        NewBody->Restitution = 0.1f;
                        NewBody->Friction = 0.7f;
                        World.Bodies.push_back(NewBody);
                    }
                    else
                    {
                        body* NewBody = new body(V2(MouseX, MouseY), 1.0f, new shape(50, 50, 23));
                        World.Bodies.push_back(NewBody);
                    }
                }
            } break;
        case SDL_MOUSEMOTION:
            {
                i32 MouseX, MouseY;
                SDL_GetMouseState(&MouseX, &MouseY);
                //World.Bodies[2]->d = V2(MouseX, MouseY) - V2(World.Bodies[2]->Shape->Width / 2, World.Bodies[2]->Shape->Height / 2);
            } break;
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
    for(u32 AIndex = 0; 
        AIndex <= World.Bodies.size() - 1;
        ++AIndex)
    {
        for(u32 BIndex = AIndex + 1;
            BIndex < World.Bodies.size();
            ++BIndex)
        {
            body* A = World.Bodies[AIndex];
            body* B = World.Bodies[BIndex];

            contact Contact = {};
            //if(A != B)
            {
                if (IsColliding(A, B, &Contact))
                {
                    ResolveCollision(&Contact);

                    A->IsColliding = true;
                    B->IsColliding = true;

                    DrawFilledCircle(Contact.Start - 2, 6, 6, 2, 0xFFFFFF00);
                    DrawFilledCircle(Contact.End - 2, 6, 6, 2, 0xFFFFFF00);
                    DrawLine(ColorBuffer, Contact.Start, Contact.Start + Contact.Normal * 15, 0xFFFFFF00);
                }
                else
                {
                    A->IsColliding = false;
                    B->IsColliding = false;
                }
            }
        }
    }

    for(body* Body : World.Bodies)
    {
        Body->AddForce(V2(0.0f, 9.8f*Body->Mass));
        Body->Update(DeltaTime);
    }
}

static void 
render(void)
{

    v2 P = {(r32)ColorBuffer->Width/2, (r32)ColorBuffer->Height/2};

    //DrawRect(ColorBuffer, World.Liquid.Min, World.Liquid.Max, 0xFF83D7EE);

    for(body* Body : World.Bodies)
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
