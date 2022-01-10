world::world(r32 Gravity = -9.8f)
{
    G = -Gravity;
}

world::~world()
{
    for(body* Body : Bodies)
    {
        delete Body;
    }
}

void world::Update(r32 DeltaTime)
{
    for(body* Body : Bodies)
    {
        Body->AddForce(V2(0.0f, G*Body->Mass));
        for(v2 Force : Forces)
        {
            Body->AddForce(Force);
        }
        for(r32 Torque : Torques)
        {
            Body->AddTorque(Torque);
        }

        Body->IntegrateForces(DeltaTime);
        Body->IntegrateVelocities(DeltaTime);
    }

    for(constraint* Constraint : Constraints)
    {
        Constraint->Solve();
    }

    CheckCollisions();
}

void world::CheckCollisions()
{
    for(u32 AIndex = 0; 
        AIndex <= Bodies.size() - 1;
        ++AIndex)
    {
        for(u32 BIndex = AIndex + 1;
            BIndex < Bodies.size();
            ++BIndex)
        {
            body* A = Bodies[AIndex];
            body* B = Bodies[BIndex];

            contact Contact = {};
            if(A != B)
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
}
