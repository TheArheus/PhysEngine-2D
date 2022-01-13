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

    for(constraint* Constraint : Constraints)
    {
        free(Constraint);
    }
}

void world::Update(r32 DeltaTime)
{
    std::vector<constraint*> Penetrations;
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

            if(A != B)
            {
                std::vector<contact> Contacts;
                if(IsColliding(A, B, &Contacts))
                {
                    A->IsColliding = true;
                    B->IsColliding = true;

                    for(contact Contact : Contacts)
                    {
                        constraint* CollisionConstraint = (constraint*)calloc(sizeof(constraint), 1);
                        CollisionConstraint->PenetrationConstraint(A, B, Contact.Start, Contact.End, Contact.Normal);
                        Penetrations.push_back(CollisionConstraint);

                        DrawFilledCircle(Contact.Start - 2, 6, 6, 2, 0xFFFFFF00);
                        DrawFilledCircle(Contact.End - 2, 6, 6, 2, 0xFFFFFF00);
                        DrawLine(ColorBuffer, Contact.Start, Contact.Start + Contact.Normal * 15, 0xFFFFFF00);
                    }
                }
                else
                {
                    A->IsColliding = false;
                    B->IsColliding = false;
                }
            }
        }
    }

    for(constraint* Constraint : Constraints)
    {
        Constraint->PreSolve(DeltaTime);
    }

    for(constraint* Constraint : Penetrations)
    {
        Constraint->PreSolve(DeltaTime);
    }

    for(u32 Index = 0;
        Index < 10;
        ++Index)
    {
        for(constraint* Constraint : Constraints)
        {
            Constraint->Solve();
        }

        for(constraint* Constraint : Penetrations)
        {
            Constraint->Solve();
        }
    }

    for(constraint* Constraint : Constraints)
    {
        Constraint->PostSolve();
    }

    for(constraint* Constraint : Penetrations)
    {
        Constraint->PostSolve();
    }

    for(constraint* Constraint : Penetrations)
    {
        free(Constraint);
    }
}
