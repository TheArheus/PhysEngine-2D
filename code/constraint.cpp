vN constraint::
GetVelocities()
{
    vN Velocities = VN(6);

    Velocities.Values[0] = A->dP.x;
    Velocities.Values[1] = A->dP.y;
    Velocities.Values[2] = A->AngularVelocity;

    Velocities.Values[3] = B->dP.x;
    Velocities.Values[4] = B->dP.y;
    Velocities.Values[5] = B->AngularVelocity;

    return Velocities;
}

mNxM constraint::
GetInvM()
{
    mNxM InvM = MNxM(6, 6);

    InvM.Vectors[0].Values[0] = A->InvMass;
    InvM.Vectors[1].Values[1] = A->InvMass;
    InvM.Vectors[2].Values[2] = A->InvI;
    InvM.Vectors[3].Values[3] = B->InvMass;
    InvM.Vectors[4].Values[4] = B->InvMass;
    InvM.Vectors[5].Values[5] = B->InvI;

    return InvM;
}

void constraint::
JointConstraint(body* BodyA, body* BodyB, v2 AnchorPoint)
{
    Jacobian = MNxM(1, 6); // Jacobian is 1 rows and 6 columns here
    CachedLambda = VN(1);

    this->A = BodyA;
    this->B = BodyB;

    this->LocalPointA = A->WorldSpaceToLocalSpace(AnchorPoint);
    this->LocalPointB = B->WorldSpaceToLocalSpace(AnchorPoint);

    this->Type = ConstraintType_Joint;
}

void constraint::
PenetrationConstraint(body* BodyA, body* BodyB, v2 CollisionPointA_, v2 CollisionPointB_, v2 Normal_, r32 Friction_ = 0.0f)
{
    Jacobian = MNxM(2, 6); // Jacobian is 1 rows and 6 columns here
    CachedLambda = VN(2);

    this->A = BodyA;
    this->B = BodyB;

    this->Friction = Friction_;

    this->Norm        = A->WorldSpaceToLocalSpace(Normal_);
    this->LocalPointA = A->WorldSpaceToLocalSpace(CollisionPointA_);
    this->LocalPointB = B->WorldSpaceToLocalSpace(CollisionPointB_);

    this->Type = ConstraintType_Penetration;
}

void constraint::
PreSolve(r32 DeltaTime)
{
    v2 PointA   = A->LocalSpaceToWorldSpace(LocalPointA);
    v2 PointB   = B->LocalSpaceToWorldSpace(LocalPointB);
    v2 Normal_  = A->LocalSpaceToWorldSpace(Norm);

    v2 RadiusA = PointA - A->d;
    v2 RadiusB = PointB - B->d;

    v2 va = A->dP + Cross(V3(0, 0, A->AngularVelocity), V3(RadiusA, 0)).xy; 
    v2 vb = B->dP + Cross(V3(0, 0, B->AngularVelocity), V3(RadiusB, 0)).xy; 
    r32 RelativeVelocityDotN = Inner(va - vb, Normal_);

    if(Type == ConstraintType_Joint)
    {
        v2 J1 = (PointA - PointB) * 2.0f;
        Jacobian.Vectors[0][0] = J1.x;
        Jacobian.Vectors[0][1] = J1.y;

        r32 J2 = 2.0f*Cross(RadiusA, PointA - PointB);
        Jacobian.Vectors[0][2] = J2;

        v2 J3 = 2.0f*(PointB - PointA);
        Jacobian.Vectors[0][3] = J3.x;
        Jacobian.Vectors[0][4] = J3.y;

        r32 J4 = 2.0f*Cross(RadiusB, PointB - PointA);
        Jacobian.Vectors[0][5] = J4;
    }
    else if(Type == ConstraintType_Penetration)
    {

        Jacobian.Vectors[0][0] = -Normal_.x;
        Jacobian.Vectors[0][1] = -Normal_.y;

        Jacobian.Vectors[0][2] = -Cross(RadiusA, Normal_);

        Jacobian.Vectors[0][3] =  Normal_.x;
        Jacobian.Vectors[0][4] =  Normal_.y;

        Jacobian.Vectors[0][5] =  Cross(RadiusB, Normal_);

        Friction = Max(A->Friction, B->Friction);
#if 1
        if(Friction > 0.0f)
        {
            v2 Tangent_ = Normal(Normal_);

            Jacobian.Vectors[1][0] = -Tangent_.x;
            Jacobian.Vectors[1][1] = -Tangent_.y;
                                  
            Jacobian.Vectors[1][2] = -Cross(RadiusA, Tangent_);
                                  
            Jacobian.Vectors[1][3] =  Tangent_.x;
            Jacobian.Vectors[1][4] =  Tangent_.y;
                                  
            Jacobian.Vectors[1][5] =  Cross(RadiusB, Tangent_);
        }
#endif
    }

    mNxM JacobianTransposed = Transpose(Jacobian);
    vN Impulses = JacobianTransposed * CachedLambda;

    A->ApplyImpulseLinear(V2(Impulses.Values[0], Impulses.Values[1]));
    A->ApplyImpulseAngular(Impulses.Values[2]);

    B->ApplyImpulseLinear(V2(Impulses.Values[3], Impulses.Values[4]));
    B->ApplyImpulseAngular(Impulses.Values[5]);

    r32 e = Min(A->Restitution, B->Restitution);
    r32 Beta = 0.1f;
    r32 PositionalError = 0;
    if(Type == ConstraintType_Joint)
    {
        PositionalError = Inner((PointB - PointA), (PointB - PointA));
        PositionalError = Max(0.0f, PositionalError - 0.01f);
        Bias = Beta / DeltaTime * PositionalError;
    }
    else if(Type == ConstraintType_Penetration)
    {
        Beta = 0.2f;
        PositionalError = Inner(PointB - PointA, -1.0f*Normal_);
        PositionalError = Min(0.0f, PositionalError + 0.01f);
        Bias = Beta / DeltaTime * PositionalError + (e * RelativeVelocityDotN);
    }

}

void constraint::
Solve()
{
    vN   Velocities = GetVelocities();
    mNxM InvMasses  = GetInvM();
    mNxM JacobianTransposed = Transpose(Jacobian);

    mNxM lhs = Jacobian * InvMasses  * JacobianTransposed;
    vN   rhs = Jacobian * Velocities * -1.0f;
    rhs.Values[0] -= Bias;

    vN Lambda = SolveGaussSeidel(lhs, rhs);
    if(Type == ConstraintType_Joint)
    {
        CachedLambda = CachedLambda + Lambda;
    }
    else if(Type == ConstraintType_Penetration)
    {
        vN OldLambda  = CachedLambda;
        CachedLambda  = CachedLambda + Lambda;
        CachedLambda.Values[0] = (CachedLambda[0] < 0.0f) ? 0.0f : CachedLambda[0];

        if(Friction > 0.0f)
        {
            r32 MaxFriction = CachedLambda[0] * Friction;
            CachedLambda.Values[1] = Clamp(-MaxFriction, CachedLambda[1], MaxFriction);
        }

        Lambda = CachedLambda - OldLambda;
    }

    vN Impulses = JacobianTransposed * Lambda;

    A->ApplyImpulseLinear(V2(Impulses[0], Impulses[1]));
    A->ApplyImpulseAngular(Impulses[2]);

    B->ApplyImpulseLinear(V2(Impulses[3], Impulses[4]));
    B->ApplyImpulseAngular(Impulses[5]);
}

void constraint::
PostSolve()
{
}

