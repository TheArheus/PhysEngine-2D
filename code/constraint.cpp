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

    this->A = BodyA;
    this->B = BodyB;

    APoint = A->WorldSpaceToLocalSpace(AnchorPoint);
    BPoint = B->WorldSpaceToLocalSpace(AnchorPoint);
}

void constraint::
Solve()
{
    v2 PointA = A->LocalSpaceToWorldSpace(APoint);
    v2 PointB = B->LocalSpaceToWorldSpace(BPoint);

    v2 RadiusA = PointA - A->d;
    v2 RadiusB = PointB - B->d;

    v2 J1 = (PointA - PointB) * 2.0f;
    Jacobian.Vectors[0].Values[0] = J1.x;
    Jacobian.Vectors[1].Values[0] = J1.y;

    r32 J2 = 2.0f*Cross(RadiusA, PointA - PointB);
    Jacobian.Vectors[2].Values[0] = J2;

    v2 J3 = 2.0f*(PointB - PointA);
    Jacobian.Vectors[3].Values[0] = J3.x;
    Jacobian.Vectors[4].Values[0] = J3.y;

    r32 J4 = 2.0f*Cross(RadiusB, PointB - PointA);
    Jacobian.Vectors[5].Values[0] = J4;
    
    vN   Velocities = GetVelocities();
    mNxM InvMasses  = GetInvM();
    mNxM JacobianTransposed = Transpose(Jacobian);

    vN   rhs = Jacobian * Velocities * -1.0f;
    mNxM lhs = Jacobian * InvMasses * JacobianTransposed;

    vN Lambda = SolveGaussSeidel(lhs, rhs);
    vN Impulses = JacobianTransposed * Lambda;

    A->ApplyImpulseLinear(V2(Impulses.Values[0], Impulses.Values[1]));
    A->ApplyImpulseAngular(Impulses.Values[2]);

    B->ApplyImpulseLinear(V2(Impulses.Values[3], Impulses.Values[4]));
    B->ApplyImpulseAngular(Impulses.Values[5]);
}


