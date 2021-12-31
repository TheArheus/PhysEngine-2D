
body::body(v2 P_, r32 Mass_)
{
    this->d = P_;
    this->Mass = Mass_;
    this->InvMass = 1 / Mass_;

    this->Rotation = 0.0f;
    this->AngularVelocity = 0.0f;
    this->AngularAcceleration = 0.0f;

    this->I = 0.0;
    this->InvI = 0.0;

    this->Restitution = 1.0f;

    ClearForces();
    ClearTorque();
}

body::body(v2 P_, r32 Mass_, shape* Shape_)
{
    this->d = P_ - V2(Shape_->Width / 2, Shape_->Height / 2);
    this->Mass = Mass_;
    this->InvMass = 1 / Mass_;

    this->Rotation = 0.0f;
    this->AngularVelocity = 0.0f;
    this->AngularAcceleration = 0.0f;

    this->Shape = Shape_;
    this->I = Shape_->GetMomentOfInertia()*Mass_;
    this->InvI = 1/this->I;

    this->Restitution = 1.0f;

    ClearForces();
    ClearTorque();
}

body::~body()
{
}

void body::IntegrateLinear(r32 DeltaTime)
{
    if(fabsf(this->InvMass) > 0.001)
    {
        this->ddP = this->SumForces * this->InvMass;
        this->dP += this->ddP*DeltaTime;
        this->d  += this->dP*DeltaTime;
        ClearForces();
    }
}

void body::IntegrateAngular(r32 DeltaTime)
{
    if(fabsf(this->InvMass) > 0.001)
    {
        this->AngularAcceleration = this->SumTorque * this->InvI;
        this->AngularVelocity += this->AngularAcceleration * DeltaTime;
        this->Rotation += this->AngularVelocity * DeltaTime;
        ClearTorque();
    }
}

void body::Update(r32 DeltaTime)
{
    IntegrateLinear(DeltaTime);
    IntegrateAngular(DeltaTime);
    b32 IsPolygon = Shape->Type == ShapeType_Box || Shape->Type == ShapeType_Polygon;
    if(IsPolygon)
    {
        Shape->UpdateVertices(d, Rotation);
    }
}

void body::AddForce(const v2& Force)
{
    this->SumForces += Force*PixelsPerMeter;
}

void body::AddTorque(r32 Torque)
{
    this->SumTorque += Torque*PixelsPerMeter;
}

void body::ApplyImpulse(v2 Impulse)
{
    if(fabsf(this->InvMass) > 0.001)
    {
        dP += Impulse * InvMass;
    }
}

void body::ClearForces()
{
    this->SumForces = V2(0.0, 0.0);
}

void body::ClearTorque()
{
    this->SumTorque = 0.0f;
}

v2 GenerateDragForce(const body& Body, r32 k)
{
    v2 DragForce = V2(0, 0);
    if(LengthSqr(Body.dP) > 0)
    {
        v2 DragDirection = -Normalize(Body.dP);
        r32 DragMagnitude = k * LengthSqr(Body.dP);
        DragForce = DragMagnitude*DragDirection;
    }
    return DragForce;
}

v2 GenerateFrictionForce(const body& Body, r32 k)
{
    v2 FrictionForce = V2(0, 0);

    v2 FrictionDirection = -Normalize(Body.dP);
    r32 FrictionMagnitude = k;
    FrictionForce = FrictionDirection*FrictionMagnitude;

    return FrictionForce;
}

v2 GenerateGravitationalForce(const body& BodyA, const body& BodyB, r32 G)
{
    v2 Direction = BodyB.d - BodyA.d;
    r32 Distance = LengthSqr(Direction);
    v2 AttractionDirection = Normalize(Direction);

    r32 AttractionMagnitude = G*BodyA.Mass*BodyB.Mass/Distance;
    v2 AttractionForce = AttractionMagnitude*AttractionDirection;

    return AttractionForce;
}

v2 GenerateSpringForce(const body& Body, v2 Anchor, r32 RestLength, r32 k)
{
    v2 Direction = Body.d - Anchor;

    r32 Displacement = Length(Direction) - RestLength;

    v2 SpringDirection = Normalize(Direction);
    r32 SpringMagnitude = -k * Displacement;

    v2 SpringForce = SpringDirection * SpringMagnitude;
    return SpringForce;
}

v2 GenerateSpringForce(const body& BodyA, const body& BodyB, r32 RestLength, r32 k)
{
    v2 Direction = BodyA.d - BodyB.d;

    r32 Displacement = Length(Direction) - RestLength;

    v2 SpringDirection = Normalize(Direction);
    r32 SpringMagnitude = -k * Displacement;

    v2 SpringForce = SpringDirection * SpringMagnitude;
    return SpringForce;
}

shape::shape(r32 Width, r32 Height)
{
    this->Width = Width;
    this->Height = Height;
    this->Type = ShapeType_Box;

    this->LocalVertices.push_back(V2(-Width/2, -Height/2));
    this->LocalVertices.push_back(V2( Width/2, -Height/2));
    this->LocalVertices.push_back(V2( Width/2,  Height/2));
    this->LocalVertices.push_back(V2(-Width/2,  Height/2));

    this->WorldVertices.resize(this->LocalVertices.size());
}

shape::shape(r32 Width, r32 Height, r32 Radius)
{
    this->Radius = Radius;
    this->Width = Width;
    this->Height = Height;
    this->Type = ShapeType_Circle;
}

shape::shape(r32 Width, r32 Height, std::vector<v2> Vertices)
{
    this->LocalVertices = Vertices;
    this->Width = Width;
    this->Height = Height;
    this->Type = ShapeType_Polygon;
}

shape::~shape()
{
}

void shape::UpdateVertices(v2 P, r32 Rotation)
{
    for(u32 PointIndex = 0;
        PointIndex < this->LocalVertices.size();
        ++PointIndex)
    {
        v2 Point = this->LocalVertices[PointIndex];
        Point = rotate(Point, Rotation);
        Point += P;
        this->WorldVertices[PointIndex] = Point;
    }
}

r32 shape::GetMomentOfInertia()
{
    r32 Result = 0.0;
    switch(this->Type)
    {
        case ShapeType_Circle:
        {
            Result = 0.5f*Square(this->Radius);
        } break;
        case ShapeType_Box:
        {
            Result = (Square(this->Width) + Square(this->Height)) * 0.0833333f;
        } break;
        case ShapeType_Polygon:
        {
        } break;
    }
    return Result;
}


b32 IsColliding(body* A, body* B, contact* ContactInfo = nullptr)
{
    b32 Result = false;
    if(A->Shape->Type == ShapeType_Circle && B->Shape->Type == ShapeType_Circle)
    {
        Result = CircleCircleCollision(A, B, ContactInfo);
    }
    return Result;
}

b32 CircleCircleCollision(body* A, body* B, contact* ContactInfo)
{
    v2 ab = B->d - A->d;

    r32 RadiusSum = A->Shape->Radius + B->Shape->Radius;
    b32 AreColliding = LengthSqr(ab) <= Square(RadiusSum);

    if(AreColliding && ContactInfo)
    {
        ContactInfo->A = A;
        ContactInfo->B = B;
        
        ContactInfo->Normal = Normalize(ab);

        ContactInfo->Start = B->d - ContactInfo->Normal*B->Shape->Radius + V2(B->Shape->Width/2, B->Shape->Height/2);
        ContactInfo->End = A->d + ContactInfo->Normal*A->Shape->Radius + V2(A->Shape->Width/2, A->Shape->Height/2);

        ContactInfo->Depth = Length(ContactInfo->End - ContactInfo->Start);
    }

    return AreColliding;
}

void ResolvePenetration(contact* ContactInfo)
{
    if((fabsf(ContactInfo->A->InvMass) > 0.001) && (fabsf(ContactInfo->B->InvMass) > 0.001))
    {
        r32 da = ContactInfo->Depth/(ContactInfo->A->InvMass + ContactInfo->B->InvMass)*ContactInfo->A->InvMass;
        r32 db = ContactInfo->Depth/(ContactInfo->A->InvMass + ContactInfo->B->InvMass)*ContactInfo->B->InvMass;

        ContactInfo->A->d -= da;
        ContactInfo->B->d += db;
    }
}

void ResolveCollision(contact* ContactInfo)
{
    ResolvePenetration(ContactInfo);

    body* A = ContactInfo->A;
    body* B = ContactInfo->B;

    r32 e = Min(A->Restitution, B->Restitution);

    v2 RelativeVelocity = (A->dP - B->dP);

    r32 ImpulseMagnitude = -(1 + e) * Inner(ContactInfo->Normal, RelativeVelocity) / (A->InvMass + B->InvMass);
    v2  ImpulseDirection = ContactInfo->Normal;

    v2 Jn = ImpulseDirection * ImpulseMagnitude;

    A->ApplyImpulse(Jn);
    B->ApplyImpulse(-Jn);
}
