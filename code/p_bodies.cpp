
#include <limits>

body::body(v2 P_, r32 Mass_, shape* Shape_)
{
    this->d = P_ - V2(Shape_->Width / 2, Shape_->Height / 2);
    this->dP = {};
    this->ddP = {};
    this->Mass = Mass_;
    if(Mass != 0.0)
    {
        this->InvMass = 1 / Mass_;
    }
    else
    {
        this->InvMass = 0.0f;
    }

    this->Rotation = 0.0f;
    this->AngularVelocity = 0.0f;
    this->AngularAcceleration = 0.0f;

    this->Shape = Shape_;
    this->I = Shape_->GetMomentOfInertia()*Mass_;
    if (this->I != 0.0f)
    {
        this->InvI = 1 / this->I;
    }
    else
    {
        this->InvI = 0.0;
    }

    this->Restitution = 1.0f;
    this->Friction = 0.7f;

    ClearForces();
    ClearTorque();
}

body::~body()
{
}

void body::IntegrateLinear(r32 DeltaTime)
{
    if(!IsStatic())
    {
        this->ddP = this->SumForces * this->InvMass;
        this->dP += this->ddP*DeltaTime;
        this->d  += this->dP*DeltaTime;
        ClearForces();
    }
}

void body::IntegrateAngular(r32 DeltaTime)
{
    if (!IsStatic())
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
    Shape->UpdateVertices(d, Rotation);
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
    if (!IsStatic())
    {
        dP += Impulse * InvMass;
    }
}

void body::ApplyImpulse(v2 Impulse, v2 Radius)
{
    if (!IsStatic())
    {
        dP += Impulse * InvMass;
        AngularVelocity += Cross(Radius, Impulse) * InvI;
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

b32 body::IsStatic()
{
    r32 Epsilon = 0.005f;
    b32 Result = fabsf(InvMass) < Epsilon;
    return Result;
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

    this->WorldVertices.resize(Vertices.size());
}

shape::~shape()
{
}

void shape::UpdateVertices(v2 P, r32 Rotation)
{
    if(Type == ShapeType_Box || Type == ShapeType_Polygon)
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
            Result = (Square(this->Width) + Square(this->Height)) * 0.083333f;
        } break;
        case ShapeType_Polygon:
        {
            Result = 500.0f;
        } break;
    }
    return Result;
}


b32 IsColliding(body* A, body* B, contact* ContactInfo = nullptr)
{
    b32 Result = false;

    b32 AIsCircle  = A->Shape->Type == ShapeType_Circle;
    b32 BIsCircle  = B->Shape->Type == ShapeType_Circle;
    b32 AIsPolygon = ((A->Shape->Type == ShapeType_Box) || (A->Shape->Type == ShapeType_Polygon));
    b32 BIsPolygon = ((B->Shape->Type == ShapeType_Box) || (B->Shape->Type == ShapeType_Polygon));

    if(AIsCircle && BIsCircle)
    {
        Result = CircleCircleCollision(A, B, ContactInfo);
    }
    else if(AIsPolygon && BIsPolygon)
    {
        Result = PolyPolyCollision(A, B, ContactInfo);
    }
    else if(AIsPolygon && BIsCircle)
    {
        Result = CirclePolyCollision(A, B, ContactInfo);
    }
    else if(BIsPolygon && AIsCircle)
    {
        Result = CirclePolyCollision(B, A, ContactInfo);
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

b32 CirclePolyCollision(body* Poly, body* Circle, contact* ContactInfo)
{
    b32 Result = false;
    b32 IsOutside = false;

    v2 MinimumCurrVertex = {};
    v2 MinimumNextVertex = {};
    r32 DistanceCircleEdge = -FLT_MAX;

    for(u32 PIndex = 0;
        PIndex < Poly->Shape->WorldVertices.size();
        ++PIndex)
    {
        v2 VertexA = Poly->Shape->WorldVertices[PIndex];
        v2 VertexB = Poly->Shape->WorldVertices[(PIndex + 1) % Poly->Shape->WorldVertices.size()];
        v2 Edge = VertexB - VertexA;
        v2 Norm = Normal(Edge);

        v2 VertexToCircleCenter = Circle->d - VertexA;
        r32 Projection = Inner(VertexToCircleCenter, Norm);

        if(Projection > 0)
        {
            DistanceCircleEdge = Projection;
            MinimumCurrVertex = VertexA;
            MinimumNextVertex = VertexB;
            IsOutside = true;
            break;
        }
        else
        {
            if(Projection > DistanceCircleEdge)
            {
                DistanceCircleEdge = Projection;
                MinimumCurrVertex = VertexA;
                MinimumNextVertex = VertexB;
            }
        }
    }

    if(IsOutside)
    {
        v2 VertexA = Circle->d - MinimumCurrVertex;
        v2 VertexB = MinimumNextVertex - MinimumCurrVertex;

        if(Inner(VertexA, VertexB) < 0)
        {
            // Region A
            if(Length(VertexA) < Circle->Shape->Radius)
            {
                ContactInfo->A = Poly;
                ContactInfo->B = Circle;
                ContactInfo->Depth = Circle->Shape->Radius - Length(VertexA);
                ContactInfo->Normal = Normalize(VertexA);
                ContactInfo->Start = Circle->d - ContactInfo->Normal * Circle->Shape->Radius;
                ContactInfo->End = ContactInfo->Start + (ContactInfo->Normal * ContactInfo->Depth);

                Result = true;
            }
        }
        else
        {
            VertexA = Circle->d - MinimumNextVertex;
            VertexB = MinimumCurrVertex - MinimumNextVertex;

            if(Inner(VertexA, VertexB) < 0)
            {
                // Region B
                if(Length(VertexA) < Circle->Shape->Radius)
                {
                    ContactInfo->A = Poly;
                    ContactInfo->B = Circle;
                    ContactInfo->Depth = Circle->Shape->Radius - Length(VertexA);
                    ContactInfo->Normal = Normalize(VertexA);
                    ContactInfo->Start = Circle->d - ContactInfo->Normal * Circle->Shape->Radius;
                    ContactInfo->End = ContactInfo->Start + (ContactInfo->Normal * ContactInfo->Depth);

                    Result = true;
                }
            }
            else
            {
                // Region C
                if(DistanceCircleEdge < Circle->Shape->Radius)
                {
                    ContactInfo->A = Poly;
                    ContactInfo->B = Circle;
                    ContactInfo->Depth = Circle->Shape->Radius - DistanceCircleEdge;
                    ContactInfo->Normal = Normal(MinimumNextVertex - MinimumCurrVertex);
                    ContactInfo->Start = Circle->d - ContactInfo->Normal * Circle->Shape->Radius;
                    ContactInfo->End = ContactInfo->Start + ContactInfo->Normal * ContactInfo->Depth;

                    Result = true;
                }
            }
        }
    }
    else
    {
        ContactInfo->A = Poly;
        ContactInfo->B = Circle;
        ContactInfo->Depth = Circle->Shape->Radius - DistanceCircleEdge;
        ContactInfo->Normal = Normal(MinimumNextVertex - MinimumCurrVertex);
        ContactInfo->Start = Circle->d - ContactInfo->Normal * Circle->Shape->Radius;
        ContactInfo->End = ContactInfo->Start + ContactInfo->Normal * ContactInfo->Depth;
        Result = true;
    }

    return Result;
}

r32 FindMinSeparation(shape* A, shape* B, v2& Axis, v2& P)
{
    r32 Separation = -FLT_MAX;
    for(u32 PointAIndex = 0; 
        PointAIndex < A->WorldVertices.size();
        ++PointAIndex)
    {
        v2 VertexA = A->WorldVertices[PointAIndex];
        v2 NextA = A->WorldVertices[(PointAIndex + 1) % A->WorldVertices.size()];
        v2 Edge = NextA - VertexA;

        v2 N = Normal(Edge);

        r32 MinimumSeparation = FLT_MAX;
        v2  MinimumVertex;
        for(u32 PointBIndex = 0; 
            PointBIndex < B->WorldVertices.size();
            ++PointBIndex)
        {
            v2 VertexB = B->WorldVertices[PointBIndex];
            r32 Projection = Inner(VertexB - VertexA, N);
            if(Projection < MinimumSeparation)
            {
                MinimumSeparation = Projection;
                MinimumVertex = VertexB;
            }
        }

        if(MinimumSeparation > Separation)
        {
            Separation = MinimumSeparation;
            Axis = Edge;
            P = MinimumVertex;
        }
    }

    return Separation;
}

b32 PolyPolyCollision(body* BodyA, body* BodyB, contact* ContactInfo)
{
    shape* A = BodyA->Shape;
    shape* B = BodyB->Shape;
    v2 AAxis, BAxis;
    v2 APoint, BPoint;

    r32 ABSeparation = FindMinSeparation(A, B, AAxis, APoint);
    r32 BASeparation = FindMinSeparation(B, A, BAxis, BPoint);

    b32 Result = ABSeparation < 0.0f && BASeparation < 0.0f;

    if(Result)
    {
        ContactInfo->A = BodyA;
        ContactInfo->B = BodyB;
        if (ABSeparation > BASeparation)
        {
            ContactInfo->Depth  = -ABSeparation;
            ContactInfo->Normal =  Normal(AAxis);
            ContactInfo->Start  =  APoint;
            ContactInfo->End    =  APoint + ContactInfo->Normal * ContactInfo->Depth;
        }
        else
        {
            ContactInfo->Depth  = -BASeparation;
            ContactInfo->Normal = -Normal(BAxis);
            ContactInfo->Start  =  BPoint - ContactInfo->Normal * ContactInfo->Depth;
            ContactInfo->End    =  BPoint;
        }

    }

    return Result;
}

void ResolvePenetration(contact* ContactInfo)
{
    body* A = ContactInfo->A;
    body* B = ContactInfo->B;
    if(!A->IsStatic() && !B->IsStatic())
    {
        r32 da = ContactInfo->Depth/(A->InvMass + B->InvMass)*A->InvMass;
        r32 db = ContactInfo->Depth/(A->InvMass + B->InvMass)*B->InvMass;

        ContactInfo->A->d -= ContactInfo->Normal*da*0.8f;
        ContactInfo->B->d += ContactInfo->Normal*db*0.8f;

        A->Shape->UpdateVertices(A->d, A->Rotation);
        B->Shape->UpdateVertices(A->d, A->Rotation);
    }
}

// NOTE: There is somewhere an error
// Body should not be sinking!!!
void ResolveCollision(contact* ContactInfo)
{
    ResolvePenetration(ContactInfo);
    body* A = ContactInfo->A;
    body* B = ContactInfo->B;
    v2 Norm  = ContactInfo->Normal;

    r32 e = Min(A->Restitution, B->Restitution);
    r32 f = Min(A->Friction, B->Friction);

    v2 ra = ContactInfo->End   - A->d;
    v2 rb = ContactInfo->Start - B->d;
    v2 va = A->dP + Cross(V3(0, 0, A->AngularVelocity), V3(ra, 0)).xy; 
    v2 vb = B->dP + Cross(V3(0, 0, B->AngularVelocity), V3(rb, 0)).xy; 

    v2  RelativeVelocity = va - vb;
    r32 RelativeVelocityDotN = Inner(RelativeVelocity,  Norm);

    r32 Denom = A->InvMass + B->InvMass + Square(Cross(ra, Norm)) * A->InvI + Square(Cross(rb, Norm)) * B->InvI;
    r32 ImpulseMagnitude = -(1 + e) * RelativeVelocityDotN / Denom;
    v2  ImpulseDirection =  Norm;

    v2 Jn = ImpulseDirection * ImpulseMagnitude;

    // Friction

    v2 Tangent = Normal(Norm);
    r32 RelativeVelocityDotT = Inner(RelativeVelocity, Tangent);

    r32 DenomT = A->InvMass + B->InvMass + Square(Cross(ra, Tangent)) * A->InvI + Square(Cross(rb, Tangent)) * B->InvI;
    r32 ImpulseMagnitudeT = -1.0f*f*(1 + e) * RelativeVelocityDotT / DenomT;
    v2  ImpulseDirectionT = Tangent;

    v2 Jt = ImpulseDirectionT * ImpulseMagnitudeT;

    v2 J = Jn + Jt;

    A->ApplyImpulse( J, ra);
    B->ApplyImpulse(-J, rb);
}

