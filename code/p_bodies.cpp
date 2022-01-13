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

void body::
IntegrateForces(r32 DeltaTime)
{
    if (!IsStatic())
    {
        ddP = SumForces * InvMass;
        dP += ddP * DeltaTime;

        AngularAcceleration = SumTorque * InvI;
        AngularVelocity += AngularAcceleration * DeltaTime;

        ClearForces();
        ClearTorque();
    }
}

void body::
IntegrateVelocities(r32 DeltaTime)
{
    if (!IsStatic())
    {
        d += dP*DeltaTime;
        Rotation += AngularVelocity * DeltaTime;
    }
    Shape->UpdateVertices(d, Rotation);
}

void body::
AddForce(const v2& Force)
{
    this->SumForces += Force*PixelsPerMeter;
}

void body::
AddTorque(r32 Torque)
{
    this->SumTorque += Torque*PixelsPerMeter;
}

void body::
ApplyImpulseLinear(v2 Impulse)
{
    if (!IsStatic())
    {
        dP += Impulse * InvMass;
    }
}

void body::
ApplyImpulseAngular(r32 Impulse)
{
    if (!IsStatic())
    {
        AngularVelocity += Impulse * InvI;
    }
}

void body::
ApplyImpulseAtPoint(v2 Impulse, v2 Radius)
{
    if (!IsStatic())
    {
        dP += Impulse * InvMass;
        AngularVelocity += Cross(Radius, Impulse) * InvI;
    }
}

void body::
ClearForces()
{
    this->SumForces = V2(0.0, 0.0);
}

void body::
ClearTorque()
{
    this->SumTorque = 0.0f;
}

b32 body::
IsStatic()
{
    r32 Epsilon = 0.005f;
    b32 Result = fabsf(InvMass) < Epsilon;
    return Result;
}

v2 body::
LocalSpaceToWorldSpace(v2 P)
{
    v2 Result = rotate(P, Rotation);
    Result += this->d;
    return Result;
}

v2 body::
WorldSpaceToLocalSpace(v2 P)
{
    v2 Result = P - this->d;
    Result = rotate(Result, -Rotation);
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

shape::
shape(r32 Width, r32 Height)
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

shape::
shape(r32 Width, r32 Height, r32 Radius)
{
    this->Radius = Radius;
    this->Width = Width;
    this->Height = Height;
    this->Type = ShapeType_Circle;
}

shape::
shape(r32 Width, r32 Height, std::vector<v2> Vertices)
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

void shape::
UpdateVertices(v2 P, r32 Rotation)
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

r32 shape::
GetMomentOfInertia()
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

v2 shape::
EdgeAt(u32 Index)
{
    v2 CurrentP = WorldVertices[Index];
    v2 NextP = WorldVertices[(Index + 1) % WorldVertices.size()];
    v2 Result = NextP - CurrentP;
    return Result;
}

#include <float.h>
i32 shape::
FindIncidentEdge(v2 Norm)
{
    i32 Result = -1.0f;
    r32 MinimumProjection = FLT_MAX;
    for(u32 VertexIndex = 0;
        VertexIndex < WorldVertices.size();
        ++VertexIndex)
    {
        v2 EdgeNormal = Normal(EdgeAt(VertexIndex));
        r32 Projection = Inner(EdgeNormal, Norm);
        if(Projection < MinimumProjection)
        {
            MinimumProjection = Projection;
            Result = VertexIndex;
        }
    }

    return Result;
}

i32 shape::
ClipSegmentToLine(std::vector<v2>& ContactsIn, std::vector<v2>& ContactsOut, v2 C0, v2 C1)
{
    i32 Result = 0;

    v2 Norm = Normalize(C1 - C0);
    r32 Dist0 = Cross(ContactsIn[0] - C0, Norm);
    r32 Dist1 = Cross(ContactsIn[1] - C1, Norm);

    if(Dist0 <= 0) ContactsOut[Result++] = ContactsIn[0];
    if(Dist1 <= 0) ContactsOut[Result++] = ContactsIn[1];

    if(Dist0 * Dist1 < 0)
    {
        r32 TotalDist = Dist0 - Dist1;

        r32 t = Dist0 / TotalDist;
        v2 Contact = ContactsIn[0] + t*(ContactsIn[1] - ContactsIn[0]);
        ContactsOut[Result] = Contact;
        Result++;
    }
    return Result;
}


b32 IsColliding(body* A, body* B, std::vector<contact>* ContactsInfo = nullptr)
{
    b32 Result = false;

    b32 AIsCircle  = A->Shape->Type == ShapeType_Circle;
    b32 BIsCircle  = B->Shape->Type == ShapeType_Circle;
    b32 AIsPolygon = ((A->Shape->Type == ShapeType_Box) || (A->Shape->Type == ShapeType_Polygon));
    b32 BIsPolygon = ((B->Shape->Type == ShapeType_Box) || (B->Shape->Type == ShapeType_Polygon));

    if(AIsCircle && BIsCircle)
    {
        Result = CircleCircleCollision(A, B, ContactsInfo);
    }
    else if(AIsPolygon && BIsPolygon)
    {
        Result = PolyPolyCollision(A, B, ContactsInfo);
    }
    else if(AIsPolygon && BIsCircle)
    {
        Result = CirclePolyCollision(A, B, ContactsInfo);
    }
    else if(BIsPolygon && AIsCircle)
    {
        Result = CirclePolyCollision(B, A, ContactsInfo);
    }

    return Result;
}

b32 CircleCircleCollision(body* A, body* B, std::vector<contact>* ContactsInfo)
{
    v2 ab = B->d - A->d;

    r32 RadiusSum = A->Shape->Radius + B->Shape->Radius;
    b32 AreColliding = LengthSqr(ab) <= Square(RadiusSum);

    contact ContactInfo = {};
    if(AreColliding)
    {
        ContactInfo.A = A;
        ContactInfo.B = B;
        
        ContactInfo.Normal = Normalize(ab);

        ContactInfo.Start = B->d - ContactInfo.Normal*B->Shape->Radius;
        ContactInfo.End = A->d + ContactInfo.Normal*A->Shape->Radius;

        ContactInfo.Depth = Length(ContactInfo.End - ContactInfo.Start);
    }
    ContactsInfo->push_back(ContactInfo);

    return AreColliding;
}

b32 CirclePolyCollision(body* Poly, body* Circle, std::vector<contact>* ContactsInfo)
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

    contact ContactInfo = {};
    if(IsOutside)
    {
        v2 VertexA = Circle->d - MinimumCurrVertex;
        v2 VertexB = MinimumNextVertex - MinimumCurrVertex;

        if(Inner(VertexA, VertexB) < 0)
        {
            // Region A
            if(Length(VertexA) < Circle->Shape->Radius)
            {
                ContactInfo.A = Poly;
                ContactInfo.B = Circle;
                ContactInfo.Depth = Circle->Shape->Radius - Length(VertexA);
                ContactInfo.Normal = Normalize(VertexA);
                ContactInfo.Start = Circle->d - ContactInfo.Normal * Circle->Shape->Radius;
                ContactInfo.End = ContactInfo.Start + (ContactInfo.Normal * ContactInfo.Depth);

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
                    ContactInfo.A = Poly;
                    ContactInfo.B = Circle;
                    ContactInfo.Depth = Circle->Shape->Radius - Length(VertexA);
                    ContactInfo.Normal = Normalize(VertexA);
                    ContactInfo.Start = Circle->d - ContactInfo.Normal * Circle->Shape->Radius;
                    ContactInfo.End = ContactInfo.Start + (ContactInfo.Normal * ContactInfo.Depth);

                    Result = true;
                }
            }
            else
            {
                // Region C
                if(DistanceCircleEdge < Circle->Shape->Radius)
                {
                    ContactInfo.A = Poly;
                    ContactInfo.B = Circle;
                    ContactInfo.Depth = Circle->Shape->Radius - DistanceCircleEdge;
                    ContactInfo.Normal = Normal(MinimumNextVertex - MinimumCurrVertex);
                    ContactInfo.Start = Circle->d - ContactInfo.Normal * Circle->Shape->Radius;
                    ContactInfo.End = ContactInfo.Start + ContactInfo.Normal * ContactInfo.Depth;

                    Result = true;
                }
            }
        }
    }
    else
    {
        ContactInfo.A = Poly;
        ContactInfo.B = Circle;
        ContactInfo.Depth = Circle->Shape->Radius - DistanceCircleEdge;
        ContactInfo.Normal = Normal(MinimumNextVertex - MinimumCurrVertex);
        ContactInfo.Start = Circle->d - ContactInfo.Normal * Circle->Shape->Radius;
        ContactInfo.End = ContactInfo.Start + ContactInfo.Normal * ContactInfo.Depth;
        Result = true;
    }
    ContactsInfo->push_back(ContactInfo);

    return Result;
}

r32 FindMinSeparation(shape* A, shape* B, int& IndexReferenceEdge, v2& SupportPoint)
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
            IndexReferenceEdge = PointAIndex;
            SupportPoint = MinimumVertex;
        }
    }

    return Separation;
}

b32 PolyPolyCollision(body* BodyA, body* BodyB, std::vector<contact>* ContactsInfo)
{
    shape* A = BodyA->Shape;
    shape* B = BodyB->Shape;
    int IndexReferenceEdgeA, IndexReferenceEdgeB;
    v2  SupportPointA, SupportPointB;

    r32 ABSeparation = FindMinSeparation(A, B, IndexReferenceEdgeA, SupportPointA);
    r32 BASeparation = FindMinSeparation(B, A, IndexReferenceEdgeB, SupportPointB);

    b32 Result = ABSeparation < 0.0f && BASeparation < 0.0f;

    shape* ReferenceShape;
    shape* IncidentShape;
    int IndexReferenceEdge;
    if(ABSeparation > BASeparation)
    {
        ReferenceShape = BodyA->Shape;
        IncidentShape  = BodyB->Shape;
        IndexReferenceEdge = IndexReferenceEdgeA;
    }
    else
    {
        ReferenceShape = BodyB->Shape;
        IncidentShape  = BodyA->Shape;
        IndexReferenceEdge = IndexReferenceEdgeB;
    }

    v2 ReferenceEdge  = ReferenceShape->WorldVertices[(IndexReferenceEdge + 1) % ReferenceShape->WorldVertices.size()]
                      - ReferenceShape->WorldVertices[IndexReferenceEdge];
    i32 IncidentIndex = IncidentShape->FindIncidentEdge(Normal(ReferenceEdge));
    i32 IncidentNextIndex = ((IncidentIndex + 1) % IncidentShape->WorldVertices.size());

    v2 Vertex0 = IncidentShape->WorldVertices[IncidentIndex];
    v2 Vertex1 = IncidentShape->WorldVertices[IncidentNextIndex];

    std::vector<v2> ContactPoints = {Vertex0, Vertex1};
    std::vector<v2> ClippedPoints =  ContactPoints;
    for(i32 VertexIndex = 0;
        VertexIndex < ReferenceShape->WorldVertices.size();
        ++VertexIndex)
    {
        if(VertexIndex == IndexReferenceEdge) continue;

        v2 C0 = ReferenceShape->WorldVertices[VertexIndex];
        v2 C1 = ReferenceShape->WorldVertices[(VertexIndex + 1) % ReferenceShape->WorldVertices.size()];
        i32 ClippedCount = ReferenceShape->ClipSegmentToLine(ContactPoints, ClippedPoints, C0, C1);

        if(ClippedCount > 2) break;

        ContactPoints = ClippedPoints;
    }

    v2 ReferenceVertex = ReferenceShape->WorldVertices[IndexReferenceEdge];

    for(v2 ClippedVertex : ClippedPoints)
    {
        r32 Separation = Inner((ClippedVertex - ReferenceVertex), Normal(ReferenceEdge));
        if(Separation <= 0)
        {
            contact ContactInfo = {};
            ContactInfo.A = BodyA;
            ContactInfo.B = BodyB;
            ContactInfo.Normal = Normal(ReferenceEdge);
            ContactInfo.Start  = ClippedVertex;
            ContactInfo.End    = ClippedVertex - ContactInfo.Normal * Separation;
            if(BASeparation >= ABSeparation)
            {
                ContactInfo.Start = ClippedVertex - ContactInfo.Normal * Separation;
                ContactInfo.End   = ClippedVertex;
                ContactInfo.Normal *= -1.0f;
            }
            ContactsInfo->push_back(ContactInfo);
        }
    }

    return Result;
}

