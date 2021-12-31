#if !defined(body_h_)
#define body_h_

enum shape_type
{
    ShapeType_Circle,
    ShapeType_Polygon,
    ShapeType_Box,
};

struct shape
{
    r32 Width;
    r32 Height;
    r32 Radius;
    std::vector<v2> LocalVertices;
    std::vector<v2> WorldVertices;
    shape_type Type;

    shape(r32 Width, r32 Height);
    shape(r32 Width, r32 Height, r32 Radius);
    shape(r32 Width, r32 Height, std::vector<v2> Vertices);
    ~shape();

    r32 GetMomentOfInertia();
    void UpdateVertices(v2 P, r32 Rotation);
};

struct body 
{
    v2 d;
    v2 dP;
    v2 ddP;

    r32 Rotation;
    r32 AngularVelocity;
    r32 AngularAcceleration;

    shape* Shape;

    r32 Mass;
    r32 InvMass;
    r32 I; // Moment Of Inertia
    r32 InvI; 

    r32 Restitution;

    v2 SumForces;
    r32 SumTorque;

    b32 IsColliding;

    body(v2 P, r32 Mass);
    body(v2 P, r32 Mass, shape* Shape);
    ~body();

    void IntegrateLinear(r32 DeltaTime);
    void IntegrateAngular(r32 DeltaTime);
    void Update(r32 DeltaTime);
    void AddForce(const v2& Force);
    void AddTorque(r32 Torque);
    void ApplyImpulse(v2 Impulse);
    void ClearForces();
    void ClearTorque();
};

struct contact 
{
    body* A;
    body* B;

    v2 Start;
    v2 End;

    v2 Normal;
    r32 Depth;

    contact() = default;
    ~contact() = default;
};

v2 GenerateDragForce(const body& Body, r32 k);
v2 GenerateFrictionForce(const body& Body, r32 k);
v2 GenerateGravitationalForce(const body& BodyA, const body& BodyB, r32 G);
v2 GenerateSpringForce(const body& Body, v2 Anchor, r32 RestLength, r32 k);
v2 GenerateSpringForce(const body& BodyA, const body& BodyB, r32 RestLength, r32 k);

b32 IsColliding(body* A, body* B, contact* ContactInfo);
b32 CircleCircleCollision(body* A, body* B, contact* ContactInfo);
void ResolvePenetration(contact* ContactInfo);
void ResolveCollision(contact* ContactInfo);

#endif
