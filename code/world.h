
struct world
{
    std::vector<body*> Bodies;
    std::vector<v2> Forces;
    std::vector<r32> Torques;
    std::vector<constraint*> Constraints;

    v2 PushForce;
    v2 Anchor;

    rectangle2 Liquid;

    r32 SpringTightness;
    r32 SpringRestLength;

    r32 G = 9.8f;

    world(r32 Gravity);
    ~world();

    void Update(r32 DeltaTime);
    void CheckCollisions();
};
