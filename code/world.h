
struct world
{
    std::vector<body*> Bodies;

    v2 PushForce;
    v2 Anchor;

    rectangle2 Liquid;

    r32 SpringTightness;
    r32 SpringRestLength;
};
