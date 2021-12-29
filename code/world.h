
struct world
{
    std::vector<particle*> Particles;
    u32 ParticlesCount;

    v2 PushForce;
    v2 Anchor;

    rectangle2 Liquid;

    r32 SpringTightness;
    r32 SpringRestLength;
};
