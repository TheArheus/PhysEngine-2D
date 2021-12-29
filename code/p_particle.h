#if !defined(particle_h_)
#define particle_h_

struct particle
{
    v2 d;
    v2 dP;
    v2 ddP;

    r32 Mass;
    r32 InvMass;
    v2 SumForces;

    particle(v2 P, r32 Mass);
    ~particle();

    void Integrate(r32 DeltaTime);
    void AddForce(const v2& Force);
    void ClearForces();
};

v2 GenerateDragForce(const particle& Particle, r32 k);
v2 GenerateFrictionForce(const particle& Particle, r32 k);
v2 GenerateGravitationalForce(const particle& ParticleA, const particle& ParticleB, r32 G);
v2 GenerateSpringForce(const particle& Particle, v2 Anchor, r32 RestLength, r32 k);
v2 GenerateSpringForce(const particle& ParticleA, const particle& ParticleB, r32 RestLength, r32 k);

#endif
