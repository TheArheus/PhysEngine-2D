
particle::particle(v2 P_, r32 Mass_)
{
    this->d = P_;
    this->Mass = Mass_;
    this->InvMass = 1 / Mass_;
    ClearForces();
}

particle::~particle()
{
}

void particle::Integrate(r32 DeltaTime)
{
    this->ddP = this->SumForces * this->InvMass;
    this->dP += this->ddP*DeltaTime;
    this->d  += this->dP*DeltaTime;
    ClearForces();
}

void particle::AddForce(const v2& Force)
{
    this->SumForces += Force*PixelsPerMeter;
}

void particle::ClearForces()
{
    this->SumForces = V2(0.0, 0.0);
}

v2 GenerateDragForce(const particle& Particle, r32 k)
{
    v2 DragForce = V2(0, 0);
    if(LengthSqr(Particle.dP) > 0)
    {
        v2 DragDirection = -Normalize(Particle.dP);
        r32 DragMagnitude = k * LengthSqr(Particle.dP);
        DragForce = DragMagnitude*DragDirection;
    }
    return DragForce;
}

v2 GenerateFrictionForce(const particle& Particle, r32 k)
{
    v2 FrictionForce = V2(0, 0);

    v2 FrictionDirection = -Normalize(Particle.dP);
    r32 FrictionMagnitude = k;
    FrictionForce = FrictionDirection*FrictionMagnitude;

    return FrictionForce;
}

v2 GenerateGravitationalForce(const particle& ParticleA, const particle& ParticleB, r32 G)
{
    v2 Direction = ParticleB.d - ParticleA.d;
    r32 Distance = LengthSqr(Direction);
    v2 AttractionDirection = Normalize(Direction);

    r32 AttractionMagnitude = G*ParticleA.Mass*ParticleB.Mass/Distance;
    v2 AttractionForce = AttractionMagnitude*AttractionDirection;

    return AttractionForce;
}

v2 GenerateSpringForce(const particle& Particle, v2 Anchor, r32 RestLength, r32 k)
{
    v2 Direction = Particle.d - Anchor;

    r32 Displacement = Length(Direction) - RestLength;

    v2 SpringDirection = Normalize(Direction);
    r32 SpringMagnitude = -k * Displacement;

    v2 SpringForce = SpringDirection * SpringMagnitude;
    return SpringForce;
}

v2 GenerateSpringForce(const particle& ParticleA, const particle& ParticleB, r32 RestLength, r32 k)
{
    v2 Direction = ParticleA.d - ParticleB.d;

    r32 Displacement = Length(Direction) - RestLength;

    v2 SpringDirection = Normalize(Direction);
    r32 SpringMagnitude = -k * Displacement;

    v2 SpringForce = SpringDirection * SpringMagnitude;
    return SpringForce;
}
