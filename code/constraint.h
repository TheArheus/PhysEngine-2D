#if !defined(CONSTRAIN_H_)
#define CONSTRAIN_H_

enum constraint_type
{
    ConstraintType_Joint,
    ConstraintType_Penetration,
};

struct constraint
{
    body* A;
    body* B;

    constraint_type Type;

    r32 Bias;
    r32 Friction;

    mNxM Jacobian;
    vN CachedLambda;

    v2 LocalPointA;
    v2 LocalPointB;
    v2 Norm;

    vN GetVelocities();
    mNxM GetInvM();

    void PreSolve(r32 DeltaTime);
    void Solve();
    void PostSolve();

    void JointConstraint(body* BodyA, body* BodyB, v2 AnchorPoint);
    void PenetrationConstraint(body* BodyA, body* BodyB, v2 CollisionPointA_, v2 CollisionPointB_, v2 Normal_, r32 Friction_);
    void PenetrationConstraint();
};

#endif
