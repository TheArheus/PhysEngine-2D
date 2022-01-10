#if !defined(CONSTRAIN_H_)
#define CONSTRAIN_H_

struct constraint
{
    body* A;
    body* B;
    mNxM Jacobian;

    v2 APoint;
    v2 BPoint;

    vN GetVelocities();
    mNxM GetInvM();
    void Solve();

    void JointConstraint(body* BodyA, body* BodyB, v2 AnchorPoint);
    void PenetrationConstraint();
};

#endif
