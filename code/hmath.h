#if !defined(HMATH_H_)
#define HMATH_H_
union v2
{
    struct
    {
        r32 x, y;        
    };
    r32 E[2];
};

union v3
{
    struct
    {
        r32 x, y, z;        
    };
    struct
    {
        v2 xy;
        r32 _Ign0;
    };
    struct
    {
        r32 _Ign1;
        v2 yz;
    };
    r32 E[3];
};

union v4
{
    struct
    {
        r32 x, y, z, w;
    };
    union
    {
        struct
        {
            v3 xyz;
        };
        r32 Ign2;
    };
    r32 E[4];
};

inline r32
Lerp(r32 A, r32 t, r32 B)
{
    r32 Result = (1 - t)*A + t*B;
    return Result;
}

inline r32
Square(r32 A)
{
    return A*A;
}

inline r32
Clamp(r32 Min, r32 V, r32 Max)
{
    r32 Result = V;
    if(Result < Min)
    {
        Result = Min;
    }
    else if(Result > Max)
    {
        Result = Max;
    }

    return Result;
}

inline r32
Clamp01(r32 V)
{
    r32 Result = Clamp(0.0f, V, 1.0f);
    return Result;
}

inline r32
Step(r32 V, r32 Edge)
{
    r32 Result = (V < Edge) ? 0.0f : 1.0f;
    return Result;
}

inline r32
Smoothstep(r32 A, r32 x, r32 B)
{
    r32 Result = Clamp01((x - A) / (B - A));
    Result = x*x*(3 - 2 * x);
    return Result;
}

inline v2
Square(v2 A)
{
    v2 Result = {};

    Result.x = Square(A.x);
    Result.y = Square(A.y);

    return Result;
}

inline r32 
Inner(v2 A, v2 B)
{
    r32 Result = A.x*B.x + A.y*B.y;
    return Result;
}

inline r32
LengthSqr(v2 A)
{
    r32 Result = Inner(A, A);
    return Result;
}

inline r32
Length(v2 A)
{
    r32 Result = sqrtf(LengthSqr(A));
    return Result;
}

inline r32
Distance(v2 A, v2 B)
{
    r32 Result = sqrtf(Square(A.x - B.x) + Square(A.y - B.y));
    return Result;
}

inline r32
SafeRatioN(r32 Numerator, r32 Divisor, r32 N)
{
    r32 Result = N;
    if(Divisor != 0.0f)
    {
        Result = Numerator / Divisor;
    }
    return Result;
}

inline r32
SafeRatio0(r32 Num, r32 Div)
{
    r32 Result = SafeRatioN(Num, Div, 0.0f);
    return Result;
}

inline r32
SafeRatio1(r32 Num, r32 Div)
{
    r32 Result = SafeRatioN(Num, Div, 1.0f);
    return Result;
}

inline r32
SquareRoot(r32 V)
{
    r32 Result = (r32)sqrtf(V);
    return Result;
}

inline v2
V2(r32 X, r32 Y)
{
    v2 Result = {};

    Result.x = X;
    Result.y = Y;

    return Result;
}

inline v2
V2i(i32 X, i32 Y)
{
    v2 Result = {(r32)X, (r32)Y};

    return Result;
}

inline v3
V3(r32 X, r32 Y, r32 Z)
{
    v3 Result = {};

    Result.x = X;
    Result.y = Y;
    Result.z = Z;

    return Result;
}

inline v3
V3(v2 XY, r32 Z)
{
    v3 Result = {};

    Result.xy = XY;
    Result.z = Z;

    return Result;
}

inline v3
V3(r32 X, v2 YZ)
{
    v3 Result = {};

    Result.x = X;
    Result.yz = YZ;

    return Result;
}

inline v4
V4(r32 V)
{
    v4 Result = {};

    Result.x = V;
    Result.y = V;
    Result.z = V;
    Result.w = V;

    return Result;
}

inline v4
V4(r32 X, r32 Y, r32 Z, r32 W)
{
    v4 Result = {};

    Result.x = X;
    Result.y = Y;
    Result.z = Z;
    Result.w = W;

    return Result;
}

inline v4
V4i(i32 X, i32 Y, i32 Z, i32 W)
{
    v4 Result = {};

    Result.x = (r32)X;
    Result.y = (r32)Y;
    Result.z = (r32)Z;
    Result.w = (r32)W;

    return Result;
}

inline v2 
operator+(v2 A, v2 B)
{
    v2 Result = {};

    Result.x = A.x + B.x;
    Result.y = A.y + B.y;

    return Result;
}

inline v2
operator+(v2 A, r32 B)
{
    v2 Result = {};

    Result.x = A.x + B;
    Result.y = A.y + B;

    return Result;
}

inline v2
operator+(r32 A, v2 B)
{
    B = B + A;

    return B;
}

inline v2&
operator+=(v2& A, v2 B)
{
    A = A + B;

    return A;
}

inline v2&
operator+=(v2& A, r32 B)
{
    A = A + B;

    return A;
}

inline v2 
operator-(v2 A, v2 B)
{
    v2 Result = {};

    Result.x = A.x - B.x;
    Result.y = A.y - B.y;

    return Result;
}

inline v2
operator-(v2 A, r32 B)
{
    v2 Result = {};

    Result.x = A.x - B;
    Result.y = A.y - B;

    return Result;
}

inline v2
operator-(r32 A, v2 B)
{
    B = B - A;

    return B;
}

inline v2
operator-(v2 A)
{
    v2 Result = {};

    Result.x = -A.x;
    Result.y = -A.y;

    return Result;
}

inline v2&
operator-=(v2& A, v2 B)
{
    A = A - B;

    return A;
}

inline v2&
operator-=(v2& A, r32 B)
{
    A = A - B;

    return A;
}

inline v2
operator*(v2 A, v2 B)
{
    v2 Result = {};

    Result.x = A.x*B.x;
    Result.y = A.y*B.y;

    return Result;
}

inline v2
operator*(v2 A, r32 B)
{
    v2 Result = {};

    Result.x = A.x*B;
    Result.y = A.y*B;

    return Result;
}

inline v2
operator*(r32 A, v2 B)
{
    B = B*A;

    return B;
}

inline v2&
operator*=(v2& A, v2 B)
{
    A = A*B;

    return A;
}

inline v2&
operator*=(v2& A, r32 B)
{
    A = A * B;

    return A;
}

inline v2
operator/(v2 A, v2 B)
{
    v2 Result = {};

    Result.x = SafeRatio0(A.x, B.x);
    Result.y = SafeRatio0(A.y, B.y);

    return Result;
}

inline v2
operator/(v2 A, r32 B)
{
    v2 Result = {};

    Result.x = SafeRatio0(A.x, B);
    Result.y = SafeRatio0(A.y, B);

    return Result;
}

inline v2
operator/(r32 A, v2 B)
{
    B = B / A;
    return B;
}

inline v2 
Perp(v2 A)
{
    v2 Result = {-A.y, A.x};
    return Result;
}

inline v2
Normalize(v2 A)
{
    v2 Result = A * (1 / Length(A));
    return Result;
}

inline v2
NOZ(v2 A)
{
    v2 Result = V2(0, 0);
    r32 LengthSquared = LengthSqr(A);
    if(LengthSquared > Square(0.00001))
    {
        Result = A * (1 / SquareRoot(LengthSquared));
    }
    return Result;
}

inline v2
rotate(v2 V, r32 A)
{
    v2 Result = {};

    Result.x = V.x*cosf(A) - V.y*sinf(A);
    Result.y = V.x*sinf(A) + V.y*cosf(A);

    return Result;
}

inline v2
rotate_c(v2 V, v2 C, r32 A)
{
    v2 Result = {};

    Result.x = C.x + (V.x - C.x)*cosf(A) - (V.y - C.y)*sinf(A);
    Result.y = C.y + (V.x - C.x)*sinf(A) + (V.y - C.y)*cosf(A);

    return Result;
}

inline v3 
operator+(v3 A, v3 B)
{
    v3 Result = {};

    Result.x = A.x + B.x;
    Result.y = A.y + B.y;
    Result.z = A.z + B.z;

    return Result;
}

inline v3
operator+(v3 A, r32 B)
{
    v3 Result = {};

    Result.x = A.x + B;
    Result.y = A.y + B;
    Result.z = A.z + B;

    return Result;
}

inline v3
operator+(r32 A, v3 B)
{
    B = B + A;

    return B;
}

inline v3&
operator+=(v3& A, v3 B)
{
    A = A + B;

    return A;
}

inline v3&
operator+=(v3& A, r32 B)
{
    A = A + B;

    return A;
}

inline v3 
operator-(v3 A, v3 B)
{
    v3 Result = {};

    Result.x = A.x - B.x;
    Result.y = A.y - B.y;
    Result.z = A.z - B.z;

    return Result;
}

inline v3
operator-(v3 A, r32 B)
{
    v3 Result = {};

    Result.x = A.x - B;
    Result.y = A.y - B;
    Result.z = A.z - B;

    return Result;
}

inline v3
operator-(r32 A, v3 B)
{
    B = B - A;

    return B;
}

inline v3
operator-(v3 A)
{
    v3 Result = {};

    Result.x = -A.x;
    Result.y = -A.y;
    Result.z = -A.z;

    return Result;
}

inline v3&
operator-=(v3& A, v3 B)
{
    A = A - B;

    return A;
}

inline v3&
operator-=(v3& A, r32 B)
{
    A = A - B;

    return A;
}

inline v3
operator*(v3 A, v3 B)
{
    v3 Result = {};

    Result.x = A.x*B.x;
    Result.y = A.y*B.y;
    Result.z = A.z*B.z;

    return Result;
}

inline v3
operator*(v3 A, r32 B)
{
    v3 Result = {};

    Result.x = A.x*B;
    Result.y = A.y*B;
    Result.z = A.z*B;

    return Result;
}

inline v3
operator*(r32 A, v3 B)
{
    B = B*A;

    return B;
}

inline v3&
operator*=(v3& A, v3 B)
{
    A = A*B;

    return A;
}

inline v3&
operator*=(v3& A, r32 B)
{
    A = A * B;

    return A;
}

inline v3
operator/(v3 A, v3 B)
{
    v3 Result = {};

    Result.x = SafeRatio0(A.x, B.x);
    Result.y = SafeRatio0(A.y, B.y);
    Result.z = SafeRatio0(A.z, B.z);

    return Result;
}

inline v3
operator/(v3 A, r32 B)
{
    v3 Result = {};

    Result.x = SafeRatio0(A.x, B);
    Result.y = SafeRatio0(A.y, B);
    Result.z = SafeRatio0(A.z, B);

    return Result;
}

inline v3
operator/(r32 A, v3 B)
{
    B = B / A;
    return B;
}

inline v3
rotate_x(v3 V, r32 A)
{
    v3 Result = {};

    Result.x = V.x;
    Result.y = V.y*cosf(A) - V.z*sinf(A);
    Result.z = V.y*sinf(A) + V.z*cosf(A);

    return Result;
}

inline v3
rotate_y(v3 V, r32 A)
{
    v3 Result = {};

    Result.x = V.x*cosf(A) - V.z*sinf(A);
    Result.y = V.y;
    Result.z = V.x*sinf(A) + V.z*cosf(A);

    return Result;
}

inline v3
rotate_z(v3 V, r32 A)
{
    v3 Result = {};

    Result.x = V.x*cosf(A) - V.y*sinf(A);
    Result.y = V.x*sinf(A) + V.y*sinf(A);
    Result.z = V.z;

    return Result;
}

inline v4 
operator+(v4 A, v4 B)
{
    v4 Result = {};

    Result.x = A.x + B.x;
    Result.y = A.y + B.y;
    Result.z = A.z + B.z;
    Result.w = A.w + B.w;

    return Result;
}

inline v4
operator+(v4 A, r32 B)
{
    v4 Result = {};

    Result.x = A.x + B;
    Result.y = A.y + B;
    Result.z = A.z + B;
    Result.w = A.w + B;

    return Result;
}

inline v4
operator+(r32 A, v4 B)
{
    B = B + A;

    return B;
}

inline v4&
operator+=(v4& A, v4 B)
{
    A = A + B;

    return A;
}

inline v4&
operator+=(v4& A, r32 B)
{
    A = A + B;

    return A;
}

inline v4 
operator-(v4 A, v4 B)
{
    v4 Result = {};

    Result.x = A.x - B.x;
    Result.y = A.y - B.y;
    Result.z = A.z - B.z;
    Result.w = A.w - B.w;

    return Result;
}

inline v4
operator-(v4 A, r32 B)
{
    v4 Result = {};

    Result.x = A.x - B;
    Result.y = A.y - B;
    Result.z = A.z - B;
    Result.w = A.w - B;

    return Result;
}

inline v4
operator-(r32 A, v4 B)
{
    B = B - A;

    return B;
}

inline v4
operator-(v4 A)
{
    v4 Result = {};

    Result.x = -A.x;
    Result.y = -A.y;
    Result.z = -A.z;
    Result.w = -A.w;

    return Result;
}

inline v4&
operator-=(v4& A, v4 B)
{
    A = A - B;

    return A;
}

inline v4&
operator-=(v4& A, r32 B)
{
    A = A - B;

    return A;
}

inline v4
operator*(v4 A, v4 B)
{
    v4 Result = {};

    Result.x = A.x*B.x;
    Result.y = A.y*B.y;
    Result.z = A.z*B.z;
    Result.w = A.w*B.w;

    return Result;
}

inline v4
operator*(v4 A, r32 B)
{
    v4 Result = {};

    Result.x = A.x*B;
    Result.y = A.y*B;
    Result.z = A.z*B;
    Result.w = A.w*B;

    return Result;
}

inline v4
operator*(r32 A, v4 B)
{
    B = B*A;

    return B;
}

inline v4&
operator*=(v4& A, v4 B)
{
    A = A*B;

    return A;
}

inline v4&
operator*=(v4& A, r32 B)
{
    A = A * B;

    return A;
}

inline v4
operator/(v4 A, v4 B)
{
    v4 Result = {};

    Result.x = SafeRatio0(A.x, B.x);
    Result.y = SafeRatio0(A.y, B.y);
    Result.z = SafeRatio0(A.z, B.z);
    Result.w = SafeRatio0(A.w, B.w);

    return Result;
}

inline v4
operator/(v4 A, r32 B)
{
    v4 Result = {};

    Result.x = SafeRatio0(A.x, B);
    Result.y = SafeRatio0(A.y, B);
    Result.z = SafeRatio0(A.z, B);
    Result.w = SafeRatio0(A.w, B);

    return Result;
}

inline v4
operator/(r32 A, v4 B)
{
    B = B / A;
    return B;
}

inline v4 
Hadamard(v4 A, v4 B)
{
    v4 Result = {};

    Result.x = A.x*B.x;
    Result.y = A.y*B.y;
    Result.z = A.z*B.z;
    Result.w = A.w*B.w;

    return Result;
}

struct rectangle2
{
    v2 Min;
    v2 Max;
};

#endif
