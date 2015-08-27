/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_MATH_H
#define B2_MATH_H

#include <Box2D/Common/b2Settings.h>
#include <math.h>
#define B2_DXMATH
#ifdef B2_DXMATH
#include "DirectXMath.h"
#endif
/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b2IsValid(float32 x)
{
	int32 ix = *reinterpret_cast<int32*>(&x);
	return (ix & 0x7f800000) != 0x7f800000;
}

/// This is a approximate yet fast inverse square-root.
inline float32 b2InvSqrt(float32 x)
{
	union
	{
		float32 x;
		int32 i;
	} convert;

	convert.x = x;
	float32 xhalf = 0.5f * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5f - xhalf * x * x);
	return x;
}

#define	b2Sqrt(x)	sqrtf(x)
#define	b2Atan2(y, x)	atan2f(y, x)

/// A 2D column vector.
struct b2Vec2
{
	/// Default constructor does nothing (for performance).
	b2Vec2() {}

	/// Construct using coordinates.
	b2Vec2(float32 x, float32 y) : x(x), y(y) {}

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_) { x = x_; y = y_; }

	/// Negate this vector.
	b2Vec2 operator -() const { b2Vec2 v; v.Set(-x, -y); return v; }
	
	/// Read from and indexed element.
	float32 operator () (int32 i) const
	{
		return (&x)[i];
	}

	/// Write to an indexed element.
	float32& operator () (int32 i)
	{
		return (&x)[i];
	}

	/// Add a vector to this vector.
	void operator += (const b2Vec2& v)
	{
		x += v.x; y += v.y;
	}
	
	/// Subtract a vector from this vector.
	void operator -= (const b2Vec2& v)
	{
		x -= v.x; y -= v.y;
	}

	/// Multiply this vector by a scalar.
	void operator *= (float32 a)
	{
		x *= a; y *= a;
	}

	/// Get the length of this vector (the norm).
	float32 Length() const
	{
#ifdef B2_DXMATH
		DirectX::XMVECTOR v1 = DirectX::XMVectorSet(x, y, 0.0f, 0.0f);
		return DirectX::XMVectorGetX(DirectX::XMVector2Length(v1));
#else
		return b2Sqrt(x * x + y * y);
#endif
	}

	/// Get the length squared. For performance, use this instead of
	/// b2Vec2::Length (if possible).
	float32 LengthSquared() const
	{
#ifdef B2_DXMATH
		DirectX::XMVECTOR v1 = DirectX::XMVectorSet(x, y, 0.0f, 0.0f);
		return DirectX::XMVectorGetX(DirectX::XMVector2LengthSq(v1));
#else
		return x * x + y * y;
#endif
	}

	/// Convert this vector into a unit vector. Returns the length.
	float32 Normalize()
	{
#ifdef B2_DXMATH
		DirectX::XMVECTOR v1 = DirectX::XMVectorSet(x, y, 0.0f, 0.0f);
		float32 length = DirectX::XMVectorGetX(DirectX::XMVector2Length(v1));
		if (length < b2_epsilon)
		{
			return 0.0f;
		}
		DirectX::XMVECTOR v2 = DirectX::XMVector2Normalize(v1);
		x = DirectX::XMVectorGetX(v2);
		y = DirectX::XMVectorGetY(v2);
		return length;
#else
		float32 length = Length();
		if (length < b2_epsilon)
		{
			return 0.0f;
		}
		float32 invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;

		return length;
#endif
	}

	/// Does this vector contain finite coordinates?
	bool IsValid() const
	{
		return b2IsValid(x) && b2IsValid(y);
	}

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	b2Vec2 Skew() const
	{
		return b2Vec2(-y, x);
	}

	float32 x, y;
};

/// A 2D column vector with 3 elements.
struct b2Vec3
{
	/// Default constructor does nothing (for performance).
	b2Vec3() {}

	/// Construct using coordinates.
	b2Vec3(float32 x, float32 y, float32 z) : x(x), y(y), z(z) {}

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_, float32 z_) { x = x_; y = y_; z = z_; }

	/// Negate this vector.
	b2Vec3 operator -() const { b2Vec3 v; v.Set(-x, -y, -z); return v; }

	/// Add a vector to this vector.
	void operator += (const b2Vec3& v)
	{
		x += v.x; y += v.y; z += v.z;
	}

	/// Subtract a vector from this vector.
	void operator -= (const b2Vec3& v)
	{
		x -= v.x; y -= v.y; z -= v.z;
	}

	/// Multiply this vector by a scalar.
	void operator *= (float32 s)
	{
		x *= s; y *= s; z *= s;
	}

	float32 x, y, z;
};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
	/// The default constructor does nothing (for performance).
	b2Mat22() {}

	/// Construct this matrix using columns.
	b2Mat22(const b2Vec2& c1, const b2Vec2& c2)
	{
		ex = c1;
		ey = c2;
	}

	/// Construct this matrix using scalars.
	b2Mat22(float32 a11, float32 a12, float32 a21, float32 a22)
	{
		ex.x = a11; ex.y = a21;
		ey.x = a12; ey.y = a22;
	}

	/// Initialize this matrix using columns.
	void Set(const b2Vec2& c1, const b2Vec2& c2)
	{
		ex = c1;
		ey = c2;
	}

	/// Set this to the identity matrix.
	void SetIdentity()
	{
		ex.x = 1.0f; ey.x = 0.0f;
		ex.y = 0.0f; ey.y = 1.0f;
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
		ex.x = 0.0f; ey.x = 0.0f;
		ex.y = 0.0f; ey.y = 0.0f;
	}

	b2Mat22 GetInverse() const
	{
		float32 a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		b2Mat22 B;
		float32 det = a * d - b * c;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		B.ex.x =  det * d;	B.ey.x = -det * b;
		B.ex.y = -det * c;	B.ey.y =  det * a;
		return B;
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b2Vec2 Solve(const b2Vec2& b) const
	{
		float32 a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		float32 det = a11 * a22 - a12 * a21;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		b2Vec2 x;
		x.x = det * (a22 * b.x - a12 * b.y);
		x.y = det * (a11 * b.y - a21 * b.x);
		return x;
	}

	b2Vec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
	/// The default constructor does nothing (for performance).
	b2Mat33() {}

	/// Construct this matrix using columns.
	b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3)
	{
		ex = c1;
		ey = c2;
		ez = c3;
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
		ex.SetZero();
		ey.SetZero();
		ez.SetZero();
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b2Vec3 Solve33(const b2Vec3& b) const;

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	b2Vec2 Solve22(const b2Vec2& b) const;

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	void GetInverse22(b2Mat33* M) const;

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	void GetSymInverse33(b2Mat33* M) const;

	b2Vec3 ex, ey, ez;
};

/// Rotation
struct b2Rot
{
	b2Rot() {}

	/// Initialize from an angle in radians
	explicit b2Rot(float32 angle)
	{
#ifdef B2_DXMATH
		 DirectX::XMScalarSinCosEst(&s, &c, angle);
#else
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
#endif
	}

	/// Set using an angle in radians.
	void Set(float32 angle)
	{
#ifdef B2_DXMATH
		 DirectX::XMScalarSinCosEst(&s, &c, angle);
#else
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
#endif
	}

	/// Set to the identity rotation
	void SetIdentity()
	{
		s = 0.0f;
		c = 1.0f;
	}

	/// Get the angle in radians
	float32 GetAngle() const
	{
		return b2Atan2(s, c);
	}

	/// Get the x-axis
	b2Vec2 GetXAxis() const
	{
		return b2Vec2(c, s);
	}

	/// Get the u-axis
	b2Vec2 GetYAxis() const
	{
		return b2Vec2(-s, c);
	}

	/// Sine and cosine
	float32 s, c;
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() {}

	/// Initialize using a position vector and a rotation.
	b2Transform(const b2Vec2& position, const b2Rot& rotation) : p(position), q(rotation) {}

	/// Set this to the identity transform.
	void SetIdentity()
	{
		p.SetZero();
		q.SetIdentity();
	}

	/// Set this based on the position and angle.
	void Set(const b2Vec2& position, float32 angle)
	{
		p = position;
		q.Set(angle);
	}

#ifdef B2_DXMATH
	DirectX::XMMATRIX GetXMMatrix(bool CCW = true) const
	{
		DirectX::XMMATRIX ret = DirectX::XMMatrixAffineTransformation2D(DirectX::XMVectorReplicate(1.0f), DirectX::XMVectorZero(), 0, DirectX::XMVectorSet(p.x, p.y, 0.0f, 0.0f));
		// Set rotation components
		DirectX::XMVectorSetX(ret.r[0], q.c);
		if (CCW)
		{
			DirectX::XMVectorSetY(ret.r[0], -q.s);
			DirectX::XMVectorSetX(ret.r[1], q.s);
		}
		else
		{
			DirectX::XMVectorSetY(ret.r[0], q.s);
			DirectX::XMVectorSetX(ret.r[1], -q.s);
		}
		DirectX::XMVectorSetY(ret.r[1], q.c);
		return ret;
	}
#endif

	b2Vec2 p;
	b2Rot q;
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	void GetTransform(b2Transform* xfb, float32 beta) const;

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	void Advance(float32 alpha);

	/// Normalize the angles.
	void Normalize();

	b2Vec2 localCenter;	///< local center of mass position
	b2Vec2 c0, c;		///< center world positions
	float32 a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float32 alpha0;
};

/// Useful constant
extern const b2Vec2 b2Vec2_zero;

/// Perform the dot product on two vectors.
inline float32 b2Dot(const b2Vec2& a, const b2Vec2& b)
{
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(a.x, a.y, b.x, b.y);
	DirectX::XMVECTOR v2 = DirectX::XMVectorShiftLeft<2>(v1,DirectX::XMVectorZero());
	return DirectX::XMVectorGetX(DirectX::XMVector2Dot(v1, v2));
#else
	return a.x * b.x + a.y * b.y;
#endif
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float32 b2Cross(const b2Vec2& a, const b2Vec2& b)
{
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(a.x, a.y, b.x, b.y);
	DirectX::XMVECTOR v2 = DirectX::XMVectorRotateLeft<2>(v1);
	return DirectX::XMVectorGetX(DirectX::XMVector2Cross(v1, v2));
#else
	return a.x * b.y - a.y * b.x;
#endif
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross(const b2Vec2& a, float32 s)
{
#if 0
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(a.y, a.x, 0.0f, 0.0f);
	v1 = DirectX::XMVectorScale(v1, s);
	return b2Vec2(DirectX::XMVectorGetX(v1), -DirectX::XMVectorGetY(v1));
#else
	return b2Vec2(s * a.y, -s * a.x);
#endif
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross(float32 s, const b2Vec2& a)
{
#if 0
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(a.y, a.x, 0.0f, 0.0f);
	v1 = DirectX::XMVectorScale(v1, s);
	return b2Vec2(-DirectX::XMVectorGetX(v1), DirectX::XMVectorGetY(v1));
#else
	return b2Vec2(-s * a.y, s * a.x);
#endif
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline b2Vec2 b2Mul(const b2Mat22& A, const b2Vec2& v)
{
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(A.ex.x, A.ey.x, A.ex.y, A.ey.y);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(v.x, v.y, v.x, v.y);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	return b2Vec2(DirectX::XMVectorGetX(v1), DirectX::XMVectorGetZ(v1));
#else
	return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
#endif
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline b2Vec2 b2MulT(const b2Mat22& A, const b2Vec2& v)
{
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(v.x, v.y, 0.0f, 0.0f);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(A.ex.x, A.ex.y, A.ey.x, A.ey.y);
	DirectX::XMVECTOR v3 = DirectX::XMVector2Dot(v1, v2);
	v1 = DirectX::XMVector2Dot(v1, DirectX::XMVectorShiftLeft<2>(v2,DirectX::XMVectorZero()));
	return b2Vec2(DirectX::XMVectorGetX(v3), DirectX::XMVectorGetX(v1));
#else
	return b2Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey));
#endif
}

/// Add two vectors component-wise.
inline b2Vec2 operator + (const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(a.x + b.x, a.y + b.y);
}

/// Subtract two vectors component-wise.
inline b2Vec2 operator - (const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(a.x - b.x, a.y - b.y);
}

inline b2Vec2 operator * (float32 s, const b2Vec2& a)
{
	return b2Vec2(s * a.x, s * a.y);
}

inline bool operator == (const b2Vec2& a, const b2Vec2& b)
{
	return a.x == b.x && a.y == b.y;
}

inline bool operator != (const b2Vec2& a, const b2Vec2& b)
{
	return a.x != b.x || a.y != b.y;
}

inline float32 b2Distance(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = a - b;
	return c.Length();
}

inline float32 b2DistanceSquared(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = a - b;
	return b2Dot(c, c);
}

inline b2Vec3 operator * (float32 s, const b2Vec3& a)
{
	return b2Vec3(s * a.x, s * a.y, s * a.z);
}

/// Add two vectors component-wise.
inline b2Vec3 operator + (const b2Vec3& a, const b2Vec3& b)
{
	return b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/// Subtract two vectors component-wise.
inline b2Vec3 operator - (const b2Vec3& a, const b2Vec3& b)
{
	return b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// Perform the dot product on two vectors.
inline float32 b2Dot(const b2Vec3& a, const b2Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
inline b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b)
{
	return b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B)
{
	return b2Mat22(A.ex + B.ex, A.ey + B.ey);
}

// A * B
inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B)
{
	return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey));
}

// A^T * B
inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B)
{
	b2Vec2 c1(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
	b2Vec2 c2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
	return b2Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
inline b2Vec3 b2Mul(const b2Mat33& A, const b2Vec3& v)
{
	return v.x * A.ex + v.y * A.ey + v.z * A.ez;
}

/// Multiply a matrix times a vector.
inline b2Vec2 b2Mul22(const b2Mat33& A, const b2Vec2& v)
{
	return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply two rotations: q * r
inline b2Rot b2Mul(const b2Rot& q, const b2Rot& r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	b2Rot qr;
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(q.s, q.c, q.c, -q.s);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(r.c, r.s, r.c, r.s);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	qr.s = DirectX::XMVectorGetX(v1);
	qr.c = DirectX::XMVectorGetZ(v1);
#else
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
#endif
	return qr;
}

/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT(const b2Rot& q, const b2Rot& r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	b2Rot qr;
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(q.c, -q.s, q.c, q.s);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(r.s, r.c, r.c, r.s);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	qr.s = DirectX::XMVectorGetX(v1);
	qr.c = DirectX::XMVectorGetZ(v1);
#else
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
#endif
	return qr;
}


/// Rotate a vector
inline b2Vec2 b2Mul(const b2Rot& q, const b2Vec2& v)
{
#ifdef B2_DXMATH
	b2Vec2 ret;
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(q.c, -q.s, q.s, q.c);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(v.x, v.y, v.x, v.y);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	ret.x = DirectX::XMVectorGetX(v1);
	ret.y = DirectX::XMVectorGetZ(v1);
	return ret;
#else
	return b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
#endif
}

#ifdef B2_DXMATH
// returns z-rotated XMVECTOR. Ignore z,w components of result
inline DirectX::XMVECTOR b2Mul(const b2Rot& q, DirectX::XMVECTOR v)
{
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(q.c, -q.s, q.s, q.c);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSwizzle<0,1,0,1>(v);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	return DirectX::XMVectorSwizzle<0,2,2,3>(v1);
}
#endif


/// Inverse rotate a vector
inline b2Vec2 b2MulT(const b2Rot& q, const b2Vec2& v)
{
#ifdef B2_DXMATH
	b2Vec2 ret;
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(q.c, q.s, -q.s, q.c);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(v.x, v.y, v.x, v.y);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	ret.x = DirectX::XMVectorGetX(v1);
	ret.y = DirectX::XMVectorGetZ(v1);
	return ret;
#else
	return b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
#endif
}

#ifdef B2_DXMATH
// returns inverse z-rotated XMVECTOR. Ignore z,w components of result
inline DirectX::XMVECTOR b2MulT(const b2Rot& q, DirectX::XMVECTOR v)
{
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(q.c, q.s, -q.s, q.c);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSwizzle<0,1,0,1>(v);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	return DirectX::XMVectorSwizzle<0,2,2,3>(v1);
}
#endif

inline b2Vec2 b2Mul(const b2Transform& T, const b2Vec2& v)
{
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(v.x, v.y, T.p.x, T.p.y);
//	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(T.p.x, T.p.y, 0.0f, 0.0f);
	DirectX::XMVECTOR v2 = b2Mul(T.q, v1);
	v1 = DirectX::XMVectorAdd(v2,  DirectX::XMVectorRotateLeft<2>(v1));
	float32 x = DirectX::XMVectorGetX(v1);
	float32 y = DirectX::XMVectorGetY(v1);
#else
	float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
#endif
	return b2Vec2(x, y);
}

#ifdef B2_DXMATH
inline DirectX::XMVECTOR b2Mul(const b2Transform& T, DirectX::XMVECTOR v)
{
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(T.p.x, T.p.y, 0.0f, 0.0f);
	DirectX::XMVECTOR v1 = b2Mul(T.q, v);
	return DirectX::XMVectorAdd(v1, v2);
}
#endif

inline b2Vec2 b2MulT(const b2Transform& T, const b2Vec2& v)
{
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(v.x, v.y, 0.0f, 0.0f);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(T.p.x, T.p.y, 0.0f, 0.0f);
	v1 =  DirectX::XMVectorSubtract(v1, v2);
	v2 = b2MulT(T.q, v1);
	float32 x = DirectX::XMVectorGetX(v2);
	float32 y = DirectX::XMVectorGetY(v2);
#else
	float32 px = v.x - T.p.x;
	float32 py = v.y - T.p.y;
	float32 x = (T.q.c * px + T.q.s * py);
	float32 y = (-T.q.s * px + T.q.c * py);
#endif
	return b2Vec2(x, y);
}

#ifdef B2_DXMATH
inline DirectX::XMVECTOR b2MulT(const b2Transform& T, DirectX::XMVECTOR v)
{
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(T.p.x, T.p.y, 0.0f, 0.0f);
	DirectX::XMVECTOR v1 = DirectX::XMVectorSubtract(v, v2);
	return b2MulT(T.q, v1);
}
#endif

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b2Transform b2Mul(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(B.p.x, B.p.y, 0.0f, 0.0f);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(A.p.x, A.p.y, 0.0f, 0.0f);
	C.q = b2Mul(A.q, B.q);
	v1 = b2Mul(A.q, v1);
	v2 = DirectX::XMVectorAdd(v1, v2);
	C.p.x = DirectX::XMVectorGetX(v2);
	C.p.y = DirectX::XMVectorGetY(v2);
#else
	C.q = b2Mul(A.q, B.q);
	C.p = b2Mul(A.q, B.p) + A.p;
#endif
	return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b2Transform b2MulT(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(B.p.x, B.p.y, 0.0f, 0.0f);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(A.p.x, A.p.y, 0.0f, 0.0f);
	C.q = b2MulT(A.q, B.q);
	v1 = DirectX::XMVectorSubtract(v1, v2);
	v2 = b2MulT(A.q, v1);
	C.p.x = DirectX::XMVectorGetX(v2);
	C.p.y = DirectX::XMVectorGetY(v2);
#else
	C.q = b2MulT(A.q, B.q);
	C.p = b2MulT(A.q, B.p - A.p);
#endif
	return C;
}

template <typename T>
inline T b2Abs(T a)
{
	return a > T(0) ? a : -a;
}

inline b2Vec2 b2Abs(const b2Vec2& a)
{
	return b2Vec2(b2Abs(a.x), b2Abs(a.y));
}

inline b2Mat22 b2Abs(const b2Mat22& A)
{
	return b2Mat22(b2Abs(A.ex), b2Abs(A.ey));
}

template <typename T>
inline T b2Min(T a, T b)
{
	return a < b ? a : b;
}

inline b2Vec2 b2Min(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
}

template <typename T>
inline T b2Max(T a, T b)
{
	return a > b ? a : b;
}

inline b2Vec2 b2Max(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
}

template <typename T>
inline T b2Clamp(T a, T low, T high)
{
	return b2Max(low, b2Min(a, high));
}

inline b2Vec2 b2Clamp(const b2Vec2& a, const b2Vec2& low, const b2Vec2& high)
{
	return b2Max(low, b2Min(a, high));
}

template<typename T> inline void b2Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline uint32 b2NextPowerOfTwo(uint32 x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

inline bool b2IsPowerOfTwo(uint32 x)
{
	bool result = x > 0 && (x & (x - 1)) == 0;
	return result;
}

inline void b2Sweep::GetTransform(b2Transform* xf, float32 beta) const
{
	xf->p = (1.0f - beta) * c0 + beta * c;
	float32 angle = (1.0f - beta) * a0 + beta * a;
	xf->q.Set(angle);

	// Shift to origin
	xf->p -= b2Mul(xf->q, localCenter);
}

inline void b2Sweep::Advance(float32 alpha)
{
	b2Assert(alpha0 < 1.0f);
	float32 beta = (alpha - alpha0) / (1.0f - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
	float32 twoPi = 2.0f * b2_pi;
	float32 d =  twoPi * floorf(a0 / twoPi);
	a0 -= d;
	a -= d;
}

#ifdef B2_DXMATH
inline float32 b2DXMathDeepestPoint(const b2Vec2* v2s, int32 count2, const b2Vec2& n, const b2Vec2& v1)
{
	// Find deepest point for normal i.
	float32 si = b2_maxFloat;
	DirectX::XMVECTOR normal = DirectX::XMVectorSet(n.x, n.y, 0.0f, 0.0f);
	DirectX::XMVECTOR vertex = DirectX::XMVectorSet(v1.x, v1.y, v1.x, v1.y);
	DirectX::XMVECTOR v22;
	DirectX::XMVECTOR diff;
	DirectX::XMVECTOR dot1;
	float32 sij;

	int32 j;
	for (j = 0; j+1 < count2; j+=2)
	{
		v22 = DirectX::XMVectorSet(v2s[j].x, v2s[j].y, v2s[j+1].x, v2s[j+1].y);
		diff = DirectX::XMVectorSubtract(v22, vertex);
		dot1 = DirectX::XMVector2Dot(normal, diff);
		diff = DirectX::XMVectorShiftLeft<2>(diff, DirectX::XMVectorZero());
		DirectX::XMVECTOR dot2 = DirectX::XMVector2Dot(normal, diff);
		sij = DirectX::XMVectorGetX(DirectX::XMVectorSelect(dot2, dot1, DirectX::XMVectorLess(dot1, dot2)));
		if (sij < si)
		{
			si = sij;
		}
	}
	if (j < count2)
	{
		v22 = DirectX::XMVectorSet(v2s[j].x, v2s[j].y, 0.0f, 0.0f);
		diff = DirectX::XMVectorSubtract(v22, vertex);
		sij = DirectX::XMVectorGetX(DirectX::XMVector2Dot(normal, diff));
		if (sij < si)
		{
			si = sij;
		}
	}
	return si;
}


#endif
#endif
