# box2d-for-windows-phone-8
Box2d WP8 lib. 
Define B2_CONCURRENT_SOLVER in b2ContactSolver.cpp in order to use parallel pattern library (ppl) optimisations.
'''C
---  B2_CONCURRENT_SOLVER example begin ---
// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints()
{
	
\#ifdef  B2_CONCURRENT_SOLVER
	concurrency::combinable<float> minSeparation;
	concurrency::parallel_for(0, m_count, [&](int32 i)
	{
		minSeparation.local() = 0.0f;
\#else
	float32 minSeparation = 0.0f;
	for (int32 i = 0; i < m_count; ++i)
	{
\#endif
//code removed
\#ifdef B2_CONCURRENT_SOLVER
			if (separation < minSeparation.local())
			{				
				minSeparation.local() = separation;
			}
\#else
			minSeparation = b2Min(minSeparation, separation);
\#endif
//code removed
\#ifdef B2_CONCURRENT_SOLVER
	});
	float result = minSeparation.combine([](float left, float right) { return (left < right) ? left : right; });
	return result >= -3.0f * b2_linearSlop;
\#else
	}
	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0f * b2_linearSlop;
\#endif
---  B2_CONCURRENT_SOLVER example end ---
'''
TODO: Use B2_CONCURRENT_SOLVER also in b2CollidePolygon.cpp.



Define B2_DXMATH in b2MAth.h in order to use DirectXMath optimised math routines.
'''C
--- B2_DXMATH example begin ---
/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT(const b2Rot& q, const b2Rot& r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	b2Rot qr;
\#ifdef B2_DXMATH
	DirectX::XMVECTOR v1 = DirectX::XMVectorSet(q.c, -q.s, q.c, q.s);
	DirectX::XMVECTOR v2 = DirectX::XMVectorSet(r.s, r.c, r.c, r.s);
	v1 = DirectX::XMVectorMultiply(v1, v2);
	v2 = DirectX::XMVectorRotateLeft<1>(v1);
	v1 = DirectX::XMVectorAdd(v1, v2);
	qr.s = DirectX::XMVectorGetX(v1);
	qr.c = DirectX::XMVectorGetZ(v1);
\#else
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
\#endif
	return qr;
}

--- B2_DXMATH example end ---
'''
