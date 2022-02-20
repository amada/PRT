#include "triangle.h"

namespace prt
{

const float kEpsilon = 0.0001f;

TriangleIntersection intersectTriangle(
    const Vector3f& org, const Vector3f& dir, const Vector3f& v0, const Vector3f& v1, const Vector3f& v2)
{
    auto o = org;
    auto d = dir;

    // Translate to ray origin
    auto v0r = v0 - o;
    auto v1r = v1 - o;
    auto v2r = v2 - o;

    // Find an element having the largest magnitude
    auto abs_d = abs(dir);

    // Set the largest magnitute element to z-axis
    auto& v = abs_d;
    if (v.x > v.y) {
        if (v.x > v.z) {
            v0r.set(v0r.y, v0r.z, v0r.x);
            v1r.set(v1r.y, v1r.z, v1r.x);
            v2r.set(v2r.y, v2r.z, v2r.x);
            d.set(d.y, d.z, d.x);
        } else {
        }
    } else {
        if (v.y > v.z) {
            v0r.set(v0r.z, v0r.x, v0r.y);
            v1r.set(v1r.z, v1r.x, v1r.y);
            v2r.set(v2r.z, v2r.x, v2r.y);
            d.set(d.z, d.x, d.y);
        } else {
        }
    }

    // TODO can pre-calculate
    auto v0z = v0r.z;
    auto v1z = v1r.z;
    auto v2z = v2r.z;
    auto inv_dz = 1.0f/d.z;
//    auto inv_dz = 1.0f/d.z;

#if 0
    v0r = v0r - v0r.z*ray.m_invDzDir;
    v1r = v1r - v1r.z*ray.m_invDzDir;//*d;
    v2r = v2r - v2r.z*ray.m_invDzDir;//*d;
#else

    v0r = v0r - v0r.z*inv_dz*d;
    v1r = v1r - v1r.z*inv_dz*d;
    v2r = v2r - v2r.z*inv_dz*d;
#endif
    auto e0 = v1r.x*v2r.y - v1r.y*v2r.x;
    auto e1 = v2r.x*v0r.y - v2r.y*v0r.x;
    auto e2 = v0r.x*v1r.y - v0r.y*v1r.x;

    if ((e0 < 0.0 || e1 < 0.0 || e2 < 0.0) && (e0 > 0.0 || e1 > 0.0 || e2 > 0.0)) {
        return TriangleIntersection(TriangleIntersection::kNoIntersection);
    }

    auto det = e0 + e1 + e2;

    if (det == 0.0) {
        return TriangleIntersection(TriangleIntersection::kNoIntersection);
    }

    auto inv_det = 1.0/det;

    auto t_scaled = (e0*v0z + e1*v1z + e2*v2z)*inv_dz;

    auto t = t_scaled*inv_det;

    // Calculate barycentric coordiante i, j, k

    if (t > kEpsilon) {
        TriangleIntersection r;
        r.t = t;
        r.i = e0*inv_det;
        r.j = e1*inv_det;
        r.k = e2*inv_det;

        return r;
    }

    return TriangleIntersection(TriangleIntersection::kNoIntersection);
}

SoaTriangleIntersection intersectTriangle(const SoaMask& _mask, const SoaVector3f& org, const SoaVector3f& dir, const SoaMask& maskX, const SoaMask& maskY, const SoaVector3f& v0, const SoaVector3f& v1, const SoaVector3f& v2)
{
    auto o = org;
    auto d = dir;

    // Translate to ray origin
    auto v0r = v0 - o;
    auto v1r = v1 - o;
    auto v2r = v2 - o;

#if 0
    // Find an element having the largest magnitude
    auto abs_d = d.abs();

    auto max_e = abs_d.maximumElement();
    auto mask_x = max_e.equals(d.getX());
    auto mask_y = max_e.equals(d.getY()).computeXor(mask_x);
#endif

// TODO: should not use SIMD intrinsics directly
#if defined(R_NEON)
    auto swap = [](floatvec_t& v0, floatvec_t& v1, maskvec_t mask) {
        auto temp = vbslq_f32(mask, v1, v0);
        v1 = vbslq_f32(mask, v0, v1);
        v0 = temp;
    };

    auto z_as_max = [&swap](SoaVector3f& v, const SoaMask& mask_x, const SoaMask& mask_y) {
        swap(v.getRefRawX(), v.getRefRawZ(), mask_x.getRawValue());
        swap(v.getRefRawY(), v.getRefRawZ(), mask_y.getRawValue());
    };

    if (maskX.anyTrue() || maskY.anyTrue()) {
        z_as_max(d, maskX, maskY);
        z_as_max(v0r, maskX, maskY);
        z_as_max(v1r, maskX, maskY);
        z_as_max(v2r, maskX, maskY);
    }
#else
    auto swap = [](floatvec_t& v0, floatvec_t& v1, maskvec_t mask) {
        auto temp = AVX_INT(blendv_ps)(v0, v1, mask);
        v1 = AVX_INT(blendv_ps)(v1, v0, mask);
        v0 = temp;
    };

    auto z_as_max = [&swap](SoaVector3f& v, const SoaMask& mask_x, const SoaMask& mask_y) {
        swap(v.getRefRawX(), v.getRefRawZ(), mask_x.getRawValue());
        swap(v.getRefRawY(), v.getRefRawZ(), mask_y.getRawValue());
    };

#if 1
    if (maskX.anyTrue() || maskY.anyTrue()) {
        z_as_max(d, maskX, maskY);
        z_as_max(v0r, maskX, maskY);
        z_as_max(v1r, maskX, maskY);
        z_as_max(v2r, maskX, maskY);
    }
#else
    z_as_max(d, maskX, mask_y);
    z_as_max(v0r, mask_x, mask_y);
    z_as_max(v1r, mask_x, mask_y);
    z_as_max(v2r, mask_x, mask_y);
#endif

#endif

    auto v0z = v0r.getZ();
    auto v1z = v1r.getZ();
    auto v2z = v2r.getZ();
    auto inv_dz = 1.0f/d.getZ();
    auto invDzD = inv_dz*d;
//    auto inv_dz = ray.m_invDz;

#if 0
    v0r = v0r - v0r.getZ()*ray.m_invDzDir;
    v1r = v1r - v1r.getZ()*ray.m_invDzDir;
    v2r = v2r - v2r.getZ()*ray.m_invDzDir;
#else
    v0r = v0r - v0r.getZ()*invDzD;
    v1r = v1r - v1r.getZ()*invDzD;
    v2r = v2r - v2r.getZ()*invDzD;
#endif
    auto e0 = v1r.getX()*v2r.getY() - v1r.getY()*v2r.getX();
    auto e1 = v2r.getX()*v0r.getY() - v2r.getY()*v0r.getX();
    auto e2 = v0r.getX()*v1r.getY() - v0r.getY()*v1r.getX();

    auto mask = _mask;

    SoaTriangleIntersection res;
#if 1
    auto mask_eb = (e0.lessThan(0.0f) | e1.lessThan(0.0f) | e2.lessThan(0.0f))
        & (e0.greaterThan(0.0f) | e1.greaterThan(0.0f) | e2.greaterThan(0.0f));

    mask = mask & mask_eb.computeNot();

    res.t = select(SoaTriangleIntersection::kNoIntersection, res.t, mask);

    if (!mask.anyTrue()) {
        // TODO: initialize other fields?
        return res;
    }
#endif

    auto det = e0 + e1 + e2;

    mask = mask & det.notEquals(0.0f);

    res.t = select(SoaTriangleIntersection::kNoIntersection, res.t, mask);

//    mask_det0 = mask_det0.computeNot();
    if (!mask.anyTrue()) {
        return res;
    }

    auto inv_det = 1.0f/det;

    auto t_scaled = (e0*v0z + e1*v1z + e2*v2z)*inv_dz;

    auto t = t_scaled*inv_det;

    // Calculate barycentric coordiante i, j, k

    mask = mask & t.greaterThan(kEpsilon);

    SoaTriangleIntersection r;

    r.t = select(SoaTriangleIntersection::kNoIntersection, t, mask);
    r.i = e0*inv_det;
    r.j = e1*inv_det;
    r.k = e2*inv_det;

    return r;
}

} // namespace prt