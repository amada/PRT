#pragma once

#include "vecmath.h"

namespace prt
{

template<typename T>
struct TriangleIntersectionT
{
    static constexpr float kNoIntersection = -1.0f;

    T t;

    // Barycentric coordinates
    T i;
    T j;
    T k;

    TriangleIntersectionT() = default;
    TriangleIntersectionT(T _t) : t(_t) {}
};

using TriangleIntersection = TriangleIntersectionT<float>;
using SoaTriangleIntersection = TriangleIntersectionT<SoaFloat>;

TriangleIntersection intersectTriangle(const Vector3f& org, const Vector3f& dir, const Vector3f& v0, const Vector3f& v1, const Vector3f& v2);

SoaTriangleIntersection intersectTriangle(const SoaMask& mask, const SoaVector3f& org, const SoaVector3f& dir, const SoaMask& swapXZ, const SoaMask& swapYZ, const SoaVector3f& v0, const SoaVector3f& v1, const SoaVector3f& v2);

} // namespace prt