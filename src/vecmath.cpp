#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <limits>
#include <type_traits>

#include "vecmath.h"
#include "triangle.h"

#include "bvh.h"

namespace prt {

static_assert(std::is_pod<Vector3f>::value, "Vector3f must be POD");
static_assert(std::is_pod<SoaVector3f>::value, "SoaVector3f must be POD");
static_assert(std::is_pod<BBox>::value, "BBox must be POD");


void Vector3f::print() const {
    printf("(%f, %f, %f)\n", x, y, z);
}


BBox BBox::init()
{
    BBox b; 
    b.lower = std::numeric_limits<float>::max();
    b.upper = std::numeric_limits<float>::lowest();
    return b;    
}

float BBox::intersect(const Vector3f& org, const Vector3f& dir, const Vector3f& invDir) const
{
    auto tmp_t0 = (lower - org)*invDir;
    auto tmp_t1 = (upper - org)*invDir;

    auto t0 = min(tmp_t0, tmp_t1);
    auto t1 = max(tmp_t0, tmp_t1);

    auto max_t0 = std::max(t0.x, std::max(t0.y, t0.z));
    auto min_t1 = std::min(t1.x, std::min(t1.y, t1.z));

    if (min_t1 < max_t0) {
        return kNoHit;
    }

    // inside bbox
    if (max_t0 < 0.0f) {
        return 0.0f;
    }

    return max_t0;
}


SoaFloat BBox::intersect(const SoaVector3f& org, const SoaVector3f& dir, const SoaVector3f& invDir) const
{
    auto tmp_t0 = (SoaVector3f(lower) - org)*invDir;
    auto tmp_t1 = (SoaVector3f(upper) - org)*invDir;

    auto t0 = tmp_t0.min(tmp_t1);
    auto t1 = tmp_t0.max(tmp_t1);

    auto max_t0 = t0.maximumElement();
    auto min_t1 = t1.minimumElement();

    auto mask = min_t1.greaterThanOrEqual(max_t0);

    SoaFloat r(max_t0);
    r = select(kNoHit, r, mask);

    // inside box
    auto temp = select(max_t0, 0.0f, max_t0.lessThan(0.0f));

    return select(r, temp, mask);
}

void BBox::grow(const BBox& bbox)
{
    lower = min(lower, bbox.lower);
    upper = max(upper, bbox.upper);
}

void BBox::grow(const Vector3f& p)
{
    lower = min(lower, p);
    upper = max(upper, p);
}

float BBox::surfaceArea() const
{
    auto e = upper - lower;
    return 2.0f*(e.x*e.y + e.y*e.z + e.z*e.x);
}

void BBox::print() const
{
    printf("(%f, %f, %f) - (%f, %f, %f)\n", lower.x, lower.y, lower.z, upper.x, upper.y, upper.z);
}



} // namespace prt