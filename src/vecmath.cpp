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


void Vector3f::print(const char* tag) const {
    if (tag == nullptr) {
        printf("(%f, %f, %f)\n", x, y, z);
    } else {
        printf("[%s] (%f, %f, %f)\n", tag, x, y, z);
    }
}


void SoaVector3f::print(const char* tag) const {
    printf("----");

    if (tag != nullptr) {
        printf(" [SoaVector3f(%s)]\n", tag);
    } else {
        printf("\n");
    }

    for (uint32_t i = 0; i < SoaConstants::kLaneCount; i++) {
        auto v = getLane(i);
        printf("[%u] (%f, %f, %f)\n", i, v.x, v.y, v.z);
    }
    printf("----\n");
}


BBox BBox::init()
{
    BBox b; 
    b.lower = std::numeric_limits<float>::max();
    b.upper = std::numeric_limits<float>::lowest();
    return b;    
}



bool BBox::contains(const Vector3f& p) const
{
    return lower.x <= p.x && p.x <= upper.x &&
           lower.y <= p.y && p.y <= upper.y &&
           lower.z <= p.z && p.z <= upper.z;
}

void BBox::merge(const BBox& bbox)
{
    lower = min(lower, bbox.lower);
    upper = max(upper, bbox.upper);
}

void BBox::merge(const Vector3f& p)
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