#pragma once

#include "vecmath.h"

namespace prt
{

// TODO
// template<typename T>
//struct RayT

class Ray
{
public:
    Ray() = default;
    Vector3f org;
    Vector3f dir;
    float maxT;
    Vector3f invDir;

    float invDz;
    Vector3f invDzDir;

//    bool m_verbose = false;

    void prepare() {
        invDir = 1.0f/dir;
        auto d = dir;
        // Find an element having the largest magnitude
        auto abs_d = abs(d);

        // Set the largest magnitute element to z-axis
        auto& v = abs_d;
        if (v.x > v.y) {
            if (v.x > v.z) {
                d.set(d.y, d.z, d.x);
            } else {
            }
        } else {
            if (v.y > v.z) {
                d.set(d.z, d.x, d.y);
            } else {
            }
        }

        invDz = 1.0f/d.z;
        invDzDir = invDz*d;        
    }
};

class SoaRay
{
public:
    SoaRay() = default;
    SoaVector3f org;
    SoaVector3f dir;
    SoaVector3f invDir;
    SoaFloat maxT;

    SoaMask swapXZ;
    SoaMask swapYZ;

    Vector3f avgDir;

//    bool m_verbose = false;

// TODO: should calculate after org, dir are set
    void prepare() {
        invDir = 1.0f/dir;
        auto d = dir;
        // Pre-calculation
        // Find an element having the largest magnitude
        auto abs_d = d.abs();

        auto max_e = abs_d.maximumElement();
        auto mask_x = max_e.equals(abs_d.getX()); // Mask if max element is x
        auto mask_y = max_e.equals(abs_d.getY()).computeAnd(mask_x.computeNot()); // Mask if max element is y

        swapXZ = mask_x;
        swapYZ = mask_y;

        Vector3f avg(0.0f);
        for (uint32_t i = 0; i < SoaConstants::kLaneCount; i++) {
            avg = avg + d.getLane(i);
        }
        avgDir = avg/SoaConstants::kLaneCount;
    }
};

} // namespace prt