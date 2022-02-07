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

    SoaMask maskX;
    SoaMask maskY;

//    bool m_verbose = false;

// TODO: should calculate after org, dir are set
    void prepare() {
        invDir = 1.0f/dir;
        auto d = dir;
        // Pre-calculation
        // Find an element having the largest magnitude
        auto abs_d = d.abs();

        auto max_e = abs_d.maximumElement();
        auto mask_x = max_e.equals(d.getX());
        auto mask_y = max_e.equals(d.getY()).computeXor(mask_x);

        maskX = mask_x;
        maskY = mask_y;
    }


};

} // namespace prt