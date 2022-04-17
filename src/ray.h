#pragma once

#include "prt.h"
#include "stats.h"
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
    }
};

struct SingleRayPacket
{
    Ray ray;

    SingleRayPacket(const Ray& ray) : ray(ray) {}
};

struct RayPacket
{
    static const uint32_t kSize = 8;
    static const uint32_t kVectorCount = kSize/SoaConstants::kLaneCount;
    SoaRay rays[kVectorCount];
    Vector3f avgDir;
//    uint32_t count;
};

struct RayPacketMask
{
    SoaMask masks[RayPacket::kVectorCount];

#if 0
    static RayPacketMask initAllTrue() {
        RayPacketMask result;
        for (uint32_t i = 0; i < RayPacket::kVectorCount; i++) {
            result.masks[i] = SoaMask::initAllTrue();
        }
        return result;
    }
#endif

    RayPacketMask operator&(const RayPacketMask& other) const {
        RayPacketMask result;
        for (uint32_t i = 0; i < RayPacket::kVectorCount; i++) {
            result.masks[i] = masks[i] & other.masks[i];
        }
        return result;
    }

    void computeAndSelf(uint32_t lane, const SoaMask& mask) {
        masks[lane] = masks[lane] & mask;
    }

    void setAll(bool b) {
        for (uint32_t i = 0; i < RayPacket::kVectorCount; i++) {
            masks[i].setAll(b);
        }
    }

    bool anyTrue(uint32_t lane) const {
        return masks[lane].anyTrue();
    }

    bool anyTrue() const {
        for (uint32_t i = 0; i < RayPacket::kVectorCount; i++) {
            if (masks[i].anyTrue()) {
                return true;
            }
        }
        return false;
    }
};

template<typename T, typename U>
struct RayHitT
{
    static const constexpr float kMissT = -1.0f;

    RayHitT() = default;
    bool isHit() const { return t != kMissT; }

    T t;
    T i, j, k; // barycentric coordinate

    U primId;
    U meshId;
};

using RayHit = RayHitT<float, uint32_t>;
using SoaRayHit = RayHitT<SoaFloat, SoaInt>;

struct SingleRayHitPacket
{
    RayHit hit;

#ifdef PRT_ENABLE_STATS
    Stats stats;
#endif

    void setMaxT(const SingleRayPacket& packet) {
        hit.t = packet.ray.maxT;
    }

    void setMissForMaxT(const SingleRayPacket& packet) {
        if (hit.t == packet.ray.maxT)
            hit.t = RayHit::kMissT;        
    }
};

struct RayHitPacket
{
    SoaRayHit hits[RayPacket::kVectorCount];

#ifdef PRT_ENABLE_STATS
    Stats stats;
#endif

    void setMaxT(const RayPacket& packet) {
        for (uint32_t i = 0; i < RayPacket::kVectorCount; i++)
            hits[i].t = packet.rays[i].maxT;
    }

    void setMissForMaxT(const RayPacket& packet) {
        for (uint32_t i = 0; i < RayPacket::kVectorCount; i++) {
            auto mask = hits[i].t.equals(packet.rays[i].maxT);
            hits[i].t = select(hits[i].t, RayHit::kMissT, mask);
        }
    }        
};

struct SingleRayOccludedPacket
{
    bool occluded;

#ifdef PRT_ENABLE_STATS
    Stats stats;
#endif
};


} // namespace prt