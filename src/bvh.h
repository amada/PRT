#pragma once

#include <vector>

#include "vecmath.h"
#include "triangle.h"
#include "ray.h"
#include "mesh.h"

namespace prt
{


class Bvh;

class BvhBuildNode
{
    friend class Bvh;
public:
    const static int32_t kMaxPrimCountInNode = 4;

    struct BuildContext
    {
        uint32_t nodeCount;
        uint32_t primCountInNode[kMaxPrimCountInNode]; // Only used for stats
    };

private:
    void calculateBounds(const uint32_t* primRemapping, const Mesh& mesh, int32_t start, int32_t end);
    void build(BuildContext& context, uint32_t* primRemapping, const Mesh& mesh, int32_t start, int32_t end);

    BBox m_bbox = BBox::init();

    const static uint32_t kChildCount = 2;
    const static int32_t kInteriorNode = -1;

    BvhBuildNode* m_children[kChildCount];
    int32_t m_primIndex = kInteriorNode;
    int16_t m_primCount = 0;
    int16_t m_splitAxis = 0;
};

#define TRI_VECTOR

struct LinearBvhNode
{
    const static int16_t kInternalNode = -1;

    BBox bbox;

    int32_t primOrSecondNodeIndex;
    int32_t triVectorIndex : 26;
    int16_t primCount : 4; // triCount?
    int16_t splitAxis : 2; // Axis splitting BVH node
};

//static_assert(sizeof(LinearBvhNode) == 64, "Size of LinearBvhNode must be multiples of cache line size");

struct TraverseStackCache
{
    static const int32_t kNodeStackCapacity = 64;
    LinearBvhNode* nodes[kNodeStackCapacity];
    int32_t size;
};

struct TriangleVector
{
    static const uint32_t kSize = 4; // Support for 8

    SoaVector3f p0; // 48B
    SoaVector3f p1;
    SoaVector3f p2;

    int32_t alphaTest[kSize];
    SoaVector2f uv0; // 32B
    SoaVector2f uv1;
    SoaVector2f uv2;
};

static_assert(sizeof(TriangleVector) == 256 || sizeof(TriangleVector) == 512, "Size of TriangleVector must be multiples of cache line size");

class Bvh
{
    friend class BvhNode;
    friend class Scene;
public:
    static constexpr float kNoHitT = -1.0f;
    // TODO constructor, destructor
    Bvh() = default;

    template<typename T, typename R>
    void intersect(T& result, const R& packet, const TraverseStackCache* stackCache) const;

    template<typename T, typename R>
    T occluded(const RayPacketMask& mask, const R& packet) const;

    void createStackCache(TraverseStackCache& stackCache, const Vector3f& pos, const Vector3f& dir) const;

    void build(Mesh&& mesh);
private:
    static constexpr float kEpsilon = 0.0001f;

    void buildLinearBvhNodes(LinearBvhNode* nodes, int32_t* index, BvhBuildNode* node, int32_t* triVectorIndex);

    Mesh m_mesh; // TODO: should have multiple meshes
    
     // Remapping index in node to mesh vertex index
     // TODO: can remove if this just maps to primitive/index, but should include mesh index
    uint32_t* m_primRemapping;
    LinearBvhNode* m_nodes;
    TriangleVector* m_triangleVectors;
};


#if 0
struct LinearBvh
{
};

template<typename T, typename U>
struct SurfacePropertiesT
{
    T normal;
    U material;
    Vector2f uv;
};

using SurfaceProperties = SurfacePropertiesT<Vector3f, const Material*>;
using SoaSurfaceProperties = SurfacePropertiesT<SoaVector3f, SoaVar<const Material*>>;


class Scene
{
public:
    template<typename T, typename U>
    void intersect(T& intr, const U& ray) const;


    template<typename T, typename U>
    void getSurfaceProperties(T& properties, const U& intr) const;

    void Add(Bvh* bvh) { 
        bvh->m_id = m_bvh.size();
        m_bvh.push_back(bvh); 
    } // shared ptr?

private:
    // TODO remove std::vector
    std::vector<Bvh*> m_bvh;
};
#endif

} // namespace prt
