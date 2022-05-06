#pragma once

#include <vector>

#include "vecmath.h"
#include "triangle.h"
#include "ray.h"
#include "mesh.h"

namespace prt
{


class ThreadPool;
class Bvh;

class BvhBuildNode
{
    friend class Bvh;
public:
    const static int32_t kMaxPrimCountInNode = SoaConstants::kLaneCount;

    struct BuildContext
    {
        ThreadPool* threadPool;
        const Mesh* mesh;
        uint32_t* primRemapping;

        // TODO: better not to use atomic here for perf, instead accumulate the values after works have been done
        std::atomic<uint32_t> nodeCount;
        std::atomic<uint32_t> primCountInNode[kMaxPrimCountInNode]; // Only used for stats
    };

private:
    void calculateBounds(const uint32_t* primRemapping, const Mesh& mesh, int32_t start, int32_t end);
    void build(BuildContext& context, int32_t start, int32_t end, int32_t level);

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
    const static uint32_t kInternalNode = 0xf;
    static_assert(kInternalNode > BvhBuildNode::kMaxPrimCountInNode, "kInternalNode is reserved to indicate whether node is internal or leaf");

    BBox bbox;

    uint32_t primOrSecondNodeIndex;
    uint32_t triVectorIndex : 26;
    uint32_t primCount : 4; // triCount?
    uint32_t splitAxis : 2; // Axis splitting BVH node
};

static_assert(sizeof(LinearBvhNode) == 32, "Size of LinearBvhNode should be multiples, or division of power of two, of cache line size");

struct TraverseStackCache
{
    static const int32_t kNodeStackCapacity = 64;
    LinearBvhNode* nodes[kNodeStackCapacity];
    int32_t size;
};

struct TriangleVector
{
    static const uint32_t kSize = SoaConstants::kLaneCount;
    static_assert(TriangleVector::kSize >= BvhBuildNode::kMaxPrimCountInNode, "TriangleVector needs to be able to contain as many triangle as BvhBuildNode::kMaxPrimCountInNode");

    static const uint32_t kVectorCount = kSize/SoaConstants::kLaneCount;

    SoaVector3f p0[kVectorCount];
    SoaVector3f p1[kVectorCount];
    SoaVector3f p2[kVectorCount];

    int32_t alphaTest[kSize];
    SoaVector2f uv0[kVectorCount];
    SoaVector2f uv1[kVectorCount];
    SoaVector2f uv2[kVectorCount];
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


} // namespace prt
