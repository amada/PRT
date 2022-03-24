#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <thread>
#include <limits>
#include <type_traits>

#include "vecmath.h"
#include "triangle.h"
#include "ray.h"

#include "bvh.h"

namespace prt
{

void BvhBuildNode::calculateBounds(const uint32_t* primRemapping, const Mesh& mesh, int32_t start, int32_t end)
{
    for (int32_t i = start; i <= end; i++) {
        const int32_t indexBase = Mesh::kVertexCountPerPrim*primRemapping[i];
        for (int32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
            m_bbox.grow(mesh.getPosition(mesh.getIndex(indexBase + j)));
        }
    }
}

void BvhBuildNode::build(BuildContext& context, uint32_t* primRemapping, const Mesh& mesh, int32_t start, int32_t end)
{
    calculateBounds(primRemapping, mesh, start, end);

    int32_t primCount = end - start + 1;
    if (primCount <= 4) {
        m_primIndex = start;
        m_primCount = primCount;
        m_children[0] = nullptr;
        m_children[1] = nullptr;
        return;
    }

    auto& bbox = m_bbox;

    BvhBuildNode* nodes[kChildCount];
    for (int32_t i = 0; i < kChildCount; i++) {
        nodes[i] = new BvhBuildNode;
    }
    context.nodeCount += kChildCount;

    const auto extent = bbox.upper - bbox.lower;
    const uint32_t dim = (bbox.upper - bbox.lower).GetLongestElement();
    const float splitExtent = extent.v[dim];
    const float lowerPos = bbox.lower.v[dim];

    const uint32_t kBucketCount = 12;

    struct Bucket {
        uint32_t count = 0;
        BBox bounds = BBox::init();
    };

    Bucket buckets[kBucketCount];

    for (int32_t i = start; i <= end; i++) {
        Vector3f temp(0.0f);
        uint32_t indexBase = Mesh::kVertexCountPerPrim*primRemapping[i];
        for (int32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
            temp = temp + mesh.getPosition(mesh.getIndex(indexBase + j));
        }
        temp = 1.0f/Mesh::kVertexCountPerPrim*temp;

        int32_t b = kBucketCount*(temp.v[dim] - lowerPos)/splitExtent;
        if (b == kBucketCount) b = b - 1;

        buckets[b].count++;
        buckets[b].bounds.grow(temp);
    }

    float lowestCost = std::numeric_limits<float>::max();
    int32_t lowestCostSplit = -1;
    for (uint32_t i = 0; i < kBucketCount - 1; i++) {
        uint32_t countLeft = 0;
        BBox boundsLeft = BBox::init();
        for (uint32_t j = 0; j <= i; j++) {
            countLeft += buckets[j].count;
            boundsLeft.grow(buckets[j].bounds);
        }

        uint32_t countRight = 0;
        BBox boundsRight = BBox::init();
        for (uint32_t j = i + 1; j < kBucketCount; j++) {
            countRight += buckets[j].count;
            boundsRight.grow(buckets[j].bounds);
        }

        float cost = 0.125f + (countLeft*boundsLeft.surfaceArea() + countRight*boundsRight.surfaceArea());
        if (lowestCost > cost) {
            lowestCost = cost;
            lowestCostSplit = i;
        }
    }

    float splitPos = lowerPos + (lowestCostSplit + 1)*splitExtent/kBucketCount;

    auto splitLeft = [&mesh, splitPos, &primRemapping, dim](int32_t i) {
        Vector3f temp(0.0f);
        uint32_t indexBase = Mesh::kVertexCountPerPrim*primRemapping[i];
        for (int32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
            temp = temp + mesh.getPosition(mesh.getIndex(indexBase + j));
        }
        temp = 1.0f/Mesh::kVertexCountPerPrim*temp;        

        return temp.v[dim] < splitPos;
    };

    int32_t cursor;
    for (cursor = start; cursor <= end; cursor++) {
        if (!splitLeft(cursor))
            break;
    }

    for (int32_t i = cursor + 1; i <= end; i++) {
        if (splitLeft(i)) {
            std::swap(primRemapping[i], primRemapping[cursor]);
            cursor++;
        }
    }

    int32_t mid = cursor - 1;
    if (mid <= start || end >= mid) {
        mid = (start + end)/2;
    } else {
    }

    nodes[0]->build(context, primRemapping, mesh, start, mid);
    nodes[1]->build(context, primRemapping, mesh, mid + 1, end);

    m_children[0] = nodes[0];
    m_children[1] = nodes[1];
}

void Bvh::build(Mesh&& mesh)
{
    m_mesh = std::move(mesh);

    // TODO: Can pre-calculate bounds for primitives
    const uint32_t primRemappingSize = m_mesh.getPrimCount();
    m_primRemapping = new uint32_t[primRemappingSize];

    for (uint32_t i = 0; i < primRemappingSize; i++) {
        m_primRemapping[i] = i;
    }

    BvhBuildNode::BuildContext context;
    context.nodeCount = 1;

    auto rootBuildNode = new BvhBuildNode;
    rootBuildNode->build(context, m_primRemapping, m_mesh, 0, primRemappingSize - 1);

    LinearBvhNode* nodes = new LinearBvhNode[context.nodeCount];

//    printf("nodes=%u\n", context.nodeCount);
    int32_t index = 0;
    buildLinearBvhNodes(nodes, &index, rootBuildNode);

    m_nodes = nodes;
}

void Bvh::buildLinearBvhNodes(LinearBvhNode* nodes, int32_t* index, BvhBuildNode* node)
{
    LinearBvhNode& l = nodes[*index];
    (*index)++;

    l.bbox = node->m_bbox;

    if (node->m_primIndex == BvhBuildNode::kInteriorNode) {
        l.primCount = LinearBvhNode::kInternalNode;
        buildLinearBvhNodes(nodes, index, node->m_children[0]);

        l.primOrSecondNodeIndex = *index;
        buildLinearBvhNodes(nodes, index, node->m_children[1]);
    } else {
        l.primCount = node->m_primCount;
        l.primOrSecondNodeIndex = node->m_primIndex;
    }

    delete node;
}

template<typename T, typename U>
void Bvh::intersect(T& intr, const U& ray) const
{
    const uint32_t kStackSize = 64;
    int32_t current = 0;
    LinearBvhNode* nodes[kStackSize];
    nodes[current] = &m_nodes[0];

    SoaMask masks[kStackSize];
    if constexpr (std::is_same<U, SoaRay>::value) {
        masks[current] = SoaMask::initAllTrue();
    }

    do {
        auto node = nodes[current];
        auto mask = masks[current];

        auto t = node->bbox.intersect(ray.org, ray.dir, ray.invDir);
        bool hit;

        if constexpr (std::is_same<U, Ray>::value) {
            hit = (t != BBox::kNoHit && t < intr.t);
        } else if constexpr (std::is_same<U, SoaRay>::value) {
            auto maskBbox = t.notEquals(BBox::kNoHit).computeAnd(t.lessThan(intr.t));

            mask = maskBbox & mask;
            hit = mask.anyTrue();
        }

        if (!hit) {
            // Do nothing
        } else if (node->primCount == LinearBvhNode::kInternalNode) {
            if (hit) {
                // TODO: order
                nodes[current + 0] = node + 1;
                nodes[current + 1] = &m_nodes[node->primOrSecondNodeIndex];
                if constexpr (std::is_same<U, SoaRay>::value) {
                    masks[current + 0] = mask;
                    masks[current + 1] = mask;
                }

                current += 2;

                if (current >= kStackSize) {
                    __builtin_trap();
                }
            }
        } else {
            auto& m = m_mesh;

            for (int32_t i = 0; i < node->primCount; i++) {
                uint32_t primIndex = m_primRemapping[node->primOrSecondNodeIndex + i];
                uint32_t indexBase = primIndex*Mesh::kVertexCountPerPrim;
                // TODO skip two-level indircection
                uint32_t v0 = m.getIndex(indexBase + 0);
                uint32_t v1 = m.getIndex(indexBase + 1);
                uint32_t v2 = m.getIndex(indexBase + 2);
                const Vector3f &p0 = m.getPosition(v0);
                const Vector3f &p1 = m.getPosition(v1);
                const Vector3f &p2 = m.getPosition(v2);

                static_assert(kEpsilon > TriangleIntersection::kNoIntersection, "kEpsilon must be larger than TriangleIntersectionT::kNoIntersection to reject no intersection");

                if constexpr (std::is_same<U, Ray>::value) {
                    auto triIntr = intersectTriangle(ray.org, ray.dir, p0, p1, p2);

                    if (triIntr.t >= kEpsilon && triIntr.t < intr.t) {
                        intr.t = triIntr.t;
                        intr.i = triIntr.i;
                        intr.j = triIntr.j;
                        intr.k = triIntr.k;
                        intr.primId = primIndex;
                        intr.meshId = m_id;
                    }
                } else if constexpr (std::is_same<U, SoaRay>::value) {
                    auto triIntr = intersectTriangle(mask, ray.org, ray.dir, ray.maskX, ray.maskY, p0, p1, p2);

                    auto maskHit = triIntr.t.greaterThanOrEqual(kEpsilon) & triIntr.t.lessThan(intr.t);

                    maskHit = maskHit & mask;

                    if (maskHit.anyTrue()) {
                        intr.t = select(intr.t, triIntr.t, maskHit);
                        intr.i = select(intr.i, triIntr.i, maskHit);
                        intr.j = select(intr.j, triIntr.j, maskHit);
                        intr.k = select(intr.k, triIntr.k, maskHit);
                        intr.primId = select(intr.primId, primIndex, maskHit);
                        intr.meshId = select(intr.meshId, m_id, maskHit);
                    }
                }
            }
        }

        current--;
    } while (current >= 0);
}


template<typename T, typename U>
void Scene::getSurfaceProperties(T& properties, const U& intr) const
{
    if constexpr (std::is_same<U, RayIntersection>::value) {
        Vector3f normal;

        auto bvh = m_bvh[intr.meshId];
        auto& mesh = bvh->m_mesh;

        uint32_t indexBase = intr.primId*Mesh::kVertexCountPerPrim;
        uint32_t v0 = mesh.getIndex(indexBase + 0);
        uint32_t v1 = mesh.getIndex(indexBase + 1);
        uint32_t v2 = mesh.getIndex(indexBase + 2);

        if (bvh->m_mesh.hasVertexNormal()) {
            const auto &n0 = mesh.getNormal(v0);
            const auto &n1 = mesh.getNormal(v1);
            const auto &n2 = mesh.getNormal(v2);
            normal = normalize(intr.i * n0 + intr.j * n1 + intr.k * n2);
        } else {
            const Vector3f &p0 = mesh.getPosition(v0);
            const Vector3f &p1 = mesh.getPosition(v1);
            const Vector3f &p2 = mesh.getPosition(v2);
            normal = normalize(cross(p1 - p0, p2 - p0));
        }

// TODO: remove indirection?
// use 3 floats at once
        v0 = mesh.getTexcoordIndex(indexBase + 0);
        v1 = mesh.getTexcoordIndex(indexBase + 1);
        v2 = mesh.getTexcoordIndex(indexBase + 2);
        #if 1
        const auto& t0 = mesh.getTexcoord(v0);
        const auto& t1 = mesh.getTexcoord(v1);
        const auto& t2 = mesh.getTexcoord(v2);
        properties.uv = intr.i*t0 + intr.j*t1 + intr.k*t2;
        #endif

        properties.normal = normal;
        properties.material = &mesh.getMaterial(mesh.getPrimToMaterial(intr.primId));
    } else if constexpr (std::is_same<U, SoaRayIntersection>::value) {
        __builtin_trap();
    } else {
        __builtin_trap();
    }
}

template void Scene::getSurfaceProperties<SurfaceProperties, RayIntersection>(SurfaceProperties&, const RayIntersection&) const;

template<typename T, typename U>
void Scene::intersect(T& intr, const U& ray) const
{
    memset(&intr, 0, sizeof(intr));

    // Find intersection closer than maxT
    intr.t = ray.maxT;

    for (auto &bvh : m_bvh) {
        // TODO ray transform per BVH

        bvh->intersect(intr, ray);
    }

    if constexpr (std::is_same<U, Ray>::value) {
        if (intr.t == ray.maxT) {
            intr.t = RayIntersection::kNoHitT;
        }
    } else if constexpr (std::is_same<U, SoaRay>::value) {
        auto mask = intr.t.equals(ray.maxT);
        intr.t = select(intr.t, SoaRayIntersection::kNoHitT, mask);
    }    
}

template void Scene::intersect<RayIntersection, Ray>(RayIntersection&, const Ray&) const;
template void Scene::intersect<SoaRayIntersection, SoaRay>(SoaRayIntersection&, const SoaRay&) const;

} // namespace prt
