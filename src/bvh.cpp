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
    const uint32_t kBucketCount = 14;

    float lowestCost = std::numeric_limits<float>::max();
    uint32_t lowestDim = 0;
    int32_t lowestCostSplit = -1;

    // Check cost for every dimension for SAH
    for (uint32_t dim = 0; dim < 3; dim++) {
        const float splitExtent = extent.v[dim] == 0.0 ? 0.0001 : extent.v[dim]; // Maybe skip this dim
        const float lowerPos = bbox.lower.v[dim];
        struct Bucket {
            uint32_t count = 0;
            BBox bounds = BBox::init();
        };

        Bucket buckets[kBucketCount];

        // Calculate computation cost for each bucket
        for (int32_t i = start; i <= end; i++) {
            Vector3f temp(0.0f);
            uint32_t indexBase = Mesh::kVertexCountPerPrim*primRemapping[i];
            auto primBounds = BBox::init();
            for (int32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
                auto v = mesh.getPosition(mesh.getIndex(indexBase + j));
                temp = temp + v;
                primBounds.grow(v);
            }
            temp = 1.0f/Mesh::kVertexCountPerPrim*temp;

            int32_t b = kBucketCount*(temp.v[dim] - lowerPos)/splitExtent;
            if (b >= kBucketCount) b = kBucketCount - 1;
            if (b < 0) { b = 0; }

            buckets[b].count++;
            buckets[b].bounds.grow(primBounds);
        }

        // Find the lowest cost split
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
                lowestDim = dim;
                lowestCost = cost;
                lowestCostSplit = i;
            }
        }
    }

    const uint32_t dim = lowestDim;
    const float splitExtent = extent.v[dim] == 0.0 ? 0.0001 : extent.v[dim];
    const float lowerPos = bbox.lower.v[dim];

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

    m_splitAxis = dim;

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
    l.splitAxis = node->m_splitAxis;

    if (node->m_primIndex == BvhBuildNode::kInteriorNode) {
        l.primCount = LinearBvhNode::kInternalNode;
        buildLinearBvhNodes(nodes, index, node->m_children[0]);

        l.primOrSecondNodeIndex = *index;
        buildLinearBvhNodes(nodes, index, node->m_children[1]);
    } else {
        l.primOrSecondNodeIndex = node->m_primIndex;
        l.primCount = node->m_primCount;
    }

    delete node;
}

template<typename T, typename R>
void Bvh::intersect(T& hitPacket, const R& packet, const TraverseStackCache* stackCache) const
{
    const uint32_t kStackSize = 64;
    int32_t current = 0;
    LinearBvhNode* nodes[kStackSize];
    nodes[current] = &m_nodes[0];

    bool reverseStackOrder[3];
    RayPacketMask masks[kStackSize];
    if constexpr (std::is_same<R, SingleRayPacket>::value) {
        for (uint32_t i = 0; i < 3; i++)
            reverseStackOrder[i] = packet.ray.dir.v[i] < 0.0f;
    } else if constexpr (std::is_same<R, RayPacket>::value) {
        masks[current].setAll(true);
        for (uint32_t i = 0; i < 3; i++)
            reverseStackOrder[i] = packet.avgDir.v[i] < 0.0f;
    }

    if (stackCache) {
        memcpy(nodes, stackCache->nodes, sizeof(LinearBvhNode*)*stackCache->size);
        current = stackCache->size - 1;
        for (uint32_t i = 0; i < stackCache->size; i++)
            masks[i].setAll(true);
    }

    do {
#ifdef PRT_ENABLE_STATS
        hitPacket.stats.nodesTraversed++;
#endif
        auto node = nodes[current];
        auto mask = masks[current];

        bool hit;

        if constexpr (std::is_same<R, SingleRayPacket>::value) {
            hit = node->bbox.intersect(packet.ray.org, packet.ray.dir, packet.ray.invDir, hitPacket.hit.t);
        } else if constexpr (std::is_same<R, RayPacket>::value) {
            // TODO: for loop packet.count
            for (uint32_t i = 0; i < RayPacket::kVectorCount; i++) {
                auto bbox_mask = node->bbox.intersect(packet.rays[i].org, packet.rays[i].dir, packet.rays[i].invDir, hitPacket.hits[i].t);
                mask.computeAndSelf(i, bbox_mask);
            }

            hit = mask.anyTrue();
        }

        int16_t primCount = node->primCount;

        if (!hit) {
            // Do nothing
        } else if (primCount == LinearBvhNode::kInternalNode) {
            // Visit closer child first
            if (reverseStackOrder[node->splitAxis]) {
                nodes[current + 0] = node + 1;
                nodes[current + 1] = &m_nodes[node->primOrSecondNodeIndex];
            } else {
                nodes[current + 0] = &m_nodes[node->primOrSecondNodeIndex];
                nodes[current + 1] = node + 1;
            }
            if constexpr (std::is_same<R, RayPacket>::value) {
                masks[current + 0] = mask;
                masks[current + 1] = mask;
            }

            current += 2;

            PRT_ASSERT(current < kStackSize);
        } else {
            auto& m = m_mesh;

// TODO: give all primIndex to m.intersect? rather than loop?
            int32_t primIndexPreMap = node->primOrSecondNodeIndex;
            for (int32_t i = 0; i < primCount; i++) {
                uint32_t primIndex = m_primRemapping[primIndexPreMap + i];
                m.intersect(hitPacket, mask, packet, primIndex);
            }
        }

        current--;
    } while (current >= 0);
}

template void Bvh::intersect<SingleRayHitPacket, SingleRayPacket>(SingleRayHitPacket&, const SingleRayPacket&, const TraverseStackCache*) const;
template void Bvh::intersect<RayHitPacket, RayPacket>(RayHitPacket&, const RayPacket&, const TraverseStackCache*) const;


template<typename T, typename R>
T Bvh::occluded(const R& ray) const
{
    const uint32_t kStackSize = 64;
    int32_t current = 0;
    LinearBvhNode* nodes[kStackSize];
    nodes[current] = &m_nodes[0];

    bool reverseStackOrder[3];
    SoaMask masks[kStackSize];
    if constexpr (std::is_same<R, Ray>::value) {
        for (uint32_t i = 0; i < 3; i++)
            reverseStackOrder[i] = ray.dir.v[i] < 0.0f;
    } else if constexpr (std::is_same<R, SoaRay>::value) {
        masks[current].setAll(true);
        for (uint32_t i = 0; i < 3; i++)
            reverseStackOrder[i] = ray.avgDir.v[i] < 0.0f;
    }

    do {
        auto node = nodes[current];
        auto mask = masks[current];

        bool hit = node->bbox.intersect(ray.org, ray.dir, ray.invDir, ray.maxT);

        int16_t primCount = node->primCount;
        if (!hit) {
            // Do nothing
        } else if (primCount == LinearBvhNode::kInternalNode) {
            // Visit closer child first
            if (reverseStackOrder[node->splitAxis]) {
                nodes[current + 0] = node + 1;
                nodes[current + 1] = &m_nodes[node->primOrSecondNodeIndex];
            } else {
                nodes[current + 0] = &m_nodes[node->primOrSecondNodeIndex];
                nodes[current + 1] = node + 1;
            }
            if constexpr (std::is_same<R, SoaRay>::value) {
                masks[current + 0] = mask;
                masks[current + 1] = mask;
            }

            current += 2;

            PRT_ASSERT(current < kStackSize);
        } else {
            auto& m = m_mesh;

            int32_t primIndexPreMap = node->primOrSecondNodeIndex;
            for (int32_t i = 0; i < primCount; i++) {
                uint32_t primIndex = m_primRemapping[primIndexPreMap + i];

                if (m.occluded(ray, primIndex))
                    return true;
            }
        }

        current--;
    } while (current >= 0);

    return false;
}

void Bvh::createStackCache(TraverseStackCache& stackCache, const Vector3f& pos, const Vector3f& dir) const
{
    int32_t current = 0;
    LinearBvhNode* nodes[TraverseStackCache::kNodeStackCapacity + TraverseStackCache::kNodeStackCapacity/2 + 1];
    nodes[current] = &m_nodes[0];

    bool reverseStackOrder[3];
    for (uint32_t i = 0; i < 3; i++)
        reverseStackOrder[i] = dir.v[i] < 0.0f;

    int32_t nodeCount = 1;

    do {
        auto node = nodes[current];

        auto hit = node->bbox.contains(pos);

        if (!hit) {
            // Do nothing
        } else if (node->primCount == LinearBvhNode::kInternalNode) {
            nodes[current] = nullptr;
#if 0
            if (reverseStackOrder[node->splitAxis]) {
                nodes[nodeCount + 0] = node + 1;
                nodes[nodeCount + 1] = &m_nodes[node->primOrSecondNodeIndex];
            } else {
                nodes[nodeCount + 0] = &m_nodes[node->primOrSecondNodeIndex];
                nodes[nodeCount + 1] = node + 1;
            }
#else
            nodes[nodeCount + 0] = node + 1;
            nodes[nodeCount + 1] = &m_nodes[node->primOrSecondNodeIndex];
#endif
            nodeCount += 2;

            if ((nodeCount + 2) > sizeof(nodes)/sizeof(nodes[0])) {
                break;
            }
        }

        current++;
    } while (current < nodeCount);

    // Remove null node pointers
    stackCache.size = 0;
    for (uint32_t i = 0; i < nodeCount; i++) {
        if (nodes[i] != nullptr) {
            stackCache.nodes[stackCache.size] = nodes[i];
            stackCache.size++;
        }
    }

    PRT_ASSERT(stackCache.size <= TraverseStackCache::kNodeStackCapacity);
}


template bool Bvh::occluded<bool, Ray>(const Ray&) const;


} // namespace prt
