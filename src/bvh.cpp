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
#include "log.h"

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
    if (primCount <= kMaxPrimCountInNode) {
        m_primIndex = start;
        m_primCount = primCount;
        m_children[0] = nullptr;
        m_children[1] = nullptr;
        PRT_ASSERT(primCount > 0);
        context.primCountInNode[primCount - 1]++;
        return;
    }

    auto& bbox = m_bbox;

    BvhBuildNode* nodes[kChildCount];
    for (int32_t i = 0; i < kChildCount; i++) {
        nodes[i] = new BvhBuildNode;
    }
    context.nodeCount += kChildCount;

    const auto extent = bbox.upper - bbox.lower;
    const uint32_t kBucketCount = 32;

    float lowestCost = std::numeric_limits<float>::max();
    uint32_t lowestDim = 0;
    int32_t lowestCostSplit = -1;

    // Check cost for every dimension for SAH
    for (uint32_t dim = 0; dim < VectorConstants::kDimensions; dim++) {
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
    memset(&context.primCountInNode[0], 0, sizeof(context.primCountInNode));

    auto rootBuildNode = new BvhBuildNode;
    rootBuildNode->build(context, m_primRemapping, m_mesh, 0, primRemappingSize - 1);

    LinearBvhNode* nodes = new LinearBvhNode[context.nodeCount];

    logPrintf(LogLevel::kVerbose, "LinearBvhNode (count=%u, size=%.3fMiB)\n", context.nodeCount, context.nodeCount*sizeof(LinearBvhNode)/1024.0f/1024.0f);
    for (uint32_t i = 0; i < sizeof(context.primCountInNode)/sizeof(context.primCountInNode[0]); i++) {
        logPrintf(LogLevel::kVerbose, "([%u] %u) ", i + 1, context.primCountInNode[i]);
    }
    logPrintf(LogLevel::kVerbose, "\n");

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

    bool reverseStackOrder[VectorConstants::kDimensions];
    RayPacketMask masks[kStackSize];
    if constexpr (std::is_same<R, SingleRayPacket>::value) {
        for (uint32_t i = 0; i < VectorConstants::kDimensions; i++)
            reverseStackOrder[i] = packet.ray.dir.v[i] < 0.0f;
    } else if constexpr (std::is_same<R, RayPacket>::value) {
        masks[current].setAll(true);
        for (uint32_t i = 0; i < VectorConstants::kDimensions; i++)
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
#ifdef PRT_ENABLE_STATS
            hitPacket.stats.primsTraversed += primCount;
#endif
// TODO: give all primIndex to m.intersect? rather than loop?
            int32_t primIndexPreMap = node->primOrSecondNodeIndex;
            m.intersect(hitPacket, mask, packet, &m_primRemapping[primIndexPreMap], primCount);
        }

        current--;
    } while (current >= 0);
}

template void Bvh::intersect<SingleRayHitPacket, SingleRayPacket>(SingleRayHitPacket&, const SingleRayPacket&, const TraverseStackCache*) const;
template void Bvh::intersect<RayHitPacket, RayPacket>(RayHitPacket&, const RayPacket&, const TraverseStackCache*) const;


template<typename T, typename R>
T Bvh::occluded(const RayPacketMask& _mask, const R& packet) const
{
    const uint32_t kStackSize = 64;
    int32_t current = 0;
    LinearBvhNode* nodes[kStackSize];
    nodes[current] = &m_nodes[0];

    bool reverseStackOrder[3];
    RayPacketMask masks[kStackSize];

    if constexpr (std::is_same<R, SingleRayPacket>::value) {
        for (uint32_t i = 0; i < VectorConstants::kDimensions; i++)
            reverseStackOrder[i] = packet.ray.dir.v[i] < 0.0f;
    } else if constexpr (std::is_same<R, RayPacket>::value) {
        masks[current] = _mask;
        for (uint32_t i = 0; i < VectorConstants::kDimensions; i++)
            reverseStackOrder[i] = packet.avgDir.v[i] < 0.0f;
    }

    RayPacketMask occludeMask = _mask.computeNot();

    do {
        auto node = nodes[current];
        auto mask = masks[current];

        mask = mask & occludeMask.computeNot();

        bool hit;

        if constexpr (std::is_same<R, SingleRayPacket>::value) {
            hit = node->bbox.intersect(packet.ray.org, packet.ray.dir, packet.ray.invDir, packet.ray.maxT);
        } else if constexpr (std::is_same<R, RayPacket>::value) {
            // TODO: for loop packet.count
            for (uint32_t i = 0; i < RayPacket::kVectorCount; i++) {
                auto bbox_mask = node->bbox.intersect(packet.rays[i].org, packet.rays[i].dir, packet.rays[i].invDir, packet.rays[i].maxT);
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

            int32_t primIndexPreMap = node->primOrSecondNodeIndex;
            static_assert(SoaConstants::kLaneCount >= BvhBuildNode::kMaxPrimCountInNode, "Lanes must be more than max primitives in leaf node");

            if constexpr (std::is_same<R, SingleRayPacket>::value) {
                if (m.occluded<bool, SingleRayPacket>(mask, packet, &m_primRemapping[primIndexPreMap], primCount))
                    return true;
            } else if constexpr (std::is_same<R, RayPacket>::value) {
                auto omask = m.occluded<RayPacketMask, RayPacket>(mask, packet, &m_primRemapping[primIndexPreMap], primCount);

                occludeMask = occludeMask | omask;
                if (occludeMask.allTrue())
                    return occludeMask;
            }

        }

        current--;
    } while (current >= 0);

    if constexpr (std::is_same<R, SingleRayPacket>::value) {
        return false;
    } else if constexpr (std::is_same<R, RayPacket>::value) {
        return occludeMask;
    } 
}

template bool Bvh::occluded<bool, SingleRayPacket>(const RayPacketMask& mask, const SingleRayPacket&) const;
template RayPacketMask Bvh::occluded<RayPacketMask, RayPacket>(const RayPacketMask& mask, const RayPacket&) const;


void Bvh::createStackCache(TraverseStackCache& stackCache, const Vector3f& pos, const Vector3f& dir) const
{
    int32_t current = 0;
    LinearBvhNode* nodes[TraverseStackCache::kNodeStackCapacity + TraverseStackCache::kNodeStackCapacity/2 + 1];
    nodes[current] = &m_nodes[0];

    bool reverseStackOrder[VectorConstants::kDimensions];
    for (uint32_t i = 0; i < VectorConstants::kDimensions; i++)
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


} // namespace prt
