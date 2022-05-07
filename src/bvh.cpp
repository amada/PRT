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
#include "thread_pool.h"
#include "log.h"

#include "bvh.h"

namespace prt
{

void BvhBuildNode::calculateBounds(const uint32_t* primRemapping, const Mesh& mesh, int32_t start, int32_t end)
{
    for (int32_t i = start; i <= end; i++) {
        const int32_t indexBase = Mesh::kVertexCountPerPrim*primRemapping[i];
        for (int32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
            m_bbox.merge(mesh.getPosition(mesh.getIndex(indexBase + j)));
        }
    }
}

void BvhBuildNode::build(BuildContext& context, int32_t start, int32_t end, int32_t level)
{
    auto& mesh = *context.mesh;
    auto primRemapping = context.primRemapping;

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
                primBounds.merge(v);
            }
            temp = 1.0f/Mesh::kVertexCountPerPrim*temp;

            int32_t b = kBucketCount*(temp.v[dim] - lowerPos)/splitExtent;
            if (b >= kBucketCount) b = kBucketCount - 1;
            if (b < 0) { b = 0; }

            buckets[b].count++;
            buckets[b].bounds.merge(primBounds);
        }

        // Find the lowest cost split
        for (uint32_t i = 0; i < kBucketCount - 1; i++) {
            uint32_t countLeft = 0;
            BBox boundsLeft = BBox::init();
            for (uint32_t j = 0; j <= i; j++) {
                countLeft += buckets[j].count;
                boundsLeft.merge(buckets[j].bounds);
            }

            uint32_t countRight = 0;
            BBox boundsRight = BBox::init();
            for (uint32_t j = i + 1; j < kBucketCount; j++) {
                countRight += buckets[j].count;
                boundsRight.merge(buckets[j].bounds);
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
    if (mid <= start || end <= mid) {
        mid = (start + end)/2;
    }

    m_splitAxis = dim;

    if (context.threadPool && level == 2) {
        context.threadPool->queue([nodes, &context, start, mid, level]() {
            nodes[0]->build(context, start, mid, level + 1);
        });
        context.threadPool->queue([nodes, &context, mid, end, level]() {
            nodes[1]->build(context, mid + 1, end, level + 1);
        });
    } else {
        nodes[0]->build(context, start, mid, level + 1);
        nodes[1]->build(context, mid + 1, end, level + 1);
    }

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
    context.threadPool = new ThreadPool;
    context.threadPool->create(-1);
    context.nodeCount = 1;
    memset(&context.primCountInNode[0], 0, sizeof(context.primCountInNode));

    auto rootBuildNode = new BvhBuildNode;
    context.mesh = &m_mesh;
    context.primRemapping = m_primRemapping;

    auto start = std::chrono::steady_clock::now();
    rootBuildNode->build(context, 0, primRemappingSize - 1, 0);


    if (context.threadPool) {
        context.threadPool->waitAllTasksDone();
        delete context.threadPool;
        context.threadPool = nullptr;
    }

    LinearBvhNode* nodes = new LinearBvhNode[context.nodeCount];

#ifdef TRI_VECTOR
    uint32_t primNodeCount = 0;
#endif
    logPrintf(LogLevel::kVerbose, "LinearBvhNode (count=%u, size=%.3fMiB)\n", context.nodeCount.load(), context.nodeCount*sizeof(LinearBvhNode)/1024.0f/1024.0f);
    for (uint32_t i = 0; i < sizeof(context.primCountInNode)/sizeof(context.primCountInNode[0]); i++) {
        logPrintf(LogLevel::kVerbose, "([%u] %u) ", i + 1, context.primCountInNode[i].load());
#ifdef TRI_VECTOR
        primNodeCount += context.primCountInNode[i];
#endif
    }
    logPrintf(LogLevel::kVerbose, "\n");

#ifdef TRI_VECTOR
    m_triangleVectors = new TriangleVector[primNodeCount];
#endif
    int32_t triVectorIndex = 0;
    int32_t index = 0;
    buildLinearBvhNodes(nodes, &index, rootBuildNode, &triVectorIndex);

#ifdef TRI_VECTOR
    logPrintf(LogLevel::kVerbose, "TriangleVector %d\n", triVectorIndex);
#endif
    auto end = std::chrono::steady_clock::now();

    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    logPrintf(LogLevel::kVerbose, "BVH build time %dms\n", elapsedMs);

    m_nodes = nodes;
}

void Bvh::buildLinearBvhNodes(LinearBvhNode* nodes, int32_t* index, BvhBuildNode* node, int32_t* triVectorIndex)
{
    LinearBvhNode& l = nodes[*index];
    (*index)++;

    l.bbox = node->m_bbox;
    l.splitAxis = node->m_splitAxis;

    if (node->m_primIndex == BvhBuildNode::kInteriorNode) {
        l.primCount = LinearBvhNode::kInternalNode;
        buildLinearBvhNodes(nodes, index, node->m_children[0], triVectorIndex);

        l.primOrSecondNodeIndex = *index;
        buildLinearBvhNodes(nodes, index, node->m_children[1], triVectorIndex);
    } else {
        auto primRemapIndex = node->m_primIndex;
        auto primCount = node->m_primCount;

        l.primOrSecondNodeIndex = primRemapIndex;
        l.primCount = primCount;
#ifdef TRI_VECTOR
        auto& m = m_mesh;

        Vector3f p[Mesh::kVertexCountPerPrim][TriangleVector::kSize];
        Vector2f uv[Mesh::kVertexCountPerPrim][TriangleVector::kSize];

        int32_t alphaTest[TriangleVector::kSize];

        for (uint32_t i = 0; i < primCount; i++) {
            uint32_t indexBase = m_primRemapping[primRemapIndex + i]*Mesh::kVertexCountPerPrim;

            auto mat = m.getMaterial(m.getPrimToMaterial(m_primRemapping[primRemapIndex + i]));
            alphaTest[i] = mat.alphaTest;

            for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
                uint32_t v = m.getIndex(indexBase + j);
                p[j][i] = m.getPosition(v);
                //v = m.getTexcoordIndex(indexBase + j);
                uv[j][i] = m.getTexcoord(v);
            }
        }

        for (uint32_t i = primCount; i < TriangleVector::kSize; i++) {
            for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
                p[j][i] = Vector3f(0.0f);
                uv[j][i] = Vector2f(0.0f);
            }
            alphaTest[i] = 0;
        }

        auto& triVector = m_triangleVectors[*triVectorIndex];
        for (uint32_t i = 0; i < TriangleVector::kVectorCount; i++) {
            auto lo = i*SoaConstants::kLaneCount;
            triVector.p0[i] = SoaVector3f(&p[0][lo]);
            triVector.p1[i] = SoaVector3f(&p[1][lo]);
            triVector.p2[i] = SoaVector3f(&p[2][lo]);
            triVector.uv0[i] = SoaVector2f(&uv[0][lo]);
            triVector.uv1[i] = SoaVector2f(&uv[1][lo]);
            triVector.uv2[i] = SoaVector2f(&uv[2][lo]);
        }

        memcpy(&triVector.alphaTest[0], alphaTest, sizeof(alphaTest));

        l.triVectorIndex = *triVectorIndex;
        (*triVectorIndex)++;
#endif
    }

    delete node;
}

enum IntersectMode {kNearest, kOcclude};
template<IntersectMode mode>
inline bool intersectSingleRay(const TriangleVector& tri, SingleRayHitPacket* _hitPacket, const SingleRayPacket& packet, const Mesh& m, const uint32_t* primIndices, uint32_t primCount, float epsilon, float maxT)
{
    int32_t maskBits = (1 << primCount) - 1;

    float nearestT = std::numeric_limits<float>::max();
    int32_t nearestVector = -1;
    int32_t nearestLane = -1;
    SoaTriangleIntersection intr[TriangleVector::kVectorCount];

    for (uint32_t v = 0; v < TriangleVector::kVectorCount; v++, maskBits >>= SoaConstants::kLaneCount) {
        auto& p0 = tri.p0[v];
        auto& p1 = tri.p1[v];
        auto& p2 = tri.p2[v];

        SoaMask mask(maskBits);
        if (maskBits == 0)
            break;

        intr[v] = intersectTriangle(mask, packet.ray.org, packet.ray.dir, packet.ray.swapXZ, packet.ray.swapYZ, p0, p1, p2);

        mask = intr[v].t.greaterThanOrEqual(epsilon) & intr[v].t.lessThan(maxT);


        for (auto bits = mask.ballot(); bits; bits &= bits -1) {
            auto index = bitScanForward(bits);

            if (tri.alphaTest[index]) {
                auto primIndex = primIndices[index];
                auto& mat = m.getMaterial(m.getPrimToMaterial(primIndex));

                auto uv0 = tri.uv0[v].getLane(index);
                auto uv1 = tri.uv1[v].getLane(index);
                auto uv2 = tri.uv2[v].getLane(index);

                auto uv = intr[v].i.getLane(index)*uv0 + intr[v].j.getLane(index)*uv1 + intr[v].k.getLane(index)*uv2;

                if (!mat.testAlpha(uv))
                    continue;
            }

            if constexpr (mode == kNearest) {
                auto t = intr[v].t.getLane(index);
                if (nearestT > t) {
                    nearestT = t;
                    nearestLane = index;
                    nearestVector = v;
                }
            } else if constexpr (mode == kOcclude) {
                return true;
            }
        }
    }

    if constexpr (mode == kNearest) {
        if (nearestLane >= 0) {
            auto& hitPacket = *_hitPacket;
            hitPacket.hit.t = nearestT;
            hitPacket.hit.i = intr[nearestVector].i.getLane(nearestLane);
            hitPacket.hit.j = intr[nearestVector].j.getLane(nearestLane);
            hitPacket.hit.k = intr[nearestVector].k.getLane(nearestLane);
            hitPacket.hit.primId = primIndices[nearestVector*SoaConstants::kLaneCount + nearestLane];
            hitPacket.hit.meshId = m.getId();
        }
    }

    return false;
}

template<IntersectMode mode>
inline RayPacketMask intersectRayPacket(const TriangleVector& tri, const RayPacketMask& mask, RayHitPacket* _hitPacket, const RayPacket& packet, const Mesh& m, const uint32_t* primIndices, uint32_t primCount, float epsilon)
{
    auto& hitPacket = *_hitPacket;
    RayPacketMask res(0);

    for (uint32_t i = 0; i < primCount; i++) {
        auto tv = i/SoaConstants::kLaneCount;
        auto l = i - tv*SoaConstants::kLaneCount;
        auto p0 = tri.p0[tv].broadcast(l);
        auto p1 = tri.p1[tv].broadcast(l);
        auto p2 = tri.p2[tv].broadcast(l);

        for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
//            if (!mask.masks[v].anyTrue()) continue;

            auto& hit = hitPacket.hits[v];
            auto& ray = packet.rays[v];
            auto intr = intersectTriangle(mask.masks[v], ray.org, ray.dir, ray.swapXZ, ray.swapYZ, p0, p1, p2);

            SoaMask maskHit;
            if constexpr (mode == kNearest) {
                maskHit = intr.t.greaterThanOrEqual(epsilon) & intr.t.lessThan(hit.t);
            } else if constexpr (mode == kOcclude) {
                maskHit = intr.t.greaterThanOrEqual(epsilon) & intr.t.lessThan(ray.maxT);
            }

            if (maskHit.anyTrue()) {
                auto primIndex = primIndices[i];

                if (tri.alphaTest[i]) {
                    auto& mat = m.getMaterial(m.getPrimToMaterial(primIndex));

                    auto uv0 = tri.uv0[tv].getLane(l); // Should store AOS in TriangleVector or do SIMD computation?
                    auto uv1 = tri.uv1[tv].getLane(l);
                    auto uv2 = tri.uv2[tv].getLane(l);

                    auto uv = intr.i*uv0 + intr.j*uv1 + intr.k*uv2;

                    maskHit = mat.testAlpha(maskHit, uv);
                }

                if constexpr (mode == kNearest) {
                    hit.t = select(hit.t, intr.t, maskHit);
                    hit.i = select(hit.i, intr.i, maskHit);
                    hit.j = select(hit.j, intr.j, maskHit);
                    hit.k = select(hit.k, intr.k, maskHit);
                    hit.primId = select(hit.primId, primIndex, maskHit);
                    hit.meshId = select(hit.meshId, m.getId(), maskHit);
                } else if constexpr (mode == kOcclude) {
                    res.computeOrSelf(v, maskHit);
                }
            }
        }
    }

    return res;
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

#define SORT_CHILDREN
#ifdef SORT_CHILDREN
    if constexpr (std::is_same<R, SingleRayPacket>::value) {
        if (!m_nodes[0].bbox.intersect(packet.ray.org, packet.ray.dir, packet.ray.invDir, hitPacket.hit.t))
            return;
    }
#endif

    do {
#ifdef PRT_ENABLE_STATS
        hitPacket.stats.nodesTraversed++;
#endif
        auto node = nodes[current];
        auto mask = masks[current];

        bool hit;

        if constexpr (std::is_same<R, SingleRayPacket>::value) {
#ifdef SORT_CHILDREN
            hit = true;
#else
            hit = node->bbox.intersect(packet.ray.org, packet.ray.dir, packet.ray.invDir, hitPacket.hit.t);
#endif
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
#ifdef SORT_CHILDREN
            // Visit closer child first
            if constexpr (std::is_same<R, SingleRayPacket>::value) {
                auto node0 = node + 1;
                auto node1 = &m_nodes[node->primOrSecondNodeIndex];

                auto t0 = node0->bbox.intersect(packet.ray.org, packet.ray.dir, packet.ray.invDir);
                auto t1 = node1->bbox.intersect(packet.ray.org, packet.ray.dir, packet.ray.invDir);

                auto hit0 = t0 < hitPacket.hit.t;
                auto hit1 = t1 < hitPacket.hit.t;

                if (!hit0) {
                    if (hit1) {
                        nodes[current] = node1;
                        current++;
                    }
                } else if (!hit1) {
                    nodes[current] = node0;
                    current++;
                } else if (t0 < t1) {
                    nodes[current + 0] = node1;
                    nodes[current + 1] = node0;
                    current += 2;
                } else {
                    nodes[current + 0] = node0;
                    nodes[current + 1] = node1;
                    current += 2;
                }
            } else if constexpr (std::is_same<R, RayPacket>::value) {
                if (reverseStackOrder[node->splitAxis]) {
                    nodes[current + 0] = node + 1;
                    nodes[current + 1] = &m_nodes[node->primOrSecondNodeIndex];
                } else {
                    nodes[current + 0] = &m_nodes[node->primOrSecondNodeIndex];
                    nodes[current + 1] = node + 1;
                }

                masks[current + 0] = mask;
                masks[current + 1] = mask;

                current += 2;
            }
#else
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
#endif

            PRT_ASSERT(current < kStackSize);
        } else {
            auto& m = m_mesh;
#ifdef PRT_ENABLE_STATS
            hitPacket.stats.primsTraversed += primCount;
#endif
            int32_t primIndexPreMap = node->primOrSecondNodeIndex;

            auto& tri = m_triangleVectors[node->triVectorIndex];
            if constexpr (std::is_same<R, SingleRayPacket>::value) {
                intersectSingleRay<kNearest>(tri, &hitPacket, packet, m, &m_primRemapping[primIndexPreMap], primCount, kEpsilon, hitPacket.hit.t);
            } else if constexpr (std::is_same<R, RayPacket>::value) {
                intersectRayPacket<kNearest>(tri, mask, &hitPacket, packet, m, &m_primRemapping[primIndexPreMap], primCount, kEpsilon);
            }
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

    RayPacketMask masks[kStackSize];

    if constexpr (std::is_same<R, RayPacket>::value) {
        masks[current] = _mask;
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
            nodes[current + 0] = &m_nodes[node->primOrSecondNodeIndex];
            nodes[current + 1] = node + 1;

            if constexpr (std::is_same<R, RayPacket>::value) {
                masks[current + 0] = mask;
                masks[current + 1] = mask;
            }

            current += 2;

            PRT_ASSERT(current < kStackSize);
        } else {
            auto& m = m_mesh;

            int32_t primIndexPreMap = node->primOrSecondNodeIndex;

            auto& tri = m_triangleVectors[node->triVectorIndex];
            if constexpr (std::is_same<R, SingleRayPacket>::value) {
                if (intersectSingleRay<kOcclude>(tri, nullptr, packet, m, &m_primRemapping[primIndexPreMap], primCount, kEpsilon, packet.ray.maxT))
                    return true;
            } else if constexpr (std::is_same<R, RayPacket>::value) {
                auto res = intersectRayPacket<kOcclude>(tri, mask, nullptr, packet, m, &m_primRemapping[primIndexPreMap], primCount, kEpsilon);

                occludeMask = occludeMask | res;
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
