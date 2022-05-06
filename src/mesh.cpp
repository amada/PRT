#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "../ext/tinyobjloader/tiny_obj_loader.h"
#include "log.h"
#include "vecmath.h"
#include "triangle.h"
#include "bvh.h"
#include "ray.h"
#include "thread_pool.h"

#include "mesh.h"


#define DELETE_ARRAY(x) { delete[] x; x = nullptr; }

namespace prt
{

Mesh::Mesh(Mesh&& m)
{
    // TODO is this needed?
    __builtin_trap();
    m_indices = m.m_indices;
    m_positions = m.m_positions;
    m_normals = m.m_normals;
    m_primMaterial = m.m_primMaterial;
    m_materials = m.m_materials;
    m_indexCount = m.m_indexCount;
    m_vertexCount = m.m_vertexCount;
    m_materialCount = m.m_materialCount;
    m_hasVertexNormal = m.m_hasVertexNormal;
    m_hasTexcoord = m.m_hasTexcoord;
    m_bbox = m.m_bbox;

    m.m_indices = nullptr;
    m.m_positions = nullptr;
    m.m_normals = nullptr;
    m.m_primMaterial = nullptr;
    m.m_materials = nullptr;
    m.m_indexCount = 0;
    m.m_vertexCount = 0;
    m.m_materialCount = 0;
}

Mesh& Mesh::operator=(Mesh&& m)
{
    m_indices = m.m_indices;
    m_texcoordIndices = m.m_texcoordIndices;
    m_positions = m.m_positions;
    m_texcoords = m.m_texcoords;
    m_normals = m.m_normals;
    m_primMaterial = m.m_primMaterial;
    m_materials = m.m_materials;
    m_indexCount = m.m_indexCount;
    m_vertexCount = m.m_vertexCount;
    m_materialCount = m.m_materialCount;
    m_texcoordsCount = m.m_texcoordsCount;
    m_hasVertexNormal = m.m_hasVertexNormal;
    m_hasTexcoord = m.m_hasTexcoord;
    m_bbox = m.m_bbox;

    m.m_indices = nullptr;
    m.m_positions = nullptr;
    m.m_normals = nullptr;
    m.m_primMaterial = nullptr;
    m.m_materials = nullptr;
    m.m_indexCount = 0;
    m.m_vertexCount = 0;
    m.m_texcoordsCount = 0;
    m.m_materialCount = 0;

    return m;
}

Mesh::~Mesh()
{
    DELETE_ARRAY(m_indices);
    DELETE_ARRAY(m_positions);
    DELETE_ARRAY(m_normals);
    DELETE_ARRAY(m_primMaterial);
    DELETE_ARRAY(m_materials);
}

void Mesh::create(uint32_t primCount, uint32_t vertexCount, uint32_t materialCount, bool hasVertexNormal)
{
    uint32_t indexCount = primCount*kVertexCountPerPrim;
    m_indices = new uint32_t[indexCount];
    m_texcoordIndices = new uint32_t[indexCount]; // TODO fixme
    m_positions = new Vector3f[vertexCount];
    m_texcoords = new Vector2f[vertexCount]; // TODO fixme
    if (hasVertexNormal) {
        m_normals = new Vector3f[vertexCount];
    }
    m_primMaterial = new uint32_t[primCount];
    m_materials = new Material[materialCount];

    m_indexCount = indexCount;
    m_vertexCount = vertexCount;
    m_materialCount = materialCount;
    m_hasVertexNormal = hasVertexNormal;
}


void Mesh::calculateVertexNormals()
{
    auto& normals = m_normals;

    delete[] normals;

    normals = new Vector3f[m_vertexCount];
    for (uint32_t i = 0; i < m_vertexCount; i++) {
        m_normals[i].set(0.0f, 0.0f, 0.0f);
    }

    // TODO: The order of processing matters because this loop doens't allow accumulated normals to be zero even temporarily
    for (int32_t i = m_indexCount/Mesh::kVertexCountPerPrim - 1; i >= 0; i--) {
//    for (uint32_t i = 0; i < m_indexCount/Mesh::kVertexCountPerPrim; i++) {
        uint32_t indexBase = i*Mesh::kVertexCountPerPrim;
        uint32_t v[Mesh::kVertexCountPerPrim];
        Vector3f p[Mesh::kVertexCountPerPrim];
        for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
            v[j] = m_indices[indexBase + j];
            p[j] = m_positions[v[j]];
        }

        auto normal = normalize(cross(p[1] - p[0], p[2] - p[0]));
        if (std::isnan(normal.x) || std::isnan(normal.y)|| std::isnan(normal.z)) {
            normal.set(0.0f, 0.0f, 0.0f);
        }

        for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
//            normals[v[j]] = normals[v[j]] + normal;
            auto n = normals[v[j]] + normal;
            if (length(n) > 0.0f) {
                normals[v[j]] = n;
            }
        }
    }

    for (uint32_t i = 0; i < m_vertexCount; i++) {
        normals[i] = normalize(normals[i]);
    }

    m_hasVertexNormal = true;
}

void Mesh::loadObj(const char* path, const Material& mat)
{
    tinyobj::ObjReader objReader;
    auto res = objReader.ParseFromFile(path);

    if (!res) {
        logPrintf(LogLevel::kError, "Failed to load %s\n", path);
        return;
    }

    auto& attr = objReader.GetAttrib();

    uint32_t vertexCount = attr.vertices.size()/3;
    std::vector<Vector2f> tex; tex.resize(vertexCount);

    std::vector<uint32_t> indices;
    std::vector<uint32_t> texcoordIndices;

    for (auto s: objReader.GetShapes()) {
        // TODO: not taking care of normal and texture coords
        auto& srcIndices = s.mesh.indices;
        indices.reserve(indices.size() + srcIndices.size());
        for (auto i: srcIndices) {
            indices.push_back(i.vertex_index);
            texcoordIndices.push_back(i.vertex_index);
            uint32_t b;
            b = 2*i.texcoord_index;
            tex[i.vertex_index] = Vector2f(attr.texcoords[b], attr.texcoords[b + 1]);        
        }
    }

    auto& srcVertices = attr.vertices;
    auto& srcTexcoords = attr.texcoords;

    uint32_t primCount = indices.size()/Mesh::kVertexCountPerPrim;
    const uint32_t kFloatCountPerVertex = 3;
    create(primCount, srcVertices.size()/kFloatCountPerVertex, 1, false);

    const uint32_t kFloatCountPerTexcoord = 2;

    memcpy(getIndexBuffer(), indices.data(), indices.size()*sizeof(uint32_t));
    memcpy(getPositionBuffer(), srcVertices.data(), srcVertices.size()*sizeof(float));

    m_texcoordsCount = srcTexcoords.size()/kFloatCountPerTexcoord;
    memcpy(m_texcoordIndices, texcoordIndices.data(), indices.size()*sizeof(uint32_t));
    memcpy(m_texcoords, srcTexcoords.data(), vertexCount*sizeof(float));

    memset(getPrimMateialBuffer(), 0, primCount*sizeof(uint32_t));
    memcpy(getMaterialBuffer(), &mat, sizeof(Material));

    calculateBounds();

    m_hasTexcoord = true;

    logPrintf(LogLevel::kVerbose, "Finished loading '%s'\n", path);
}

void Mesh::loadObj(const char* path)
{
    tinyobj::ObjReader objReader;
    auto res = objReader.ParseFromFile(path);

    if (!res) {
        logPrintf(LogLevel::kError, "Failed to load %s\n", path);
        return;
    }

    std::filesystem::path objPath(path);
    std::vector<Material> materials;
    materials.resize(objReader.GetMaterials().size());
    uint32_t mi = 0;

    ThreadPool threadPool;
    threadPool.create(-1);

    for (auto srcMat: objReader.GetMaterials()) {
        auto& m = materials[mi];

        threadPool.queue([&m, objPath, srcMat]() {
#ifdef _WIN32
            std::wstring parentPath = objPath.parent_path();
            m.load(srcMat, std::string(parentPath.begin(), parentPath.end()));
#else
            m.load(srcMat, objPath.parent_path());
#endif
        });
        mi++;
    }
    threadPool.waitAllTasksDone();

    std::vector<uint32_t> primMat;
    std::vector<uint32_t> indices;
    std::vector<uint32_t> texcoordIndices;

    auto& attr = objReader.GetAttrib();

    uint32_t vertexCount = attr.vertices.size()/3;
    std::vector<Vector2f> tex; tex.resize(vertexCount);

    for (auto s: objReader.GetShapes()) {
        // TODO: not taking care of normal and texture coords
        auto& srcIndices = s.mesh.indices;
        indices.reserve(indices.size() + srcIndices.size());
        // TODO can be insert
        for (auto i: srcIndices) {
            indices.push_back(i.vertex_index);
//            texcoordIndices.push_back(i.texcoord_index);
            texcoordIndices.push_back(i.vertex_index);
            uint32_t b;
            b = 2*i.texcoord_index;
            tex[i.vertex_index] = Vector2f(attr.texcoords[b], 1.0f - attr.texcoords[b + 1]);
        }

        primMat.insert(primMat.end(), s.mesh.material_ids.begin(), s.mesh.material_ids.end());
    }

    auto& srcVertices = attr.vertices;
    auto& srcTexcoords = attr.texcoords;

    uint32_t primCount = indices.size()/Mesh::kVertexCountPerPrim;
    const uint32_t kFloatCountPerVertex = 3;
    create(primCount, srcVertices.size()/kFloatCountPerVertex, materials.size(), false);

    const uint32_t kFloatCountPerTexcoord = 2;

    memcpy(getIndexBuffer(), indices.data(), indices.size()*sizeof(uint32_t));
    memcpy(getPositionBuffer(), srcVertices.data(), srcVertices.size()*sizeof(float));

    m_texcoordsCount = srcTexcoords.size()/kFloatCountPerTexcoord;
    memcpy(m_texcoordIndices, texcoordIndices.data(), indices.size()*sizeof(uint32_t));
//    memcpy(m_texcoords, srcTexcoords.data(), srcTexcoords.size()*sizeof(Vector2f));
    memcpy(m_texcoords, tex.data(), tex.size()*sizeof(Vector2f));

    memcpy(getPrimMateialBuffer(), primMat.data(), primCount*sizeof(uint32_t));
    memcpy(getMaterialBuffer(), materials.data(), materials.size()*sizeof(Material));

    calculateBounds();

    m_hasTexcoord = true;

    logPrintf(LogLevel::kVerbose, "Finished loading '%s' prim=%u, vtx=%u, tex=%u\n", path, primCount, srcVertices.size()/3, srcTexcoords.size()/2);
}

void Mesh::calculateBounds()
{
    BBox bbox = BBox::init();

    for (uint32_t i = 0; i < m_vertexCount; i++) {
        bbox.merge(m_positions[i]);
    }

    m_bbox = bbox;
}


template<typename T, typename R>
void Mesh::intersect(T& hitPacket, const RayPacketMask& mask, const R& packet, const uint32_t* primIndices, uint32_t primCount) const
{
    static_assert(kEpsilon > TriangleIntersection::kNoIntersection, "kEpsilon must be larger than TriangleIntersectionT::kNoIntersection to reject no intersection");

    if constexpr (std::is_same<R, SingleRayPacket>::value) {
#if 1
        Vector3f p[Mesh::kVertexCountPerPrim][SoaConstants::kLaneCount];

        // TODO: memset p with 0?

        for (uint32_t i = 0; i < primCount; i++) {
            uint32_t indexBase = primIndices[i]*Mesh::kVertexCountPerPrim;

            for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
                uint32_t v = getIndex(indexBase + j);
                p[j][i] = getPosition(v);
            }
        }
        SoaVector3f p0(&p[0][0]);
        SoaVector3f p1(&p[1][0]);
        SoaVector3f p2(&p[2][0]);

        SoaMask mask((1 << primCount) - 1);

        auto intr = intersectTriangle(mask, packet.ray.org, packet.ray.dir, packet.ray.swapXZ, packet.ray.swapYZ, p0, p1, p2);

        mask = intr.t.greaterThanOrEqual(kEpsilon) & intr.t.lessThan(hitPacket.hit.t);

        float nearestT = std::numeric_limits<float>::max();
        int32_t nearestIndex = -1;

        for (auto bits = mask.ballot(); bits; bits &= bits -1) {
            auto index = bitScanForward(bits);
            auto primIndex = primIndices[index];
            auto& mat = getMaterial(getPrimToMaterial(primIndex));
            if (mat.alphaTest) {
                uint32_t indexBase = primIndex*Mesh::kVertexCountPerPrim;

                auto v0 = getTexcoordIndex(indexBase + 0);
                auto v1 = getTexcoordIndex(indexBase + 1);
                auto v2 = getTexcoordIndex(indexBase + 2);

                const auto& t0 = getTexcoord(v0);
                const auto& t1 = getTexcoord(v1);
                const auto& t2 = getTexcoord(v2);
                auto uv = intr.i.getLane(index)*t0 + intr.j.getLane(index)*t1 + intr.k.getLane(index)*t2;

                if (!mat.testAlpha(uv))
                    continue;
            }

            auto t = intr.t.getLane(index);
            if (nearestT > t) {
                nearestT = t;
                nearestIndex = index;
            }
        }

        if (nearestIndex >= 0) {
            hitPacket.hit.t = nearestT;
            hitPacket.hit.i = intr.i.getLane(nearestIndex);
            hitPacket.hit.j = intr.j.getLane(nearestIndex);
            hitPacket.hit.k = intr.k.getLane(nearestIndex);
            hitPacket.hit.primId = primIndices[nearestIndex];
            hitPacket.hit.meshId = m_id;
        }
#else
        for (uint32_t i = 0; i < primCount; i++) {
            auto primIndex = primIndices[i];
            uint32_t indexBase = primIndex*Mesh::kVertexCountPerPrim;

            // TODO skip two-level indircection
            uint32_t v0 = getIndex(indexBase + 0);
            uint32_t v1 = getIndex(indexBase + 1);
            uint32_t v2 = getIndex(indexBase + 2);
            const Vector3f& p0 = getPosition(v0);
            const Vector3f& p1 = getPosition(v1);
            const Vector3f& p2 = getPosition(v2);

            auto intr = intersectTriangle(packet.ray.org, packet.ray.dir, p0, p1, p2);

            if (intr.t >= kEpsilon && intr.t < hitPacket.hit.t) {
                auto& mat = getMaterial(getPrimToMaterial(primIndex));

                if (mat.alphaTest) {
    // access to texcoord index can be removed??
                    v0 = getTexcoordIndex(indexBase + 0);
                    v1 = getTexcoordIndex(indexBase + 1);
                    v2 = getTexcoordIndex(indexBase + 2);

                    auto& t0 = getTexcoord(v0);
                    auto& t1 = getTexcoord(v1);
                    auto& t2 = getTexcoord(v2);
                    auto uv = intr.i*t0 + intr.j*t1 + intr.k*t2;

                    if (!mat.testAlpha(uv))
                        continue;
                }

                hitPacket.hit.t = intr.t;
                hitPacket.hit.i = intr.i;
                hitPacket.hit.j = intr.j;
                hitPacket.hit.k = intr.k;
                hitPacket.hit.primId = primIndex;
                hitPacket.hit.meshId = m_id;
            }
        }
#endif
    } else if constexpr (std::is_same<R, RayPacket>::value) {
        for (uint32_t i = 0; i < primCount; i++) {
            auto primIndex = primIndices[i];
            uint32_t indexBase = primIndex*Mesh::kVertexCountPerPrim;
            // TODO skip two-level indircection
            uint32_t v0 = getIndex(indexBase + 0);
            uint32_t v1 = getIndex(indexBase + 1);
            uint32_t v2 = getIndex(indexBase + 2);
            const Vector3f& p0 = getPosition(v0);
            const Vector3f& p1 = getPosition(v1);
            const Vector3f& p2 = getPosition(v2);
            for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
                // TODO
    //            if (!mask.masks[i].anyTrue()) continue;

                auto& hit = hitPacket.hits[v];
                auto& ray = packet.rays[v];
                auto intr = intersectTriangle(mask.masks[v], ray.org, ray.dir, ray.swapXZ, ray.swapYZ, p0, p1, p2);

                auto maskHit = intr.t.greaterThanOrEqual(kEpsilon) & intr.t.lessThan(hit.t);

// ?? because mask is already given to intersectTriangle
//                maskHit = maskHit & mask.masks[v];

                if (maskHit.anyTrue()) {
                    auto& mat = getMaterial(getPrimToMaterial(primIndex));

                    if (mat.alphaTest) {
        // access to texcoord index can be removed??
                        v0 = getTexcoordIndex(indexBase + 0);
                        v1 = getTexcoordIndex(indexBase + 1);
                        v2 = getTexcoordIndex(indexBase + 2);

                        auto& t0 = getTexcoord(v0);
                        auto& t1 = getTexcoord(v1);
                        auto& t2 = getTexcoord(v2);
                        auto uv = intr.i*t0 + intr.j*t1 + intr.k*t2;

                        maskHit = mat.testAlpha(maskHit, uv);
                    }

                    hit.t = select(hit.t, intr.t, maskHit);
                    hit.i = select(hit.i, intr.i, maskHit);
                    hit.j = select(hit.j, intr.j, maskHit);
                    hit.k = select(hit.k, intr.k, maskHit);
                    hit.primId = select(hit.primId, primIndex, maskHit);
                    hit.meshId = select(hit.meshId, m_id, maskHit);
                }
            }
        }
    }
}

template void Mesh::intersect<SingleRayHitPacket, SingleRayPacket>(SingleRayHitPacket& hitPacket, const RayPacketMask& mask, const SingleRayPacket& packet, const uint32_t* primIndices, uint32_t primCount) const;
template void Mesh::intersect<RayHitPacket, RayPacket>(RayHitPacket& hitPacket, const RayPacketMask& mask, const RayPacket& packet, const uint32_t* primIndices, uint32_t primCount) const;

template<typename T, typename R>
T Mesh::occluded(const RayPacketMask& mask, const R& packet, const uint32_t* primIndices, uint32_t primCount) const
{
#if 1
    if constexpr (std::is_same<R, SingleRayPacket>::value) {
        Vector3f p[Mesh::kVertexCountPerPrim][SoaConstants::kLaneCount];

        // TODO: memset p with 0?

        for (uint32_t i = 0; i < primCount; i++) {
            uint32_t indexBase = primIndices[i]*Mesh::kVertexCountPerPrim;

            for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
                uint32_t v = getIndex(indexBase + j);
                p[j][i] = getPosition(v);
            }
        }

        SoaVector3f p0(&p[0][0]);
        SoaVector3f p1(&p[1][0]);
        SoaVector3f p2(&p[2][0]);

        SoaMask mask((1 << primCount) - 1);

        // TODO use intersectTriangle dedicated for occluded
        auto intr = intersectTriangle(mask, packet.ray.org, packet.ray.dir, packet.ray.swapXZ, packet.ray.swapYZ, p0, p1, p2);

        mask = intr.t.greaterThanOrEqual(kEpsilon) & intr.t.lessThan(packet.ray.maxT);

        for (auto bits = mask.ballot(); bits; bits &= bits - 1) {
            auto index = bitScanForward(bits);
            auto& mat = getMaterial(getPrimToMaterial(primIndices[index]));
            if (mat.alphaTest) {
                uint32_t indexBase = primIndices[index]*Mesh::kVertexCountPerPrim;

                auto v0 = getTexcoordIndex(indexBase + 0);
                auto v1 = getTexcoordIndex(indexBase + 1);
                auto v2 = getTexcoordIndex(indexBase + 2);

                const auto& t0 = getTexcoord(v0);
                const auto& t1 = getTexcoord(v1);
                const auto& t2 = getTexcoord(v2);
                auto uv = intr.i.getLane(index)*t0 + intr.j.getLane(index)*t1 + intr.k.getLane(index)*t2;

                if (!mat.testAlpha(uv))
                    continue;
            }
            return true;
        }

        return false;
#else
    for (uint32_t i = 0; i < primCount; i++) {
        auto primIndex = primIndices[i];
        uint32_t indexBase = primIndex*Mesh::kVertexCountPerPrim;

        // TODO skip two-level indircection
        uint32_t v0 = getIndex(indexBase + 0);
        uint32_t v1 = getIndex(indexBase + 1);
        uint32_t v2 = getIndex(indexBase + 2);
        const Vector3f &p0 = getPosition(v0);
        const Vector3f &p1 = getPosition(v1);
        const Vector3f &p2 = getPosition(v2);

        static_assert(kEpsilon > TriangleIntersection::kNoIntersection, "kEpsilon must be larger than TriangleIntersectionT::kNoIntersection to reject no intersection");

        if constexpr (std::is_same<R, Ray>::value) {
            // TODO use intersectTriangle dedicated for occluded
            auto triIntr = intersectTriangle(ray.org, ray.dir, p0, p1, p2);

            auto& mat = getMaterial(getPrimToMaterial(primIndex));

            if (triIntr.t >= kEpsilon && triIntr.t < ray.maxT) {
                if (mat.alphaTest) {
                    v0 = getTexcoordIndex(indexBase + 0);
                    v1 = getTexcoordIndex(indexBase + 1);
                    v2 = getTexcoordIndex(indexBase + 2);

                    const auto& t0 = getTexcoord(v0);
                    const auto& t1 = getTexcoord(v1);
                    const auto& t2 = getTexcoord(v2);
                    auto uv = triIntr.i*t0 + triIntr.j*t1 + triIntr.k*t2;

                    if (mat.testAlpha(uv)) {
                        return true;
                    }
                } else {
                    return true;
                }
            }
        }
    }

    return false;
#endif
    } else if constexpr (std::is_same<R, RayPacket>::value) {
        // Mask is true if occluded; opposite to exec mask
        RayPacketMask res(0);

        for (uint32_t i = 0; i < primCount; i++) {
            auto primIndex = primIndices[i];
            uint32_t indexBase = primIndex*Mesh::kVertexCountPerPrim;
            // TODO skip two-level indircection
            uint32_t v0 = getIndex(indexBase + 0);
            uint32_t v1 = getIndex(indexBase + 1);
            uint32_t v2 = getIndex(indexBase + 2);
            const Vector3f& p0 = getPosition(v0);
            const Vector3f& p1 = getPosition(v1);
            const Vector3f& p2 = getPosition(v2);
            for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
                if (!mask.masks[v].anyTrue()) continue;

                auto& ray = packet.rays[v];
                auto intr = intersectTriangle(mask.masks[v], ray.org, ray.dir, ray.swapXZ, ray.swapYZ, p0, p1, p2);

                auto maskHit = intr.t.greaterThanOrEqual(kEpsilon) & intr.t.lessThan(ray.maxT);

                if (maskHit.anyTrue()) {
                    auto& mat = getMaterial(getPrimToMaterial(primIndex));

                    if (mat.alphaTest) {
        // access to texcoord index can be removed??
                        v0 = getTexcoordIndex(indexBase + 0);
                        v1 = getTexcoordIndex(indexBase + 1);
                        v2 = getTexcoordIndex(indexBase + 2);

                        auto& t0 = getTexcoord(v0);
                        auto& t1 = getTexcoord(v1);
                        auto& t2 = getTexcoord(v2);
                        auto uv = intr.i*t0 + intr.j*t1 + intr.k*t2;

                        maskHit = mat.testAlpha(maskHit, uv);
                    }
                    res.computeOrSelf(v, maskHit);
                }
            }
        }

        return res;
    }
}

template bool Mesh::occluded<bool, SingleRayPacket>(const RayPacketMask&, const SingleRayPacket&, const uint32_t*, uint32_t) const;
template RayPacketMask Mesh::occluded<RayPacketMask, RayPacket>(const RayPacketMask&, const RayPacket&, const uint32_t*, uint32_t) const;


template<typename T, typename U>
void Mesh::getSurfaceProperties(T& prop, const U& hit) const
{
    if constexpr (std::is_same<U, RayHit>::value) {
        Vector3f normal;

        uint32_t indexBase = hit.primId*Mesh::kVertexCountPerPrim;
        uint32_t v0 = getIndex(indexBase + 0);
        uint32_t v1 = getIndex(indexBase + 1);
        uint32_t v2 = getIndex(indexBase + 2);

        const Vector3f& p0 = getPosition(v0);
        const Vector3f& p1 = getPosition(v1);
        const Vector3f& p2 = getPosition(v2);

        if (hasVertexNormal()) {
            const auto &n0 = getNormal(v0);
            const auto &n1 = getNormal(v1);
            const auto &n2 = getNormal(v2);
            normal = normalize(hit.i * n0 + hit.j * n1 + hit.k * n2);
        } else {
//            const Vector3f &p0 = getPosition(v0);
//            const Vector3f &p1 = getPosition(v1);
//            const Vector3f &p2 = getPosition(v2);
            normal = normalize(cross(p1 - p0, p2 - p0));
        }

// TODO: remove indirection?
// use 3 floats at once
        Vector2f t0;
        Vector2f t1;
        Vector2f t2;
        if (m_hasTexcoord) {
            v0 = getTexcoordIndex(indexBase + 0);
            v1 = getTexcoordIndex(indexBase + 1);
            v2 = getTexcoordIndex(indexBase + 2);
            t0 = getTexcoord(v0);
            t1 = getTexcoord(v1);
            t2 = getTexcoord(v2);
        } else {
            t0 = Vector2f(0.0f, 0.0f);
            t1 = Vector2f(1.0f, 0.0f);
            t2 = Vector2f(0.0f, 1.0f);
        }
        prop.uv = hit.i*t0 + hit.j*t1 + hit.k*t2;

        prop.normal = normal;
        prop.material = &getMaterial(getPrimToMaterial(hit.primId));

        prop.dp01 = normalize(p1 - p0);
        prop.dp02 = normalize(p2 - p0);
        prop.duv01 = safeNormalize(t1 - t0);
        prop.duv02 = safeNormalize(t2 - t0);

    } else if constexpr (std::is_same<U, SoaRayHit>::value) {
        __builtin_trap();
    } else {
        __builtin_trap();
    }
}

template void Mesh::getSurfaceProperties<SurfaceProperties, RayHit>(SurfaceProperties&, const RayHit&) const;



}