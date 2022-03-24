#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "../ext/tinyobjloader/tiny_obj_loader.h"
#include "log.h"
#include "vecmath.h"
#include "triangle.h"

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
    m_hasVertexNormal = m.m_hasVertexNormal;

    m.m_indices = nullptr;
    m.m_positions = nullptr;
    m.m_normals = nullptr;
    m.m_primMaterial = nullptr;
    m.m_materials = nullptr;
    m.m_indexCount = 0;
    m.m_vertexCount = 0;
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
    m_texcoordsCount = m.m_texcoordsCount;
    m_hasVertexNormal = m.m_hasVertexNormal;

    m.m_indices = nullptr;
    m.m_positions = nullptr;
    m.m_normals = nullptr;
    m.m_primMaterial = nullptr;
    m.m_materials = nullptr;
    m.m_indexCount = 0;
    m.m_vertexCount = 0;

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

    for (uint32_t i = 0; i < m_indexCount/Mesh::kVertexCountPerPrim; i++) {
        uint32_t indexBase = i*Mesh::kVertexCountPerPrim;
        uint32_t v[Mesh::kVertexCountPerPrim];
        Vector3f p[Mesh::kVertexCountPerPrim];
        for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
            v[j] = m_indices[indexBase + j];
            p[j] = m_positions[v[j]];
        }

        auto normal = normalize(cross(p[1] - p[0], p[2] - p[0]));

        for (uint32_t j = 0; j < Mesh::kVertexCountPerPrim; j++) {
            normals[v[j]] = normals[v[j]] + normal;
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

    std::vector<uint32_t> indices;
    std::vector<uint32_t> texcoordIndices;

    for (auto s: objReader.GetShapes()) {
        // TODO: not taking care of normal and texture coords
        auto& srcIndices = s.mesh.indices;
        indices.reserve(indices.size() + srcIndices.size());
        // TODO can be insert
        for (auto i: srcIndices) {
            indices.push_back(i.vertex_index);
            texcoordIndices.push_back(i.texcoord_index);
        }
    }

    auto& attr = objReader.GetAttrib();
    auto& srcVertices = attr.vertices;
    auto& srcTexcoords = attr.texcoords;

    uint32_t primCount = indices.size()/Mesh::kVertexCountPerPrim;
    const uint32_t kFloatCountPerVertex = 3;
    create(primCount, srcVertices.size()/kFloatCountPerVertex, 1, false);

    const uint32_t kFloatCountPerTexcoord = 2;

    memcpy(getIndexBuffer(), indices.data(), indices.size()*sizeof(uint32_t));
    memcpy(getPositionBuffer(), srcVertices.data(), srcVertices.size()*sizeof(float));

    m_texcoordsCount = srcTexcoords.size()/kFloatCountPerTexcoord;
    memcpy(m_texcoordIndices, texcoordIndices.data(), texcoordIndices.size()*sizeof(uint32_t));
    memcpy(m_texcoords, srcTexcoords.data(), srcTexcoords.size()*sizeof(float));

    memset(getPrimMateialBuffer(), 0, primCount*sizeof(uint32_t));
    memcpy(getMaterialBuffer(), &mat, sizeof(Material));

    logPrintf(LogLevel::kVerbose, "finished loading '%s'\n", path);
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
    for (auto srcMat: objReader.GetMaterials()) {
        auto& m = materials[mi];
        m.load(srcMat, objPath.parent_path());
        mi++;
    }

    std::vector<uint32_t> primMat;
    std::vector<uint32_t> indices;
    std::vector<uint32_t> texcoordIndices;

    for (auto s: objReader.GetShapes()) {
        // TODO: not taking care of normal and texture coords
        auto& srcIndices = s.mesh.indices;
        indices.reserve(indices.size() + srcIndices.size());
        // TODO can be insert
        for (auto i: srcIndices) {
            indices.push_back(i.vertex_index);
            texcoordIndices.push_back(i.texcoord_index);
        }

        primMat.insert(primMat.end(), s.mesh.material_ids.begin(), s.mesh.material_ids.end());
    }

    auto& attr = objReader.GetAttrib();
    auto& srcVertices = attr.vertices;
    auto& srcTexcoords = attr.texcoords;

    uint32_t primCount = indices.size()/Mesh::kVertexCountPerPrim;
    const uint32_t kFloatCountPerVertex = 3;
    create(primCount, srcVertices.size()/kFloatCountPerVertex, materials.size(), false);

    const uint32_t kFloatCountPerTexcoord = 2;

    memcpy(getIndexBuffer(), indices.data(), indices.size()*sizeof(uint32_t));
    memcpy(getPositionBuffer(), srcVertices.data(), srcVertices.size()*sizeof(float));

    m_texcoordsCount = srcTexcoords.size()/kFloatCountPerTexcoord;
    memcpy(m_texcoordIndices, texcoordIndices.data(), texcoordIndices.size()*sizeof(uint32_t));
    memcpy(m_texcoords, srcTexcoords.data(), srcTexcoords.size()*sizeof(float));

    memcpy(getPrimMateialBuffer(), primMat.data(), primCount*sizeof(uint32_t));
    memcpy(getMaterialBuffer(), materials.data(), materials.size()*sizeof(Material));

    logPrintf(LogLevel::kVerbose, "finished loading '%s'\n", path);
}


}