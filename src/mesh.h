#pragma once

#include "vecmath.h"

namespace prt
{

enum class ReflectionType : uint32_t {
    kDiffuse = 0,
    kSpecular,
    kRefraction
};

struct Material
{
    Vector3f diffuse = {0, 0, 0};
    Vector3f emissive = {0, 0, 0};
    ReflectionType reflectionType = ReflectionType::kDiffuse;
};


class Mesh
{
public:
    static const uint32_t kVertexCountPerPrim = 3;

    Mesh() = default;
    Mesh(Mesh&& m);
    Mesh& operator=(Mesh&& m);
    virtual ~Mesh();

    void loadObj(const char* path, const Material& mat);
    void create(uint32_t primCount, uint32_t vertexCount, uint32_t materialCount, bool hasVertexNormal);

    void calculateVertexNormals();

    uint32_t getPrimCount() const { return m_indexCount/kVertexCountPerPrim; }
    uint32_t getIndexCount() const { return m_indexCount; }
    uint32_t getIndex(uint32_t index) const { return m_indices[index]; };

    uint32_t getVertexCount() const { return m_vertexCount; }
    const Vector3f& getPosition(uint32_t index) const { return m_positions[index]; }
    const Vector3f& getNormal(uint32_t index) const { return m_normals[index]; }
    uint32_t getPrimToMaterial(uint32_t index) const { return m_primMaterial[index]; }
    const Material& getMaterial(uint32_t index) const { return m_materials[index]; }

    bool hasVertexNormal() const { return m_hasVertexNormal; }

    // For initialization
    uint32_t* getIndexBuffer() { return m_indices; }
    Vector3f* getPositionBuffer() { return m_positions; }
    Vector3f* getNormalBuffer() { return m_normals; }
    uint32_t* getPrimMateialBuffer() { return m_primMaterial; }
    Material* getMaterialBuffer() { return m_materials; }

private:
    uint32_t* m_indices = nullptr;
    Vector3f* m_positions = nullptr;
    Vector3f* m_normals = nullptr;
    uint32_t* m_primMaterial = nullptr; // TODO prim material map?
    Material* m_materials = nullptr;
    uint32_t m_indexCount = 0;
    uint32_t m_vertexCount = 0;
    bool m_hasVertexNormal = false;
};

} // namespace prt