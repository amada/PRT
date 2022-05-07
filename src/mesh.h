#pragma once

#include "vecmath.h"
#include "ray.h"
#include "material.h"

namespace prt
{

template<typename T>
struct SoaVar
{
    T lanes[SoaConstants::kLaneCount];
};


template<typename T, typename U>
struct SurfacePropertiesT
{
    T normal;
    U material;
    Vector2f uv;
    Vector2f duv01;
    Vector2f duv02;
    Vector3f dp01;
    Vector3f dp02;
};

using SurfaceProperties = SurfacePropertiesT<Vector3f, const Material*>;
//using SoaSurfaceProperties = SurfacePropertiesT<SoaVector3f, SoaVar<const Material*>>;

// struct?
class Mesh
{
    friend class Scene;
public:
    static const uint32_t kVertexCountPerPrim = 3;

    Mesh() = default;
    Mesh(Mesh&& m);
    Mesh& operator=(Mesh&& m);
    virtual ~Mesh();

    void loadObj(const char* path);
    void loadObj(const char* path, const Material& mat);

    // TODO make create config
    // Add texcoordCount
    void create(uint32_t primCount, uint32_t vertexCount, uint32_t materialCount, bool hasVertexNormal);

    template<typename T, typename U>
    void getSurfaceProperties(T& properties, const U& hit) const;

    const BBox& getBBox() const { return m_bbox; }
    void calculateVertexNormals();
    void calculateBounds();

    uint32_t getPrimCount() const { return m_indexCount/kVertexCountPerPrim; }
    uint32_t getIndexCount() const { return m_indexCount; }
    uint32_t getIndex(uint32_t index) const { return m_indices[index]; };
    uint32_t getTexcoordIndex(uint32_t index) const { return m_texcoordIndices[index]; }

    uint32_t getVertexCount() const { return m_vertexCount; }
    const Vector3f& getPosition(uint32_t index) const { return m_positions[index]; }
    const Vector2f& getTexcoord(uint32_t index) const { return m_texcoords[index]; }
    const Vector3f& getNormal(uint32_t index) const { return m_normals[index]; }
    uint32_t getPrimToMaterial(uint32_t index) const { return m_primMaterial[index]; }
    const Material& getMaterial(uint32_t index) const { return m_materials[index]; }

    uint32_t getMaterialCount() const { return m_materialCount; }

    bool hasVertexNormal() const { return m_hasVertexNormal; }

    // For initialization
    uint32_t* getIndexBuffer() { return m_indices; }
    Vector3f* getPositionBuffer() { return m_positions; }
    Vector3f* getNormalBuffer() { return m_normals; }
    uint32_t* getPrimMateialBuffer() { return m_primMaterial; }
    Material* getMaterialBuffer() { return m_materials; }
    Vector2f* getTexcoordBuffer() { return m_texcoords; }

    int32_t getId() const { return m_id; }

private:
    static constexpr float kEpsilon = 0.0001f;

    uint32_t* m_indices = nullptr;
    uint32_t* m_texcoordIndices = nullptr;
    Vector3f* m_positions = nullptr;
    Vector3f* m_normals = nullptr;
    Vector2f* m_texcoords = nullptr;
    uint32_t* m_primMaterial = nullptr; // TODO prim material map?
    Material* m_materials = nullptr;
    uint32_t m_indexCount = 0;
    uint32_t m_vertexCount = 0;
    uint32_t m_materialCount = 0;
    uint32_t m_texcoordsCount = 0;
    BBox m_bbox;

    bool m_hasVertexNormal = false;
    bool m_hasTexcoord = false;
    uint32_t m_id = 0; // TODO mesh id
};

} // namespace prt
