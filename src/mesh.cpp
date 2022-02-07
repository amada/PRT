#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <fstream>

#include "vecmath.h"
#include "triangle.h"

#include "mesh.h"


#define DELETE_ARRAY(x) { delete[] x; x = nullptr; }

namespace prt
{

Mesh::Mesh(Mesh&& m)
{
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
    m_positions = new Vector3f[vertexCount];
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
    std::ifstream file(path);

    if (!file.is_open()) {
        printf("Failed to open '%s'\n", path);
    }

    auto findNs = [](const std::string& str, std::string::size_type pos, char d) {
        for (auto i = pos; i < str.length(); i++) {
            if (str[i] != d)
                return i;
        }
        return std::string::npos;
    };

    // Apparently this allocates std::string from heap every time, which isn't good for performance
    auto findToken = [&findNs](const std::string& str, std::string::size_type& pos, char d) {
        auto vstart = findNs(str, pos, d);
        auto vend = str.find(d, vstart);
        pos = vend;
        return str.substr(vstart, vend - vstart + 1);
    };

    std::vector<Vector3f> vertices;
    std::vector<uint32_t> indices;
    std::string line;
    while (getline(file, line)) {
        auto spos = line.find(' ');
        auto str = line.substr(0, spos);
        if (str == "v") {
            float v[3];
            auto pos = spos;
            for (uint32_t i = 0; i < 3; i++) {
                auto vstr = findToken(line, pos, ' ');
                v[i] = std::atof(vstr.c_str());
            }
            vertices.push_back(Vector3f(v[0], v[1], v[2]));
//            printf("(%f, %f, %f)\n", v[0], v[1], v[2]);
        } else if (str == "vn") {
        } else if (str == "f") {
            int f[4]; // Can be quad
            auto pos = spos;
            for (uint32_t i = 0; i < std::size(f); i++) {
                auto fstr = findToken(line, pos, ' ');
                std::string::size_type temp = 0;
                auto fvstr = findToken(fstr, temp, '/');
                f[i] = std::atoi(fvstr.c_str()) - 1;
            }

            const int32_t kInvalidIndex = -1; // When std::atoi fails to convert, it returns 0; 0 isn't valid index for .obj format
            indices.insert(indices.end(), {(uint32_t)f[0], (uint32_t)f[1], (uint32_t)f[2]});
            if (f[3] != kInvalidIndex)
                indices.insert(indices.end(), {(uint32_t)f[0], (uint32_t)f[2], (uint32_t)f[3]});

//            printf("(%i, %i, %i, %i)\n", f[0], f[1], f[2], f[3]);
        }
    }
    file.close();

    uint32_t primCount = indices.size()/Mesh::kVertexCountPerPrim;
    create(primCount, vertices.size(), 1, false);

    memcpy(getIndexBuffer(), &indices[0], indices.size()*sizeof(uint32_t));
    memcpy(getPositionBuffer(), &vertices[0], vertices.size()*sizeof(Vector3f));
    memset(getPrimMateialBuffer(), 0, primCount*sizeof(uint32_t));
    memcpy(getMaterialBuffer(), &mat, sizeof(Material));
}


}