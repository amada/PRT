#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <thread>
#include <limits>

#include "sample_models.h"

namespace prt
{

Mesh SampleModels::getCornellBox(bool box)
{
    Mesh mesh;

    std::vector<Material> materials;
    std::vector<uint32_t> indices;
    std::vector<uint32_t> primMaterial;

    Vector3f verts[] = {
        // Floor
        {-1.01, 0.00, 0.99},
        {1.00, 0.00, 0.99},
        {1.00, 0.00, -1.04},
        {-0.99, 0.00, -1.04},

        // Ceiling
        {-1.02, 1.99, 0.99},
        {-1.02, 1.99, -1.04},
        {1.00, 1.99, -1.04},
        {1.00, 1.99, 0.99},

        // BackWall
        {-0.99, 0.00, -1.04},
        {1.00, 0.00, -1.04},
        {1.00, 1.99, -1.04},
        {-1.02, 1.99, -1.04},

        // Right wall
        {1.00, 0.00, -1.04},
        {1.00, 0.00, 0.99},
        {1.00, 1.99, 0.99},
        {1.00, 1.99, -1.04},

        // Left wall
        {-1.01, 0.00, 0.99},
        {-0.99, 0.00, -1.04},
        {-1.02, 1.99, -1.04},
        {-1.02, 1.99, 0.99},

        // Short Box
        // Top Face
        {0.53, 0.60 , 0.75 },
        {0.70, 0.60 , 0.17, },
        {0.13, 0.60 , 0.00},
        {-0.05, 0.60 , 0.57},

        // Left Face
        {-0.05, 0.00 , 0.57},
        {-0.05, 0.60 , 0.57},
        { 0.13, 0.60 , 0.00 },
        { 0.13, 0.00 , 0.00},

        // Front Face
        {0.53, 0.00 , 0.75},
        {0.53, 0.60 , 0.75},
        {-0.05, 0.60 , 0.57},
        {-0.05, 0.00 , 0.57},

        // Right Face
        {0.70, 0.00 , 0.17},
        {0.70, 0.60 , 0.17},
        {0.53, 0.60 , 0.75},
        {0.53, 0.00 , 0.75},

        // Back Face
        {0.13, 0.00 , 0.00},
        {0.13, 0.60 , 0.00},
        {0.70, 0.60 , 0.17},
        {0.70, 0.00 , 0.17},

        // Bottom Face
        {0.53, 0.00 , 0.75 },
        {0.70, 0.00 , 0.17, },
        {0.13, 0.00 , 0.00},
        {-0.05, 0.00 , 0.57},

        // Tall box
        // Top Face
        {-0.53, 1.20 , 0.09},
        {0.04, 1.20, -0.09},
        {-0.14, 1.20, -0.67},
        {-0.71, 1.20, -0.49},

        // Left Face
        {-0.53, 0.00 , 0.09},
        {-0.53, 1.20 , 0.09},
        {-0.71, 1.20, -0.49},
        {-0.71, 0.00, -0.49},

        // Back Face
        {-0.71, 0.00, -0.49},
        {-0.71, 1.20, -0.49},
        {-0.14, 1.20, -0.67},
        {-0.14, 0.00, -0.67},

        // Right Face
        {-0.14, 0.00, -0.67},
        {-0.14, 1.20, -0.67},
        {0.04, 1.20, -0.09},
        {0.04, 0.00, -0.09},

        // Front Face
        {0.04, 0.00, -0.09},
        {0.04, 1.20, -0.09},
        {-0.53, 1.20 , 0.09},
        {-0.53, 0.00 , 0.09},

        // Bottom Face
        {-0.53, 0.00 , 0.09},
        {0.04, 0.00, -0.09},
        {-0.14, 0.00, -0.67},
        {-0.71, 0.00, -0.49},

        // Light
        {-0.24, 1.98 , 0.16},
        {-0.24, 1.98, -0.22},
        {0.23, 1.98, -0.22},
        {0.23, 1.98 , 0.16}};

    indices = {
            0, 1, 2, 0, 2, 3, // floor
            4, 5, 6, 4, 6, 7, // ceiling
            8, 9, 10, 8, 10, 11, // back
            12, 13, 14, 12, 14, 15, // right
            16, 17, 18, 16, 18, 19, // left
    };
        
    auto createDiffuse = [](const Vector3f& color) {
        Material m;
        m.init();
        m.diffuse = color;
        return m;
    };

    // floor
    materials.push_back(createDiffuse(Vector3f(0.725, 0.71, 0.68)));

    // ceiling
    materials.push_back(createDiffuse(Vector3f(0.725, 0.71, 0.68)));

    // back
    materials.push_back(createDiffuse(Vector3f(0.725, 0.71, 0.68)));

    // right
    materials.push_back(createDiffuse(Vector3f(0.14, 0.45, 0.091)));

    // left
    materials.push_back(createDiffuse(Vector3f(0.63, 0.065, 0.05)));

    if (box) {
        // Short box and tall box
        for (uint32_t i = 0; i < 12; i++) {
            uint32_t b = 4 * i;

#if 1 // glass box
            if (i >= 6) {
                auto m = createDiffuse(Vector3f(0.725, 0.71, 0.68));
                m.reflectionType = ReflectionType::kDiffuse;
                materials.push_back(m);
            } else {
                materials.push_back(createDiffuse(Vector3f(0.725, 0.71, 0.68)));
            }
#else
            materials.push_back(createDiffuse(Vector3f{0.725, 0.71, 0.68));
#endif
            indices.insert(indices.end(), {20 + b, 21 + b, 22 + b, 20 + b, 22 + b, 23 + b});
        }
    }

    // light
    Material em;
    em.init();
    em.emissive = Vector3f(17, 12, 4);
    materials.push_back(em);
    indices.insert(indices.end(), {68, 69, 70, 68, 70, 71});

    primMaterial.resize(materials.size()*2);
    for (uint32_t i = 0; i < materials.size(); i++) {
        uint32_t base = 2*i;
        primMaterial[base + 0] = i;
        primMaterial[base + 1] = i;
    }

    // TODO prim material size?

    mesh.create(indices.size()/Mesh::kVertexCountPerPrim, sizeof(verts)/sizeof(verts[0]), materials.size(), false);

    memcpy(mesh.getIndexBuffer(), &indices[0], indices.size()*sizeof(uint32_t));
    memcpy(mesh.getPositionBuffer(), &verts[0], sizeof(verts));
    memcpy(mesh.getPrimMateialBuffer(), &primMaterial[0], primMaterial.size()*sizeof(uint32_t));
    memcpy(mesh.getMaterialBuffer(), &materials[0], materials.size()*sizeof(Material));
    memset(mesh.getTexcoordBuffer(), 0, mesh.getVertexCount()*sizeof(Vector2f));

    mesh.calculateBounds();

    return mesh;
}

} // namespace prt
