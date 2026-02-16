#ifndef GEOMETRYGENERATOR_HPP
#define GEOMETRYGENERATOR_HPP

#include "geometry.hpp"
#include <vector>
#include <corecrt_math_defines.h>

//================================//
struct MeshData
{
    std::vector<Vertex>   vertices;
    std::vector<uint32_t> indices;

    uint32_t vertexCount() const { return static_cast<uint32_t>(vertices.size()); }
    uint32_t indexCount()  const { return static_cast<uint32_t>(indices.size()); }
};

//================================//
namespace GeometryGenerator
{
    //================================//
    inline MeshData GenerateCube()
    {
        MeshData mesh;

        struct FaceDesc 
        {
            Eigen::Vector3f normal;
            Eigen::Vector3f up;
            Eigen::Vector3f right;
        };

        const FaceDesc faces[6] = 
        {
            { { 0, 0, 1}, { 0, 1, 0}, { 1, 0, 0} },  // +Z
            { { 0, 0,-1}, { 0, 1, 0}, {-1, 0, 0} },  // -Z
            { { 1, 0, 0}, { 0, 1, 0}, { 0, 0,-1} },  // +X
            { {-1, 0, 0}, { 0, 1, 0}, { 0, 0, 1} },  // -X
            { { 0, 1, 0}, { 0, 0,-1}, { 1, 0, 0} },  // +Y
            { { 0,-1, 0}, { 0, 0, 1}, { 1, 0, 0} },  // -Y
        };

        mesh.vertices.reserve(24);
        mesh.indices.reserve(36);

        for (int f = 0; f < 6; ++f)
        {
            const auto& face = faces[f];
            Eigen::Vector3f center = face.normal * 0.5f;
            uint32_t base = static_cast<uint32_t>(mesh.vertices.size());

            for (int j = 0; j < 4; ++j)
            {
                float signR = (j & 1) ? 0.5f : -0.5f;
                float signU = (j & 2) ? 0.5f : -0.5f;

                Vertex v;
                v.position = center + face.right * signR + face.up * signU;
                v.normal   = face.normal;
                v.uv       = { (j & 1) ? 1.0f : 0.0f, (j & 2) ? 1.0f : 0.0f };
                mesh.vertices.push_back(v);
            }

            mesh.indices.push_back(base + 0);
            mesh.indices.push_back(base + 1);
            mesh.indices.push_back(base + 3);
            mesh.indices.push_back(base + 0);
            mesh.indices.push_back(base + 3);
            mesh.indices.push_back(base + 2);
        }

        return mesh;
    }

    //================================//
    inline MeshData GenerateSphere(uint32_t stacks = 16, uint32_t slices = 32)
    {
        MeshData mesh;
        mesh.vertices.reserve((stacks + 1) * (slices + 1));
        mesh.indices.reserve(stacks * slices * 6);

        for (uint32_t i = 0; i <= stacks; ++i)
        {
            float phi = static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(stacks);
            float sinPhi = std::sin(phi);
            float cosPhi = std::cos(phi);

            for (uint32_t j = 0; j <= slices; ++j)
            {
                float theta = 2.0f * static_cast<float>(M_PI) * static_cast<float>(j) / static_cast<float>(slices);

                Vertex v;
                v.normal = { sinPhi * std::cos(theta), cosPhi, sinPhi * std::sin(theta) };
                v.position = v.normal * 0.5f;
                v.uv = { static_cast<float>(j) / slices, static_cast<float>(i) / stacks };
                mesh.vertices.push_back(v);
            }
        }

        for (uint32_t i = 0; i < stacks; ++i)
        {
            for (uint32_t j = 0; j < slices; ++j)
            {
                uint32_t a = i * (slices + 1) + j;
                uint32_t b = a + slices + 1;
                mesh.indices.insert(mesh.indices.end(), { a, b, a + 1, a + 1, b, b + 1 });
            }
        }

        return mesh;
    }
}

#endif // GEOMETRYGENERATOR_HPP