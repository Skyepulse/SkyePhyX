#ifndef GEOMETRYGENERATOR_HPP
#define GEOMETRYGENERATOR_HPP

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>

#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

//================================//
enum ModelType
{
    ModelType_Cube = 0,
    ModelType_Sphere = 1,
    ModelType_TestConvexMesh = 2,
    ModelType_Capsule = 3,
};

//================================//
struct Vertex
{
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector2f uv;
};

struct Triangle
{
    uint32_t vertexIndices[3];
};

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
                mesh.indices.insert(mesh.indices.end(), { a, a + 1, b, a + 1, b + 1, b });            
            }
        }

        return mesh;
    }

    //================================//
    inline MeshData GenerateCapsule(uint32_t hemiStacks = 8, uint32_t bodyStacks = 1, uint32_t slices = 32)
    {
        constexpr float height = 1.0f;
        constexpr float radius = 0.25f;
        constexpr float cylinderHeight = height - 2.0f * radius;
        constexpr float halfCylinder   = cylinderHeight * 0.5f;

        MeshData mesh;

        const uint32_t ringCount = 2 * hemiStacks + bodyStacks + 1;
        const uint32_t quadRows  = ringCount - 1;

        mesh.vertices.reserve(ringCount * (slices + 1));
        mesh.indices.reserve(quadRows * slices * 6);

        uint32_t ring = 0;

        auto PushRing = [&](float y, float ringRadius, float normalY, float normalXZ)
        {
            for (uint32_t j = 0; j <= slices; ++j)
            {
                float theta = 2.0f * static_cast<float>(M_PI) * static_cast<float>(j) / static_cast<float>(slices);
                float c = std::cos(theta);
                float s = std::sin(theta);

                Vertex v;
                v.normal   = { normalXZ * c, normalY, normalXZ * s };
                v.position = { ringRadius * c, y, ringRadius * s };
                v.uv       = { static_cast<float>(j) / static_cast<float>(slices), static_cast<float>(ring) / static_cast<float>(quadRows) };

                mesh.vertices.push_back(v);
            }

            ring++;
        };

        // Top -> ring of capsule
        for (uint32_t i = 0; i <= hemiStacks; ++i)
        {
            float a = 0.5f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(hemiStacks);
            float sinA = std::sin(a);
            float cosA = std::cos(a);

            float y          = halfCylinder + cosA * radius;
            float ringRadius = sinA * radius;

            PushRing(y, ringRadius, cosA, sinA);
        }

        // Body of capsule
        for (uint32_t i = 1; i <= bodyStacks; ++i)
        {
            float t = static_cast<float>(i) / static_cast<float>(bodyStacks);
            float y = halfCylinder - t * cylinderHeight;

            PushRing(y, radius, 0.0f, 1.0f);
        }

        // Ring of capsule -> bottom
        for (uint32_t i = 0; i <= hemiStacks; ++i)
        {
            float a = 0.5f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(hemiStacks);
            float sinA = std::sin(a);
            float cosA = std::cos(a);

            float y          = -halfCylinder - sinA * radius;
            float ringRadius =  cosA * radius;

            PushRing(y, ringRadius, -sinA, cosA);
        }

        for (uint32_t i = 0; i < quadRows; ++i)
        {
            for (uint32_t j = 0; j < slices; ++j)
            {
                uint32_t a = i * (slices + 1) + j;
                uint32_t b = a + (slices + 1);

                mesh.indices.insert(mesh.indices.end(), { a, a + 1, b, a + 1, b + 1, b });
            }
        }

        return mesh;
    }

    //================================//
    inline MeshData GenerateTestConvexMesh()
    {
        MeshData mesh;

        const std::array<Eigen::Vector2f, 6> profile =
        {{
            { -0.45f, -0.50f },
            {  0.18f, -0.50f },
            {  0.50f, -0.12f },
            {  0.36f,  0.50f },
            { -0.18f,  0.50f },
            { -0.50f,  0.10f }
        }};

        const float halfDepth = 0.35f;

        auto appendFace = [&](const std::vector<Eigen::Vector3f>& positions, const Eigen::Vector3f& normal)
        {
            if (positions.size() < 3)
                return;

            const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
            for (size_t i = 0; i < positions.size(); ++i)
            {
                Vertex vertex;
                vertex.position = positions[i];
                vertex.normal = normal;
                vertex.uv = Eigen::Vector2f(
                    (positions[i].x() + 0.5f),
                    (positions[i].y() + 0.5f)
                );
                mesh.vertices.push_back(vertex);
            }

            for (uint32_t i = 1; i + 1 < positions.size(); ++i)
            {
                mesh.indices.push_back(base);
                mesh.indices.push_back(base + i);
                mesh.indices.push_back(base + i + 1);
            }
        };

        std::vector<Eigen::Vector3f> frontFace;
        std::vector<Eigen::Vector3f> backFace;
        frontFace.reserve(profile.size());
        backFace.reserve(profile.size());

        for (const Eigen::Vector2f& point : profile)
        {
            frontFace.push_back(Eigen::Vector3f(point.x(), point.y(), halfDepth));
            backFace.push_back(Eigen::Vector3f(point.x(), point.y(), -halfDepth));
        }

        appendFace(frontFace, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        std::reverse(backFace.begin(), backFace.end());
        appendFace(backFace, Eigen::Vector3f(0.0f, 0.0f, -1.0f));

        for (size_t i = 0; i < profile.size(); ++i)
        {
            const size_t j = (i + 1) % profile.size();
            const Eigen::Vector2f edge = profile[j] - profile[i];
            Eigen::Vector3f sideNormal(edge.y(), -edge.x(), 0.0f);
            sideNormal.normalize();

            appendFace(
                {
                    Eigen::Vector3f(profile[i].x(), profile[i].y(),  halfDepth),
                    Eigen::Vector3f(profile[i].x(), profile[i].y(), -halfDepth),
                    Eigen::Vector3f(profile[j].x(), profile[j].y(), -halfDepth),
                    Eigen::Vector3f(profile[j].x(), profile[j].y(),  halfDepth)
                },
                sideNormal
            );
        }

        return mesh;
    }

    //================================//
    inline MeshData GenerateMeshDataForModelType(ModelType modelType)
    {
        switch (modelType)
        {
            case ModelType_Cube:
                return GenerateCube();
            case ModelType_Sphere:
                return GenerateSphere(32, 16);
            case ModelType_TestConvexMesh:
                return GenerateTestConvexMesh();
            case ModelType_Capsule:
                return GenerateCapsule(16, 1, 32);
            default:
                return MeshData{};
        }
    }
}

#endif // GEOMETRYGENERATOR_HPP
