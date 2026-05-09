
#include "geometry.hpp"
#include "../physics/solver.hpp"

#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullVertexSet.h>

#include <limits>
#include <map>
#include <unordered_map>

namespace
{
    struct RawHullTriangle
    {
        uint32_t vertexIndices[3] = { 0u, 0u, 0u };
        Eigen::Vector3f normal = Eigen::Vector3f::Zero();
        float planeOffset = 0.0f;
    };

    //================================//
    static bool AreCoplanarTriangles(const RawHullTriangle& a, const RawHullTriangle& b)
    {
        return (a.normal.dot(b.normal) > 1.0f - 1e-4f) &&
               (std::abs(a.planeOffset - b.planeOffset) <= 1e-4f);
    }

    //================================//
    static HullFace BuildMergedFace(const std::vector<RawHullTriangle>& triangles, const std::vector<HullVertex>& vertices)
    {
        HullFace face;
        if (triangles.empty())
            return face;

        face.normal = triangles[0].normal;
        face.planeOffset = triangles[0].planeOffset;

        std::map<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> orientedBoundaryEdges;
        for (const RawHullTriangle& triangle : triangles)
        {
            for (int edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
            {
                const uint32_t a = triangle.vertexIndices[edgeIndex];
                const uint32_t b = triangle.vertexIndices[(edgeIndex + 1) % 3];

                const std::pair<uint32_t, uint32_t> edge(a, b);
                const std::pair<uint32_t, uint32_t> reversedEdge(b, a);

                auto reversedIt = orientedBoundaryEdges.find(reversedEdge);
                if (reversedIt != orientedBoundaryEdges.end())
                    orientedBoundaryEdges.erase(reversedIt);
                else
                    orientedBoundaryEdges.emplace(edge, edge);
            }
        }

        if (orientedBoundaryEdges.size() < 3)
            return HullFace{};

        std::map<uint32_t, uint32_t> nextVertex;
        for (const auto& [edgeKey, edge] : orientedBoundaryEdges)
        {
            (void)edgeKey;
            nextVertex[edge.first] = edge.second;
        }

        if (nextVertex.size() < 3)
            return HullFace{};

        const uint32_t startVertex = nextVertex.begin()->first;
        uint32_t currentVertex = startVertex;

        for (size_t i = 0; i < nextVertex.size(); ++i)
        {
            face.vertexIndices.push_back(currentVertex);

            auto nextIt = nextVertex.find(currentVertex);
            if (nextIt == nextVertex.end())
                return HullFace{};

            currentVertex = nextIt->second;
            if (currentVertex == startVertex)
                break;
        }

        if (face.vertexIndices.size() < 3 || currentVertex != startVertex)
            return HullFace{};

        const Eigen::Vector3f& p0 = vertices[face.vertexIndices[0]].position;
        const Eigen::Vector3f& p1 = vertices[face.vertexIndices[1]].position;
        const Eigen::Vector3f& p2 = vertices[face.vertexIndices[2]].position;
        const Eigen::Vector3f geometricNormal = (p1 - p0).cross(p2 - p0);
        if (geometricNormal.dot(face.normal) < 0.0f)
            std::reverse(face.vertexIndices.begin(), face.vertexIndices.end());

        return face;
    }
}

//================================//
static AABB ComputeLocalAABB(const MeshData& meshData)
{
    AABB aabb;
    if (meshData.vertices.empty())
    {
        aabb.min = Eigen::Vector3f::Zero();
        aabb.max = Eigen::Vector3f::Zero();
        return aabb;
    }

    Eigen::Vector3f min = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f max = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

    for (const Vertex& vertex : meshData.vertices)
    {
        min = min.cwiseMin(vertex.position);
        max = max.cwiseMax(vertex.position);
    }

    // Inflate a little bit the AABB
    Eigen::Vector3f padding(0.01f, 0.01f, 0.01f);
    min -= padding;
    max += padding;
    aabb.min = min;
    aabb.max = max;

    return aabb;
}

//================================//
static ConvexHull ComputeConvexHull(const MeshData& meshData)
{
    ConvexHull hull;

    if (meshData.vertices.empty())
        return hull;

    std::vector<Eigen::Vector3f> uniquePositions;
    uniquePositions.reserve(meshData.vertices.size());

    for (const Vertex& vertex : meshData.vertices)
    {
        bool alreadyPresent = false;
        for (const Eigen::Vector3f& existingPosition : uniquePositions)
        {
            if (existingPosition.isApprox(vertex.position, 1e-6f))
            {
                alreadyPresent = true;
                break;
            }
        }

        if (!alreadyPresent)
            uniquePositions.push_back(vertex.position);
    }

    if (uniquePositions.size() < 4)
        return hull;

    std::vector<double> pointCoordinates;
    pointCoordinates.reserve(uniquePositions.size() * 3);
    for (const Eigen::Vector3f& position : uniquePositions)
    {
        pointCoordinates.push_back(static_cast<double>(position.x()));
        pointCoordinates.push_back(static_cast<double>(position.y()));
        pointCoordinates.push_back(static_cast<double>(position.z()));
    }

    orgQhull::Qhull qhull;
    qhull.runQhull("", 3, static_cast<int>(uniquePositions.size()), pointCoordinates.data(), "Qt");

    std::unordered_map<uint32_t, uint32_t> qhullVertexIdToHullIndex;
    hull.vertices.reserve(static_cast<size_t>(qhull.vertexCount()));

    for (orgQhull::QhullVertex vertex = qhull.beginVertex(); vertex != qhull.endVertex(); vertex = vertex.next())
    {
        const orgQhull::QhullPoint point = vertex.point();

        HullVertex hullVertex;
        hullVertex.position = Eigen::Vector3f(
            static_cast<float>(point[0]),
            static_cast<float>(point[1]),
            static_cast<float>(point[2])
        );

        const uint32_t hullVertexIndex = static_cast<uint32_t>(hull.vertices.size());
        qhullVertexIdToHullIndex[static_cast<uint32_t>(vertex.id())] = hullVertexIndex;
        hull.vertices.push_back(hullVertex);
        hull.centroid += hullVertex.position;
    }

    if (!hull.vertices.empty())
        hull.centroid /= static_cast<float>(hull.vertices.size());

    std::vector<RawHullTriangle> rawTriangles;

    for (orgQhull::QhullFacet facet = qhull.beginFacet(); facet != qhull.endFacet(); facet = facet.next())
    {
        if (!facet.isGood())
            continue;

        const std::vector<orgQhull::QhullVertex> facetVertices = facet.vertices().toStdVector();
        if (facetVertices.size() != 3)
            continue;

        RawHullTriangle triangle;
        for (int i = 0; i < 3; ++i)
        {
            const uint32_t qhullVertexId = static_cast<uint32_t>(facetVertices[i].id());
            auto vertexIt = qhullVertexIdToHullIndex.find(qhullVertexId);
            if (vertexIt == qhullVertexIdToHullIndex.end())
                return ConvexHull{};

            triangle.vertexIndices[i] = vertexIt->second;
        }

        const orgQhull::QhullHyperplane hyperplane = facet.hyperplane();
        triangle.normal = Eigen::Vector3f(
            static_cast<float>(hyperplane[0]),
            static_cast<float>(hyperplane[1]),
            static_cast<float>(hyperplane[2])
        );

        const Eigen::Vector3f& p0 = hull.vertices[triangle.vertexIndices[0]].position;
        const Eigen::Vector3f& p1 = hull.vertices[triangle.vertexIndices[1]].position;
        const Eigen::Vector3f& p2 = hull.vertices[triangle.vertexIndices[2]].position;

        Eigen::Vector3f geometricNormal = (p1 - p0).cross(p2 - p0);
        if (geometricNormal.squaredNorm() <= 1e-12f)
            continue;

        if (geometricNormal.dot(triangle.normal) < 0.0f)
            std::swap(triangle.vertexIndices[1], triangle.vertexIndices[2]);

        triangle.planeOffset = triangle.normal.dot(hull.vertices[triangle.vertexIndices[0]].position);
        rawTriangles.push_back(triangle);
    }

    std::vector<bool> triangleConsumed(rawTriangles.size(), false);
    for (size_t triangleIndex = 0; triangleIndex < rawTriangles.size(); ++triangleIndex)
    {
        if (triangleConsumed[triangleIndex])
            continue;

        std::vector<RawHullTriangle> coplanarGroup;
        coplanarGroup.push_back(rawTriangles[triangleIndex]);
        triangleConsumed[triangleIndex] = true;

        for (size_t otherTriangleIndex = triangleIndex + 1; otherTriangleIndex < rawTriangles.size(); ++otherTriangleIndex)
        {
            if (triangleConsumed[otherTriangleIndex])
                continue;

            if (!AreCoplanarTriangles(rawTriangles[triangleIndex], rawTriangles[otherTriangleIndex]))
                continue;

            coplanarGroup.push_back(rawTriangles[otherTriangleIndex]);
            triangleConsumed[otherTriangleIndex] = true;
        }

        HullFace mergedFace = BuildMergedFace(coplanarGroup, hull.vertices);
        if (mergedFace.vertexIndices.size() >= 3)
            hull.faces.push_back(std::move(mergedFace));
    }

    std::map<std::pair<uint32_t, uint32_t>, HullEdge> edgeMap;
    for (uint32_t faceIndex = 0; faceIndex < hull.faces.size(); ++faceIndex)
    {
        const HullFace& face = hull.faces[faceIndex];
        const size_t vertexCount = face.vertexIndices.size();
        if (vertexCount < 3)
            continue;

        for (size_t edgeIndex = 0; edgeIndex < vertexCount; ++edgeIndex)
        {
            const uint32_t a = face.vertexIndices[edgeIndex];
            const uint32_t b = face.vertexIndices[(edgeIndex + 1) % vertexCount];
            const std::pair<uint32_t, uint32_t> edgeKey = (a < b)
                ? std::make_pair(a, b)
                : std::make_pair(b, a);

            auto [edgeIt, inserted] = edgeMap.emplace(edgeKey, HullEdge{});
            if (inserted)
            {
                edgeIt->second.vertexIndices[0] = edgeKey.first;
                edgeIt->second.vertexIndices[1] = edgeKey.second;
                edgeIt->second.faceIndices[0] = faceIndex;
                edgeIt->second.faceIndices[1] = faceIndex;
            }
            else
            {
                edgeIt->second.faceIndices[1] = faceIndex;
            }
        }
    }

    hull.edges.reserve(edgeMap.size());
    for (const auto& [edgeKey, edge] : edgeMap)
    {
        (void)edgeKey;

        if (edge.faceIndices[0] >= hull.faces.size() || edge.faceIndices[1] >= hull.faces.size())
            continue;
        if (edge.faceIndices[0] == edge.faceIndices[1])
            continue;

        hull.edges.push_back(edge);
    }

    return hull;
}

//================================//
Mesh::Mesh(Solver* solver, ModelType modelType, const Eigen::Vector3f& color) : modelType(modelType), color(color), solver(solver)
{
    return;
}

//================================//
Mesh::~Mesh()   
{
    auto forcesCopy = forces;
    for (Force* f : forcesCopy)
    {
        solver->RemoveForce(f);
    }

    auto energiesCopy = energies;
    for (Energy* e : energiesCopy)
    {
        solver->RemoveEnergy(e);
    }
};

//================================//
AABB Mesh::GetWorldAABB() const
{
    AABB localAABB = solver->GetModelAABB(modelType);

    const Eigen::Vector3f localCenter = 0.5f * (localAABB.min + localAABB.max);
    const Eigen::Vector3f localExtent = 0.5f * (localAABB.max - localAABB.min);

    const Eigen::Matrix3f rotationMatrix = transform.GetRotation().toRotationMatrix();
    const Eigen::Matrix3f scaleMatrix = transform.GetScale().asDiagonal();
    const Eigen::Matrix3f linearTransform = rotationMatrix * scaleMatrix;

    const Eigen::Vector3f worldCenter = transform.TransformPoint(localCenter);
    const Eigen::Vector3f worldExtent = linearTransform.cwiseAbs() * localExtent;

    return AABB
    {
        worldCenter - worldExtent,
        worldCenter + worldExtent
    };
}

//================================//
namespace GeometryHelpers
{
    //================================//
    void computeModelGeometry(ModelGeometry& outModelGeometry)
    {
        outModelGeometry.perModelConvexHulls.clear();
        outModelGeometry.perModelLocalAABBs.clear();
        outModelGeometry.perModelMeshData.clear();

        for (ModelType modelType : { ModelType_Cube, ModelType_Sphere, ModelType_TestConvexMesh, ModelType_Capsule })
        {
            MeshData meshData = GeometryGenerator::GenerateMeshDataForModelType(modelType);
            outModelGeometry.perModelMeshData[modelType] = meshData;
            outModelGeometry.perModelLocalAABBs[modelType] = ComputeLocalAABB(meshData);
            outModelGeometry.perModelConvexHulls[modelType] = ComputeConvexHull(meshData);
        }

        return;
    }

    //================================//
    void makeTetVolume(Solver* solver, float ox, float oy, float oz, int nx, int ny, int nz, float cellSize, float E, float nu, const Eigen::Vector3f& color, float particleMass, float friction, bool pinMinX)
    {
        std::vector<Mesh*> parts(nx * ny * nz);

        auto idx = [&](int i, int j, int k)
        {
            return i + j * nx + k * nx * ny;
        };

        for (int k = 0; k < nz; k++)
        {
            for (int j = 0; j < ny; j++)
            {
                for (int i = 0; i < nx; i++)
                {
                    Eigen::Vector3f pos(ox + i * cellSize, oy + j * cellSize, oz + k * cellSize);
                    bool isStatic = pinMinX && (i == 0);
                    Mesh* p = solver->AddParticle(
                        particleMass, friction,
                        pos, Eigen::Vector3f::Zero(),
                        isStatic, color
                    );
                    p->isParticle = true;
                    p->name = "V_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);
                    parts[idx(i, j, k)] = p;
                }
            }
        }

        for (int ci = 0; ci < nx - 1; ci++)
        {
            for (int cj = 0; cj < ny - 1; cj++)
            {
                for (int ck = 0; ck < nz - 1; ck++)
                {
                    Mesh* v[8] = {
                        parts[idx(ci,   cj,   ck  )],  // v0
                        parts[idx(ci+1, cj,   ck  )],  // v1
                        parts[idx(ci,   cj+1, ck  )],  // v2
                        parts[idx(ci+1, cj+1, ck  )],  // v3
                        parts[idx(ci,   cj,   ck+1)],  // v4
                        parts[idx(ci+1, cj,   ck+1)],  // v5
                        parts[idx(ci,   cj+1, ck+1)],  // v6
                        parts[idx(ci+1, cj+1, ck+1)],  // v7
                    };

                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[0], v[1], v[2], v[4], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[3], v[2], v[1], v[7], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[5], v[1], v[4], v[7], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[6], v[4], v[2], v[7], E, nu
                    ));
                    solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
                        solver, v[1], v[2], v[4], v[7], E, nu
                    ));
                }
            }
        }
    }

    //================================//
    void makeTetSphere(Solver* solver, float cx, float cy, float cz, float radius, float res, float E, float nu, const Eigen::Vector3f& color, float mass, float friction)
    {
        int N      = std::max(1, (int)std::roundf(res));
        int nLon   = 6 * N;
        int nLat   = 3 * N;
        int nShells = N;

        int vps    = 2 + nLat * nLon;
        int nVerts = 1 + nShells * vps;
        std::vector<Mesh*> pts(nVerts, nullptr);

        auto sBase  = [&](int s)            { return 1 + s * vps; };
        auto sSouth = [&](int s)            { return sBase(s); };
        auto sNorth = [&](int s)            { return sBase(s) + 1; };
        auto sV     = [&](int s, int j, int i) { return sBase(s) + 2 + j * nLon + ((i % nLon + nLon) % nLon); };

        auto makeP = [&](float x, float y, float z) -> Mesh* 
        {
            Mesh* p = solver->AddParticle(mass, friction, Eigen::Vector3f(x, y, z), Eigen::Vector3f::Zero(), false, color);
            p->isParticle = true;
            return p;
        };

        pts[0] = makeP(cx, cy, cz);

        for (int s = 0; s < nShells; s++)
        {
            float r = radius * (s + 1) / (float)nShells;
            pts[sSouth(s)] = makeP(cx, cy - r, cz);
            pts[sNorth(s)] = makeP(cx, cy + r, cz);
            for (int j = 0; j < nLat; j++)
            {
                float theta = (float)M_PI * (j + 1) / (float)(nLat + 1);
                float sinT  = std::sin(theta);
                float cosT  = std::cos(theta);
                for (int i = 0; i < nLon; i++)
                {
                    float phi = 2.f * (float)M_PI * i / (float)nLon;
                    pts[sV(s, j, i)] = makeP(
                        cx + r * sinT * std::cos(phi),
                        cy + r * cosT,
                        cz + r * sinT * std::sin(phi)
                    );
                }
            }
        }

        auto addTet = [&](Mesh* a, Mesh* b, Mesh* c, Mesh* d)
        {
            Eigen::Matrix3f Dm;
            Dm.col(0) = b->transform.GetPosition() - a->transform.GetPosition();
            Dm.col(1) = c->transform.GetPosition() - a->transform.GetPosition();
            Dm.col(2) = d->transform.GetPosition() - a->transform.GetPosition();
            if (Dm.determinant() < 0.f) std::swap(c, d);
            solver->AddEnergy(std::make_unique<NeoHookeanFEM>(solver, a, b, c, d, E, nu));
        };

        auto addPrism = [&](Mesh* P0, Mesh* P1, Mesh* P2,
                            Mesh* Q0, Mesh* Q1, Mesh* Q2)
        {
            addTet(P0, P1, P2, Q2);
            addTet(P0, Q1, P1, Q2);
            addTet(P0, Q0, Q1, Q2);
        };

        for (int s = 0; s < nShells; s++)
        {
            bool fanToCenter = (s == 0);
            int  sIn         = s - 1;

            for (int i = 0; i < nLon; i++)
            {
                int i1 = (i + 1) % nLon;

                if (fanToCenter)
                    addTet(pts[0], pts[sNorth(s)], pts[sV(s,0,i)], pts[sV(s,0,i1)]);
                else
                    addPrism(pts[sNorth(sIn)], pts[sV(sIn,0,i)], pts[sV(sIn,0,i1)],
                             pts[sNorth(s  )], pts[sV(s,  0,i)], pts[sV(s,  0,i1)]);

                int jS = nLat - 1;
                if (fanToCenter)
                    addTet(pts[0], pts[sSouth(s)], pts[sV(s,jS,i)], pts[sV(s,jS,i1)]);
                else
                    addPrism(pts[sSouth(sIn)], pts[sV(sIn,jS,i)], pts[sV(sIn,jS,i1)],
                             pts[sSouth(s  )], pts[sV(s,  jS,i)], pts[sV(s,  jS,i1)]);
            }

            for (int j = 0; j < nLat - 1; j++)
            {
                for (int i = 0; i < nLon; i++)
                {
                    int i1 = (i + 1) % nLon;
                    if (fanToCenter)
                    {
                        addTet(pts[0], pts[sV(s,j,i)],  pts[sV(s,j,i1)],  pts[sV(s,j+1,i1)]);
                        addTet(pts[0], pts[sV(s,j,i)],  pts[sV(s,j+1,i1)],pts[sV(s,j+1,i )]);
                    }
                    else
                    {
                        addPrism(pts[sV(sIn,j,  i )], pts[sV(sIn,j,  i1)], pts[sV(sIn,j+1,i1)],
                                 pts[sV(s,  j,  i )], pts[sV(s,  j,  i1)], pts[sV(s,  j+1,i1)]);
                        addPrism(pts[sV(sIn,j,  i )], pts[sV(sIn,j+1,i1)], pts[sV(sIn,j+1,i )],
                                 pts[sV(s,  j,  i )], pts[sV(s,  j+1,i1)], pts[sV(s,  j+1,i )]);
                    }
                }
            }
        }
    }

    //================================//
    void makeTet(Solver* solver, float cx, float cy, float cz, float edge, float E, float nu, const Eigen::Vector3f& color, float mass, float friction)
    {
        float a = edge;
        float inv_sqrt3  = 1.f / std::sqrt(3.f);
        float inv_sqrt6  = 1.f / std::sqrt(6.f);
        float sqrt_2_3   = std::sqrt(2.f / 3.f);

        Eigen::Vector3f pyramidVertices[4] = 
        {
            { 0.f,            0.f,              sqrt_2_3 * a },
            { -0.5f * a,     -0.5f * inv_sqrt3 * a,  -inv_sqrt6 * a },
            {  0.5f * a,     -0.5f * inv_sqrt3 * a,  -inv_sqrt6 * a },
            {  0.f,           inv_sqrt3 * a,          -inv_sqrt6 * a },
        };

        Eigen::Vector3f centroid = (pyramidVertices[0] + pyramidVertices[1] + pyramidVertices[2] + pyramidVertices[3]) / 4.f;
        Eigen::Vector3f offset(cx, cy, cz);
        for (int i = 0; i < 4; ++i)
            pyramidVertices[i] += offset - centroid;

        Mesh* particles[4];
        for (int i = 0; i < 4; ++i)
        {
            particles[i] = solver->AddParticle(
                mass, friction,
                pyramidVertices[i], Eigen::Vector3f::Zero(),
                false, color
            );
            particles[i]->isParticle = true;
            particles[i]->name = "Tet_" + std::to_string((int)E) + "_" + std::to_string(i);
        }

        solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
            solver,
            particles[0], particles[1], particles[2], particles[3],
            E, nu
        ));
    };
}
