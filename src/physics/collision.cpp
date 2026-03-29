#include "collision.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include "solver.hpp"

//================================//
namespace CollisionSpace
{
    static constexpr float FACE_CONTACT_EDGE_BIAS = 0.005f;

    //================================//
    static Eigen::Vector3f TransformNormalToWorld(const Transform& transform, const Eigen::Vector3f& localNormal)
    {
        const Eigen::Matrix3f linear = transform.GetRotation().toRotationMatrix() * transform.GetScale().asDiagonal();
        Eigen::Vector3f worldNormal = linear.inverse().transpose() * localNormal;
        const float lengthSq = worldNormal.squaredNorm();
        if (lengthSq <= 1e-12f)
            return Eigen::Vector3f::Zero();

        return worldNormal / std::sqrt(lengthSq);
    }

    //================================//
    static Eigen::Vector3f ToLocalOffset(const Transform& transform, const Eigen::Vector3f& worldPoint)
    {
        return transform.GetRotation().toRotationMatrix().transpose() * (worldPoint - transform.GetPosition());
    }

    //================================//
    static bool IsMinkowskiFace(Eigen::Vector3f normalA0, Eigen::Vector3f normalA1, const Eigen::Vector3f& edgeDirectionA,
                                Eigen::Vector3f normalB0, Eigen::Vector3f normalB1, const Eigen::Vector3f& edgeDirectionB)
    {
        if ((normalA1.cross(normalA0)).dot(edgeDirectionA) < 0.0f)
            std::swap(normalA0, normalA1);

        if ((normalB1.cross(normalB0)).dot(edgeDirectionB) < 0.0f)
            std::swap(normalB0, normalB1);

        const float cba = normalB0.dot(normalA1.cross(normalA0));
        const float dba = normalB1.dot(normalA1.cross(normalA0));
        const float adc = normalA0.dot(normalB1.cross(normalB0));
        const float bdc = normalA1.dot(normalB1.cross(normalB0));

        return (cba * dba < 0.0f) &&
               (adc * bdc < 0.0f) &&
               (cba * bdc > 0.0f);
    }

    //================================//
    static std::vector<Eigen::Vector3f> ClipPolygonAgainstPlane(const std::vector<Eigen::Vector3f>& polygon, const Eigen::Vector3f& planeNormal, float planeOffset)
    {
        std::vector<Eigen::Vector3f> clipped;
        if (polygon.empty())
            return clipped;

        clipped.reserve(polygon.size() + 1);

        const int count = static_cast<int>(polygon.size());
        for (int i = 0; i < count; ++i)
        {
            const Eigen::Vector3f& a = polygon[i];
            const Eigen::Vector3f& b = polygon[(i + 1) % count];

            const float da = planeNormal.dot(a) - planeOffset;
            const float db = planeNormal.dot(b) - planeOffset;

            const bool insideA = da <= 0.0f;
            const bool insideB = db <= 0.0f;

            if (insideA)
                clipped.push_back(a);

            if (insideA != insideB)
            {
                const float t = da / (da - db);
                clipped.push_back(a + t * (b - a));
            }
        }

        return clipped;
    }

    //================================//
    static void ClosestPointsOnSegments(const Eigen::Vector3f& a0, const Eigen::Vector3f& a1,
                                        const Eigen::Vector3f& b0, const Eigen::Vector3f& b1,
                                        Eigen::Vector3f& pointA, Eigen::Vector3f& pointB)
    {
        const Eigen::Vector3f d1 = a1 - a0;
        const Eigen::Vector3f d2 = b1 - b0;
        const Eigen::Vector3f r = a0 - b0;

        const float a = d1.dot(d1);
        const float e = d2.dot(d2);
        const float f = d2.dot(r);

        float s = 0.0f;
        float t = 0.0f;

        if (a <= 1e-12f && e <= 1e-12f)
        {
            pointA = a0;
            pointB = b0;
            return;
        }

        if (a <= 1e-12f)
        {
            t = std::clamp(f / e, 0.0f, 1.0f);
        }
        else
        {
            const float c = d1.dot(r);
            if (e <= 1e-12f)
            {
                s = std::clamp(-c / a, 0.0f, 1.0f);
            }
            else
            {
                const float b = d1.dot(d2);
                const float denom = a * e - b * b;

                if (denom > 1e-12f)
                    s = std::clamp((b * f - c * e) / denom, 0.0f, 1.0f);

                t = (b * s + f) / e;

                if (t < 0.0f)
                {
                    t = 0.0f;
                    s = std::clamp(-c / a, 0.0f, 1.0f);
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;
                    s = std::clamp((b - c) / a, 0.0f, 1.0f);
                }
            }
        }

        pointA = a0 + d1 * s;
        pointB = b0 + d2 * t;
    }

    //================================//
    static void CreateFaceContact(const FaceQuery& faceQuery, const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB, CollisionResult& result);
    static void CreateEdgeContact(const EdgeQuery& edgeQuery, const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB, CollisionResult& result);
    static FaceQuery QueryFaceDirections(const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB);
    static EdgeQuery QueryEdgeDirections(const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB);

    //================================//
    CollisionResult CollisionHullHull(const Mesh* A, const Mesh* B)
    {
        CollisionResult result;
        result.numContacts = 0;

        const ConvexHull& hullA = A->solver->GetModelConvexHull(A->modelType);
        const ConvexHull& hullB = B->solver->GetModelConvexHull(B->modelType);
        if (hullA.vertexCount() == 0 || hullB.vertexCount() == 0 ||
            hullA.faceCount() == 0 || hullB.faceCount() == 0)
        {
            return result;
        }

        const Transform& transformA = A->transform;
        const Transform& transformB = B->transform;

        FaceQuery faceQueryA = QueryFaceDirections(transformA, hullA, transformB, hullB);
        if (faceQueryA.separation > 0.0)
            return result;

        FaceQuery faceQueryB = QueryFaceDirections(transformB, hullB, transformA, hullA);
        if (faceQueryB.separation > 0.0)
            return result;

        EdgeQuery edgeQuery = QueryEdgeDirections(transformA, hullA, transformB, hullB);
        if (edgeQuery.separation > 0.0)
            return result;

        // HULL INTERSECTION
        bool isFaceContactA = faceQueryA.separation >= edgeQuery.separation;
        bool isFaceContactB = faceQueryB.separation >= edgeQuery.separation;

        if (isFaceContactA && faceQueryA.separation >= faceQueryB.separation - FACE_CONTACT_EDGE_BIAS)
        {
            CreateFaceContact(faceQueryA, transformA, hullA, transformB, hullB, result);
        }
        else if (isFaceContactB && faceQueryB.separation >= faceQueryA.separation - FACE_CONTACT_EDGE_BIAS)
        {
            CreateFaceContact(faceQueryB, transformB, hullB, transformA, hullA, result);

            for (int i = 0; i < result.numContacts; ++i)
            {
                std::swap(result.contactPoints[i].rA, result.contactPoints[i].rB);
                result.contactPoints[i].normal = -result.contactPoints[i].normal;
            }
        }
        else
        {
            CreateEdgeContact(edgeQuery, transformA, hullA, transformB, hullB, result);
        }

        return result;
    }

    //================================//
    static FaceQuery QueryFaceDirections(const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB)
    {
        FaceQuery query;
        query.separation = -INFINITY;
        query.faceIndex = 0;

        if (hullA.faceCount() == 0 || hullB.vertexCount() == 0)
        {
            query.separation = std::numeric_limits<double>::infinity();
            return query;
        }

        const Eigen::Matrix3f rotationA = transformA.GetRotation().toRotationMatrix();
        const Eigen::Matrix3f rotationB = transformB.GetRotation().toRotationMatrix();

        const Eigen::Matrix3f linearA = rotationA * transformA.GetScale().asDiagonal();
        const Eigen::Matrix3f linearB = rotationB * transformB.GetScale().asDiagonal();

        const Eigen::Matrix3f normalMatrixA = linearA.inverse().transpose();

        int N = hullA.faceCount();
        for (int i = 0; i < N; i++)
        {
            const HullFace& faceA = hullA.faces[i];
            if (faceA.vertexIndices.empty() || faceA.vertexIndices[0] >= hullA.vertices.size())
                continue;

            Eigen::Vector3f planeNormal = normalMatrixA * faceA.normal;
            const float normalLengthSq = planeNormal.squaredNorm();
            if (normalLengthSq <= 1e-12f)
                continue;

            planeNormal /= std::sqrt(normalLengthSq);

            const Eigen::Vector3f planePoint = transformA.TransformPoint(hullA.vertices[faceA.vertexIndices[0]].position);

            const Eigen::Vector3f supportDirectionLocalB = linearB.transpose() * (-planeNormal);
            const Eigen::Vector3f supportPointLocalB = hullB.GetSupport(supportDirectionLocalB);
            const Eigen::Vector3f supportPointWorldB = transformB.TransformPoint(supportPointLocalB);

            const double separation = static_cast<double>(planeNormal.dot(supportPointWorldB - planePoint));
            if (separation > query.separation)
            {
                query.separation = separation;
                query.faceIndex = static_cast<uint16_t>(i);
            }
        }

        return query;
    }

    //================================//
    static EdgeQuery QueryEdgeDirections(const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB)
    {
        EdgeQuery query;
        query.separation = -INFINITY;
        query.edgeIndexA = 0;
        query.edgeIndexB = 0;

        if (hullA.edgeCount() == 0 || hullB.edgeCount() == 0 ||
            hullA.vertexCount() == 0 || hullB.vertexCount() == 0)
        {
            query.separation = std::numeric_limits<double>::infinity();
            return query;
        }

        int edgeCountA = hullA.edgeCount();
        int edgeCountB = hullB.edgeCount();

        const Eigen::Matrix3f rotationA = transformA.GetRotation().toRotationMatrix();
        const Eigen::Matrix3f rotationB = transformB.GetRotation().toRotationMatrix();

        const Eigen::Matrix3f linearA = rotationA * transformA.GetScale().asDiagonal();
        const Eigen::Matrix3f linearB = rotationB * transformB.GetScale().asDiagonal();

        const Eigen::Vector3f hullCenterWorldA = transformA.TransformPoint(hullA.centroid);

        for(int ecA = 0; ecA < edgeCountA; ecA++)
        {
            const HullEdge& edgeA = hullA.edges[ecA];
            if (edgeA.vertexIndices[0] >= hullA.vertices.size() || edgeA.vertexIndices[1] >= hullA.vertices.size())
                continue;
            if (edgeA.faceIndices[0] >= hullA.faces.size() || edgeA.faceIndices[1] >= hullA.faces.size())
                continue;

            const Eigen::Vector3f edgeAOriginWorld = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[0]].position);
            const Eigen::Vector3f edgeAEndWorld = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[1]].position);
            const Eigen::Vector3f edgeADirectionWorld = edgeAEndWorld - edgeAOriginWorld;

            if (edgeADirectionWorld.squaredNorm() <= 1e-12f)
                continue;

            const Eigen::Vector3f edgeAFaceNormal0 = TransformNormalToWorld(transformA, hullA.faces[edgeA.faceIndices[0]].normal);
            const Eigen::Vector3f edgeAFaceNormal1 = TransformNormalToWorld(transformA, hullA.faces[edgeA.faceIndices[1]].normal);

            for (int ecB = 0; ecB < edgeCountB; ecB++)
            {
                const HullEdge& edgeB = hullB.edges[ecB];
                if (edgeB.vertexIndices[0] >= hullB.vertices.size() || edgeB.vertexIndices[1] >= hullB.vertices.size())
                    continue;
                if (edgeB.faceIndices[0] >= hullB.faces.size() || edgeB.faceIndices[1] >= hullB.faces.size())
                    continue;

                const Eigen::Vector3f edgeBOriginWorld = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[0]].position);
                const Eigen::Vector3f edgeBEndWorld = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[1]].position);
                const Eigen::Vector3f edgeBDirectionWorld = edgeBEndWorld - edgeBOriginWorld;

                if (edgeBDirectionWorld.squaredNorm() <= 1e-12f)
                    continue;

                const Eigen::Vector3f edgeBFaceNormal0 = TransformNormalToWorld(transformB, hullB.faces[edgeB.faceIndices[0]].normal);
                const Eigen::Vector3f edgeBFaceNormal1 = TransformNormalToWorld(transformB, hullB.faces[edgeB.faceIndices[1]].normal);

                if (!IsMinkowskiFace(edgeAFaceNormal0, edgeAFaceNormal1, edgeADirectionWorld,
                                     edgeBFaceNormal0, edgeBFaceNormal1, edgeBDirectionWorld))
                {
                    continue;
                }

                Eigen::Vector3f axis = edgeADirectionWorld.cross(edgeBDirectionWorld);
                const float axisLengthSq = axis.squaredNorm();
                if (axisLengthSq <= 1e-12f)
                    continue;

                axis /= std::sqrt(axisLengthSq);

                if (axis.dot(edgeAOriginWorld - hullCenterWorldA) < 0.0f)
                    axis = -axis;

                const Eigen::Vector3f supportDirectionLocalB = linearB.transpose() * (-axis);
                const Eigen::Vector3f supportPointLocalB = hullB.GetSupport(supportDirectionLocalB);
                const Eigen::Vector3f supportPointWorldB = transformB.TransformPoint(supportPointLocalB);

                const double separation = static_cast<double>(axis.dot(supportPointWorldB - edgeAOriginWorld));
                if (separation > query.separation)
                {
                    query.separation = separation;
                    query.edgeIndexA = static_cast<uint16_t>(ecA);
                    query.edgeIndexB = static_cast<uint16_t>(ecB);
                }
            }
        }

        return query;
    }

    //================================//
    static void CreateFaceContact(const FaceQuery& faceQuery, const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB, CollisionResult& result)
    {
        result.numContacts = 0;
        if (faceQuery.faceIndex >= hullA.faceCount())
            return;

        const HullFace& referenceFace = hullA.faces[faceQuery.faceIndex];
        const Eigen::Vector3f referenceNormal = TransformNormalToWorld(transformA, referenceFace.normal);
        if (referenceNormal.squaredNorm() <= 1e-12f)
            return;

        int incidentFaceIndex = -1;
        float minDot = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(hullB.faceCount()); i++)
        {
            const Eigen::Vector3f incidentNormal = TransformNormalToWorld(transformB, hullB.faces[i].normal);
            const float d = referenceNormal.dot(incidentNormal);
            if (d < minDot)
            {
                minDot = d;
                incidentFaceIndex = i;
            }
        }

        if (incidentFaceIndex < 0)
            return;

        std::vector<Eigen::Vector3f> incidentPolygon;
        incidentPolygon.reserve(3);
        const HullFace& incidentFace = hullB.faces[incidentFaceIndex];
        incidentPolygon.reserve(incidentFace.vertexIndices.size());
        for (size_t j = 0; j < incidentFace.vertexIndices.size(); ++j)
        {
            if (incidentFace.vertexIndices[j] >= hullB.vertices.size())
                return;

            incidentPolygon.push_back(transformB.TransformPoint(hullB.vertices[incidentFace.vertexIndices[j]].position));
        }

        std::vector<Eigen::Vector3f> referenceVertices(referenceFace.vertexIndices.size());
        if (referenceVertices.size() < 3)
            return;

        for (size_t j = 0; j < referenceFace.vertexIndices.size(); ++j)
        {
            if (referenceFace.vertexIndices[j] >= hullA.vertices.size())
                return;

            referenceVertices[j] = transformA.TransformPoint(hullA.vertices[referenceFace.vertexIndices[j]].position);
        }

        for (size_t j = 0; j < referenceVertices.size(); ++j)
        {
            const Eigen::Vector3f edge = referenceVertices[(j + 1) % referenceVertices.size()] - referenceVertices[j];
            const Eigen::Vector3f sidePlaneNormal = edge.cross(referenceNormal);
            const float sidePlaneOffset = sidePlaneNormal.dot(referenceVertices[j]);
            incidentPolygon = ClipPolygonAgainstPlane(incidentPolygon, sidePlaneNormal, sidePlaneOffset);
            if (incidentPolygon.empty())
                return;
        }

        const float referencePlaneOffset = referenceNormal.dot(referenceVertices[0]);
        for (int i = 0; i < static_cast<int>(incidentPolygon.size()) && result.numContacts < 8; i++)
        {
            const Eigen::Vector3f& point = incidentPolygon[i];
            const float separation = referenceNormal.dot(point) - referencePlaneOffset;
            if (separation > 0.0f)
                continue;

            const Eigen::Vector3f projectedPoint = point - separation * referenceNormal;

            bool isDuplicate = false;
            for (int contactIndex = 0; contactIndex < result.numContacts; ++contactIndex)
            {
                if ((result.contactPoints[contactIndex].position - projectedPoint).squaredNorm() <= 1e-10f)
                {
                    isDuplicate = true;
                    break;
                }
            }
            if (isDuplicate)
                continue;

            ContactPoint& contact = result.contactPoints[result.numContacts];
            contact.position = projectedPoint;
            contact.normal = referenceNormal;
            contact.penetration = -separation;
            contact.rA = ToLocalOffset(transformA, projectedPoint);
            contact.rB = ToLocalOffset(transformB, point);
            contact.id = (static_cast<uint32_t>(faceQuery.faceIndex) << 8) | static_cast<uint32_t>(i);
            result.numContacts++;
        }
    }

    //================================//
    static void CreateEdgeContact(const EdgeQuery& edgeQuery, const Transform& transformA, const ConvexHull& hullA, const Transform& transformB, const ConvexHull& hullB, CollisionResult& result)
    {  
        result.numContacts = 0;
        if (edgeQuery.edgeIndexA >= hullA.edges.size() || edgeQuery.edgeIndexB >= hullB.edges.size())
            return;

        const HullEdge& edgeA = hullA.edges[edgeQuery.edgeIndexA];
        const HullEdge& edgeB = hullB.edges[edgeQuery.edgeIndexB];
        if (edgeA.vertexIndices[0] >= hullA.vertices.size() || edgeA.vertexIndices[1] >= hullA.vertices.size())
            return;
        if (edgeB.vertexIndices[0] >= hullB.vertices.size() || edgeB.vertexIndices[1] >= hullB.vertices.size())
            return;

        const Eigen::Vector3f a0 = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[0]].position);
        const Eigen::Vector3f a1 = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[1]].position);
        const Eigen::Vector3f b0 = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[0]].position);
        const Eigen::Vector3f b1 = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[1]].position);

        Eigen::Vector3f axis = (a1 - a0).cross(b1 - b0);
        const float axisLengthSq = axis.squaredNorm();
        if (axisLengthSq <= 1e-12f)
            return;

        axis /= std::sqrt(axisLengthSq);

        const Eigen::Vector3f hullCenterWorldA = transformA.TransformPoint(hullA.centroid);
        if (axis.dot(a0 - hullCenterWorldA) < 0.0f)
            axis = -axis;

        Eigen::Vector3f pointA, pointB;
        ClosestPointsOnSegments(a0, a1, b0, b1, pointA, pointB);

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointA + pointB);
        contact.normal = axis;
        contact.penetration = std::max(0.0f, -axis.dot(pointB - pointA));
        contact.rA = ToLocalOffset(transformA, pointA);
        contact.rB = ToLocalOffset(transformB, pointB);
        contact.id =
            (1u << 31) |
            (static_cast<uint32_t>(edgeQuery.edgeIndexA) << 16) |
            static_cast<uint32_t>(edgeQuery.edgeIndexB);

        result.numContacts = 1;
    }
}
