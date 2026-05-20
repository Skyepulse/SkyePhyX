#include "sat.hpp"
#include "support.hpp"
#include "../solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

//================================//
namespace CollisionSpace
{
    static constexpr float SEPARATING_AXIS_RELATIVE_TOLERANCE = 1.002f;
    static constexpr float SEPARATING_AXIS_ABSOLUTE_TOLERANCE = 0.0005f;
    //================================//
    struct CapsuleFaceQuery
    {
        double separation = -std::numeric_limits<double>::infinity();
        uint16_t faceIndex = 0;
    };

    //================================//
    struct CapsuleEdgeQuery
    {
        double separation = -std::numeric_limits<double>::infinity();
        uint16_t edgeIndex = 0;
    };

    //================================//
    static bool ShouldPreferEdgeAxis(double edgeSeparation, double bestFaceSeparation, double shapeMargin)
    {
        const double edgePenetration = shapeMargin - edgeSeparation;
        const double facePenetration = shapeMargin - bestFaceSeparation;

        // ReactPhysics3D compares positive penetration depths here.  We store
        // signed separations, so convert before applying the same bias.
        return edgePenetration * SEPARATING_AXIS_RELATIVE_TOLERANCE +
               SEPARATING_AXIS_ABSOLUTE_TOLERANCE < facePenetration;
    }

    //================================//
    static std::vector<Eigen::Vector3f> ClipPolygonAgainstPlane(const std::vector<Eigen::Vector3f>& polygon,
                                                                const Eigen::Vector3f& planeNormal,
                                                                float planeOffset)
    {
        std::vector<Eigen::Vector3f> clipped;
        if (polygon.empty())
            return clipped;

        clipped.reserve(polygon.size() + 1);

        const int count = static_cast<int>(polygon.size());
        for (int i = 0; i < count; ++i)
        {
            const Eigen::Vector3f pointA = polygon[i];
            const Eigen::Vector3f pointB = polygon[(i + 1) % count];

            const float distanceA = planeNormal.dot(pointA) - planeOffset;
            const float distanceB = planeNormal.dot(pointB) - planeOffset;

            const bool insideA = distanceA <= 0.0f;
            const bool insideB = distanceB <= 0.0f;

            if (insideA)
                clipped.push_back(pointA);

            if (insideA != insideB)
            {
                const float t = distanceA / (distanceA - distanceB);
                clipped.push_back(pointA + t * (pointB - pointA));
            }
        }

        return clipped;
    }

    //================================//
    static bool IsMinkowskiFace(Eigen::Vector3f normalA0,
                                Eigen::Vector3f normalA1,
                                const Eigen::Vector3f& edgeDirectionA,
                                Eigen::Vector3f normalB0,
                                Eigen::Vector3f normalB1,
                                const Eigen::Vector3f& edgeDirectionB)
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
    static FaceQuery QueryHullFaceDirections(const Transform& transformA,
                                             const ConvexHull& hullA,
                                             const Transform& transformB,
                                             const ConvexHull& hullB)
    {
        FaceQuery query;
        query.separation = -std::numeric_limits<double>::infinity();
        query.faceIndex = 0;

        // Empty hull shape
        if (hullA.faceCount() == 0 || hullB.vertexCount() == 0)
        {
            query.separation = std::numeric_limits<double>::infinity();
            return query;
        }

        const Eigen::Matrix3f linearB = transformB.GetRotation().toRotationMatrix() * transformB.GetScale().asDiagonal();
        for (int faceIndex = 0; faceIndex < static_cast<int>(hullA.faceCount()); ++faceIndex)
        {
            const HullFace& faceA = hullA.faces[faceIndex];
            if (faceA.vertexIndices.empty() || faceA.vertexIndices[0] >= hullA.vertices.size())
                continue;

            const Eigen::Vector3f planeNormal = TransformNormalToWorld(transformA, faceA.normal);
            if (planeNormal.squaredNorm() <= 1e-12f)
                continue;

            const Eigen::Vector3f planePoint = transformA.TransformPoint(hullA.vertices[faceA.vertexIndices[0]].position);
            const Eigen::Vector3f supportDirectionLocalB = linearB.transpose() * (-planeNormal);
            const Eigen::Vector3f supportPointLocalB = hullB.GetSupport(supportDirectionLocalB);
            const Eigen::Vector3f supportPointWorldB = transformB.TransformPoint(supportPointLocalB);

            const double separation = static_cast<double>(planeNormal.dot(supportPointWorldB - planePoint));

            if (separation > query.separation)
            {
                query.separation = separation;
                query.faceIndex = static_cast<uint16_t>(faceIndex);
            }
        }

        return query;
    }

    //================================//
    static EdgeQuery QueryHullEdgeDirections(const Transform& transformA,
                                             const ConvexHull& hullA,
                                             const Transform& transformB,
                                             const ConvexHull& hullB)
    {
        EdgeQuery query;
        query.separation = -std::numeric_limits<double>::infinity();
        query.edgeIndexA = 0;
        query.edgeIndexB = 0;

        if (hullA.edgeCount() == 0 || hullB.edgeCount() == 0)
        {
            query.separation = std::numeric_limits<double>::infinity();
            return query;
        }

        const Eigen::Matrix3f linearB = transformB.GetRotation().toRotationMatrix() * transformB.GetScale().asDiagonal();
        const Eigen::Vector3f hullCenterWorldA = transformA.TransformPoint(hullA.centroid);

        for (int edgeIndexA = 0; edgeIndexA < static_cast<int>(hullA.edgeCount()); ++edgeIndexA)
        {
            const HullEdge& edgeA = hullA.edges[edgeIndexA];
            if (edgeA.vertexIndices[0] >= hullA.vertices.size() || edgeA.vertexIndices[1] >= hullA.vertices.size())
                continue;
            if (edgeA.faceIndices[0] >= hullA.faces.size() || edgeA.faceIndices[1] >= hullA.faces.size())
                continue;

            const Eigen::Vector3f edgeAStart = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[0]].position);
            const Eigen::Vector3f edgeAEnd = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[1]].position);
            const Eigen::Vector3f edgeDirectionA = edgeAEnd - edgeAStart;
            if (edgeDirectionA.squaredNorm() <= 1e-12f)
                continue;

            const Eigen::Vector3f normalA0 = TransformNormalToWorld(transformA, hullA.faces[edgeA.faceIndices[0]].normal);
            const Eigen::Vector3f normalA1 = TransformNormalToWorld(transformA, hullA.faces[edgeA.faceIndices[1]].normal);

            for (int edgeIndexB = 0; edgeIndexB < static_cast<int>(hullB.edgeCount()); ++edgeIndexB)
            {
                const HullEdge& edgeB = hullB.edges[edgeIndexB];
                if (edgeB.vertexIndices[0] >= hullB.vertices.size() || edgeB.vertexIndices[1] >= hullB.vertices.size())
                    continue;
                if (edgeB.faceIndices[0] >= hullB.faces.size() || edgeB.faceIndices[1] >= hullB.faces.size())
                    continue;

                const Eigen::Vector3f edgeBStart = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[0]].position);
                const Eigen::Vector3f edgeBEnd = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[1]].position);
                const Eigen::Vector3f edgeDirectionB = edgeBEnd - edgeBStart;
                if (edgeDirectionB.squaredNorm() <= 1e-12f)
                    continue;

                const Eigen::Vector3f normalB0 = TransformNormalToWorld(transformB, hullB.faces[edgeB.faceIndices[0]].normal);
                const Eigen::Vector3f normalB1 = TransformNormalToWorld(transformB, hullB.faces[edgeB.faceIndices[1]].normal);

                // A Minkowski face is a pair of edges from the two hulls such that the cross product of their 
                // directions is a separating axis. 
                // If the cross product is not a separating axis, we can skip this pair of edges.
                if (!IsMinkowskiFace(normalA0, normalA1, edgeDirectionA, normalB0, normalB1, edgeDirectionB))
                    continue;

                Eigen::Vector3f axis = edgeDirectionA.cross(edgeDirectionB);
                const float axisLengthSquared = axis.squaredNorm();
                if (axisLengthSquared <= 1e-12f)
                    continue;

                axis /= std::sqrt(axisLengthSquared);
                if (axis.dot(edgeAStart - hullCenterWorldA) < 0.0f)
                    axis = -axis;

                const Eigen::Vector3f supportDirectionLocalB = linearB.transpose() * (-axis);
                const Eigen::Vector3f supportPointLocalB = hullB.GetSupport(supportDirectionLocalB);
                const Eigen::Vector3f supportPointWorldB = transformB.TransformPoint(supportPointLocalB);
                const double separation = static_cast<double>(axis.dot(supportPointWorldB - edgeAStart));

                if (separation > query.separation)
                {
                    query.separation = separation;
                    query.edgeIndexA = static_cast<uint16_t>(edgeIndexA);
                    query.edgeIndexB = static_cast<uint16_t>(edgeIndexB);
                }
            }
        }

        return query;
    }

    //================================//
    static void CreateHullFaceContacts(const Mesh* meshA,
                                       const ConvexHull& hullA,
                                       const Mesh* meshB,
                                       const ConvexHull& hullB,
                                       const FaceQuery& faceQuery,
                                       CollisionResult& result)
    {
        result.numContacts = 0;
        if (faceQuery.faceIndex >= hullA.faceCount())
            return;

        const Transform& transformA = meshA->transform;
        const Transform& transformB = meshB->transform;
        const HullFace& referenceFace = hullA.faces[faceQuery.faceIndex];
        const Eigen::Vector3f referenceNormal = TransformNormalToWorld(transformA, referenceFace.normal);
        if (referenceNormal.squaredNorm() <= 1e-12f)
            return;

        int incidentFaceIndex = -1;
        float minDot = std::numeric_limits<float>::max();
        for (int faceIndex = 0; faceIndex < static_cast<int>(hullB.faceCount()); ++faceIndex)
        {
            const Eigen::Vector3f incidentNormal = TransformNormalToWorld(transformB, hullB.faces[faceIndex].normal);
            const float dot = referenceNormal.dot(incidentNormal);
            if (dot < minDot)
            {
                minDot = dot;
                incidentFaceIndex = faceIndex;
            }
        }

        if (incidentFaceIndex < 0)
            return;

        std::vector<Eigen::Vector3f> incidentPolygon;
        const HullFace& incidentFace = hullB.faces[incidentFaceIndex];
        incidentPolygon.reserve(incidentFace.vertexIndices.size());

        for (int i = 0; i < static_cast<int>(incidentFace.vertexIndices.size()); ++i)
        {
            if (incidentFace.vertexIndices[i] >= hullB.vertices.size())
                return;

            incidentPolygon.push_back(transformB.TransformPoint(hullB.vertices[incidentFace.vertexIndices[i]].position));
        }

        std::vector<Eigen::Vector3f> referenceVertices(referenceFace.vertexIndices.size());
        if (referenceVertices.size() < 3)
            return;

        for (int i = 0; i < static_cast<int>(referenceFace.vertexIndices.size()); ++i)
        {
            if (referenceFace.vertexIndices[i] >= hullA.vertices.size())
                return;

            referenceVertices[i] = transformA.TransformPoint(hullA.vertices[referenceFace.vertexIndices[i]].position);
        }

        for (int i = 0; i < static_cast<int>(referenceVertices.size()); ++i)
        {
            const Eigen::Vector3f edge = referenceVertices[(i + 1) % referenceVertices.size()] - referenceVertices[i];
            const Eigen::Vector3f sidePlaneNormal = edge.cross(referenceNormal);
            const float sidePlaneOffset = sidePlaneNormal.dot(referenceVertices[i]);

            incidentPolygon = ClipPolygonAgainstPlane(incidentPolygon, sidePlaneNormal, sidePlaneOffset);
            if (incidentPolygon.empty())
                return;
        }

        const float referencePlaneOffset = referenceNormal.dot(referenceVertices[0]);
        for (int i = 0; i < static_cast<int>(incidentPolygon.size()) && result.numContacts < 8; ++i)
        {
            const Eigen::Vector3f incidentPoint = incidentPolygon[i];
            const float separation = referenceNormal.dot(incidentPoint) - referencePlaneOffset;
            if (separation > 0.0f)
                continue;

            const Eigen::Vector3f referencePoint = incidentPoint - separation * referenceNormal;

            bool duplicate = false;
            for (int contactIndex = 0; contactIndex < result.numContacts; ++contactIndex)
            {
                if ((result.contactPoints[contactIndex].position - referencePoint).squaredNorm() <= 1e-10f)
                {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate)
                continue;

            ContactPoint& contact = result.contactPoints[result.numContacts];
            contact.position = 0.5f * (referencePoint + incidentPoint);
            contact.normal = referenceNormal;
            contact.penetration = -separation;
            contact.rA = ToLocalOffset(meshA, referencePoint);
            contact.rB = ToLocalOffset(meshB, incidentPoint);
            contact.id = MakeContactID(1u, faceQuery.faceIndex, static_cast<uint32_t>(incidentFaceIndex), static_cast<uint32_t>(i));

            result.numContacts++;
        }
    }

    //================================//
    static void CreateHullEdgeContact(const Mesh* meshA,
                                      const ConvexHull& hullA,
                                      const Mesh* meshB,
                                      const ConvexHull& hullB,
                                      const EdgeQuery& edgeQuery,
                                      CollisionResult& result)
    {
        result.numContacts = 0;
        if (edgeQuery.edgeIndexA >= hullA.edgeCount() || edgeQuery.edgeIndexB >= hullB.edgeCount())
            return;

        const Transform& transformA = meshA->transform;
        const Transform& transformB = meshB->transform;
        const HullEdge& edgeA = hullA.edges[edgeQuery.edgeIndexA];
        const HullEdge& edgeB = hullB.edges[edgeQuery.edgeIndexB];

        if (edgeA.vertexIndices[0] >= hullA.vertices.size() || edgeA.vertexIndices[1] >= hullA.vertices.size())
            return;
        if (edgeB.vertexIndices[0] >= hullB.vertices.size() || edgeB.vertexIndices[1] >= hullB.vertices.size())
            return;

        const Eigen::Vector3f edgeAStart = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[0]].position);
        const Eigen::Vector3f edgeAEnd = transformA.TransformPoint(hullA.vertices[edgeA.vertexIndices[1]].position);
        const Eigen::Vector3f edgeBStart = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[0]].position);
        const Eigen::Vector3f edgeBEnd = transformB.TransformPoint(hullB.vertices[edgeB.vertexIndices[1]].position);

        Eigen::Vector3f normal = (edgeAEnd - edgeAStart).cross(edgeBEnd - edgeBStart);
        const float normalLengthSquared = normal.squaredNorm();
        if (normalLengthSquared <= 1e-12f)
            return;

        normal /= std::sqrt(normalLengthSquared);
        if (normal.dot(edgeAStart - transformA.TransformPoint(hullA.centroid)) < 0.0f)
            normal = -normal;

        Eigen::Vector3f pointA = Eigen::Vector3f::Zero();
        Eigen::Vector3f pointB = Eigen::Vector3f::Zero();
        ClosestPointsOnSegments(edgeAStart, edgeAEnd, edgeBStart, edgeBEnd, pointA, pointB);

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointA + pointB);
        contact.normal = normal;
        contact.penetration = std::max(0.0f, -normal.dot(pointB - pointA));
        contact.rA = ToLocalOffset(meshA, pointA);
        contact.rB = ToLocalOffset(meshB, pointB);
        contact.id = MakeContactID(2u, edgeQuery.edgeIndexA, edgeQuery.edgeIndexB, 0u);

        result.numContacts = 1;
    }

    //================================//
    static CapsuleFaceQuery QueryCapsuleFaceDirections(const Mesh* hullMesh,
                                                       const ConvexHull& hull,
                                                       const CapsuleSegment& capsule)
    {
        CapsuleFaceQuery query;

        for (int faceIndex = 0; faceIndex < static_cast<int>(hull.faceCount()); ++faceIndex)
        {
            const HullFace& face = hull.faces[faceIndex];
            if (face.vertexIndices.empty() || face.vertexIndices[0] >= hull.vertices.size())
                continue;

            const Eigen::Vector3f faceNormal = TransformNormalToWorld(hullMesh->transform, face.normal);
            if (faceNormal.squaredNorm() <= 1e-12f)
                continue;

            const Eigen::Vector3f planePoint = hullMesh->transform.TransformPoint(hull.vertices[face.vertexIndices[0]].position);
            const Eigen::Vector3f capsuleSupport = SupportSegment(capsule, -faceNormal);
            const double separation = static_cast<double>(faceNormal.dot(capsuleSupport - planePoint));

            if (separation > query.separation)
            {
                query.separation = separation;
                query.faceIndex = static_cast<uint16_t>(faceIndex);
            }
        }

        return query;
    }

    //================================//
    static CapsuleEdgeQuery QueryCapsuleEdgeDirections(const Mesh* hullMesh,
                                                       const ConvexHull& hull,
                                                       const CapsuleSegment& capsule)
    {
        CapsuleEdgeQuery query;
        const Eigen::Vector3f hullCenter = hullMesh->transform.TransformPoint(hull.centroid);

        for (int edgeIndex = 0; edgeIndex < static_cast<int>(hull.edgeCount()); ++edgeIndex)
        {
            const HullEdge& edge = hull.edges[edgeIndex];
            if (edge.vertexIndices[0] >= hull.vertices.size() || edge.vertexIndices[1] >= hull.vertices.size())
                continue;
            if (edge.faceIndices[0] >= hull.faces.size() || edge.faceIndices[1] >= hull.faces.size())
                continue;

            const Eigen::Vector3f edgeStart = hullMesh->transform.TransformPoint(hull.vertices[edge.vertexIndices[0]].position);
            const Eigen::Vector3f edgeEnd = hullMesh->transform.TransformPoint(hull.vertices[edge.vertexIndices[1]].position);
            const Eigen::Vector3f edgeDirection = edgeEnd - edgeStart;
            if (edgeDirection.squaredNorm() <= 1e-12f)
                continue;

            const Eigen::Vector3f faceNormal0 = TransformNormalToWorld(hullMesh->transform, hull.faces[edge.faceIndices[0]].normal);
            const Eigen::Vector3f faceNormal1 = TransformNormalToWorld(hullMesh->transform, hull.faces[edge.faceIndices[1]].normal);
            const Eigen::Vector3f segmentDirection = capsule.end - capsule.start;

            if (segmentDirection.dot(faceNormal0) * segmentDirection.dot(faceNormal1) >= 0.0f)
                continue;

            Eigen::Vector3f axis = edgeDirection.cross(segmentDirection);
            const float axisLengthSquared = axis.squaredNorm();
            if (axisLengthSquared <= 1e-12f)
                continue;

            axis /= std::sqrt(axisLengthSquared);
            if (axis.dot(capsule.center - hullCenter) < 0.0f)
                axis = -axis;

            const Eigen::Vector3f capsuleSupport = SupportSegment(capsule, -axis);
            const double separation = static_cast<double>(axis.dot(capsuleSupport - edgeStart));

            if (separation > query.separation)
            {
                query.separation = separation;
                query.edgeIndex = static_cast<uint16_t>(edgeIndex);
            }
        }

        return query;
    }

    //================================//
    static void BuildCapsuleFaceContacts(const Mesh* hullMesh,
                                         const Mesh* capsuleMesh,
                                         const ConvexHull& hull,
                                         uint16_t faceIndex,
                                         const Eigen::Vector3f& normalHullToCapsule,
                                         const CapsuleSegment& capsule,
                                         CollisionResult& result)
    {
        result.numContacts = 0;
        if (faceIndex >= hull.faceCount())
            return;

        const HullFace& referenceFace = hull.faces[faceIndex];
        std::vector<Eigen::Vector3f> clippedSegment;
        clippedSegment.push_back(capsule.start);
        clippedSegment.push_back(capsule.end);

        std::vector<Eigen::Vector3f> referenceVertices(referenceFace.vertexIndices.size());
        if (referenceVertices.size() < 3)
            return;

        for (int i = 0; i < static_cast<int>(referenceFace.vertexIndices.size()); ++i)
        {
            if (referenceFace.vertexIndices[i] >= hull.vertices.size())
                return;

            referenceVertices[i] = hullMesh->transform.TransformPoint(hull.vertices[referenceFace.vertexIndices[i]].position);
        }

        for (int i = 0; i < static_cast<int>(referenceVertices.size()); ++i)
        {
            const Eigen::Vector3f edge = referenceVertices[(i + 1) % referenceVertices.size()] - referenceVertices[i];
            const Eigen::Vector3f sideNormal = edge.cross(normalHullToCapsule);
            const float sideOffset = sideNormal.dot(referenceVertices[i]);

            clippedSegment = ClipPolygonAgainstPlane(clippedSegment, sideNormal, sideOffset);
            if (clippedSegment.empty())
                return;
        }

        const float planeOffset = normalHullToCapsule.dot(referenceVertices[0]);
        for (int i = 0; i < static_cast<int>(clippedSegment.size()) && result.numContacts < 8; ++i)
        {
            const Eigen::Vector3f pointOnSegment = clippedSegment[i];
            const float separation = normalHullToCapsule.dot(pointOnSegment) - planeOffset;
            if (separation > capsule.radius)
                continue;

            const Eigen::Vector3f pointOnHull = pointOnSegment - separation * normalHullToCapsule;
            // A bit of a compile magic, without the cast it returned a template of
            // CwiseUnaryOp<internal::scalar_opposite_op<double>, const Vector3f>
            const Eigen::Vector3f capsuleOffset = (separation <= 0.0f) ? Eigen::Vector3f(normalHullToCapsule * capsule.radius) 
                : Eigen::Vector3f(-normalHullToCapsule * capsule.radius);
            const Eigen::Vector3f pointOnCapsule = pointOnSegment + capsuleOffset;

            ContactPoint& contact = result.contactPoints[result.numContacts];
            contact.position = 0.5f * (pointOnHull + pointOnCapsule);
            contact.normal = normalHullToCapsule;
            contact.penetration = capsule.radius - separation;
            contact.rA = ToLocalOffset(hullMesh, pointOnHull);
            contact.rB = ToLocalOffset(capsuleMesh, pointOnCapsule);
            contact.id = MakeContactID(3u, faceIndex, 0u, static_cast<uint32_t>(i));

            result.numContacts++;
        }
    }

    //================================//
    static void BuildCapsuleEdgeContact(const Mesh* hullMesh,
                                        const Mesh* capsuleMesh,
                                        const ConvexHull& hull,
                                        uint16_t edgeIndex,
                                        const Eigen::Vector3f& normalHullToCapsule,
                                        const CapsuleSegment& capsule,
                                        CollisionResult& result)
    {
        result.numContacts = 0;
        if (edgeIndex >= hull.edgeCount())
            return;

        const HullEdge& edge = hull.edges[edgeIndex];
        if (edge.vertexIndices[0] >= hull.vertices.size() || edge.vertexIndices[1] >= hull.vertices.size())
            return;

        const Eigen::Vector3f edgeStart = hullMesh->transform.TransformPoint(hull.vertices[edge.vertexIndices[0]].position);
        const Eigen::Vector3f edgeEnd = hullMesh->transform.TransformPoint(hull.vertices[edge.vertexIndices[1]].position);

        Eigen::Vector3f pointOnHull = Eigen::Vector3f::Zero();
        Eigen::Vector3f pointOnSegment = Eigen::Vector3f::Zero();
        ClosestPointsOnSegments(edgeStart, edgeEnd, capsule.start, capsule.end, pointOnHull, pointOnSegment);

        const float separation = normalHullToCapsule.dot(pointOnSegment - pointOnHull);
        const float penetration = capsule.radius - separation;
        if (penetration <= 0.0f)
            return;

        // Same compile magic as in BuildCapsuleFaceContacts, without the cast it returned a template of
        // CwiseUnaryOp<internal::scalar_opposite_op<double>, const Vector3f
        const Eigen::Vector3f capsuleOffset =
            (separation <= 0.0f) ? Eigen::Vector3f(normalHullToCapsule * capsule.radius) : Eigen::Vector3f(-normalHullToCapsule * capsule.radius);
        const Eigen::Vector3f pointOnCapsule = pointOnSegment + capsuleOffset;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnHull + pointOnCapsule);
        contact.normal = normalHullToCapsule;
        contact.penetration = penetration;
        contact.rA = ToLocalOffset(hullMesh, pointOnHull);
        contact.rB = ToLocalOffset(capsuleMesh, pointOnCapsule);
        contact.id = MakeContactID(4u, edgeIndex, 0u, 0u);

        result.numContacts = 1;
    }

    //================================//
    CollisionResult CollideSphereHullSAT(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;
        const bool sphereIsA = (meshA->modelType == ModelType_Sphere);
        const Mesh* sphereMesh = sphereIsA ? meshA : meshB;
        const Mesh* hullMesh = sphereIsA ? meshB : meshA;
        const ConvexHull& hull = hullMesh->solver->GetModelConvexHull(hullMesh->modelType);

        if (hull.faceCount() == 0)
            return result;

        const Eigen::Vector3f sphereCenter = sphereMesh->transform.GetPosition();
        const float sphereRadius = 0.5f * sphereMesh->transform.GetScale().x();

        int bestFaceIndex = -1;
        float bestSeparation = -std::numeric_limits<float>::infinity();
        Eigen::Vector3f normalHullToSphere = Eigen::Vector3f::UnitY();

        for (int faceIndex = 0; faceIndex < static_cast<int>(hull.faceCount()); ++faceIndex)
        {
            const HullFace& face = hull.faces[faceIndex];
            if (face.vertexIndices.empty() || face.vertexIndices[0] >= hull.vertices.size())
                continue;

            const Eigen::Vector3f faceNormal = TransformNormalToWorld(hullMesh->transform, face.normal);
            const Eigen::Vector3f planePoint = hullMesh->transform.TransformPoint(hull.vertices[face.vertexIndices[0]].position);
            const float separation = faceNormal.dot(sphereCenter - planePoint);

            if (separation > sphereRadius)
                return result;

            if (separation > bestSeparation)
            {
                bestSeparation = separation;
                bestFaceIndex = faceIndex;
                normalHullToSphere = faceNormal;
            }
        }

        if (bestFaceIndex < 0)
            return result;

        const Eigen::Vector3f pointOnHull = sphereCenter - bestSeparation * normalHullToSphere;
        // Same here...
        const Eigen::Vector3f sphereOffset = (bestSeparation <= 0.0f) ? Eigen::Vector3f(normalHullToSphere * sphereRadius) : Eigen::Vector3f(-normalHullToSphere * sphereRadius);
        const Eigen::Vector3f pointOnSphere = sphereCenter + sphereOffset;
        const Eigen::Vector3f normalAToB = sphereIsA ? -normalHullToSphere : normalHullToSphere;
        const Eigen::Vector3f pointOnA = sphereIsA ? pointOnSphere : pointOnHull;
        const Eigen::Vector3f pointOnB = sphereIsA ? pointOnHull : pointOnSphere;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnA + pointOnB);
        contact.normal = normalAToB;
        contact.penetration = sphereRadius - bestSeparation;
        contact.rA = ToLocalOffset(meshA, pointOnA);
        contact.rB = ToLocalOffset(meshB, pointOnB);
        contact.id = MakeContactID(5u, static_cast<uint32_t>(bestFaceIndex), 0u, 0u);

        result.numContacts = 1;
        return result;
    }

    //================================//
    CollisionResult CollideHullCapsuleSAT(const Mesh* meshA, const Mesh* meshB)
    {
        const bool hullIsA = (meshA->modelType != ModelType_Capsule);
        const Mesh* hullMesh = hullIsA ? meshA : meshB;
        const Mesh* capsuleMesh = hullIsA ? meshB : meshA;

        CollisionResult result;
        const ConvexHull& hull = hullMesh->solver->GetModelConvexHull(hullMesh->modelType);
        if (hull.faceCount() == 0 || hull.edgeCount() == 0)
            return result;

        const CapsuleSegment capsule = GetCapsuleSegment(capsuleMesh);
        const CapsuleFaceQuery faceQuery = QueryCapsuleFaceDirections(hullMesh, hull, capsule);
        if (faceQuery.separation > capsule.radius)
            return result;

        const CapsuleEdgeQuery edgeQuery = QueryCapsuleEdgeDirections(hullMesh, hull, capsule);
        if (edgeQuery.separation > capsule.radius)
            return result;

        const double bestFaceSeparation = faceQuery.separation;
        if (!ShouldPreferEdgeAxis(edgeQuery.separation, bestFaceSeparation, capsule.radius))
        {
            const Eigen::Vector3f referenceNormal = TransformNormalToWorld(hullMesh->transform, hull.faces[faceQuery.faceIndex].normal);
            BuildCapsuleFaceContacts(hullMesh, capsuleMesh, hull, faceQuery.faceIndex, referenceNormal, capsule, result);
        }
        else
        {
            const HullEdge& edge = hull.edges[edgeQuery.edgeIndex];
            const Eigen::Vector3f edgeStart = hullMesh->transform.TransformPoint(hull.vertices[edge.vertexIndices[0]].position);
            const Eigen::Vector3f edgeEnd = hullMesh->transform.TransformPoint(hull.vertices[edge.vertexIndices[1]].position);

            Eigen::Vector3f normalHullToCapsule = (edgeEnd - edgeStart).cross(capsule.end - capsule.start);
            const float normalLengthSquared = normalHullToCapsule.squaredNorm();
            if (normalLengthSquared <= 1e-12f)
                normalHullToCapsule = AnyPerpendicular(capsule.axis);
            else
                normalHullToCapsule /= std::sqrt(normalLengthSquared);

            if (normalHullToCapsule.dot(capsule.center - hullMesh->transform.TransformPoint(hull.centroid)) < 0.0f)
                normalHullToCapsule = -normalHullToCapsule;

            BuildCapsuleEdgeContact(hullMesh, capsuleMesh, hull, edgeQuery.edgeIndex, normalHullToCapsule, capsule, result);
        }

        if (!hullIsA)
        {
            for (int contactIndex = 0; contactIndex < result.numContacts; ++contactIndex)
            {
                std::swap(result.contactPoints[contactIndex].rA, result.contactPoints[contactIndex].rB);
                result.contactPoints[contactIndex].normal = -result.contactPoints[contactIndex].normal;
            }
        }

        return result;
    }

    //================================//
    CollisionResult CollideHullHullSAT(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;

        const ConvexHull& hullA = meshA->solver->GetModelConvexHull(meshA->modelType);
        const ConvexHull& hullB = meshB->solver->GetModelConvexHull(meshB->modelType);
        if (hullA.vertexCount() == 0 || hullB.vertexCount() == 0 ||
            hullA.faceCount() == 0 || hullB.faceCount() == 0)
        {
            return result;
        }

        const FaceQuery faceQueryA = QueryHullFaceDirections(meshA->transform, hullA, meshB->transform, hullB);
        if (faceQueryA.separation > 0.0)
            return result;

        const FaceQuery faceQueryB = QueryHullFaceDirections(meshB->transform, hullB, meshA->transform, hullA);
        if (faceQueryB.separation > 0.0)
            return result;

        const EdgeQuery edgeQuery = QueryHullEdgeDirections(meshA->transform, hullA, meshB->transform, hullB);
        if (edgeQuery.separation > 0.0)
            return result;

        const double bestFaceSeparation = std::max(faceQueryA.separation, faceQueryB.separation);
        if (ShouldPreferEdgeAxis(edgeQuery.separation, bestFaceSeparation, 0.0))
        {
            CreateHullEdgeContact(meshA, hullA, meshB, hullB, edgeQuery, result);
            return result;
        }

        const double penetrationA = -faceQueryA.separation;
        const double penetrationB = -faceQueryB.separation;
        if (penetrationA < penetrationB * SEPARATING_AXIS_RELATIVE_TOLERANCE +
                           SEPARATING_AXIS_ABSOLUTE_TOLERANCE)
        {
            CreateHullFaceContacts(meshA, hullA, meshB, hullB, faceQueryA, result);
        }
        else
        {
            CreateHullFaceContacts(meshB, hullB, meshA, hullA, faceQueryB, result);
            for (int contactIndex = 0; contactIndex < result.numContacts; ++contactIndex)
            {
                std::swap(result.contactPoints[contactIndex].rA, result.contactPoints[contactIndex].rB);
                result.contactPoints[contactIndex].normal = -result.contactPoints[contactIndex].normal;
            }
        }

        return result;
    }
}
