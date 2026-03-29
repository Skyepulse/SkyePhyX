#include "collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "solver.hpp"

//================================//
namespace CollisionSpace
{
    static constexpr float FACE_CONTACT_EDGE_BIAS = 0.005f;

    enum AxisType
    {
        FACE_A = 0,
        FACE_B = 1,
        EDGE = 2
    };

    struct BoxTransformData
    {
        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector3f scale = Eigen::Vector3f::Ones();
        Eigen::Vector3f halfExtents = Eigen::Vector3f::Constant(0.5f);
        Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    };

    struct BoxFaceQuery
    {
        float penetration = std::numeric_limits<float>::infinity();
        uint16_t axisIndex = 0;
        float sign = 1.0f;
        bool referenceOnA = true;
    };

    struct BoxEdgeQuery
    {
        float penetration = std::numeric_limits<float>::infinity();
        uint16_t axisA = 0;
        uint16_t axisB = 0;
    };

    //================================//
    static uint32_t CreateContactID(AxisType axisType, int axisA, int axisB, int featureIndex)
    {
        return (static_cast<uint32_t>(axisType) & 0x3u)
             | ((static_cast<uint32_t>(axisA) & 0x7u) << 2)
             | ((static_cast<uint32_t>(axisB) & 0x7u) << 5)
             | ((static_cast<uint32_t>(featureIndex) & 0xFu) << 8);
    }

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
    // BOX-BOX HELPERS
    //================================//

    //================================//
    static BoxTransformData GetBoxTransformData(const Mesh* mesh)
    {
        BoxTransformData box;
        box.position = mesh->transform.GetPosition();
        box.scale = mesh->transform.GetScale();
        box.halfExtents = 0.5f * box.scale;
        box.rotation = mesh->transform.GetRotation().toRotationMatrix();
        return box;
    }

    //================================//
    static void GetBoxFaceVerticesWorld(const BoxTransformData& box, int faceAxis, float faceSign, Eigen::Vector3f outVertices[4])
    {
        const int u = (faceAxis + 1) % 3;
        const int v = (faceAxis + 2) % 3;

        const Eigen::Vector3f faceCenter = box.position + box.rotation.col(faceAxis) * (faceSign * box.halfExtents[faceAxis]);
        const Eigen::Vector3f uVec = box.rotation.col(u) * box.halfExtents[u];
        const Eigen::Vector3f vVec = box.rotation.col(v) * box.halfExtents[v];

        if (faceSign > 0.0f)
        {
            outVertices[0] = faceCenter + uVec + vVec;
            outVertices[1] = faceCenter + uVec - vVec;
            outVertices[2] = faceCenter - uVec - vVec;
            outVertices[3] = faceCenter - uVec + vVec;
        }
        else
        {
            outVertices[0] = faceCenter - uVec - vVec;
            outVertices[1] = faceCenter - uVec + vVec;
            outVertices[2] = faceCenter + uVec + vVec;
            outVertices[3] = faceCenter + uVec - vVec;
        }
    }

    //================================//
    static void FindIncidentBoxFace(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& referenceNormal, int& outFaceAxis, float& outFaceSign)
    {
        float minDot = std::numeric_limits<float>::max();
        outFaceAxis = 0;
        outFaceSign = 1.0f;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float d = rotation.col(axis).dot(referenceNormal);
            if (d < minDot)
            {
                minDot = d;
                outFaceAxis = axis;
                outFaceSign = 1.0f;
            }
            if (-d < minDot)
            {
                minDot = -d;
                outFaceAxis = axis;
                outFaceSign = -1.0f;
            }
        }
    }

    //================================//
    static int PolygonFaceClip(const Eigen::Vector3f* inVertices, int numIn,
                               const Eigen::Vector3f& planeNormal, float planeOffset,
                               Eigen::Vector3f* outVertices)
    {
        if (numIn < 1)
            return 0;

        int outCount = 0;

        for (int i = 0; i < numIn; ++i)
        {
            const int j = (i + 1) % numIn;
            const float di = planeNormal.dot(inVertices[i]) - planeOffset;
            const float dj = planeNormal.dot(inVertices[j]) - planeOffset;

            if (di <= 0.0f)
                outVertices[outCount++] = inVertices[i];
            if (outCount >= 8)
                return outCount;

            if ((di > 0.0f) != (dj > 0.0f))
            {
                const float t = di / (di - dj);
                outVertices[outCount++] = inVertices[i] + t * (inVertices[j] - inVertices[i]);
                if (outCount >= 8)
                    return outCount;
            }
        }

        return outCount;
    }

    //================================//
    static bool QueryBoxFaceDirectionsA(const BoxTransformData& boxA, const BoxTransformData& boxB,
                                        const Eigen::Matrix3f& absoluteRotation, const Eigen::Vector3f& T,
                                        BoxFaceQuery& query)
    {
        query.penetration = std::numeric_limits<float>::infinity();
        query.axisIndex = 0;
        query.sign = 1.0f;
        query.referenceOnA = true;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float rA = boxA.halfExtents[axis];
            const float rB = absoluteRotation.row(axis).dot(boxB.halfExtents);
            const float distance = std::abs(T[axis]);
            const float penetration = rA + rB - distance;
            const float sign = (T[axis] > 0.0f) ? 1.0f : -1.0f;

            if (penetration < 0.0f)
                return false;

            if (penetration < query.penetration)
            {
                query.penetration = penetration;
                query.axisIndex = static_cast<uint16_t>(axis);
                query.sign = sign;
            }
        }

        return true;
    }

    //================================//
    static bool QueryBoxFaceDirectionsB(const BoxTransformData& boxA, const BoxTransformData& boxB,
                                        const Eigen::Matrix3f& C, const Eigen::Matrix3f& absoluteRotation,
                                        const Eigen::Vector3f& T, BoxFaceQuery& query)
    {
        query.penetration = std::numeric_limits<float>::infinity();
        query.axisIndex = 0;
        query.sign = 1.0f;
        query.referenceOnA = false;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float rA = absoluteRotation.col(axis).dot(boxA.halfExtents);
            const float rB = boxB.halfExtents[axis];
            const float distance = std::abs(C.col(axis).dot(T));
            const float penetration = rA + rB - distance;
            const float sign = (T.dot(C.col(axis)) > 0.0f) ? 1.0f : -1.0f;

            if (penetration < 0.0f)
                return false;

            if (penetration < query.penetration)
            {
                query.penetration = penetration;
                query.axisIndex = static_cast<uint16_t>(axis);
                query.sign = sign;
            }
        }

        return true;
    }

    //================================//
    static bool QueryBoxEdgeDirections(const BoxTransformData& boxA, const BoxTransformData& boxB,
                                       const Eigen::Matrix3f& C, const Eigen::Matrix3f& absoluteRotation,
                                       const Eigen::Vector3f& T, float bestFacePenetration,
                                       BoxEdgeQuery& query)
    {
        query.penetration = std::numeric_limits<float>::infinity();
        query.axisA = 0;
        query.axisB = 0;

        const float faceBiasRelative = 0.95f;
        const float faceBiasAbsolute = 0.005f;

        for (int axisA = 0; axisA < 3; ++axisA)
        {
            for (int axisB = 0; axisB < 3; ++axisB)
            {
                const float crossLengthSq = 1.0f - C(axisA, axisB) * C(axisA, axisB);
                if (crossLengthSq < 1e-6f)
                    continue;

                const float invLength = 1.0f / std::sqrt(crossLengthSq);

                const int i1 = (axisA + 1) % 3;
                const int i2 = (axisA + 2) % 3;
                const int j1 = (axisB + 1) % 3;
                const int j2 = (axisB + 2) % 3;

                const float rA = boxA.halfExtents[i1] * absoluteRotation(i2, axisB) +
                                 boxA.halfExtents[i2] * absoluteRotation(i1, axisB);
                const float rB = boxB.halfExtents[j1] * absoluteRotation(axisA, j2) +
                                 boxB.halfExtents[j2] * absoluteRotation(axisA, j1);
                const float rawDistance = T[i2] * C(i1, axisB) - T[i1] * C(i2, axisB);
                const float distance = std::abs(rawDistance);
                const float penetration = (rA + rB - distance) * invLength;

                if (penetration < 0.0f)
                    return false;

                if (penetration >= faceBiasRelative * bestFacePenetration + faceBiasAbsolute)
                    continue;

                if (penetration < query.penetration)
                {
                    query.penetration = penetration;
                    query.axisA = static_cast<uint16_t>(axisA);
                    query.axisB = static_cast<uint16_t>(axisB);
                }
            }
        }

        return true;
    }

    //================================//
    static void CreateBoxFaceContacts(const BoxTransformData& referenceBox, const BoxTransformData& incidentBox,
                                      int referenceAxis, float referenceFaceSign,
                                      const Eigen::Vector3f& collisionNormal,
                                      AxisType axisType, int axisA, int axisB,
                                      CollisionResult& result)
    {
        result.numContacts = 0;

        const Eigen::Vector3f referenceFaceNormal = referenceBox.rotation.col(referenceAxis) * referenceFaceSign;
        const float referenceFaceOffset =
            referenceFaceNormal.dot(referenceBox.position) + referenceBox.halfExtents[referenceAxis];

        int incidentAxis = 0;
        float incidentFaceSign = 1.0f;
        FindIncidentBoxFace(incidentBox.rotation, referenceFaceNormal, incidentAxis, incidentFaceSign);

        Eigen::Vector3f incidentFaceVertices[4];
        GetBoxFaceVerticesWorld(incidentBox, incidentAxis, incidentFaceSign, incidentFaceVertices);

        const int u = (referenceAxis + 1) % 3;
        const int v = (referenceAxis + 2) % 3;

        struct ClipPlane
        {
            Eigen::Vector3f normal;
            float offset = 0.0f;
        };

        ClipPlane sidePlanes[4];
        sidePlanes[0].normal =  referenceBox.rotation.col(u);
        sidePlanes[0].offset =  referenceBox.rotation.col(u).dot(referenceBox.position) + referenceBox.halfExtents[u];
        sidePlanes[1].normal = -referenceBox.rotation.col(u);
        sidePlanes[1].offset = -referenceBox.rotation.col(u).dot(referenceBox.position) + referenceBox.halfExtents[u];
        sidePlanes[2].normal =  referenceBox.rotation.col(v);
        sidePlanes[2].offset =  referenceBox.rotation.col(v).dot(referenceBox.position) + referenceBox.halfExtents[v];
        sidePlanes[3].normal = -referenceBox.rotation.col(v);
        sidePlanes[3].offset = -referenceBox.rotation.col(v).dot(referenceBox.position) + referenceBox.halfExtents[v];

        Eigen::Vector3f clipBufferA[8];
        Eigen::Vector3f clipBufferB[8];
        for (int i = 0; i < 4; ++i)
            clipBufferA[i] = incidentFaceVertices[i];

        int numClipPoints = 4;
        for (int planeIndex = 0; planeIndex < 4; ++planeIndex)
        {
            Eigen::Vector3f* source = (planeIndex % 2 == 0) ? clipBufferA : clipBufferB;
            Eigen::Vector3f* dest = (planeIndex % 2 == 0) ? clipBufferB : clipBufferA;

            numClipPoints = PolygonFaceClip(source, numClipPoints, sidePlanes[planeIndex].normal, sidePlanes[planeIndex].offset, dest);
            if (numClipPoints < 1)
                return;
        }

        Eigen::Vector3f* finalClipPoints = (4 % 2 == 0) ? clipBufferA : clipBufferB;
        const Eigen::Matrix3f invReferenceRotation = referenceBox.rotation.transpose();
        const Eigen::Matrix3f invIncidentRotation = incidentBox.rotation.transpose();

        for (int i = 0; i < numClipPoints && result.numContacts < 8; ++i)
        {
            const float separation = referenceFaceNormal.dot(finalClipPoints[i]) - referenceFaceOffset;
            if (separation > 0.0f)
                continue;

            ContactPoint& contact = result.contactPoints[result.numContacts];
            const Eigen::Vector3f projectedPoint = finalClipPoints[i] - referenceFaceNormal * separation;

            contact.position = projectedPoint;
            contact.normal = collisionNormal;
            contact.penetration = -separation;
            contact.rA = invReferenceRotation * (projectedPoint - referenceBox.position);
            contact.rB = invIncidentRotation * (finalClipPoints[i] - incidentBox.position);
            contact.id = CreateContactID(axisType, axisA, axisB, i);

            ++result.numContacts;
        }
    }

    //================================//
    static void CreateBoxEdgeContact(const BoxTransformData& boxA, const BoxTransformData& boxB,
                                     int axisA, int axisB, const Eigen::Vector3f& normal,
                                     CollisionResult& result)
    {
        result.numContacts = 0;

        const Eigen::Vector3f d = boxB.position - boxA.position;

        const int uA = (axisA + 1) % 3;
        const int vA = (axisA + 2) % 3;
        const float signUA = (boxA.rotation.col(uA).dot(d) > 0.0f) ? 1.0f : -1.0f;
        const float signVA = (boxA.rotation.col(vA).dot(d) > 0.0f) ? 1.0f : -1.0f;

        const Eigen::Vector3f midA =
            boxA.position +
            boxA.rotation.col(uA) * (signUA * boxA.halfExtents[uA]) +
            boxA.rotation.col(vA) * (signVA * boxA.halfExtents[vA]);
        const Eigen::Vector3f dirA = boxA.rotation.col(axisA);
        const Eigen::Vector3f startA = midA - dirA * boxA.halfExtents[axisA];
        const Eigen::Vector3f endA = midA + dirA * boxA.halfExtents[axisA];

        const int uB = (axisB + 1) % 3;
        const int vB = (axisB + 2) % 3;
        const float signUB = (boxB.rotation.col(uB).dot(-d) > 0.0f) ? 1.0f : -1.0f;
        const float signVB = (boxB.rotation.col(vB).dot(-d) > 0.0f) ? 1.0f : -1.0f;

        const Eigen::Vector3f midB =
            boxB.position +
            boxB.rotation.col(uB) * (signUB * boxB.halfExtents[uB]) +
            boxB.rotation.col(vB) * (signVB * boxB.halfExtents[vB]);
        const Eigen::Vector3f dirB = boxB.rotation.col(axisB);
        const Eigen::Vector3f startB = midB - dirB * boxB.halfExtents[axisB];
        const Eigen::Vector3f endB = midB + dirB * boxB.halfExtents[axisB];

        Eigen::Vector3f pointA;
        Eigen::Vector3f pointB;
        ClosestPointsOnSegments(startA, endA, startB, endB, pointA, pointB);

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointA + pointB);
        contact.normal = normal;
        contact.penetration = std::abs((pointB - pointA).dot(normal));
        contact.rA = boxA.rotation.transpose() * (pointA - boxA.position);
        contact.rB = boxB.rotation.transpose() * (pointB - boxB.position);
        contact.id = CreateContactID(EDGE, axisA, axisB, 0);

        result.numContacts = 1;
    }

    //================================//
    // HULL-HULL FORWARD DECLARATIONS
    //================================//

    //================================//
    static void CreateHullFaceContact(const FaceQuery& faceQuery, const Transform& transformA, const ConvexHull& hullA,
                                      const Transform& transformB, const ConvexHull& hullB, CollisionResult& result);
    static void CreateHullEdgeContact(const EdgeQuery& edgeQuery, const Transform& transformA, const ConvexHull& hullA,
                                      const Transform& transformB, const ConvexHull& hullB, CollisionResult& result);
    static FaceQuery QueryHullFaceDirections(const Transform& transformA, const ConvexHull& hullA,
                                             const Transform& transformB, const ConvexHull& hullB);
    static EdgeQuery QueryHullEdgeDirections(const Transform& transformA, const ConvexHull& hullA,
                                             const Transform& transformB, const ConvexHull& hullB);

    //================================//
    // HULL-HULL HELPERS
    //================================//

    //================================//
    static FaceQuery QueryHullFaceDirections(const Transform& transformA, const ConvexHull& hullA,
                                             const Transform& transformB, const ConvexHull& hullB)
    {
        FaceQuery query;
        query.separation = -INFINITY;
        query.faceIndex = 0;

        if (hullA.faceCount() == 0 || hullB.vertexCount() == 0)
        {
            query.separation = std::numeric_limits<double>::infinity();
            return query;
        }

        const Eigen::Matrix3f linearA = transformA.GetRotation().toRotationMatrix() * transformA.GetScale().asDiagonal();
        const Eigen::Matrix3f linearB = transformB.GetRotation().toRotationMatrix() * transformB.GetScale().asDiagonal();
        const Eigen::Matrix3f normalMatrixA = linearA.inverse().transpose();

        for (int i = 0; i < static_cast<int>(hullA.faceCount()); ++i)
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
    static EdgeQuery QueryHullEdgeDirections(const Transform& transformA, const ConvexHull& hullA,
                                             const Transform& transformB, const ConvexHull& hullB)
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

        const Eigen::Matrix3f linearB = transformB.GetRotation().toRotationMatrix() * transformB.GetScale().asDiagonal();
        const Eigen::Vector3f hullCenterWorldA = transformA.TransformPoint(hullA.centroid);

        for (int edgeIndexA = 0; edgeIndexA < static_cast<int>(hullA.edgeCount()); ++edgeIndexA)
        {
            const HullEdge& edgeA = hullA.edges[edgeIndexA];
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

            for (int edgeIndexB = 0; edgeIndexB < static_cast<int>(hullB.edgeCount()); ++edgeIndexB)
            {
                const HullEdge& edgeB = hullB.edges[edgeIndexB];
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
                    query.edgeIndexA = static_cast<uint16_t>(edgeIndexA);
                    query.edgeIndexB = static_cast<uint16_t>(edgeIndexB);
                }
            }
        }

        return query;
    }

    //================================//
    static void CreateHullFaceContact(const FaceQuery& faceQuery, const Transform& transformA, const ConvexHull& hullA,
                                      const Transform& transformB, const ConvexHull& hullB, CollisionResult& result)
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
        for (int i = 0; i < static_cast<int>(hullB.faceCount()); ++i)
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
        const HullFace& incidentFace = hullB.faces[incidentFaceIndex];
        incidentPolygon.reserve(incidentFace.vertexIndices.size());
        for (size_t i = 0; i < incidentFace.vertexIndices.size(); ++i)
        {
            if (incidentFace.vertexIndices[i] >= hullB.vertices.size())
                return;

            incidentPolygon.push_back(transformB.TransformPoint(hullB.vertices[incidentFace.vertexIndices[i]].position));
        }

        std::vector<Eigen::Vector3f> referenceVertices(referenceFace.vertexIndices.size());
        if (referenceVertices.size() < 3)
            return;

        for (size_t i = 0; i < referenceFace.vertexIndices.size(); ++i)
        {
            if (referenceFace.vertexIndices[i] >= hullA.vertices.size())
                return;

            referenceVertices[i] = transformA.TransformPoint(hullA.vertices[referenceFace.vertexIndices[i]].position);
        }

        for (size_t i = 0; i < referenceVertices.size(); ++i)
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
            const Eigen::Vector3f& point = incidentPolygon[i];
            const float separation = referenceNormal.dot(point) - referencePlaneOffset;
            if (separation > 0.0f)
                continue;

            const Eigen::Vector3f projectedPoint = point - separation * referenceNormal;

            bool isDuplicate = false;
            for (int existingIndex = 0; existingIndex < result.numContacts; ++existingIndex)
            {
                if ((result.contactPoints[existingIndex].position - projectedPoint).squaredNorm() <= 1e-10f)
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

            ++result.numContacts;
        }
    }

    //================================//
    static void CreateHullEdgeContact(const EdgeQuery& edgeQuery, const Transform& transformA, const ConvexHull& hullA,
                                      const Transform& transformB, const ConvexHull& hullB, CollisionResult& result)
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

        Eigen::Vector3f pointA;
        Eigen::Vector3f pointB;
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

    //================================//
    // BOX-BOX IMPLEMENTATION
    //================================//

    //================================//
    CollisionResult CollisionBoxBox(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result{};
        result.numContacts = 0;

        const BoxTransformData boxA = GetBoxTransformData(meshA);
        const BoxTransformData boxB = GetBoxTransformData(meshB);

        const Eigen::Vector3f d = boxB.position - boxA.position;
        const Eigen::Matrix3f C = boxA.rotation.transpose() * boxB.rotation;
        const Eigen::Vector3f T = boxA.rotation.transpose() * d;

        Eigen::Matrix3f absoluteRotation;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
                absoluteRotation(i, j) = std::abs(C(i, j)) + 1e-6f;
        }

        BoxFaceQuery faceQueryA;
        if (!QueryBoxFaceDirectionsA(boxA, boxB, absoluteRotation, T, faceQueryA))
            return result;

        BoxFaceQuery faceQueryB;
        if (!QueryBoxFaceDirectionsB(boxA, boxB, C, absoluteRotation, T, faceQueryB))
            return result;

        const float bestFacePenetration = std::min(faceQueryA.penetration, faceQueryB.penetration);

        BoxEdgeQuery edgeQuery;
        if (!QueryBoxEdgeDirections(boxA, boxB, C, absoluteRotation, T, bestFacePenetration, edgeQuery))
            return result;

        if (faceQueryA.penetration <= faceQueryB.penetration && faceQueryA.penetration <= edgeQuery.penetration)
        {
            const Eigen::Vector3f normal = boxA.rotation.col(faceQueryA.axisIndex) * faceQueryA.sign;

            CreateBoxFaceContacts(
                boxA, boxB,
                faceQueryA.axisIndex, faceQueryA.sign,
                normal,
                FACE_A, faceQueryA.axisIndex, 0,
                result
            );
        }
        else if (faceQueryB.penetration <= edgeQuery.penetration)
        {
            const Eigen::Vector3f normal = boxB.rotation.col(faceQueryB.axisIndex) * faceQueryB.sign;

            CreateBoxFaceContacts(
                boxB, boxA,
                faceQueryB.axisIndex, -faceQueryB.sign,
                normal,
                FACE_B, 0, faceQueryB.axisIndex,
                result
            );

            for (int i = 0; i < result.numContacts; ++i)
                std::swap(result.contactPoints[i].rA, result.contactPoints[i].rB);
        }
        else
        {
            Eigen::Vector3f normal = boxA.rotation.col(edgeQuery.axisA).cross(boxB.rotation.col(edgeQuery.axisB));
            const float normalLength = normal.norm();
            if (normalLength <= 1e-6f)
                return result;

            normal /= normalLength;
            if (normal.dot(d) < 0.0f)
                normal = -normal;

            CreateBoxEdgeContact(boxA, boxB, edgeQuery.axisA, edgeQuery.axisB, normal, result);
        }

        return result;
    }

    //================================//
    // HULL-HULL IMPLEMENTATION
    //================================//

    //================================//
    CollisionResult CollisionHullHull(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;
        result.numContacts = 0;

        const ConvexHull& hullA = meshA->solver->GetModelConvexHull(meshA->modelType);
        const ConvexHull& hullB = meshB->solver->GetModelConvexHull(meshB->modelType);
        if (hullA.vertexCount() == 0 || hullB.vertexCount() == 0 ||
            hullA.faceCount() == 0 || hullB.faceCount() == 0)
        {
            return result;
        }

        const Transform& transformA = meshA->transform;
        const Transform& transformB = meshB->transform;

        const FaceQuery faceQueryA = QueryHullFaceDirections(transformA, hullA, transformB, hullB);
        if (faceQueryA.separation > 0.0)
            return result;

        const FaceQuery faceQueryB = QueryHullFaceDirections(transformB, hullB, transformA, hullA);
        if (faceQueryB.separation > 0.0)
            return result;

        const EdgeQuery edgeQuery = QueryHullEdgeDirections(transformA, hullA, transformB, hullB);
        if (edgeQuery.separation > 0.0)
            return result;

        const bool useFaceContactA = faceQueryA.separation >= edgeQuery.separation;
        const bool useFaceContactB = faceQueryB.separation >= edgeQuery.separation;

        if (useFaceContactA && faceQueryA.separation >= faceQueryB.separation - FACE_CONTACT_EDGE_BIAS)
        {
            CreateHullFaceContact(faceQueryA, transformA, hullA, transformB, hullB, result);
        }
        else if (useFaceContactB && faceQueryB.separation >= faceQueryA.separation - FACE_CONTACT_EDGE_BIAS)
        {
            CreateHullFaceContact(faceQueryB, transformB, hullB, transformA, hullA, result);

            for (int i = 0; i < result.numContacts; ++i)
            {
                std::swap(result.contactPoints[i].rA, result.contactPoints[i].rB);
                result.contactPoints[i].normal = -result.contactPoints[i].normal;
            }
        }
        else
        {
            CreateHullEdgeContact(edgeQuery, transformA, hullA, transformB, hullB, result);
        }

        return result;
    }

    //================================//
    // DISPATCH
    //================================//

    //================================//
    CollisionResult CollisionMeshMesh(const Mesh* meshA, const Mesh* meshB)
    {
        if (!meshA || !meshB)
            return CollisionResult{};

        if (meshA->modelType == ModelType_Cube && meshB->modelType == ModelType_Cube)
            return CollisionBoxBox(meshA, meshB);

        return CollisionHullHull(meshA, meshB);
    }
}
