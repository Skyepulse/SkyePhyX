#include <algorithm>
#include <cmath>
#include <limits>
#include "../collision.hpp"

//================================//
namespace CollisionSpace
{
    enum BoxAxisType
    {
        BOX_FACE_A = 0,
        BOX_FACE_B = 1,
        BOX_EDGE = 2
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
    };

    struct BoxEdgeQuery
    {
        float penetration = std::numeric_limits<float>::infinity();
        uint16_t axisA = 0;
        uint16_t axisB = 0;
    };

    //================================//
    static uint32_t CreateBoxContactID(BoxAxisType axisType, int axisA, int axisB, int featureIndex)
    {
        return (static_cast<uint32_t>(axisType) & 0x3u)
             | ((static_cast<uint32_t>(axisA) & 0x7u) << 2)
             | ((static_cast<uint32_t>(axisB) & 0x7u) << 5)
             | ((static_cast<uint32_t>(featureIndex) & 0xFu) << 8);
    }

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
                                      BoxAxisType axisType, int axisA, int axisB,
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
            contact.rA = referenceBox.rotation.transpose() * (projectedPoint - referenceBox.position);
            contact.rB = incidentBox.rotation.transpose() * (finalClipPoints[i] - incidentBox.position);
            contact.id = CreateBoxContactID(axisType, axisA, axisB, i);

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
        contact.id = CreateBoxContactID(BOX_EDGE, axisA, axisB, 0);

        result.numContacts = 1;
    }

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
            for (int j = 0; j < 3; ++j)
                absoluteRotation(i, j) = std::abs(C(i, j)) + 1e-6f;

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
                BOX_FACE_A, faceQueryA.axisIndex, 0,
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
                BOX_FACE_B, 0, faceQueryB.axisIndex,
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
}
