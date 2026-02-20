#include "collision.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

//================================//
namespace CollisionSpace
{
    enum AxisType { FACE_A = 0, FACE_B = 1, EDGE = 2 };

    //================================//
    struct SATResult
    {
        AxisType axisType;

        int AxisA;
        int AxisB;

        float sign;
        float penetration;

        Eigen::Vector3f normal; // points A -> B
    };

    //================================//
    uint32_t createID(AxisType axisType, int axisA, int axisB, int vertexIndex)
    {
        return (static_cast<uint32_t>(axisType) & 0x3)
             | ((static_cast<uint32_t>(axisA) & 0x7) << 2)
             | ((static_cast<uint32_t>(axisB) & 0x7) << 5)
             | ((static_cast<uint32_t>(vertexIndex) & 0xF) << 8);
    }

    //================================//
    // (Vertices in CCW order)
    void GetBoxVerticesWorld(const Eigen::Vector3f& position, const Eigen::Vector3f& halfs, const Eigen::Matrix3f& rot, int faceAxis, float faceSign, Eigen::Vector3f outVertices[4])
    {
        int u = (faceAxis + 1) % 3;
        int v = (faceAxis + 2) % 3;

        Eigen::Vector3f faceCenter = position + rot.col(faceAxis) * (faceSign * halfs[faceAxis]);
        Eigen::Vector3f uVec = rot.col(u) * (halfs[u]);
        Eigen::Vector3f vVec = rot.col(v) * (halfs[v]);

        if (faceSign > 0)
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
    // Find which face of a box is most opposing a given normal.
    // Returns the face whose outward normal has the smallest
    // (most negative) dot product with refNormal.
    void FindIncidentFace(const Eigen::Matrix3f& rot, const Eigen::Vector3f& refNormal, int& outFaceAxis, float& outFaceSign)
    {
        float minDot = std::numeric_limits<float>::max();
        outFaceAxis = 0;
        outFaceSign = 0.0f;

        for (int axis = 0; axis < 3; axis++)
        {
            float d = rot.col(axis).dot(refNormal);
            if (d < minDot) { minDot = d; outFaceAxis = axis; outFaceSign = 1.0f; }
            if (-d < minDot) { minDot = -d; outFaceAxis = axis; outFaceSign = -1.0f; }
        }
    }

    //================================//
    // Sutherland-Hodgman: clip a convex polygon against a plane.
    int PolygonFaceClip(const Eigen::Vector3f* in, int numIn, const Eigen::Vector3f& planeNormal, float planeOffset, Eigen::Vector3f* out)
    {
        if (numIn < 1) return 0;

        int outIndex = 0;

        for (int i = 0; i < numIn; i++)
        {
            int j = (i + 1) % numIn;
            float di = planeNormal.dot(in[i]) - planeOffset;
            float dj = planeNormal.dot(in[j]) - planeOffset;

            if (di <= 0.f) out[outIndex++] = in[i];
            if (outIndex >= 8) return outIndex; // 8 would mean whole box clipped

            if ((di > 0.f) != (dj > 0.f)) // Meaning an edge is clipping the plane
            {
                float t = di / (di - dj);
                out[outIndex++] = in[i] + t * (in[j] - in[i]);
                if (outIndex >= 8) return outIndex;
            }
        }

        return outIndex;
    }

    //================================//
    void FaceFaceContacts(
        const Eigen::Vector3f& posA, const Eigen::Vector3f& scaleA, const Eigen::Matrix3f& rotA,
        const Eigen::Vector3f& posB, const Eigen::Vector3f& scaleB, const Eigen::Matrix3f& rotB,
        const Eigen::Vector3f& refFaceCenter, const Eigen::Matrix3f& refFaceRot, const Eigen::Vector3f& refFaceHalfs,
        const Eigen::Vector3f& incFaceCenter, const Eigen::Matrix3f& incFaceRot, const Eigen::Vector3f& incFaceHalfs,
        int refAxis, float refSign,
        const Eigen::Vector3f& normal, //A->B
        AxisType axisType, int axisA, int axisB,
        CollisionResult& outCollision)
    {
        Eigen::Vector3f refFaceNormal = refFaceRot.col(refAxis) * refSign;
        float refFaceOffset = refFaceNormal.dot(refFaceCenter) + refFaceHalfs[refAxis];

        int incFaceAxis; float incFaceSign;
        FindIncidentFace(incFaceRot, refFaceNormal, incFaceAxis, incFaceSign);

        Eigen::Vector3f incFaceVertices[4];
        GetBoxVerticesWorld(incFaceCenter, incFaceHalfs, incFaceRot, incFaceAxis, incFaceSign, incFaceVertices);

        int u = (refAxis + 1) % 3;
        int v = (refAxis + 2) % 3;

        struct ClipPlane { Eigen::Vector3f normal; float offset; };

        ClipPlane sidePlanes[4]{};
        sidePlanes[0].normal = refFaceRot.col(u); sidePlanes[0].offset = refFaceRot.col(u).dot(refFaceCenter) + refFaceHalfs[u];
        sidePlanes[1].normal = -refFaceRot.col(u); sidePlanes[1].offset = -refFaceRot.col(u).dot(refFaceCenter) + refFaceHalfs[u];
        sidePlanes[2].normal = refFaceRot.col(v); sidePlanes[2].offset = refFaceRot.col(v).dot(refFaceCenter) + refFaceHalfs[v];
        sidePlanes[3].normal = -refFaceRot.col(v); sidePlanes[3].offset = -refFaceRot.col(v).dot(refFaceCenter) + refFaceHalfs[v];

        // Collision "clip in incident face" points
        Eigen::Vector3f clipInputBuffer1[8], clipInputBuffer2[8];
        for (int i = 0; i < 4; ++i) clipInputBuffer1[i] = incFaceVertices[i];
        int numClipPoints = 4;

        for (int p = 0; p < 4; p++) // ping pong between two buffers to avoid copying clipped points
        {
            Eigen::Vector3f* source = (p % 2 == 0) ? clipInputBuffer1 : clipInputBuffer2;
            Eigen::Vector3f* dest = (p % 2 == 0) ? clipInputBuffer2 : clipInputBuffer1;

            numClipPoints = PolygonFaceClip(source, numClipPoints, sidePlanes[p].normal, sidePlanes[p].offset, dest);
            if (numClipPoints < 1) return;
        }

        Eigen::Vector3f* finalClipPoints = (4 % 2 == 0) ? clipInputBuffer1 : clipInputBuffer2;

        Eigen::Matrix3f invRotA = rotA.transpose();
        Eigen::Matrix3f invRotB = rotB.transpose();

        // Cull points in front of the reference face
        for (int i = 0; i < numClipPoints; i++)
        {
           float separation = refFaceNormal.dot(finalClipPoints[i]) - refFaceOffset;

            if (separation <= 0.f)
            {
                ContactPoint& contact = outCollision.contactPoints[outCollision.numContacts];

                Eigen::Vector3f projectedPoint = finalClipPoints[i] - refFaceNormal * separation;
                
                contact.position = projectedPoint;
                contact.normal = normal;
                contact.penetration = -separation;

                // Reference body → projected point (on the face surface)
                // Incident body  → original clip point (where it actually is)
                contact.rA = invRotA * (projectedPoint - posA);
                contact.rB = invRotB * (finalClipPoints[i] - posB);

                contact.id = createID(axisType, axisA, axisB, i);

                outCollision.numContacts++;
                if (outCollision.numContacts >= 8) return;
            }
        }
    }

    //================================//
    void ClosestPointTwoLineSegments(const Eigen::Vector3f& p1, const Eigen::Vector3f& d1, const Eigen::Vector3f& p2, const Eigen::Vector3f& d2, float& s, float& t)
    {
        // Line 1: p1 + s * d1
        // Line 2: p2 + t * d2, s, t in [0, 1]

        Eigen::Vector3f r = p1 - p2;
        float a = d1.dot(d1);
        float b = d1.dot(d2);
        float c = d2.dot(d2);
        float d = d1.dot(r);
        float e = d2.dot(r);

        float denom = a * c - b * b;

        if (denom < 1e-6f)
        {
            s = 0.0f; // Parallel line (or quasi parallet at least)
            t = (b > c ? d / b : e / c);
        }
        else
        {
            s = (b * e - c * d) / denom;
            t = (a * e - b * d) / denom;
        }

        s = std::clamp(s, 0.0f, 1.0f);
        t = std::clamp(t, 0.0f, 1.0f);
    }

    //================================//
    void LineLineContacts(
        const Eigen::Vector3f& posA, const Eigen::Vector3f& scaleA, const Eigen::Matrix3f& rotA,
        const Eigen::Vector3f& posB, const Eigen::Vector3f& scaleB, const Eigen::Matrix3f& rotB,
        const Eigen::Vector3f& normal, int axisA, int axisB, CollisionResult& outCollision)
    {
        Eigen::Vector3f d = posB - posA;

        int uA = (axisA + 1) % 3;
        int vA = (axisA + 2) % 3;

        float signUA = (rotA.col(uA).dot(d) > 0.f) ? 1.0f : -1.0f;
        float signVA = (rotA.col(vA).dot(d) > 0.f) ? 1.0f : -1.0f;

        Eigen::Vector3f midA = posA + rotA.col(uA) * (signUA * scaleA[uA] * 0.5f) + rotA.col(vA) * (signVA * scaleA[vA] * 0.5f);
        Eigen::Vector3f dirA = rotA.col(axisA);
        float lenA = scaleA[axisA];
        Eigen::Vector3f startA = midA - dirA * (lenA * 0.5f);
        Eigen::Vector3f endA = midA + dirA * (lenA * 0.5f);

        int uB = (axisB + 1) % 3;
        int vB = (axisB + 2) % 3;

        float signUB = (rotB.col(uB).dot(-d) > 0.f) ? 1.0f : -1.0f;
        float signVB = (rotB.col(vB).dot(-d) > 0.f) ? 1.0f : -1.0f;

        Eigen::Vector3f midB = posB + rotB.col(uB) * (signUB * scaleB[uB] * 0.5f) + rotB.col(vB) * (signVB * scaleB[vB] * 0.5f);
        Eigen::Vector3f dirB = rotB.col(axisB);
        float lenB = scaleB[axisB];
        Eigen::Vector3f startB = midB - dirB * (lenB * 0.5f);
        Eigen::Vector3f endB = midB + dirB * (lenB * 0.5f);

        float s, t;
        ClosestPointTwoLineSegments(startA, dirA * lenA, startB, dirB * lenB, s, t);

        Eigen::Vector3f closestCollisionPointOnA = startA + dirA * (s * lenA);
        Eigen::Vector3f closestCollisionPointOnB = startB + dirB * (t * lenB);

        ContactPoint& contact = outCollision.contactPoints[0];
        contact.position = 0.5f * (closestCollisionPointOnA + closestCollisionPointOnB);
        contact.normal = normal;
        contact.penetration = (closestCollisionPointOnB - closestCollisionPointOnA).dot(normal);
        if (contact.penetration < 0.f) contact.penetration = -contact.penetration;

        Eigen::Matrix3f invRotA = rotA.transpose();
        Eigen::Matrix3f invRotB = rotB.transpose();

        contact.rA = invRotA * (contact.position - posA);
        contact.rB = invRotB * (contact.position - posB);

        contact.id = createID(AxisType::EDGE, axisA, axisB, 0);
        outCollision.numContacts = 1; // Only one possible contact for sure
    }
}

//================================//
CollisionResult CollisionSpace::CollisionBoxBox(const Mesh* meshA, const Mesh* meshB)
{
    CollisionResult result{};
    result.numContacts = 0;

    Eigen::Vector3f posA, posB;
    Quaternionf     qA, qB;
    Eigen::Vector3f scaleA, scaleB;

    meshA->transform.GetPosition(posA);
    meshB->transform.GetPosition(posB);
    meshA->transform.GetRotation(qA);
    meshB->transform.GetRotation(qB);
    meshA->transform.GetScale(scaleA);
    meshB->transform.GetScale(scaleB);

    Eigen::Matrix3f rotA = qA.toRotationMatrix();
    Eigen::Matrix3f rotB = qB.toRotationMatrix();

    Eigen::Vector3f d = posB - posA;
    Eigen::Matrix3f C = rotA.transpose() * rotB;
    Eigen::Vector3f T = rotA.transpose() * d;

    Eigen::Matrix3f absoluteRotation;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            absoluteRotation(i, j) = std::abs(C(i, j)) + 1e-6f;

    // idea: test 15 ptential axis,
    float bestPenetration = std::numeric_limits<float>::max();
    int bestAxis = -1;
    float bestSign = 1.f;

    auto TestAxis = [&](int axisIndex, float separation, float sign) -> bool
    {
        if (separation < 0.f) return false; // SA found

        if (separation < bestPenetration)
        {
            bestPenetration = separation;
            bestAxis = axisIndex;
            bestSign = sign;
        }
        return true;
    };

    // First A faces (0, 1, 2)
    for (int i = 0; i < 3; i++)
    {
        float rA = scaleA[i] * 0.5f;
        float rB = absoluteRotation.row(i).dot(scaleB * 0.5f);
        float distance = std::abs(T[i]);
        float penetration = rA + rB - distance;
        float s = (T[i] > 0.f) ? 1.0f : -1.0f;

        if (!TestAxis(i, penetration, s)) return result;
    }

    // Then B faces (face axis of course I mean) (3, 4, 5)
    for (int i = 0; i < 3; i++)
    {
        float rA = absoluteRotation.col(i).dot(scaleA * 0.5f);
        float rB = scaleB[i] * 0.5f;
        float distance = std::abs(C.col(i).dot(T));
        float penetration = rA + rB - distance;
        float s = (T.dot(C.col(i)) > 0.f) ? 1.0f : -1.0f;

        if (!TestAxis(3 + i, penetration, s)) return result;
    }

    float bestFacePen = bestPenetration;


    // still need to test Edge Edge axes (6-14)
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float crossLenSQ = 1.f - C(i, j) * C(i, j);
            if (crossLenSQ < 1e-6f) continue; // PARALLEL

            float invLen = 1.f / std::sqrt(crossLenSQ);

            // OBB - OBB
            int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
            int j1 = (j + 1) % 3, j2 = (j + 2) % 3;

            float rA = scaleA[i1] * 0.5f * absoluteRotation(i2, j) + scaleA[i2] * 0.5f * absoluteRotation(i1, j);
            float rB = scaleB[j1] * 0.5f * absoluteRotation(i, j2) + scaleB[j2] * 0.5f * absoluteRotation(i, j1);
            float raw = T[i2] * C(i1, j) - T[i1] * C(i2, j);
            float distance = std::abs(raw);
            float penetration = (rA + rB - distance) * invLen;
            float s = (raw > 0.f) ? 1.0f : -1.0f;

            // Add a bias towards Face contacts
            const float FACE_BIAS_REL = 0.95f;
            const float FACE_BIAS_ABS = 0.005f;
            if (penetration < FACE_BIAS_REL * bestFacePen + FACE_BIAS_ABS)
            {
                if (!TestAxis(6 + i * 3 + j, penetration, s)) return result;
            }
        }
    }

    if (bestAxis < 0) return result; // No collision

    Eigen::Vector3f normal;
    if (bestAxis < 3)
    {
        normal = rotA.col(bestAxis) * bestSign;
    }
    else if (bestAxis < 6)
    {
        int bestAxisB = bestAxis - 3;
        normal = rotB.col(bestAxisB) * bestSign;
    }
    else
    {
        int i = (bestAxis - 6) / 3;
        int j = (bestAxis - 6) % 3;
        normal = rotA.col(i).cross(rotB.col(j));

        float len = normal.norm();
        if (len < 1e-6f) return result;
        normal /= len;

        if (normal.dot(d) < 0.f) normal = -normal; // Always point from A to B (convention)
    }

    // NOW is it a face contact situation? An edge contact situation?
    // remember: up to 4 contact points, so 8 relative contacts

    if (bestAxis < 3) // First case of testing agains faces of box A
    {
        int referenceFaceAxis = bestAxis;
        float referenceFaceSign = bestSign;

        // here the ref box is A
        FaceFaceContacts(
            posA, scaleA, rotA,
            posB, scaleB, rotB,
            posA, rotA, scaleA * 0.5f,
            posB, rotB, scaleB * 0.5f,
            referenceFaceAxis, referenceFaceSign,
            normal, AxisType::FACE_A, referenceFaceAxis, 0,
            result);
    }
    else if (bestAxis < 6) // Testing against faces of box B
    {
        int referenceFaceAxis = bestAxis - 3;
        float referenceFaceSign = -bestSign;

        // here the ref box is B
        FaceFaceContacts(
            posB, scaleB, rotB,
            posA, scaleA, rotA,
            posB, rotB, scaleB * 0.5f,
            posA, rotA, scaleA * 0.5f,
            referenceFaceAxis, referenceFaceSign,
            normal, AxisType::FACE_B, 0, referenceFaceAxis,
            result);

        for (int i = 0; i < result.numContacts; i++)
            std::swap(result.contactPoints[i].rA, result.contactPoints[i].rB);
    }
    else // edge edge
    {
        int edgeAxisA = (bestAxis - 6) / 3;
        int edgeAxisB = (bestAxis - 6) % 3;

        LineLineContacts(
            posA, scaleA, rotA,
            posB, scaleB, rotB,
            normal, edgeAxisA, edgeAxisB,
            result);
    }

    return result;
}