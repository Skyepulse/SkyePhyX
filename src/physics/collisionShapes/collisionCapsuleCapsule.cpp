#include "../collision.hpp"
#include "../collisionAlgorithms/support.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

//================================//
namespace CollisionSpace
{
    //================================//
    namespace
    {
        //================================//
        static bool ClipSegmentAgainstSlab(const Eigen::Vector3f& segmentStart,
                                           const Eigen::Vector3f& segmentEnd,
                                           const Eigen::Vector3f& slabStart,
                                           const Eigen::Vector3f& slabEnd,
                                           const Eigen::Vector3f& slabAxis,
                                           Eigen::Vector3f& outStart,
                                           Eigen::Vector3f& outEnd)
        {
            const float slabMin = slabAxis.dot(slabStart);
            const float slabMax = slabAxis.dot(slabEnd);
            const float distanceStart = slabAxis.dot(segmentStart);
            const float distanceEnd = slabAxis.dot(segmentEnd);
            const float distanceDelta = distanceEnd - distanceStart;

            float tMin = 0.0f;
            float tMax = 1.0f;

            if (std::abs(distanceDelta) <= 1e-12f)
            {
                if (distanceStart < slabMin || distanceStart > slabMax)
                    return false;
            }
            else
            {
                float t0 = (slabMin - distanceStart) / distanceDelta;
                float t1 = (slabMax - distanceStart) / distanceDelta;
                if (t0 > t1)
                    std::swap(t0, t1);

                tMin = std::max(tMin, t0);
                tMax = std::min(tMax, t1);
                if (tMin > tMax)
                    return false;
            }

            const Eigen::Vector3f segment = segmentEnd - segmentStart;
            outStart = segmentStart + segment * tMin;
            outEnd = segmentStart + segment * tMax;
            return true;
        }

        //================================//
        static void AddCapsuleContact(const Mesh* meshA,
                                      const Mesh* meshB,
                                      const CapsuleSegment& capsuleA,
                                      const CapsuleSegment& capsuleB,
                                      const Eigen::Vector3f& linePointA,
                                      const Eigen::Vector3f& linePointB,
                                      uint32_t contactIndex,
                                      CollisionResult& result)
        {
            if (result.numContacts >= 8)
                return;

            const Eigen::Vector3f delta = linePointB - linePointA;
            const float distanceSquared = delta.squaredNorm();
            const float combinedRadius = capsuleA.radius + capsuleB.radius;
            if (distanceSquared > combinedRadius * combinedRadius)
                return;

            float distance = 0.0f;
            Eigen::Vector3f normalAToB = capsuleA.axis.cross(capsuleB.axis);
            if (distanceSquared > 1e-12f)
            {
                distance = std::sqrt(distanceSquared);
                normalAToB = delta / distance;
            }
            else if (normalAToB.squaredNorm() > 1e-12f)
            {
                normalAToB.normalize();
                if (normalAToB.dot(capsuleB.center - capsuleA.center) < 0.0f)
                    normalAToB = -normalAToB;
            }
            else
            {
                normalAToB = AnyPerpendicular(capsuleA.axis);
            }

            const Eigen::Vector3f pointOnA = linePointA + normalAToB * capsuleA.radius;
            const Eigen::Vector3f pointOnB = linePointB - normalAToB * capsuleB.radius;

            ContactPoint& contact = result.contactPoints[result.numContacts];
            contact.position = 0.5f * (pointOnA + pointOnB);
            contact.normal = normalAToB;
            contact.penetration = combinedRadius - distance;
            contact.rA = ToLocalOffset(meshA, pointOnA);
            contact.rB = ToLocalOffset(meshB, pointOnB);
            contact.id = MakeContactID(15u, 2u, 0u, contactIndex);

            result.numContacts++;
        }
    }

    //================================//
    CollisionResult CollisionCapsuleCapsule(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;

        const CapsuleSegment capsuleA = GetCapsuleSegment(meshA);
        const CapsuleSegment capsuleB = GetCapsuleSegment(meshB);

        const Eigen::Vector3f axisCross = capsuleA.axis.cross(capsuleB.axis);
        const bool parallelSegments = axisCross.squaredNorm() <= 1e-4f;

        if (parallelSegments)
        {
            Eigen::Vector3f clippedStartB = Eigen::Vector3f::Zero();
            Eigen::Vector3f clippedEndB = Eigen::Vector3f::Zero();
            if (ClipSegmentAgainstSlab(capsuleB.start, capsuleB.end,
                                       capsuleA.start, capsuleA.end,
                                       capsuleA.axis,
                                       clippedStartB, clippedEndB))
            {
                std::vector<Eigen::Vector3f> candidates;
                candidates.push_back(clippedStartB);
                if ((clippedEndB - clippedStartB).squaredNorm() > 1e-12f)
                    candidates.push_back(clippedEndB);

                for (int i = 0; i < static_cast<int>(candidates.size()); ++i)
                {
                    const Eigen::Vector3f candidateB = candidates[i];
                    const float segmentLength = (capsuleA.end - capsuleA.start).norm();
                    const float projection = std::clamp(capsuleA.axis.dot(candidateB - capsuleA.start), 0.0f, segmentLength);
                    const Eigen::Vector3f candidateA = capsuleA.start + capsuleA.axis * projection;

                    AddCapsuleContact(meshA, meshB, capsuleA, capsuleB,
                                      candidateA, candidateB,
                                      static_cast<uint32_t>(i), result);
                }

                if (result.numContacts > 0)
                    return result;
            }
        }

        Eigen::Vector3f pointA = Eigen::Vector3f::Zero();
        Eigen::Vector3f pointB = Eigen::Vector3f::Zero();
        ClosestPointsOnSegments(capsuleA.start, capsuleA.end,
                                capsuleB.start, capsuleB.end,
                                pointA, pointB);

        AddCapsuleContact(meshA, meshB, capsuleA, capsuleB, pointA, pointB, 0u, result);
        return result;
    }
}
