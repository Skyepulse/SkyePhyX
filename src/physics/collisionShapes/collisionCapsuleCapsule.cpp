#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "../collision.hpp"

//================================//
namespace CollisionSpace
{
    namespace
    {
        //================================//
        static Eigen::Vector3f AnyPerpendicular(const Eigen::Vector3f& axis)
        {
            Eigen::Vector3f perpendicular =
                (std::abs(axis.x()) < 0.9f)
                    ? axis.cross(Eigen::Vector3f::UnitX())
                    : axis.cross(Eigen::Vector3f::UnitZ());

            const float lengthSquared = perpendicular.squaredNorm();
            if (lengthSquared <= 1e-12f)
                return Eigen::Vector3f::UnitY();

            return perpendicular / std::sqrt(lengthSquared);
        }

        //================================//
        static void GetCapsuleInnerSegment(const Mesh* mesh, Eigen::Vector3f& outStart, Eigen::Vector3f& outEnd, Eigen::Vector3f& outAxis, float& outRadius)
        {
            const Transform& transform = mesh->transform;
            const float scale = transform.GetScale().x();
            const float bodyHeight = 0.5f * scale;
            const float halfSegmentLength = 0.5f * bodyHeight;

            outAxis = transform.GetRotation().toRotationMatrix().col(1).normalized();
            outRadius = 0.25f * scale;
            outStart = transform.GetPosition() - outAxis * halfSegmentLength;
            outEnd = transform.GetPosition() + outAxis * halfSegmentLength;
        }

        //================================//
        static bool ClipSegmentAgainstCapsuleSlab(const Eigen::Vector3f& segmentStart,
                                                  const Eigen::Vector3f& segmentEnd,
                                                  const Eigen::Vector3f& slabStart,
                                                  const Eigen::Vector3f& slabEnd,
                                                  const Eigen::Vector3f& slabAxis,
                                                  Eigen::Vector3f& outStart,
                                                  Eigen::Vector3f& outEnd)
        {
            const float slabMin = slabAxis.dot(slabStart);
            const float slabMax = slabAxis.dot(slabEnd);
            const float d0 = slabAxis.dot(segmentStart);
            const float d1 = slabAxis.dot(segmentEnd);
            const float dd = d1 - d0;

            float tMin = 0.0f;
            float tMax = 1.0f;

            if (std::abs(dd) <= 1e-12f)
            {
                if (d0 < slabMin || d0 > slabMax)
                    return false;
            }
            else
            {
                float t0 = (slabMin - d0) / dd;
                float t1 = (slabMax - d0) / dd;
                if (t0 > t1)
                    std::swap(t0, t1);

                tMin = std::max(tMin, t0);
                tMax = std::min(tMax, t1);
                if (tMin > tMax)
                    return false;
            }

            const Eigen::Vector3f segment = segmentEnd - segmentStart;
            outStart = segmentStart + tMin * segment;
            outEnd = segmentStart + tMax * segment;
            return true;
        }

        //================================//
        static void AddCapsuleCapsuleContact(const Mesh* meshA, const Mesh* meshB,
                                             const Eigen::Vector3f& linePointA,
                                             const Eigen::Vector3f& linePointB,
                                             float radiusA, float radiusB,
                                             const Eigen::Vector3f& capsuleAxisA,
                                             const Eigen::Vector3f& capsuleAxisB,
                                             uint32_t contactId,
                                             CollisionResult& result)
        {
            if (result.numContacts >= 8)
                return;

            const Eigen::Vector3f delta = linePointB - linePointA;
            const float distanceSquared = delta.squaredNorm();
            const float combinedRadius = radiusA + radiusB;
            if (distanceSquared > combinedRadius * combinedRadius)
                return;

            Eigen::Vector3f normal = Eigen::Vector3f::Zero();
            float distance = 0.0f;

            if (distanceSquared > 1e-12f)
            {
                distance = std::sqrt(distanceSquared);
                normal = delta / distance;
            }
            else
            {
                normal = capsuleAxisA.cross(capsuleAxisB);
                const float normalLengthSquared = normal.squaredNorm();
                if (normalLengthSquared > 1e-12f)
                {
                    normal /= std::sqrt(normalLengthSquared);
                    const Eigen::Vector3f centerDelta = meshB->transform.GetPosition() - meshA->transform.GetPosition();
                    if (normal.dot(centerDelta) < 0.0f)
                        normal = -normal;
                }
                else
                {
                    normal = AnyPerpendicular(capsuleAxisA);
                }
            }

            const Eigen::Vector3f pointOnA = linePointA + normal * radiusA;
            const Eigen::Vector3f pointOnB = linePointB - normal * radiusB;

            ContactPoint& contact = result.contactPoints[result.numContacts];
            contact.position = 0.5f * (pointOnA + pointOnB);
            contact.normal = normal;
            contact.penetration = combinedRadius - distance;
            contact.rA = meshA->transform.GetRotation().toRotationMatrix().transpose() * (pointOnA - meshA->transform.GetPosition());
            contact.rB = meshB->transform.GetRotation().toRotationMatrix().transpose() * (pointOnB - meshB->transform.GetPosition());
            contact.id = contactId;
            result.numContacts++;
        }
    }

    //================================//
    CollisionResult CollisionCapsuleCapsule(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;
        result.numContacts = 0;

        Eigen::Vector3f segmentStartA = Eigen::Vector3f::Zero();
        Eigen::Vector3f segmentEndA = Eigen::Vector3f::Zero();
        Eigen::Vector3f segmentStartB = Eigen::Vector3f::Zero();
        Eigen::Vector3f segmentEndB = Eigen::Vector3f::Zero();
        Eigen::Vector3f capsuleAxisA = Eigen::Vector3f::UnitY();
        Eigen::Vector3f capsuleAxisB = Eigen::Vector3f::UnitY();
        float radiusA = 0.0f;
        float radiusB = 0.0f;

        GetCapsuleInnerSegment(meshA, segmentStartA, segmentEndA, capsuleAxisA, radiusA);
        GetCapsuleInnerSegment(meshB, segmentStartB, segmentEndB, capsuleAxisB, radiusB);

        const float combinedRadius = radiusA + radiusB;
        const Eigen::Vector3f axisCross = capsuleAxisA.cross(capsuleAxisB);
        const bool areParallel = axisCross.squaredNorm() <= 1e-4f;

        if (areParallel) // allow stacking
        {
            Eigen::Vector3f clippedStartB = Eigen::Vector3f::Zero();
            Eigen::Vector3f clippedEndB = Eigen::Vector3f::Zero();
            if (ClipSegmentAgainstCapsuleSlab(segmentStartB, segmentEndB, segmentStartA, segmentEndA, capsuleAxisA, clippedStartB, clippedEndB))
            {
                std::vector<Eigen::Vector3f> candidates;
                candidates.push_back(clippedStartB);
                if ((clippedEndB - clippedStartB).squaredNorm() > 1e-12f)
                    candidates.push_back(clippedEndB);

                for (size_t i = 0; i < candidates.size(); ++i)
                {
                    const Eigen::Vector3f pointB = candidates[i];
                    const float projection = std::clamp(
                        capsuleAxisA.dot(pointB - segmentStartA),
                        0.0f,
                        (segmentEndA - segmentStartA).norm()
                    );
                    const Eigen::Vector3f pointA = segmentStartA + projection * capsuleAxisA;

                    AddCapsuleCapsuleContact(
                        meshA, meshB,
                        pointA, pointB,
                        radiusA, radiusB,
                        capsuleAxisA, capsuleAxisB,
                        0xFFFFFFF0u + static_cast<uint32_t>(i),
                        result
                    );
                }

                if (result.numContacts > 0)
                    return result;
            }
        }

        Eigen::Vector3f closestPointA = Eigen::Vector3f::Zero();
        Eigen::Vector3f closestPointB = Eigen::Vector3f::Zero();
        ClosestPointsOnSegments(segmentStartA, segmentEndA, segmentStartB, segmentEndB, closestPointA, closestPointB);

        AddCapsuleCapsuleContact(
            meshA, meshB,
            closestPointA, closestPointB,
            radiusA, radiusB,
            capsuleAxisA, capsuleAxisB,
            0xFFFFFFF2u,
            result
        );

        return result;
    }
}
