#include "../collision.hpp"
#include "../collisionAlgorithms/support.hpp"

#include <algorithm>
#include <cmath>

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionSphereCapsule(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;

        const bool sphereIsA = meshA->modelType == ModelType_Sphere;
        const Mesh* sphereMesh = sphereIsA ? meshA : meshB;
        const Mesh* capsuleMesh = sphereIsA ? meshB : meshA;

        const Eigen::Vector3f sphereCenter = sphereMesh->transform.GetPosition();
        const float sphereRadius = 0.5f * sphereMesh->transform.GetScale().x();
        const CapsuleSegment capsule = GetCapsuleSegment(capsuleMesh);

        const Eigen::Vector3f segment = capsule.end - capsule.start;
        const float segmentLengthSquared = segment.squaredNorm();

        Eigen::Vector3f pointOnSegment = capsule.center;
        if (segmentLengthSquared > 1e-12f)
        {
            const float t = std::clamp((sphereCenter - capsule.start).dot(segment) / segmentLengthSquared, 0.0f, 1.0f);
            pointOnSegment = capsule.start + t * segment;
        }

        const Eigen::Vector3f delta = sphereCenter - pointOnSegment;
        const float distanceSquared = delta.squaredNorm();
        const float combinedRadius = sphereRadius + capsule.radius;
        if (distanceSquared > combinedRadius * combinedRadius)
            return result;

        float distance = 0.0f;
        Eigen::Vector3f normalCapsuleToSphere = AnyPerpendicular(capsule.axis);
        if (distanceSquared > 1e-12f)
        {
            distance = std::sqrt(distanceSquared);
            normalCapsuleToSphere = delta / distance;
        }

        const Eigen::Vector3f pointOnCapsule = pointOnSegment + normalCapsuleToSphere * capsule.radius;
        const Eigen::Vector3f pointOnSphere = sphereCenter - normalCapsuleToSphere * sphereRadius;

        const Eigen::Vector3f normalAToB = sphereIsA ? -normalCapsuleToSphere : normalCapsuleToSphere;
        const Eigen::Vector3f pointOnA = sphereIsA ? pointOnSphere : pointOnCapsule;
        const Eigen::Vector3f pointOnB = sphereIsA ? pointOnCapsule : pointOnSphere;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnA + pointOnB);
        contact.normal = normalAToB;
        contact.penetration = combinedRadius - distance;
        contact.rA = ToLocalOffset(meshA, pointOnA);
        contact.rB = ToLocalOffset(meshB, pointOnB);
        contact.id = MakeContactID(15u, 1u, 0u, 0u);

        result.numContacts = 1;
        return result;
    }
}
