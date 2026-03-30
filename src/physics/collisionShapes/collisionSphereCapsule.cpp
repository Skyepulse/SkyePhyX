#include <algorithm>
#include <cmath>
#include <limits>
#include "../collision.hpp"

//================================//
namespace CollisionSpace
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
    CollisionResult CollisionSphereCapsule(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;
        result.numContacts = 0;

        const bool sphereIsA = (meshA->modelType == ModelType_Sphere);
        const Mesh* sphereMesh = sphereIsA ? meshA : meshB;
        const Mesh* capsuleMesh = sphereIsA ? meshB : meshA;

        const Transform& sphereTransform = sphereMesh->transform;
        const Transform& capsuleTransform = capsuleMesh->transform;

        const Eigen::Vector3f sphereCenter = sphereTransform.GetPosition();
        const Eigen::Vector3f capsuleCenter = capsuleTransform.GetPosition();
        const Eigen::Matrix3f capsuleRotation = capsuleTransform.GetRotation().toRotationMatrix();
        const float unitScale = capsuleTransform.GetScale().x();

        float capsuleTotalHeight = 1.0f * unitScale;
        float bodyHeight = 0.5f * capsuleTotalHeight;
        float capsuleRadius = 0.25f * unitScale;
        float sphereRadius = 0.5f * sphereTransform.GetScale().x();
        float combinedRadius = sphereRadius + capsuleRadius;

        const Eigen::Vector3f capsuleAxis = capsuleRotation.col(1).normalized();
        const float halfSegmentLength = 0.5f * bodyHeight;
        const Eigen::Vector3f segmentStart = capsuleCenter - capsuleAxis * halfSegmentLength;
        const Eigen::Vector3f segmentEnd = capsuleCenter + capsuleAxis * halfSegmentLength;
        const Eigen::Vector3f segment = segmentEnd - segmentStart;

        Eigen::Vector3f L = capsuleCenter;
        const float segmentLengthSquared = segment.squaredNorm();
        if (segmentLengthSquared > 1e-12f)
        {
            const float t = std::clamp((sphereCenter - segmentStart).dot(segment) / segmentLengthSquared, 0.0f, 1.0f);
            L = segmentStart + t * segment;
        }

        const Eigen::Vector3f delta = sphereCenter - L;
        const float distanceSquared = delta.squaredNorm();
        if (distanceSquared > combinedRadius * combinedRadius)
            return result;

        Eigen::Vector3f normalCapsuleToSphere = Eigen::Vector3f::Zero();
        float distance = 0.0f;

        if (distanceSquared > 1e-12f)
        {
            distance = std::sqrt(distanceSquared);
            normalCapsuleToSphere = delta / distance;
        }
        else
        {
            normalCapsuleToSphere = AnyPerpendicular(capsuleAxis);
        }

        const Eigen::Vector3f pointOnCapsule = L + normalCapsuleToSphere * capsuleRadius;
        const Eigen::Vector3f pointOnSphere = sphereCenter - normalCapsuleToSphere * sphereRadius;
        const Eigen::Vector3f normalAToB = sphereIsA ? -normalCapsuleToSphere : normalCapsuleToSphere;
        const Eigen::Vector3f pointOnA = sphereIsA ? pointOnSphere : pointOnCapsule;
        const Eigen::Vector3f pointOnB = sphereIsA ? pointOnCapsule : pointOnSphere;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnA + pointOnB);
        contact.normal = normalAToB;
        contact.penetration = combinedRadius - distance;

        if (meshA->modelType == ModelType_Sphere)
            contact.rA = pointOnA - meshA->transform.GetPosition();
        else
            contact.rA = meshA->transform.GetRotation().toRotationMatrix().transpose() * (pointOnA - meshA->transform.GetPosition());

        if (meshB->modelType == ModelType_Sphere)
            contact.rB = pointOnB - meshB->transform.GetPosition();
        else
            contact.rB = meshB->transform.GetRotation().toRotationMatrix().transpose() * (pointOnB - meshB->transform.GetPosition());

        contact.id = 0xFFFFFFFAu;
        result.numContacts = 1;

        return result;
    }
}
