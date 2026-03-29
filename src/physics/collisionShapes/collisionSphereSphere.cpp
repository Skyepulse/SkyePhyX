#include <algorithm>
#include <cmath>
#include <limits>
#include "../collision.hpp"

namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionSphereSphere(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;
        result.numContacts = 0;

        const Transform& transformA = meshA->transform;
        const Transform& transformB = meshB->transform;

        const Eigen::Vector3f positionA = transformA.GetPosition();
        const Eigen::Vector3f positionB = transformB.GetPosition();

        const Eigen::Vector3f scaleA = transformA.GetScale();
        const Eigen::Vector3f scaleB = transformB.GetScale();

        // Base sphere mesh has radius 0.5 in model space.
        const float radiusA = 0.5f * scaleA.x();
        const float radiusB = 0.5f * scaleB.x();

        const Eigen::Vector3f delta = positionB - positionA;
        const float distanceSquared = delta.squaredNorm();
        const float radiusSum = radiusA + radiusB;

        if (distanceSquared > radiusSum * radiusSum)
            return result;

        Eigen::Vector3f normal = Eigen::Vector3f::UnitY();
        float distance = 0.0f;

        if (distanceSquared > 1e-12f)
        {
            distance = std::sqrt(distanceSquared);
            normal = delta / distance;
        }

        // One manifold point is enough for sphere-sphere collision
        const Eigen::Vector3f pointOnA = positionA + normal * radiusA;
        const Eigen::Vector3f pointOnB = positionB - normal * radiusB;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnA + pointOnB);
        contact.normal = normal;
        contact.penetration = radiusSum - distance;
        contact.rA = pointOnA - positionA;
        contact.rB = pointOnB - positionB;
        contact.id = 0xFFFFFFFEu;

        result.numContacts = 1;
        return result;
    }
}
