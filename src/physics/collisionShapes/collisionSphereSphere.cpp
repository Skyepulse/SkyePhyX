#include "../collision.hpp"
#include "../collisionAlgorithms/support.hpp"

#include <cmath>

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionSphereSphere(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;

        const Eigen::Vector3f centerA = meshA->transform.GetPosition();
        const Eigen::Vector3f centerB = meshB->transform.GetPosition();
        const float radiusA = 0.5f * meshA->transform.GetScale().x();
        const float radiusB = 0.5f * meshB->transform.GetScale().x();
        const float combinedRadius = radiusA + radiusB;

        const Eigen::Vector3f delta = centerB - centerA;
        const float distanceSquared = delta.squaredNorm();
        if (distanceSquared > combinedRadius * combinedRadius)
            return result;

        float distance = 0.0f;
        Eigen::Vector3f normalAToB = Eigen::Vector3f::UnitY();
        if (distanceSquared > 1e-12f)
        {
            distance = std::sqrt(distanceSquared);
            normalAToB = delta / distance;
        }

        // Sphere contacts are a single point on the line of centers.
        const Eigen::Vector3f pointOnA = centerA + normalAToB * radiusA;
        const Eigen::Vector3f pointOnB = centerB - normalAToB * radiusB;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnA + pointOnB);
        contact.normal = normalAToB;
        contact.penetration = combinedRadius - distance;
        contact.rA = ToLocalOffset(meshA, pointOnA);
        contact.rB = ToLocalOffset(meshB, pointOnB);
        contact.id = MakeContactID(15u, 0u, 0u, 0u);

        result.numContacts = 1;
        return result;
    }
}
