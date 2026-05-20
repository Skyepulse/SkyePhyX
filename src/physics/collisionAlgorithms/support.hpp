#ifndef COLLISION_SUPPORT_HPP
#define COLLISION_SUPPORT_HPP

#include "../../helpers/geometry.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    enum class CollisionProxyType
    {
        Sphere,
        Capsule,
        ConvexHull
    };

    //================================//
    struct CapsuleSegment
    {
        Eigen::Vector3f start = Eigen::Vector3f::Zero();
        Eigen::Vector3f end = Eigen::Vector3f::Zero();
        Eigen::Vector3f axis = Eigen::Vector3f::UnitY();
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        float radius = 0.0f;
    };

    //================================//
    // A kind of all-in-one toy we can hurl all the content we need 
    // to determine correct collision support.
    struct CollisionShapeProxy
    {
        const Mesh* mesh = nullptr;
        CollisionProxyType type = CollisionProxyType::ConvexHull;
        const ConvexHull* hull = nullptr;
        CapsuleSegment capsule;
        float margin = 0.0f;
    };

    //================================//
    CollisionShapeProxy MakeCollisionProxy(const Mesh* mesh);
    CapsuleSegment GetCapsuleSegment(const Mesh* mesh);

    Eigen::Vector3f AnyPerpendicular(const Eigen::Vector3f& axis);
    Eigen::Vector3f GetProxyCenter(const CollisionShapeProxy& proxy);
    Eigen::Vector3f SupportSegment(const CapsuleSegment& capsule, const Eigen::Vector3f& direction);
    Eigen::Vector3f SupportWithoutMargin(const CollisionShapeProxy& proxy, const Eigen::Vector3f& directionWorld);
    Eigen::Vector3f SupportWithMargin(const CollisionShapeProxy& proxy, const Eigen::Vector3f& directionWorld);

    Eigen::Vector3f TransformNormalToWorld(const Transform& transform, const Eigen::Vector3f& localNormal);
    Eigen::Vector3f ToLocalOffset(const Mesh* mesh, const Eigen::Vector3f& worldPoint);
    Eigen::Vector3f ToLocalOffset(const Transform& transform, const Eigen::Vector3f& worldPoint);

    uint32_t MakeContactID(uint32_t type, uint32_t featureA, uint32_t featureB, uint32_t index);
}

#endif // COLLISION_SUPPORT_HPP
