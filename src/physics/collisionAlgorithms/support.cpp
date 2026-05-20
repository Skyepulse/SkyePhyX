#include "support.hpp"
#include "../solver.hpp"

#include <algorithm>
#include <cmath>

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionShapeProxy MakeCollisionProxy(const Mesh* mesh)
    {
        CollisionShapeProxy proxy;
        proxy.mesh = mesh;

        if (!mesh)
            return proxy;

        if (mesh->modelType == ModelType_Sphere)
        {
            proxy.type = CollisionProxyType::Sphere;
            proxy.margin = 0.5f * mesh->transform.GetScale().x();
            return proxy;
        }

        if (mesh->modelType == ModelType_Capsule)
        {
            proxy.type = CollisionProxyType::Capsule;
            proxy.capsule = GetCapsuleSegment(mesh);
            proxy.margin = proxy.capsule.radius;
            return proxy;
        }

        proxy.type = CollisionProxyType::ConvexHull;
        proxy.hull = &mesh->solver->GetModelConvexHull(mesh->modelType);
        proxy.margin = 0.0f;
        return proxy;
    }

    //================================//
    CapsuleSegment GetCapsuleSegment(const Mesh* mesh)
    {
        CapsuleSegment capsule;
        if (!mesh)
            return capsule;

        const Transform& transform = mesh->transform;
        const float scale = transform.GetScale().x();
        const float halfSegmentLength = 0.25f * scale;

        capsule.center = transform.GetPosition();
        capsule.axis = transform.GetRotation().toRotationMatrix().col(1);
        if (capsule.axis.squaredNorm() <= 1e-12f)
            capsule.axis = Eigen::Vector3f::UnitY();
        else
            capsule.axis.normalize();

        capsule.start = capsule.center - capsule.axis * halfSegmentLength;
        capsule.end = capsule.center + capsule.axis * halfSegmentLength;
        capsule.radius = 0.25f * scale;

        return capsule;
    }

    //================================//
    Eigen::Vector3f AnyPerpendicular(const Eigen::Vector3f& axis)
    {
        Eigen::Vector3f perpendicular = (std::abs(axis.x()) < 0.9f) ? axis.cross(Eigen::Vector3f::UnitX()) : axis.cross(Eigen::Vector3f::UnitZ());

        const float lengthSquared = perpendicular.squaredNorm();
        if (lengthSquared <= 1e-12f)
            return Eigen::Vector3f::UnitY();

        return perpendicular / std::sqrt(lengthSquared);
    }

    //================================//
    Eigen::Vector3f GetProxyCenter(const CollisionShapeProxy& proxy)
    {
        if (proxy.type == CollisionProxyType::Capsule)
            return proxy.capsule.center;

        if (proxy.mesh)
            return proxy.mesh->transform.GetPosition();

        return Eigen::Vector3f::Zero();
    }

    //================================//
    Eigen::Vector3f SupportSegment(const CapsuleSegment& capsule, const Eigen::Vector3f& direction)
    {
        return (capsule.start.dot(direction) > capsule.end.dot(direction)) ? capsule.start : capsule.end;
    }

    //================================//
    Eigen::Vector3f SupportWithoutMargin(const CollisionShapeProxy& proxy, const Eigen::Vector3f& directionWorld)
    {
        if (!proxy.mesh)
            return Eigen::Vector3f::Zero();

        if (proxy.type == CollisionProxyType::Sphere)
            return proxy.mesh->transform.GetPosition();

        if (proxy.type == CollisionProxyType::Capsule)
            return SupportSegment(proxy.capsule, directionWorld);

        if (!proxy.hull || proxy.hull->vertices.empty())
            return proxy.mesh->transform.GetPosition();

        const Transform& transform = proxy.mesh->transform;
        const Eigen::Matrix3f linear = transform.GetRotation().toRotationMatrix() * transform.GetScale().asDiagonal();
        const Eigen::Vector3f directionLocal = linear.transpose() * directionWorld;
        const Eigen::Vector3f supportLocal = proxy.hull->GetSupport(directionLocal);

        return transform.TransformPoint(supportLocal);
    }

    //================================//
    Eigen::Vector3f SupportWithMargin(const CollisionShapeProxy& proxy, const Eigen::Vector3f& directionWorld)
    {
        Eigen::Vector3f support = SupportWithoutMargin(proxy, directionWorld);

        if (proxy.margin <= 0.0f)
            return support;

        // Add margin to the support in order to prevent the GJK algorithm from failing due to numerical issues
        // when the shapes are very close.
        Eigen::Vector3f direction = directionWorld;
        const float lengthSquared = direction.squaredNorm();
        if (lengthSquared <= 1e-12f)
            direction = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
        else
            direction /= std::sqrt(lengthSquared);

        return support + direction * proxy.margin;
    }

    //================================//
    Eigen::Vector3f TransformNormalToWorld(const Transform& transform, const Eigen::Vector3f& localNormal)
    {
        const Eigen::Matrix3f linear =
            transform.GetRotation().toRotationMatrix() * transform.GetScale().asDiagonal();
        Eigen::Vector3f worldNormal = linear.inverse().transpose() * localNormal;

        const float lengthSquared = worldNormal.squaredNorm();
        if (lengthSquared <= 1e-12f)
            return Eigen::Vector3f::Zero();

        return worldNormal / std::sqrt(lengthSquared);
    }

    //================================//
    Eigen::Vector3f ToLocalOffset(const Mesh* mesh, const Eigen::Vector3f& worldPoint)
    {
        if (!mesh)
            return Eigen::Vector3f::Zero();

        if (mesh->modelType == ModelType_Sphere)
            return worldPoint - mesh->transform.GetPosition();

        return ToLocalOffset(mesh->transform, worldPoint);
    }

    //================================//
    Eigen::Vector3f ToLocalOffset(const Transform& transform, const Eigen::Vector3f& worldPoint)
    {
        return transform.GetRotation().toRotationMatrix().transpose() * (worldPoint - transform.GetPosition());
    }

    //================================//
    uint32_t MakeContactID(uint32_t type, uint32_t featureA, uint32_t featureB, uint32_t index)
    {
        return ((type & 0xFu) << 28)
             | ((featureA & 0xFFFu) << 16)
             | ((featureB & 0xFFFu) << 4)
             | (index & 0xFu);
    }
}
