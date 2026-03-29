#include <algorithm>
#include <cmath>
#include "../collision.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionSphereBox(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;
        result.numContacts = 0;

        const bool sphereIsA = (meshA->modelType == ModelType_Sphere);
        const Mesh* sphereMesh = sphereIsA ? meshA : meshB;
        const Mesh* boxMesh = sphereIsA ? meshB : meshA;

        const Transform& sphereTransform = sphereMesh->transform;
        const Transform& boxTransform = boxMesh->transform;

        const Eigen::Vector3f sphereCenter = sphereTransform.GetPosition();
        const Eigen::Vector3f boxCenter = boxTransform.GetPosition();
        const Eigen::Matrix3f boxRotation = boxTransform.GetRotation().toRotationMatrix();
        const Eigen::Vector3f boxHalfExtents = 0.5f * boxTransform.GetScale();

        // Base sphere mesh has radius 0.5 in model space.
        const float sphereRadius = 0.5f * sphereTransform.GetScale().x();

        const Eigen::Vector3f sphereCenterLocal =
            boxRotation.transpose() * (sphereCenter - boxCenter);

        const Eigen::Vector3f closestPointLocal = sphereCenterLocal.cwiseMax(-boxHalfExtents).cwiseMin(boxHalfExtents);
        const Eigen::Vector3f deltaLocal = sphereCenterLocal - closestPointLocal;
        const float distanceSquared = deltaLocal.squaredNorm();

        Eigen::Vector3f pointOnBoxWorld = Eigen::Vector3f::Zero();
        Eigen::Vector3f pointOnSphereWorld = Eigen::Vector3f::Zero();
        Eigen::Vector3f normalWorldBoxToSphere = Eigen::Vector3f::Zero();
        float penetration = 0.0f;

        if (distanceSquared > 1e-12f)
        {
            const float distanceToBox = std::sqrt(distanceSquared);
            if (distanceToBox > sphereRadius)
                return result;

            const Eigen::Vector3f normalLocalBoxToSphere = deltaLocal / distanceToBox;
            normalWorldBoxToSphere = boxRotation * normalLocalBoxToSphere;
            penetration = sphereRadius - distanceToBox;

            pointOnBoxWorld = boxCenter + boxRotation * closestPointLocal;
            pointOnSphereWorld = sphereCenter - normalWorldBoxToSphere * sphereRadius;
        }
        else
        {
            int bestAxis = 0;
            float bestSign = 1.0f;
            float minDistanceToFace = boxHalfExtents.x() - std::abs(sphereCenterLocal.x());

            for (int axis = 1; axis < 3; ++axis)
            {
                const float distanceToFace = boxHalfExtents[axis] - std::abs(sphereCenterLocal[axis]);
                if (distanceToFace < minDistanceToFace)
                {
                    minDistanceToFace = distanceToFace;
                    bestAxis = axis;
                }
            }

            bestSign = (sphereCenterLocal[bestAxis] >= 0.0f) ? 1.0f : -1.0f;
            Eigen::Vector3f pointOnBoxLocal = sphereCenterLocal;
            pointOnBoxLocal[bestAxis] = bestSign * boxHalfExtents[bestAxis];

            Eigen::Vector3f normalLocalBoxToSphere = Eigen::Vector3f::Zero();
            normalLocalBoxToSphere[bestAxis] = bestSign;

            normalWorldBoxToSphere = boxRotation * normalLocalBoxToSphere;
            penetration = sphereRadius + minDistanceToFace;

            pointOnBoxWorld = boxCenter + boxRotation * pointOnBoxLocal;
            pointOnSphereWorld = sphereCenter + normalWorldBoxToSphere * sphereRadius;
        }

        const Eigen::Vector3f normalAToB = sphereIsA ? -normalWorldBoxToSphere : normalWorldBoxToSphere;
        const Eigen::Vector3f pointOnA = sphereIsA ? pointOnSphereWorld : pointOnBoxWorld;
        const Eigen::Vector3f pointOnB = sphereIsA ? pointOnBoxWorld : pointOnSphereWorld;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnA + pointOnB);
        contact.normal = normalAToB;
        contact.penetration = penetration;

        if (meshA->modelType == ModelType_Sphere)
            contact.rA = pointOnA - meshA->transform.GetPosition();
        else
            contact.rA = meshA->transform.GetRotation().toRotationMatrix().transpose() * (pointOnA - meshA->transform.GetPosition());

        if (meshB->modelType == ModelType_Sphere)
            contact.rB = pointOnB - meshB->transform.GetPosition();
        else
            contact.rB = meshB->transform.GetRotation().toRotationMatrix().transpose() * (pointOnB - meshB->transform.GetPosition());

        contact.id = 0xFFFFFFFCu;

        result.numContacts = 1;
        return result;
    }
}
