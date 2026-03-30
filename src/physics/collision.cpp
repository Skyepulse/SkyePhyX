#include <algorithm>
#include <cmath>
#include "collision.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    void ClosestPointsOnSegments(const Eigen::Vector3f& a0, const Eigen::Vector3f& a1,
                                 const Eigen::Vector3f& b0, const Eigen::Vector3f& b1,
                                 Eigen::Vector3f& pointA, Eigen::Vector3f& pointB)
    {
        const Eigen::Vector3f d1 = a1 - a0;
        const Eigen::Vector3f d2 = b1 - b0;
        const Eigen::Vector3f r = a0 - b0;

        const float a = d1.dot(d1);
        const float e = d2.dot(d2);
        const float f = d2.dot(r);

        float s = 0.0f;
        float t = 0.0f;

        if (a <= 1e-12f && e <= 1e-12f)
        {
            pointA = a0;
            pointB = b0;
            return;
        }

        if (a <= 1e-12f)
        {
            t = std::clamp(f / e, 0.0f, 1.0f);
        }
        else
        {
            const float c = d1.dot(r);
            if (e <= 1e-12f)
            {
                s = std::clamp(-c / a, 0.0f, 1.0f);
            }
            else
            {
                const float b = d1.dot(d2);
                const float denom = a * e - b * b;

                if (denom > 1e-12f)
                    s = std::clamp((b * f - c * e) / denom, 0.0f, 1.0f);

                t = (b * s + f) / e;

                if (t < 0.0f)
                {
                    t = 0.0f;
                    s = std::clamp(-c / a, 0.0f, 1.0f);
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;
                    s = std::clamp((b - c) / a, 0.0f, 1.0f);
                }
            }
        }

        pointA = a0 + d1 * s;
        pointB = b0 + d2 * t;
    }

    //================================//
    CollisionResult CollisionMeshMesh(const Mesh* meshA, const Mesh* meshB)
    {
        if (!meshA || !meshB)
            return CollisionResult{};

        if ((meshA->modelType == ModelType_Sphere && meshB->modelType == ModelType_Cube) ||
            (meshA->modelType == ModelType_Cube && meshB->modelType == ModelType_Sphere))
        {
            return CollisionSphereBox(meshA, meshB);
        }

        if (meshA->modelType == ModelType_Sphere && meshB->modelType == ModelType_Capsule ||
            meshA->modelType == ModelType_Capsule && meshB->modelType == ModelType_Sphere)
        {
            return CollisionSphereCapsule(meshA, meshB);
        }

        if (meshA->modelType == ModelType_Sphere && meshB->modelType == ModelType_Sphere)
            return CollisionSphereSphere(meshA, meshB);

        if (meshA->modelType == ModelType_Sphere || meshB->modelType == ModelType_Sphere)
            return CollisionSphereHull(meshA, meshB);

        if (meshA->modelType == ModelType_Cube && meshB->modelType == ModelType_Capsule ||
            meshA->modelType == ModelType_Capsule && meshB->modelType == ModelType_Cube)
        {
            return CollisionBoxCapsule(meshA, meshB);
        }

        if (meshA->modelType == ModelType_Cube && meshB->modelType == ModelType_Cube)
            return CollisionBoxBox(meshA, meshB);

        if (meshA->modelType == ModelType_Capsule && meshB->modelType == ModelType_Capsule)
            return CollisionCapsuleCapsule(meshA, meshB);

        if (meshA->modelType == ModelType_Capsule || meshB->modelType == ModelType_Capsule)
            return CollisionHullCapsule(meshA, meshB);

        return CollisionHullHull(meshA, meshB);
    }
}
