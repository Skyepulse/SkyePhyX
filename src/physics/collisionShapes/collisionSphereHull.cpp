#include "../collision.hpp"
#include "../collisionAlgorithms/gjk.hpp"
#include "../collisionAlgorithms/sat.hpp"
#include "../collisionAlgorithms/support.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    namespace
    {
        //================================//
        static CollisionResult CreateContactFromGJK(const Mesh* meshA,
                                                    const Mesh* meshB,
                                                    const GJKResult& gjkResult)
        {
            CollisionResult result;

            ContactPoint& contact = result.contactPoints[0];
            contact.position = 0.5f * (gjkResult.pointOnA + gjkResult.pointOnB);
            contact.normal = gjkResult.normalAToB;
            contact.penetration = gjkResult.penetration;
            contact.rA = ToLocalOffset(meshA, gjkResult.pointOnA);
            contact.rB = ToLocalOffset(meshB, gjkResult.pointOnB);
            contact.id = MakeContactID(6u, 0u, 0u, 0u);

            result.numContacts = 1;
            return result;
        }
    }

    //================================//
    CollisionResult CollisionSphereHull(const Mesh* meshA, const Mesh* meshB)
    {
        const CollisionShapeProxy proxyA = MakeCollisionProxy(meshA);
        const CollisionShapeProxy proxyB = MakeCollisionProxy(meshB);
        const GJKResult gjkResult = RunGJK(proxyA, proxyB);

        if (gjkResult.status == GJKStatus::Separated)
            return CollisionResult{};

        // GJK is simple here for shallow first test (simple as a how far the shapes are apart)
        if (gjkResult.status == GJKStatus::CollideInMargin)
            return CreateContactFromGJK(meshA, meshB, gjkResult);

        // When not reliable enough (penetration) fallback to classic
        // SAT to decide correctly which direction to push the sphere (ALSO NOT MAKE IT STUCK INSIDE THE HULL)
        return CollideSphereHullSAT(meshA, meshB);
    }
}
