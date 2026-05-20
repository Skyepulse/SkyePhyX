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
            contact.id = MakeContactID(7u, 0u, 0u, 0u);

            result.numContacts = 1;
            return result;
        }
    }

    //================================//
    CollisionResult CollisionHullCapsule(const Mesh* meshA, const Mesh* meshB)
    {
        const CollisionShapeProxy proxyA = MakeCollisionProxy(meshA);
        const CollisionShapeProxy proxyB = MakeCollisionProxy(meshB);
        const GJKResult gjkResult = RunGJK(proxyA, proxyB);

        if (gjkResult.status == GJKStatus::Separated)
            return CollisionResult{};

        // Shallow capsule contacts are exactly the ReactPhysics3D margin case:
        // core hull/segment closest points, then expand by capsule radius.
        if (gjkResult.status == GJKStatus::CollideInMargin)
            return CreateContactFromGJK(meshA, meshB, gjkResult);

        // Deep segment-vs-hull overlap is decomposed by SAT into face or edge
        // contacts, which is where stacks and resting contact patches come from.
        return CollideHullCapsuleSAT(meshA, meshB);
    }
}
