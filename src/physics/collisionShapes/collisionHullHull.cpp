#include "../collision.hpp"
#include "../collisionAlgorithms/gjk.hpp"
#include "../collisionAlgorithms/sat.hpp"
#include "../collisionAlgorithms/support.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionHullHull(const Mesh* meshA, const Mesh* meshB)
    {
        const CollisionShapeProxy proxyA = MakeCollisionProxy(meshA);
        const CollisionShapeProxy proxyB = MakeCollisionProxy(meshB);
        const GJKResult gjkResult = RunGJK(proxyA, proxyB);

        if (gjkResult.status == GJKStatus::Separated)
            return CollisionResult{};

        // For polygonal shapes we use GJK as the fast overlap gate, then SAT for
        // the real manifold because SAT can clip a full face patch.
        return CollideHullHullSAT(meshA, meshB);
    }
}
