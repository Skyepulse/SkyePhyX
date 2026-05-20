#include "../collision.hpp"
#include "../collisionAlgorithms/sat.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionHullHull(const Mesh* meshA, const Mesh* meshB)
    {
        // ReactPhysics3D runs convex polyhedron pairs directly through SAT.
        // Keeping GJK out of this path avoids tolerance-driven contact flicker
        // before the SAT face clipper has a chance to build a manifold.
        return CollideHullHullSAT(meshA, meshB);
    }
}
