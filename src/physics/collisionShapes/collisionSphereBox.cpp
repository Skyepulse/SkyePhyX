#include "../collision.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionSphereBox(const Mesh* meshA, const Mesh* meshB)
    {
        return CollisionSphereHull(meshA, meshB);
    }
}
