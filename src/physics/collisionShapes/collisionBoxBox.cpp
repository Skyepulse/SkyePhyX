#include "../collision.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionBoxBox(const Mesh* meshA, const Mesh* meshB)
    {
        return CollisionHullHull(meshA, meshB);
    }
}
