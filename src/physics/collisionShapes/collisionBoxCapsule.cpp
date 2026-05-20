#include "../collision.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollisionBoxCapsule(const Mesh* meshA, const Mesh* meshB)
    {
        return CollisionHullCapsule(meshA, meshB);
    }
}
