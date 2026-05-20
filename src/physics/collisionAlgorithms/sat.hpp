#ifndef COLLISION_SAT_HPP
#define COLLISION_SAT_HPP

/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *                 
*                                                                               *
* Modified by Rios Mael 2026                                                    *
********************************************************************************/

#include "../collision.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    CollisionResult CollideSphereHullSAT(const Mesh* meshA, const Mesh* meshB);
    CollisionResult CollideHullCapsuleSAT(const Mesh* meshA, const Mesh* meshB);
    CollisionResult CollideHullHullSAT(const Mesh* meshA, const Mesh* meshB);
}

#endif // COLLISION_SAT_HPP
