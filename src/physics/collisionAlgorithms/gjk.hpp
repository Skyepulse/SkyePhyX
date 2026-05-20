#ifndef COLLISION_GJK_HPP
#define COLLISION_GJK_HPP

/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *                 
*                                                                               *
* Modified by Rios Mael 2026                                                    *
********************************************************************************/

#include "support.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    enum class GJKStatus
    {
        Separated,
        CollideInMargin,
        Interpenetrating
    };

    //================================//
    struct GJKResult
    {
        GJKStatus status = GJKStatus::Separated;

        Eigen::Vector3f normalAToB = Eigen::Vector3f::UnitY();
        Eigen::Vector3f pointOnA = Eigen::Vector3f::Zero();
        Eigen::Vector3f pointOnB = Eigen::Vector3f::Zero();

        float penetration = 0.0f;
        float distance = 0.0f;
    };

    //================================//
    GJKResult RunGJK(const CollisionShapeProxy& proxyA, const CollisionShapeProxy& proxyB);
}

#endif // COLLISION_GJK_HPP
