#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "../helpers/geometry.hpp"
#include <array>

//================================//
struct ContactPoint
{
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    float penetration = 0.0f; // If positive: overlapping

    Eigen::Vector3f rA;
    Eigen::Vector3f rB;

    uint32_t id = 0;
};

//================================//
struct CollisionResult
{
    ContactPoint contactPoints[8];
    int numContacts = 0;
};

//================================//
namespace CollisionSpace
{
    CollisionResult CollisionBoxBox(const Mesh* A, const Mesh* B);
}

#endif // COLLISION_HPP