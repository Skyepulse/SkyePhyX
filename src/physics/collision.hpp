#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "../helpers/geometry.hpp"
#include <array>

class Solver;

//================================//
struct FaceQuery
{
    double separation;
    uint16_t faceIndex;
};

//================================//
struct EdgeQuery
{
    double separation;
    uint16_t edgeIndexA;
    uint16_t edgeIndexB;
};

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
    void ClosestPointsOnSegments(const Eigen::Vector3f& a0, const Eigen::Vector3f& a1,
                                 const Eigen::Vector3f& b0, const Eigen::Vector3f& b1,
                                 Eigen::Vector3f& pointA, Eigen::Vector3f& pointB);

    CollisionResult CollisionMeshMesh(const Mesh* A, const Mesh* B);
    CollisionResult CollisionSphereSphere(const Mesh* A, const Mesh* B);
    CollisionResult CollisionSphereBox(const Mesh* A, const Mesh* B);
    CollisionResult CollisionSphereHull(const Mesh* A, const Mesh* B);
    CollisionResult CollisionBoxBox(const Mesh* A, const Mesh* B);
    CollisionResult CollisionHullHull(const Mesh* A, const Mesh* B);
}

#endif // COLLISION_HPP
