#ifndef COLLISION_VORONOI_SIMPLEX_HPP
#define COLLISION_VORONOI_SIMPLEX_HPP

/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *                 
*                                                                               *
* Modified by Rios Mael 2026                                                    *
********************************************************************************/

#include "../../helpers/geometry.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    struct GJKSupportPoint
    {
        Eigen::Vector3f point = Eigen::Vector3f::Zero();
        Eigen::Vector3f supportA = Eigen::Vector3f::Zero();
        Eigen::Vector3f supportB = Eigen::Vector3f::Zero();
    };

    //================================//
    class VoronoiSimplex
    {
    public:
        VoronoiSimplex();

        bool IsFull() const { return numPoints == 4; }
        int GetNumPoints() const { return numPoints; }

        void AddPoint(const GJKSupportPoint& supportPoint);
        bool ContainsPoint(const Eigen::Vector3f& point) const;
        bool IsAffinelyDependent() const;
        bool ComputeClosestPoint(Eigen::Vector3f& outClosestPoint);
        void ComputeClosestPoints(Eigen::Vector3f& outPointA, Eigen::Vector3f& outPointB) const;
        float GetMaxLengthSquared() const;
        void BackupClosestPoint(Eigen::Vector3f& outClosestPoint) const;

    private:
        GJKSupportPoint points[4];
        float barycentricCoords[4];
        Eigen::Vector3f closestPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f closestPointA = Eigen::Vector3f::Zero();
        Eigen::Vector3f closestPointB = Eigen::Vector3f::Zero();
        int numPoints = 0;
        bool recomputeClosestPoint = false;
        bool isClosestPointValid = false;

        void SetBarycentricCoords(float a, float b, float c, float d);
        void RemovePoint(int index);
        void ReduceSimplex(int usedPointsBits);
        bool RecomputeClosestPoint();
        bool CheckClosestPointValid() const;

        void ComputeClosestPointOnSegment(const Eigen::Vector3f& a, const Eigen::Vector3f& b, int& outUsedPointsBits, float& outT) const;
        void ComputeClosestPointOnTriangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c,
                                           int& outUsedPointsBits, Eigen::Vector3f& outBarycentricCoords) const;
        bool ComputeClosestPointOnTetrahedron(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& d,
                                              int& outUsedPointsBits, Eigen::Vector4f& outBarycentricCoords, bool& outIsDegenerate) const;
        int TestOriginOutsidePlane(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& d) const;
        int MapTriangleUsedVerticesToTetrahedron(int triangleUsedVertices, int first, int second, int third) const;
    };
}

#endif // COLLISION_VORONOI_SIMPLEX_HPP
