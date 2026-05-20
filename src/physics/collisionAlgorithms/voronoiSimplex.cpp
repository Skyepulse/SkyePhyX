#include "voronoiSimplex.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

//================================//
namespace CollisionSpace
{
    static constexpr float SIMPLEX_EPSILON = 0.0001f;

    //================================//
    VoronoiSimplex::VoronoiSimplex()
    {
        SetBarycentricCoords(0.0f, 0.0f, 0.0f, 0.0f);
    }

    //================================//
    void VoronoiSimplex::AddPoint(const GJKSupportPoint& supportPoint)
    {
        assert(!IsFull());

        points[numPoints] = supportPoint;
        numPoints++;
        recomputeClosestPoint = true;
    }

    //================================//
    bool VoronoiSimplex::ContainsPoint(const Eigen::Vector3f& point) const
    {
        for (int i = 0; i < numPoints; ++i)
        {
            if ((points[i].point - point).squaredNorm() <= SIMPLEX_EPSILON)
                return true;
        }

        return false;
    }

    //================================//
    bool VoronoiSimplex::IsAffinelyDependent() const
    {
        switch (numPoints)
        {
            case 0:
            case 1:
                return false;
            case 2:
                return (points[1].point - points[0].point).squaredNorm() <= SIMPLEX_EPSILON;
            case 3:
                return (points[1].point - points[0].point).cross(points[2].point - points[0].point).squaredNorm() <= SIMPLEX_EPSILON;
            case 4:
                return std::abs((points[1].point - points[0].point).dot((points[2].point - points[0].point).cross(points[3].point - points[0].point))) <= SIMPLEX_EPSILON;
            default:
                return false;
        }
    }

    //================================//
    bool VoronoiSimplex::ComputeClosestPoint(Eigen::Vector3f& outClosestPoint)
    {
        const bool valid = RecomputeClosestPoint();
        outClosestPoint = closestPoint;
        return valid;
    }

    //================================//
    void VoronoiSimplex::ComputeClosestPoints(Eigen::Vector3f& outPointA, Eigen::Vector3f& outPointB) const
    {
        outPointA = closestPointA;
        outPointB = closestPointB;
    }

    //================================//
    float VoronoiSimplex::GetMaxLengthSquared() const
    {
        float maxLengthSquared = 0.0f;
        for (int i = 0; i < numPoints; ++i)
            maxLengthSquared = std::max(maxLengthSquared, points[i].point.squaredNorm());

        return maxLengthSquared;
    }

    //================================//
    void VoronoiSimplex::BackupClosestPoint(Eigen::Vector3f& outClosestPoint) const
    {
        outClosestPoint = closestPoint;
    }

    //================================//
    void VoronoiSimplex::SetBarycentricCoords(float a, float b, float c, float d)
    {
        barycentricCoords[0] = a;
        barycentricCoords[1] = b;
        barycentricCoords[2] = c;
        barycentricCoords[3] = d;
    }

    //================================//
    void VoronoiSimplex::RemovePoint(int index)
    {
        assert(numPoints > 0);

        numPoints--;
        points[index] = points[numPoints];
    }

    //================================//
    void VoronoiSimplex::ReduceSimplex(int usedPointsBits)
    {
        if (numPoints >= 4 && (usedPointsBits & 8) == 0)
            RemovePoint(3);
        if (numPoints >= 3 && (usedPointsBits & 4) == 0)
            RemovePoint(2);
        if (numPoints >= 2 && (usedPointsBits & 2) == 0)
            RemovePoint(1);
        if (numPoints >= 1 && (usedPointsBits & 1) == 0)
            RemovePoint(0);
    }

    //================================//
    bool VoronoiSimplex::RecomputeClosestPoint()
    {
        if (!recomputeClosestPoint)
            return isClosestPointValid;

        recomputeClosestPoint = false;

        switch (numPoints)
        {
            case 0:
            {
                isClosestPointValid = false;
                break;
            }
            case 1:
            {
                closestPoint = points[0].point;
                closestPointA = points[0].supportA;
                closestPointB = points[0].supportB;
                SetBarycentricCoords(1.0f, 0.0f, 0.0f, 0.0f);
                isClosestPointValid = CheckClosestPointValid();
                break;
            }
            case 2:
            {
                int usedPointsBits = 0;
                float t = 0.0f;
                ComputeClosestPointOnSegment(points[0].point, points[1].point, usedPointsBits, t);

                closestPointA = points[0].supportA + t * (points[1].supportA - points[0].supportA);
                closestPointB = points[0].supportB + t * (points[1].supportB - points[0].supportB);
                closestPoint = closestPointA - closestPointB;
                SetBarycentricCoords(1.0f - t, t, 0.0f, 0.0f);
                isClosestPointValid = CheckClosestPointValid();
                ReduceSimplex(usedPointsBits);
                break;
            }
            case 3:
            {
                int usedPointsBits = 0;
                Eigen::Vector3f barycentric = Eigen::Vector3f::Zero();
                ComputeClosestPointOnTriangle(points[0].point, points[1].point, points[2].point, usedPointsBits, barycentric);

                closestPointA = barycentric.x() * points[0].supportA + barycentric.y() * points[1].supportA + barycentric.z() * points[2].supportA;
                closestPointB = barycentric.x() * points[0].supportB + barycentric.y() * points[1].supportB + barycentric.z() * points[2].supportB;
                closestPoint = closestPointA - closestPointB;
                SetBarycentricCoords(barycentric.x(), barycentric.y(), barycentric.z(), 0.0f);
                isClosestPointValid = CheckClosestPointValid();
                ReduceSimplex(usedPointsBits);
                break;
            }
            case 4:
            {
                int usedPointsBits = 0;
                bool isDegenerate = false;
                Eigen::Vector4f barycentric = Eigen::Vector4f::Zero();
                const bool isOutside = ComputeClosestPointOnTetrahedron(points[0].point, points[1].point, points[2].point, points[3].point,
                                                                        usedPointsBits, barycentric, isDegenerate);
                if (isOutside)
                {
                    closestPointA = barycentric.x() * points[0].supportA + barycentric.y() * points[1].supportA +
                                    barycentric.z() * points[2].supportA + barycentric.w() * points[3].supportA;
                    closestPointB = barycentric.x() * points[0].supportB + barycentric.y() * points[1].supportB +
                                    barycentric.z() * points[2].supportB + barycentric.w() * points[3].supportB;
                    closestPoint = closestPointA - closestPointB;
                    SetBarycentricCoords(barycentric.x(), barycentric.y(), barycentric.z(), barycentric.w());
                    ReduceSimplex(usedPointsBits);
                    isClosestPointValid = CheckClosestPointValid();
                }
                else
                {
                    SetBarycentricCoords(0.0f, 0.0f, 0.0f, 0.0f);
                    closestPoint.setZero();
                    closestPointA.setZero();
                    closestPointB.setZero();
                    isClosestPointValid = !isDegenerate;
                }
                break;
            }
        }

        return isClosestPointValid;
    }

    //================================//
    bool VoronoiSimplex::CheckClosestPointValid() const
    {
        return barycentricCoords[0] >= 0.0f &&
               barycentricCoords[1] >= 0.0f &&
               barycentricCoords[2] >= 0.0f &&
               barycentricCoords[3] >= 0.0f;
    }

    //================================//
    void VoronoiSimplex::ComputeClosestPointOnSegment(const Eigen::Vector3f& a, const Eigen::Vector3f& b, int& outUsedPointsBits, float& outT) const
    {
        const Eigen::Vector3f ap = -a;
        const Eigen::Vector3f ab = b - a;
        const float apDotAb = ap.dot(ab);

        if (apDotAb > 0.0f)
        {
            const float lengthAbSquared = ab.squaredNorm();
            if (apDotAb < lengthAbSquared)
            {
                outT = apDotAb / lengthAbSquared;
                outUsedPointsBits = 3;
            }
            else
            {
                outT = 1.0f;
                outUsedPointsBits = 2;
            }
        }
        else
        {
            outT = 0.0f;
            outUsedPointsBits = 1;
        }
    }

    //================================//
    void VoronoiSimplex::ComputeClosestPointOnTriangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c,
                                                       int& outUsedPointsBits, Eigen::Vector3f& outBarycentricCoords) const
    {
        const Eigen::Vector3f ab = b - a;
        const Eigen::Vector3f ac = c - a;
        const Eigen::Vector3f ap = -a;
        const float d1 = ab.dot(ap);
        const float d2 = ac.dot(ap);

        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            outBarycentricCoords = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
            outUsedPointsBits = 1;
            return;
        }

        const Eigen::Vector3f bp = -b;
        const float d3 = ab.dot(bp);
        const float d4 = ac.dot(bp);
        if (d3 >= 0.0f && d4 <= d3)
        {
            outBarycentricCoords = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            outUsedPointsBits = 2;
            return;
        }

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            const float v = d1 / (d1 - d3);
            outBarycentricCoords = Eigen::Vector3f(1.0f - v, v, 0.0f);
            outUsedPointsBits = 3;
            return;
        }

        const Eigen::Vector3f cp = -c;
        const float d5 = ab.dot(cp);
        const float d6 = ac.dot(cp);
        if (d6 >= 0.0f && d5 <= d6)
        {
            outBarycentricCoords = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            outUsedPointsBits = 4;
            return;
        }

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            const float w = d2 / (d2 - d6);
            outBarycentricCoords = Eigen::Vector3f(1.0f - w, 0.0f, w);
            outUsedPointsBits = 5;
            return;
        }

        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            outBarycentricCoords = Eigen::Vector3f(0.0f, 1.0f - w, w);
            outUsedPointsBits = 6;
            return;
        }

        const float denominator = 1.0f / (va + vb + vc);
        const float v = vb * denominator;
        const float w = vc * denominator;
        outBarycentricCoords = Eigen::Vector3f(1.0f - v - w, v, w);
        outUsedPointsBits = 7;
    }

    //================================//
    bool VoronoiSimplex::ComputeClosestPointOnTetrahedron(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& d,
                                                          int& outUsedPointsBits, Eigen::Vector4f& outBarycentricCoords, bool& outIsDegenerate) const
    {
        outIsDegenerate = false;
        outUsedPointsBits = 15;
        outBarycentricCoords.setZero();

        const int outsideABC = TestOriginOutsidePlane(a, b, c, d);
        const int outsideACD = TestOriginOutsidePlane(a, c, d, b);
        const int outsideADB = TestOriginOutsidePlane(a, d, b, c);
        const int outsideBDC = TestOriginOutsidePlane(b, d, c, a);

        if (outsideABC < 0 || outsideACD < 0 || outsideADB < 0 || outsideBDC < 0)
        {
            outIsDegenerate = true;
            return false;
        }

        if (outsideABC == 0 && outsideACD == 0 && outsideADB == 0 && outsideBDC == 0)
            return false;

        float closestDistanceSquared = std::numeric_limits<float>::max();

        if (outsideABC != 0)
        {
            int used = 0;
            Eigen::Vector3f bary = Eigen::Vector3f::Zero();
            ComputeClosestPointOnTriangle(a, b, c, used, bary);
            const Eigen::Vector3f point = bary.x() * a + bary.y() * b + bary.z() * c;
            const float distanceSquared = point.squaredNorm();
            if (distanceSquared < closestDistanceSquared)
            {
                closestDistanceSquared = distanceSquared;
                outBarycentricCoords = Eigen::Vector4f(bary.x(), bary.y(), bary.z(), 0.0f);
                outUsedPointsBits = used;
            }
        }

        if (outsideACD != 0)
        {
            int used = 0;
            Eigen::Vector3f bary = Eigen::Vector3f::Zero();
            ComputeClosestPointOnTriangle(a, c, d, used, bary);
            const Eigen::Vector3f point = bary.x() * a + bary.y() * c + bary.z() * d;
            const float distanceSquared = point.squaredNorm();
            if (distanceSquared < closestDistanceSquared)
            {
                closestDistanceSquared = distanceSquared;
                outBarycentricCoords = Eigen::Vector4f(bary.x(), 0.0f, bary.y(), bary.z());
                outUsedPointsBits = MapTriangleUsedVerticesToTetrahedron(used, 0, 2, 3);
            }
        }

        if (outsideADB != 0)
        {
            int used = 0;
            Eigen::Vector3f bary = Eigen::Vector3f::Zero();
            ComputeClosestPointOnTriangle(a, d, b, used, bary);
            const Eigen::Vector3f point = bary.x() * a + bary.y() * d + bary.z() * b;
            const float distanceSquared = point.squaredNorm();
            if (distanceSquared < closestDistanceSquared)
            {
                closestDistanceSquared = distanceSquared;
                outBarycentricCoords = Eigen::Vector4f(bary.x(), bary.z(), 0.0f, bary.y());
                outUsedPointsBits = MapTriangleUsedVerticesToTetrahedron(used, 0, 3, 1);
            }
        }

        if (outsideBDC != 0)
        {
            int used = 0;
            Eigen::Vector3f bary = Eigen::Vector3f::Zero();
            ComputeClosestPointOnTriangle(b, d, c, used, bary);
            const Eigen::Vector3f point = bary.x() * b + bary.y() * d + bary.z() * c;
            const float distanceSquared = point.squaredNorm();
            if (distanceSquared < closestDistanceSquared)
            {
                outBarycentricCoords = Eigen::Vector4f(0.0f, bary.x(), bary.z(), bary.y());
                outUsedPointsBits = MapTriangleUsedVerticesToTetrahedron(used, 1, 3, 2);
            }
        }

        return true;
    }

    //================================//
    int VoronoiSimplex::TestOriginOutsidePlane(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& d) const
    {
        const Eigen::Vector3f normal = (b - a).cross(c - a);
        const float signOrigin = (-a).dot(normal);
        const float signD = (d - a).dot(normal);

        if (signD * signD < SIMPLEX_EPSILON * SIMPLEX_EPSILON)
            return -1;

        return (signOrigin * signD < 0.0f) ? 1 : 0;
    }

    //================================//
    int VoronoiSimplex::MapTriangleUsedVerticesToTetrahedron(int triangleUsedVertices, int first, int second, int third) const
    {
        int tetrahedronUsedVertices = 0;

        if ((triangleUsedVertices & 1) != 0)
            tetrahedronUsedVertices |= (1 << first);
        if ((triangleUsedVertices & 2) != 0)
            tetrahedronUsedVertices |= (1 << second);
        if ((triangleUsedVertices & 4) != 0)
            tetrahedronUsedVertices |= (1 << third);

        return tetrahedronUsedVertices;
    }
}
