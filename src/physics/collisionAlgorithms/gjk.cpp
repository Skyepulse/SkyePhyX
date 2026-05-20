#include "gjk.hpp"
#include "voronoiSimplex.hpp"

#include <algorithm>
#include <cmath>

//================================//
namespace CollisionSpace
{
    static constexpr int GJK_MAX_ITERATIONS = 32;
    static constexpr float GJK_EPSILON = 1e-6f;
    static constexpr float GJK_REL_ERROR = 1e-3f;

    //================================//
    static GJKSupportPoint ComputeSupportPoint(const CollisionShapeProxy& proxyA,
                                               const CollisionShapeProxy& proxyB,
                                               const Eigen::Vector3f& direction)
    {
        GJKSupportPoint supportPoint;

        // The simplex lives in the Minkowski difference A - B.  ReactPhysics3D
        // queries the support that moves this difference toward the origin.
        // {CAREFUL: this is a (slightly) MODIFIED and (probably) worse version of the reactPhysics3D implementation. 
        // To see the original, check its codebase }
        supportPoint.supportA = SupportWithoutMargin(proxyA, direction);
        supportPoint.supportB = SupportWithoutMargin(proxyB, -direction);
        supportPoint.point = supportPoint.supportA - supportPoint.supportB;

        return supportPoint;
    }

    //================================//
    static Eigen::Vector3f NormalizeOrFallback(const Eigen::Vector3f& vector,
                                               const CollisionShapeProxy& proxyA,
                                               const CollisionShapeProxy& proxyB)
    {
        const float lengthSquared = vector.squaredNorm();
        if (lengthSquared > GJK_EPSILON * GJK_EPSILON)
            return vector / std::sqrt(lengthSquared);

        Eigen::Vector3f fallback = GetProxyCenter(proxyB) - GetProxyCenter(proxyA);
        const float fallbackLengthSquared = fallback.squaredNorm();
        if (fallbackLengthSquared > GJK_EPSILON * GJK_EPSILON)
            return fallback / std::sqrt(fallbackLengthSquared);

        return Eigen::Vector3f::UnitY();
    }

    //================================//
    static GJKResult BuildResultFromClosestPoints(const CollisionShapeProxy& proxyA,
                                                  const CollisionShapeProxy& proxyB,
                                                  const VoronoiSimplex& simplex,
                                                  const Eigen::Vector3f& closestPoint)
    {
        GJKResult result;

        Eigen::Vector3f pointCoreA = Eigen::Vector3f::Zero();
        Eigen::Vector3f pointCoreB = Eigen::Vector3f::Zero();
        simplex.ComputeClosestPoints(pointCoreA, pointCoreB);

        const float distanceSquared = closestPoint.squaredNorm();
        result.distance = std::sqrt(std::max(0.0f, distanceSquared));
        result.normalAToB = NormalizeOrFallback(-closestPoint, proxyA, proxyB);

        const float marginSum = proxyA.margin + proxyB.margin;
        if (result.distance > marginSum)
        {
            result.status = GJKStatus::Separated;
            result.pointOnA = pointCoreA;
            result.pointOnB = pointCoreB;
            result.penetration = 0.0f;
            return result;
        }

        result.status = GJKStatus::CollideInMargin;
        result.pointOnA = pointCoreA + result.normalAToB * proxyA.margin;
        result.pointOnB = pointCoreB - result.normalAToB * proxyB.margin;
        result.penetration = marginSum - result.distance;
        return result;
    }

    //================================//
    GJKResult RunGJK(const CollisionShapeProxy& proxyA, const CollisionShapeProxy& proxyB)
    {
        GJKResult result;

        if (!proxyA.mesh || !proxyB.mesh)
            return result;

        Eigen::Vector3f direction = GetProxyCenter(proxyA) - GetProxyCenter(proxyB);
        if (direction.squaredNorm() <= GJK_EPSILON * GJK_EPSILON)
            direction = Eigen::Vector3f::UnitX();

        VoronoiSimplex simplex;
        simplex.AddPoint(ComputeSupportPoint(proxyA, proxyB, direction));

        Eigen::Vector3f closestPoint = Eigen::Vector3f::Zero();
        if (!simplex.ComputeClosestPoint(closestPoint))
        {
            result.status = GJKStatus::Interpenetrating;
            return result;
        }

        float previousDistanceSquared = closestPoint.squaredNorm();

        for (int iteration = 0; iteration < GJK_MAX_ITERATIONS; ++iteration)
        {
            if (previousDistanceSquared <= GJK_EPSILON * GJK_EPSILON)
            {
                result.status = GJKStatus::Interpenetrating;
                return result;
            }

            direction = -closestPoint;
            const GJKSupportPoint supportPoint = ComputeSupportPoint(proxyA, proxyB, direction);

            if (simplex.ContainsPoint(supportPoint.point))
                return BuildResultFromClosestPoints(proxyA, proxyB, simplex, closestPoint);

            const float supportProgress = previousDistanceSquared - closestPoint.dot(supportPoint.point);
            const float progressTolerance = std::max(GJK_EPSILON, GJK_REL_ERROR * previousDistanceSquared);
            if (supportProgress <= progressTolerance)
                return BuildResultFromClosestPoints(proxyA, proxyB, simplex, closestPoint);

            simplex.AddPoint(supportPoint);

            if (simplex.IsAffinelyDependent())
                return BuildResultFromClosestPoints(proxyA, proxyB, simplex, closestPoint);

            if (!simplex.ComputeClosestPoint(closestPoint))
            {
                result.status = GJKStatus::Interpenetrating;
                return result;
            }

            const float distanceSquared = closestPoint.squaredNorm();
            if (distanceSquared <= GJK_EPSILON * GJK_EPSILON)
            {
                result.status = GJKStatus::Interpenetrating;
                return result;
            }

            if (simplex.IsFull())
            {
                result.status = GJKStatus::Interpenetrating;
                return result;
            }

            if (previousDistanceSquared - distanceSquared <= progressTolerance)
                return BuildResultFromClosestPoints(proxyA, proxyB, simplex, closestPoint);

            previousDistanceSquared = distanceSquared;
        }

        return BuildResultFromClosestPoints(proxyA, proxyB, simplex, closestPoint);
    }
}
