#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "../collision.hpp"
#include "../solver.hpp"

//================================//
namespace CollisionSpace
{
    //================================//
    namespace
    {
        //================================//
        struct GJKSupportPoint
        {
            Eigen::Vector3f csoPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f hullPoint = Eigen::Vector3f::Zero();
        };

        //================================//
        struct PointHullGJKResult
        {
            bool containsOrigin = false;
            Eigen::Vector3f closestCSOPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f closestHullPoint = Eigen::Vector3f::Zero();
            float distanceSquared = std::numeric_limits<float>::infinity();
        };

        //================================//
        static Eigen::Vector3f ToLocalOffset(const Transform& transform, const Eigen::Vector3f& worldPoint)
        {
            return transform.GetRotation().toRotationMatrix().transpose() * (worldPoint - transform.GetPosition());
        }

        //================================//
        static Eigen::Vector3f TransformNormalToWorld(const Transform& transform, const Eigen::Vector3f& localNormal)
        {
            const Eigen::Matrix3f linear = transform.GetRotation().toRotationMatrix() * transform.GetScale().asDiagonal();
            Eigen::Vector3f worldNormal = linear.inverse().transpose() * localNormal;
            const float lengthSquared = worldNormal.squaredNorm();
            if (lengthSquared <= 1e-12f)
                return Eigen::Vector3f::Zero();

            return worldNormal / std::sqrt(lengthSquared);
        }

        //================================//
        static GJKSupportPoint SupportHullAgainstPoint(const Transform& hullTransform, const ConvexHull& hull, const Eigen::Vector3f& pointWorld, const Eigen::Vector3f& directionWorld)
        {
            const Eigen::Matrix3f linear = hullTransform.GetRotation().toRotationMatrix() * hullTransform.GetScale().asDiagonal();
            const Eigen::Vector3f directionLocal = linear.transpose() * directionWorld;
            const Eigen::Vector3f hullSupportLocal = hull.GetSupport(directionLocal);
            const Eigen::Vector3f hullSupportWorld = hullTransform.TransformPoint(hullSupportLocal);

            GJKSupportPoint support;
            support.csoPoint = hullSupportWorld - pointWorld;
            support.hullPoint = hullSupportWorld;
            return support;
        }

        //================================//
        static void ReduceSegmentSimplex(std::vector<GJKSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint)
        {
            const GJKSupportPoint a = simplex[0];
            const GJKSupportPoint b = simplex[1];
            const Eigen::Vector3f ab = b.csoPoint - a.csoPoint;
            const float abLengthSquared = ab.squaredNorm();

            if (abLengthSquared <= 1e-12f)
            {
                simplex = { a };
                closestCSOPoint = a.csoPoint;
                closestHullPoint = a.hullPoint;
                return;
            }

            const float t = std::clamp(-a.csoPoint.dot(ab) / abLengthSquared, 0.0f, 1.0f);
            const float wA = 1.0f - t;
            const float wB = t;

            closestCSOPoint = wA * a.csoPoint + wB * b.csoPoint;
            closestHullPoint = wA * a.hullPoint + wB * b.hullPoint;

            if (t <= 1e-6f)
                simplex = { a };
            else if (t >= 1.0f - 1e-6f)
                simplex = { b };
            else
                simplex = { a, b };
        }

        //================================//
        static void ReduceTriangleSimplex(std::vector<GJKSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint)
        {
            const GJKSupportPoint a = simplex[0];
            const GJKSupportPoint b = simplex[1];
            const GJKSupportPoint c = simplex[2];

            const Eigen::Vector3f ab = b.csoPoint - a.csoPoint;
            const Eigen::Vector3f ac = c.csoPoint - a.csoPoint;
            const Eigen::Vector3f ap = -a.csoPoint;

            const float d1 = ab.dot(ap);
            const float d2 = ac.dot(ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                simplex = { a };
                closestCSOPoint = a.csoPoint;
                closestHullPoint = a.hullPoint;
                return;
            }

            const Eigen::Vector3f bp = -b.csoPoint;
            const float d3 = ab.dot(bp);
            const float d4 = ac.dot(bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                simplex = { b };
                closestCSOPoint = b.csoPoint;
                closestHullPoint = b.hullPoint;
                return;
            }

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                const float v = d1 / (d1 - d3);
                const float u = 1.0f - v;
                simplex = { a, b };
                closestCSOPoint = u * a.csoPoint + v * b.csoPoint;
                closestHullPoint = u * a.hullPoint + v * b.hullPoint;
                return;
            }

            const Eigen::Vector3f cp = -c.csoPoint;
            const float d5 = ab.dot(cp);
            const float d6 = ac.dot(cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                simplex = { c };
                closestCSOPoint = c.csoPoint;
                closestHullPoint = c.hullPoint;
                return;
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                const float w = d2 / (d2 - d6);
                const float u = 1.0f - w;
                simplex = { a, c };
                closestCSOPoint = u * a.csoPoint + w * c.csoPoint;
                closestHullPoint = u * a.hullPoint + w * c.hullPoint;
                return;
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                const float v = 1.0f - w;
                simplex = { b, c };
                closestCSOPoint = v * b.csoPoint + w * c.csoPoint;
                closestHullPoint = v * b.hullPoint + w * c.hullPoint;
                return;
            }

            const float denom = 1.0f / (va + vb + vc);
            const float v = vb * denom;
            const float w = vc * denom;
            const float u = 1.0f - v - w;

            simplex = { a, b, c };
            closestCSOPoint = u * a.csoPoint + v * b.csoPoint + w * c.csoPoint;
            closestHullPoint = u * a.hullPoint + v * b.hullPoint + w * c.hullPoint;
        }

        //================================//
        static bool OriginOutsidePlane(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& d)
        {
            const Eigen::Vector3f normal = (b - a).cross(c - a);
            const float signOrigin = (-a).dot(normal);
            const float signOpposite = (d - a).dot(normal);

            if (std::abs(signOpposite) <= 1e-6f)
                return false;

            return signOrigin * signOpposite < -1e-6f;
        }

        //================================//
        static bool ReduceTetrahedronSimplex(std::vector<GJKSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint)
        {
            const std::vector<GJKSupportPoint> tetrahedron = simplex;

            float bestDistanceSquared = std::numeric_limits<float>::infinity();
            std::vector<GJKSupportPoint> bestFaceSimplex;
            Eigen::Vector3f bestClosestCSO = Eigen::Vector3f::Zero();
            Eigen::Vector3f bestClosestHull = Eigen::Vector3f::Zero();

            auto testFace = [&](int ia, int ib, int ic, int id)
            {
                const Eigen::Vector3f& a = tetrahedron[ia].csoPoint;
                const Eigen::Vector3f& b = tetrahedron[ib].csoPoint;
                const Eigen::Vector3f& c = tetrahedron[ic].csoPoint;
                const Eigen::Vector3f& d = tetrahedron[id].csoPoint;

                if (!OriginOutsidePlane(a, b, c, d))
                    return;

                std::vector<GJKSupportPoint> faceSimplex = { tetrahedron[ia], tetrahedron[ib], tetrahedron[ic] };
                Eigen::Vector3f faceClosestCSO = Eigen::Vector3f::Zero();
                Eigen::Vector3f faceClosestHull = Eigen::Vector3f::Zero();
                ReduceTriangleSimplex(faceSimplex, faceClosestCSO, faceClosestHull);

                const float distanceSquared = faceClosestCSO.squaredNorm();
                if (distanceSquared < bestDistanceSquared)
                {
                    bestDistanceSquared = distanceSquared;
                    bestFaceSimplex = faceSimplex;
                    bestClosestCSO = faceClosestCSO;
                    bestClosestHull = faceClosestHull;
                }
            };

            testFace(0, 1, 2, 3);
            testFace(0, 2, 3, 1);
            testFace(0, 3, 1, 2);
            testFace(1, 3, 2, 0);

            if (bestFaceSimplex.empty())
                return true;

            simplex = std::move(bestFaceSimplex);
            closestCSOPoint = bestClosestCSO;
            closestHullPoint = bestClosestHull;
            return false;
        }

        //================================//
        static bool ReduceSimplex(std::vector<GJKSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint)
        {
            switch (simplex.size())
            {
                case 1:
                    closestCSOPoint = simplex[0].csoPoint;
                    closestHullPoint = simplex[0].hullPoint;
                    return closestCSOPoint.squaredNorm() <= 1e-12f;
                case 2:
                    ReduceSegmentSimplex(simplex, closestCSOPoint, closestHullPoint);
                    return closestCSOPoint.squaredNorm() <= 1e-12f;
                case 3:
                    ReduceTriangleSimplex(simplex, closestCSOPoint, closestHullPoint);
                    return closestCSOPoint.squaredNorm() <= 1e-12f;
                case 4:
                    return ReduceTetrahedronSimplex(simplex, closestCSOPoint, closestHullPoint);
                default:
                    closestCSOPoint = Eigen::Vector3f::Zero();
                    closestHullPoint = Eigen::Vector3f::Zero();
                    return false;
            }
        }

        //================================//
        static PointHullGJKResult ComputePointHullDistance(const Transform& hullTransform, const ConvexHull& hull, const Eigen::Vector3f& pointWorld)
        {
            PointHullGJKResult result;
            if (hull.vertexCount() == 0)
                return result;

            std::vector<GJKSupportPoint> simplex;
            simplex.reserve(4);

            Eigen::Vector3f direction = hullTransform.TransformPoint(hull.centroid) - pointWorld;
            if (direction.squaredNorm() <= 1e-12f)
                direction = hullTransform.TransformPoint(hull.vertices[0].position) - pointWorld;
            if (direction.squaredNorm() <= 1e-12f)
                direction = Eigen::Vector3f::UnitX();

            simplex.push_back(SupportHullAgainstPoint(hullTransform, hull, pointWorld, direction));

            Eigen::Vector3f closestCSOPoint = simplex[0].csoPoint;
            Eigen::Vector3f closestHullPoint = simplex[0].hullPoint;

            for (int iteration = 0; iteration < 32; ++iteration)
            {
                if (ReduceSimplex(simplex, closestCSOPoint, closestHullPoint))
                {
                    result.containsOrigin = true;
                    result.closestCSOPoint = closestCSOPoint;
                    result.closestHullPoint = closestHullPoint;
                    result.distanceSquared = closestCSOPoint.squaredNorm();
                    return result;
                }

                const Eigen::Vector3f searchDirection = -closestCSOPoint;
                if (searchDirection.squaredNorm() <= 1e-12f)
                {
                    result.containsOrigin = true;
                    result.closestCSOPoint = closestCSOPoint;
                    result.closestHullPoint = closestHullPoint;
                    result.distanceSquared = closestCSOPoint.squaredNorm();
                    return result;
                }

                const GJKSupportPoint support = SupportHullAgainstPoint(hullTransform, hull, pointWorld, searchDirection);

                bool duplicateSupport = false;
                for (const GJKSupportPoint& simplexPoint : simplex)
                {
                    if ((simplexPoint.csoPoint - support.csoPoint).squaredNorm() <= 1e-12f)
                    {
                        duplicateSupport = true;
                        break;
                    }
                }

                const float progress = closestCSOPoint.squaredNorm() - closestCSOPoint.dot(support.csoPoint);
                if (duplicateSupport || progress <= 1e-6f)
                {
                    result.containsOrigin = false;
                    result.closestCSOPoint = closestCSOPoint;
                    result.closestHullPoint = closestHullPoint;
                    result.distanceSquared = closestCSOPoint.squaredNorm();
                    return result;
                }

                simplex.push_back(support);
            }

            result.containsOrigin = false;
            result.closestCSOPoint = closestCSOPoint;
            result.closestHullPoint = closestHullPoint;
            result.distanceSquared = closestCSOPoint.squaredNorm();
            return result;
        }

        //================================//
        static bool QueryDeepPenetrationFace(const Transform& hullTransform, const ConvexHull& hull, const Eigen::Vector3f& sphereCenter, uint16_t& outFaceIndex, float& outSeparation, Eigen::Vector3f& outNormal)
        {
            outFaceIndex = 0;
            outSeparation = -std::numeric_limits<float>::infinity();
            outNormal = Eigen::Vector3f::Zero();

            for (uint16_t faceIndex = 0; faceIndex < hull.faceCount(); ++faceIndex)
            {
                const HullFace& face = hull.faces[faceIndex];
                if (face.vertexIndices.empty() || face.vertexIndices[0] >= hull.vertices.size())
                    continue;

                const Eigen::Vector3f faceNormalWorld = TransformNormalToWorld(hullTransform, face.normal);
                if (faceNormalWorld.squaredNorm() <= 1e-12f)
                    continue;

                const Eigen::Vector3f planePoint = hullTransform.TransformPoint(hull.vertices[face.vertexIndices[0]].position);
                const float separation = faceNormalWorld.dot(sphereCenter - planePoint);

                if (separation > outSeparation)
                {
                    outSeparation = separation;
                    outFaceIndex = faceIndex;
                    outNormal = faceNormalWorld;
                }
            }

            return outNormal.squaredNorm() > 1e-12f;
        }
    }

    //================================//
    // Details:
    // We first run GJK algorithm on the origin of the sphere
    // if shallow penetration, we get the closest point on the hull and construct the contact
    // if deep penetration, we find the face with the least separation (SAT)
    CollisionResult CollisionSphereHull(const Mesh* meshA, const Mesh* meshB)
    {
        CollisionResult result;
        result.numContacts = 0;

        const bool sphereIsA = (meshA->modelType == ModelType_Sphere);
        const Mesh* sphereMesh = sphereIsA ? meshA : meshB;
        const Mesh* hullMesh = sphereIsA ? meshB : meshA;

        const Transform& sphereTransform = sphereMesh->transform;
        const Transform& hullTransform = hullMesh->transform;
        const ConvexHull& hull = hullMesh->solver->GetModelConvexHull(hullMesh->modelType);

        if (hull.vertices.empty() || hull.faces.empty())
            return result;

        const Eigen::Vector3f sphereCenter = sphereTransform.GetPosition();
        const float sphereRadius = 0.5f * sphereTransform.GetScale().x();

        const PointHullGJKResult gjkResult = ComputePointHullDistance(hullTransform, hull, sphereCenter);

        Eigen::Vector3f pointOnHullWorld = Eigen::Vector3f::Zero();
        Eigen::Vector3f pointOnSphereWorld = Eigen::Vector3f::Zero();
        Eigen::Vector3f normalHullToSphere = Eigen::Vector3f::Zero();
        float penetration = 0.0f;
        uint32_t contactId = 0xFFF20000u;

        if (!gjkResult.containsOrigin && gjkResult.distanceSquared > 1e-12f)
        {
            const float distance = std::sqrt(gjkResult.distanceSquared);
            if (distance > sphereRadius)
                return result;

            pointOnHullWorld = gjkResult.closestHullPoint;
            normalHullToSphere = (sphereCenter - pointOnHullWorld) / distance;
            pointOnSphereWorld = sphereCenter - normalHullToSphere * sphereRadius;
            penetration = sphereRadius - distance;
        }
        else
        {
            uint16_t faceIndex = 0;
            float faceSeparation = 0.0f;
            Eigen::Vector3f faceNormalWorld = Eigen::Vector3f::Zero();
            if (!QueryDeepPenetrationFace(hullTransform, hull, sphereCenter, faceIndex, faceSeparation, faceNormalWorld))
                return result;

            pointOnHullWorld = sphereCenter - faceSeparation * faceNormalWorld;
            normalHullToSphere = faceNormalWorld;
            pointOnSphereWorld = sphereCenter + normalHullToSphere * sphereRadius;
            penetration = sphereRadius - faceSeparation;
            contactId = 0xFFF30000u | static_cast<uint32_t>(faceIndex);
        }

        const Eigen::Vector3f normalAToB = sphereIsA ? -normalHullToSphere : normalHullToSphere;
        const Eigen::Vector3f pointOnA = sphereIsA ? pointOnSphereWorld : pointOnHullWorld;
        const Eigen::Vector3f pointOnB = sphereIsA ? pointOnHullWorld : pointOnSphereWorld;

        ContactPoint& contact = result.contactPoints[0];
        contact.position = 0.5f * (pointOnA + pointOnB);
        contact.normal = normalAToB;
        contact.penetration = penetration;

        if (meshA->modelType == ModelType_Sphere)
            contact.rA = pointOnA - meshA->transform.GetPosition();
        else
            contact.rA = ToLocalOffset(meshA->transform, pointOnA);

        if (meshB->modelType == ModelType_Sphere)
            contact.rB = pointOnB - meshB->transform.GetPosition();
        else
            contact.rB = ToLocalOffset(meshB->transform, pointOnB);

        contact.id = contactId;
        result.numContacts = 1;
        return result;
    }
}