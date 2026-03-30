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
        static constexpr float FACE_CONTACT_EDGE_BIAS = 0.005f;
        static constexpr float PARALLEL_FACE_THRESHOLD = 0.95f;

        //================================//
        struct SegmentHullSupportPoint
        {
            Eigen::Vector3f csoPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f hullPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f segmentPoint = Eigen::Vector3f::Zero();
        };

        struct SegmentHullGJKResult
        {
            bool containsOrigin = false;
            Eigen::Vector3f closestCSOPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f closestHullPoint = Eigen::Vector3f::Zero();
            Eigen::Vector3f closestSegmentPoint = Eigen::Vector3f::Zero();
            float distanceSquared = std::numeric_limits<float>::infinity();
        };

        struct CapsuleSegment
        {
            Eigen::Vector3f start = Eigen::Vector3f::Zero();
            Eigen::Vector3f end = Eigen::Vector3f::Zero();
            Eigen::Vector3f axis = Eigen::Vector3f::UnitY();
            Eigen::Vector3f center = Eigen::Vector3f::Zero();
            float radius = 0.0f;
        };

        struct CapsuleFaceQuery
        {
            double separation = -std::numeric_limits<double>::infinity();
            uint16_t faceIndex = 0;
        };

        struct CapsuleEdgeQuery
        {
            double separation = -std::numeric_limits<double>::infinity();
            uint16_t edgeIndex = 0;
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
            if (lengthSquared <= 1e-12f) return Eigen::Vector3f::Zero();
            return worldNormal / std::sqrt(lengthSquared);
        }

        //================================//
        static Eigen::Vector3f AnyPerpendicular(const Eigen::Vector3f& axis)
        {
            Eigen::Vector3f perpendicular = (std::abs(axis.x()) < 0.9f) ? axis.cross(Eigen::Vector3f::UnitX()) : axis.cross(Eigen::Vector3f::UnitZ());
            const float lengthSquared = perpendicular.squaredNorm();
            if (lengthSquared <= 1e-12f) return Eigen::Vector3f::UnitY();
            return perpendicular / std::sqrt(lengthSquared);
        }

        //================================//
        static CapsuleSegment GetCapsuleSegment(const Mesh* capsuleMesh)
        {
            CapsuleSegment capsule;
            const Transform& transform = capsuleMesh->transform;
            const float scale = transform.GetScale().x();
            const float halfSegmentLength = 0.25f * scale;
            capsule.axis = transform.GetRotation().toRotationMatrix().col(1).normalized();
            capsule.center = transform.GetPosition();
            capsule.start = capsule.center - capsule.axis * halfSegmentLength;
            capsule.end = capsule.center + capsule.axis * halfSegmentLength;
            capsule.radius = 0.25f * scale;
            return capsule;
        }

        //================================//
        static Eigen::Vector3f SupportSegment(const CapsuleSegment& capsule, const Eigen::Vector3f& direction)
        {
            return (capsule.start.dot(direction) > capsule.end.dot(direction)) ? capsule.start : capsule.end;
        }

        //================================//
        static SegmentHullSupportPoint SupportHullAgainstSegment(const Transform& hullTransform, const ConvexHull& hull, const CapsuleSegment& capsule, const Eigen::Vector3f& directionWorld)
        {
            const Eigen::Matrix3f linear = hullTransform.GetRotation().toRotationMatrix() * hullTransform.GetScale().asDiagonal();
            const Eigen::Vector3f directionLocal = linear.transpose() * directionWorld;
            const Eigen::Vector3f hullSupportWorld = hullTransform.TransformPoint(hull.GetSupport(directionLocal));
            const Eigen::Vector3f segmentSupportWorld = SupportSegment(capsule, -directionWorld);

            SegmentHullSupportPoint support;
            support.csoPoint = hullSupportWorld - segmentSupportWorld;
            support.hullPoint = hullSupportWorld;
            support.segmentPoint = segmentSupportWorld;
            return support;
        }

        //================================//
        static void ReduceSegmentSimplex(std::vector<SegmentHullSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint, Eigen::Vector3f& closestSegmentPoint)
        {
            const auto a = simplex[0];
            const auto b = simplex[1];
            const Eigen::Vector3f ab = b.csoPoint - a.csoPoint;
            const float abLengthSquared = ab.squaredNorm();
            if (abLengthSquared <= 1e-12f)
            {
                simplex = { a };
                closestCSOPoint = a.csoPoint;
                closestHullPoint = a.hullPoint;
                closestSegmentPoint = a.segmentPoint;
                return;
            }

            const float t = std::clamp(-a.csoPoint.dot(ab) / abLengthSquared, 0.0f, 1.0f);
            const float u = 1.0f - t;
            closestCSOPoint = u * a.csoPoint + t * b.csoPoint;
            closestHullPoint = u * a.hullPoint + t * b.hullPoint;
            closestSegmentPoint = u * a.segmentPoint + t * b.segmentPoint;

            if (t <= 1e-6f) simplex = { a };
            else if (t >= 1.0f - 1e-6f) simplex = { b };
            else simplex = { a, b };
        }

        //================================//
        static void ReduceTriangleSimplex(std::vector<SegmentHullSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint, Eigen::Vector3f& closestSegmentPoint)
        {
            const auto a = simplex[0];
            const auto b = simplex[1];
            const auto c = simplex[2];
            const Eigen::Vector3f ab = b.csoPoint - a.csoPoint;
            const Eigen::Vector3f ac = c.csoPoint - a.csoPoint;
            const Eigen::Vector3f ap = -a.csoPoint;
            const float d1 = ab.dot(ap), d2 = ac.dot(ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                simplex = { a };
                closestCSOPoint = a.csoPoint; closestHullPoint = a.hullPoint; closestSegmentPoint = a.segmentPoint;
                return;
            }

            const Eigen::Vector3f bp = -b.csoPoint;
            const float d3 = ab.dot(bp), d4 = ac.dot(bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                simplex = { b };
                closestCSOPoint = b.csoPoint; closestHullPoint = b.hullPoint; closestSegmentPoint = b.segmentPoint;
                return;
            }

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                const float v = d1 / (d1 - d3), u = 1.0f - v;
                simplex = { a, b };
                closestCSOPoint = u * a.csoPoint + v * b.csoPoint;
                closestHullPoint = u * a.hullPoint + v * b.hullPoint;
                closestSegmentPoint = u * a.segmentPoint + v * b.segmentPoint;
                return;
            }

            const Eigen::Vector3f cp = -c.csoPoint;
            const float d5 = ab.dot(cp), d6 = ac.dot(cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                simplex = { c };
                closestCSOPoint = c.csoPoint; closestHullPoint = c.hullPoint; closestSegmentPoint = c.segmentPoint;
                return;
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                const float w = d2 / (d2 - d6), u = 1.0f - w;
                simplex = { a, c };
                closestCSOPoint = u * a.csoPoint + w * c.csoPoint;
                closestHullPoint = u * a.hullPoint + w * c.hullPoint;
                closestSegmentPoint = u * a.segmentPoint + w * c.segmentPoint;
                return;
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6)), v = 1.0f - w;
                simplex = { b, c };
                closestCSOPoint = v * b.csoPoint + w * c.csoPoint;
                closestHullPoint = v * b.hullPoint + w * c.hullPoint;
                closestSegmentPoint = v * b.segmentPoint + w * c.segmentPoint;
                return;
            }

            const float denom = 1.0f / (va + vb + vc);
            const float v = vb * denom, w = vc * denom, u = 1.0f - v - w;
            simplex = { a, b, c };
            closestCSOPoint = u * a.csoPoint + v * b.csoPoint + w * c.csoPoint;
            closestHullPoint = u * a.hullPoint + v * b.hullPoint + w * c.hullPoint;
            closestSegmentPoint = u * a.segmentPoint + v * b.segmentPoint + w * c.segmentPoint;
        }

        //================================//
        static bool OriginOutsidePlane(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& d)
        {
            const Eigen::Vector3f normal = (b - a).cross(c - a);
            const float signOrigin = (-a).dot(normal);
            const float signOpposite = (d - a).dot(normal);
            if (std::abs(signOpposite) <= 1e-6f) return false;
            return signOrigin * signOpposite < -1e-6f;
        }

        //================================//
        static bool ReduceTetrahedronSimplex(std::vector<SegmentHullSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint, Eigen::Vector3f& closestSegmentPoint)
        {
            const std::vector<SegmentHullSupportPoint> tetrahedron = simplex;
            float bestDistanceSquared = std::numeric_limits<float>::infinity();
            std::vector<SegmentHullSupportPoint> bestFaceSimplex;
            Eigen::Vector3f bestClosestCSO = Eigen::Vector3f::Zero();
            Eigen::Vector3f bestClosestHull = Eigen::Vector3f::Zero();
            Eigen::Vector3f bestClosestSegment = Eigen::Vector3f::Zero();

            auto testFace = [&](int ia, int ib, int ic, int id)
            {
                const Eigen::Vector3f& a = tetrahedron[ia].csoPoint;
                const Eigen::Vector3f& b = tetrahedron[ib].csoPoint;
                const Eigen::Vector3f& c = tetrahedron[ic].csoPoint;
                const Eigen::Vector3f& d = tetrahedron[id].csoPoint;
                if (!OriginOutsidePlane(a, b, c, d)) return;

                std::vector<SegmentHullSupportPoint> faceSimplex = { tetrahedron[ia], tetrahedron[ib], tetrahedron[ic] };
                Eigen::Vector3f faceClosestCSO = Eigen::Vector3f::Zero();
                Eigen::Vector3f faceClosestHull = Eigen::Vector3f::Zero();
                Eigen::Vector3f faceClosestSegment = Eigen::Vector3f::Zero();
                ReduceTriangleSimplex(faceSimplex, faceClosestCSO, faceClosestHull, faceClosestSegment);

                const float distanceSquared = faceClosestCSO.squaredNorm();
                if (distanceSquared < bestDistanceSquared)
                {
                    bestDistanceSquared = distanceSquared;
                    bestFaceSimplex = faceSimplex;
                    bestClosestCSO = faceClosestCSO;
                    bestClosestHull = faceClosestHull;
                    bestClosestSegment = faceClosestSegment;
                }
            };

            testFace(0, 1, 2, 3);
            testFace(0, 2, 3, 1);
            testFace(0, 3, 1, 2);
            testFace(1, 3, 2, 0);

            if (bestFaceSimplex.empty()) return true;

            simplex = std::move(bestFaceSimplex);
            closestCSOPoint = bestClosestCSO;
            closestHullPoint = bestClosestHull;
            closestSegmentPoint = bestClosestSegment;
            return false;
        }

        //================================//
        static bool ReduceSimplex(std::vector<SegmentHullSupportPoint>& simplex, Eigen::Vector3f& closestCSOPoint, Eigen::Vector3f& closestHullPoint, Eigen::Vector3f& closestSegmentPoint)
        {
            switch (simplex.size())
            {
                case 1:
                    closestCSOPoint = simplex[0].csoPoint;
                    closestHullPoint = simplex[0].hullPoint;
                    closestSegmentPoint = simplex[0].segmentPoint;
                    return closestCSOPoint.squaredNorm() <= 1e-12f;
                case 2:
                    ReduceSegmentSimplex(simplex, closestCSOPoint, closestHullPoint, closestSegmentPoint);
                    return closestCSOPoint.squaredNorm() <= 1e-12f;
                case 3:
                    ReduceTriangleSimplex(simplex, closestCSOPoint, closestHullPoint, closestSegmentPoint);
                    return closestCSOPoint.squaredNorm() <= 1e-12f;
                case 4:
                    return ReduceTetrahedronSimplex(simplex, closestCSOPoint, closestHullPoint, closestSegmentPoint);
                default:
                    closestCSOPoint = Eigen::Vector3f::Zero();
                    closestHullPoint = Eigen::Vector3f::Zero();
                    closestSegmentPoint = Eigen::Vector3f::Zero();
                    return false;
            }
        }

        //================================//
        static SegmentHullGJKResult ComputeSegmentHullDistance(const Transform& hullTransform, const ConvexHull& hull, const CapsuleSegment& capsule)
        {
            SegmentHullGJKResult result;
            if (hull.vertexCount() == 0) return result;

            std::vector<SegmentHullSupportPoint> simplex;
            simplex.reserve(4);
            Eigen::Vector3f direction = hullTransform.TransformPoint(hull.centroid) - capsule.center;
            if (direction.squaredNorm() <= 1e-12f) direction = hullTransform.TransformPoint(hull.vertices[0].position) - capsule.center;
            if (direction.squaredNorm() <= 1e-12f) direction = Eigen::Vector3f::UnitX();

            simplex.push_back(SupportHullAgainstSegment(hullTransform, hull, capsule, direction));
            Eigen::Vector3f closestCSOPoint = simplex[0].csoPoint;
            Eigen::Vector3f closestHullPoint = simplex[0].hullPoint;
            Eigen::Vector3f closestSegmentPoint = simplex[0].segmentPoint;

            for (int iteration = 0; iteration < 32; ++iteration)
            {
                if (ReduceSimplex(simplex, closestCSOPoint, closestHullPoint, closestSegmentPoint))
                {
                    result.containsOrigin = true;
                    result.closestCSOPoint = closestCSOPoint;
                    result.closestHullPoint = closestHullPoint;
                    result.closestSegmentPoint = closestSegmentPoint;
                    result.distanceSquared = closestCSOPoint.squaredNorm();
                    return result;
                }

                const Eigen::Vector3f searchDirection = -closestCSOPoint;
                if (searchDirection.squaredNorm() <= 1e-12f)
                {
                    result.containsOrigin = true;
                    result.closestCSOPoint = closestCSOPoint;
                    result.closestHullPoint = closestHullPoint;
                    result.closestSegmentPoint = closestSegmentPoint;
                    result.distanceSquared = closestCSOPoint.squaredNorm();
                    return result;
                }

                const SegmentHullSupportPoint support = SupportHullAgainstSegment(hullTransform, hull, capsule, searchDirection);
                bool duplicateSupport = false;
                for (const SegmentHullSupportPoint& simplexPoint : simplex)
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
                    result.closestSegmentPoint = closestSegmentPoint;
                    result.distanceSquared = closestCSOPoint.squaredNorm();
                    return result;
                }

                simplex.push_back(support);
            }

            result.containsOrigin = false;
            result.closestCSOPoint = closestCSOPoint;
            result.closestHullPoint = closestHullPoint;
            result.closestSegmentPoint = closestSegmentPoint;
            result.distanceSquared = closestCSOPoint.squaredNorm();
            return result;
        }

        //================================//
        static std::vector<Eigen::Vector3f> ClipPolygonAgainstPlane(const std::vector<Eigen::Vector3f>& polygon, const Eigen::Vector3f& planeNormal, float planeOffset)
        {
            std::vector<Eigen::Vector3f> clipped;
            if (polygon.empty()) return clipped;
            clipped.reserve(polygon.size() + 1);
            const int count = static_cast<int>(polygon.size());
            for (int i = 0; i < count; ++i)
            {
                const Eigen::Vector3f& a = polygon[i];
                const Eigen::Vector3f& b = polygon[(i + 1) % count];
                const float da = planeNormal.dot(a) - planeOffset;
                const float db = planeNormal.dot(b) - planeOffset;
                const bool insideA = da <= 0.0f;
                const bool insideB = db <= 0.0f;
                if (insideA) clipped.push_back(a);
                if (insideA != insideB)
                {
                    const float t = da / (da - db);
                    clipped.push_back(a + t * (b - a));
                }
            }
            return clipped;
        }

        //================================//
        static std::vector<Eigen::Vector3f> ClipSegmentAgainstFace(const std::vector<Eigen::Vector3f>& referenceVertices, const Eigen::Vector3f& referenceNormal, const CapsuleSegment& capsule)
        {
            std::vector<Eigen::Vector3f> clipped = { capsule.start, capsule.end };
            if (referenceVertices.size() < 3) return {};
            for (size_t i = 0; i < referenceVertices.size(); ++i)
            {
                const Eigen::Vector3f edge = referenceVertices[(i + 1) % referenceVertices.size()] - referenceVertices[i];
                const Eigen::Vector3f sidePlaneNormal = edge.cross(referenceNormal);
                const float sidePlaneOffset = sidePlaneNormal.dot(referenceVertices[i]);
                clipped = ClipPolygonAgainstPlane(clipped, sidePlaneNormal, sidePlaneOffset);
                if (clipped.empty()) return {};
            }
            return clipped;
        }

        //================================//
        static int FindReferenceFaceForNormal(const Transform& hullTransform, const ConvexHull& hull, const Eigen::Vector3f& normalHullToCapsule)
        {
            int faceIndex = -1;
            float bestDot = PARALLEL_FACE_THRESHOLD;
            for (int i = 0; i < static_cast<int>(hull.faceCount()); ++i)
            {
                const Eigen::Vector3f faceNormalWorld = TransformNormalToWorld(hullTransform, hull.faces[i].normal);
                const float alignment = faceNormalWorld.dot(normalHullToCapsule);
                if (alignment > bestDot)
                {
                    bestDot = alignment;
                    faceIndex = i;
                }
            }
            return faceIndex;
        }

        //================================//
        static CapsuleFaceQuery QueryCapsuleFaceDirections(const Transform& hullTransform, const ConvexHull& hull, const CapsuleSegment& capsule)
        {
            CapsuleFaceQuery query;
            for (int i = 0; i < static_cast<int>(hull.faceCount()); ++i)
            {
                const HullFace& face = hull.faces[i];
                if (face.vertexIndices.empty() || face.vertexIndices[0] >= hull.vertices.size()) continue;
                const Eigen::Vector3f faceNormalWorld = TransformNormalToWorld(hullTransform, face.normal);
                if (faceNormalWorld.squaredNorm() <= 1e-12f) continue;
                const Eigen::Vector3f planePoint = hullTransform.TransformPoint(hull.vertices[face.vertexIndices[0]].position);
                const Eigen::Vector3f capsuleSupport = SupportSegment(capsule, -faceNormalWorld);
                const double separation = static_cast<double>(faceNormalWorld.dot(capsuleSupport - planePoint));
                if (separation > query.separation)
                {
                    query.separation = separation;
                    query.faceIndex = static_cast<uint16_t>(i);
                }
            }
            return query;
        }

        //================================//
        static CapsuleEdgeQuery QueryCapsuleEdgeDirections(const Transform& hullTransform, const ConvexHull& hull, const CapsuleSegment& capsule)
        {
            CapsuleEdgeQuery query;
            const Eigen::Vector3f hullCenterWorld = hullTransform.TransformPoint(hull.centroid);
            for (int edgeIndex = 0; edgeIndex < static_cast<int>(hull.edgeCount()); ++edgeIndex)
            {
                const HullEdge& edge = hull.edges[edgeIndex];
                if (edge.vertexIndices[0] >= hull.vertices.size() || edge.vertexIndices[1] >= hull.vertices.size()) continue;
                const Eigen::Vector3f edgeStart = hullTransform.TransformPoint(hull.vertices[edge.vertexIndices[0]].position);
                const Eigen::Vector3f edgeEnd = hullTransform.TransformPoint(hull.vertices[edge.vertexIndices[1]].position);
                const Eigen::Vector3f edgeDirection = edgeEnd - edgeStart;
                if (edgeDirection.squaredNorm() <= 1e-12f) continue;

                Eigen::Vector3f axis = edgeDirection.cross(capsule.end - capsule.start);
                const float axisLengthSquared = axis.squaredNorm();
                if (axisLengthSquared <= 1e-12f) continue;
                axis /= std::sqrt(axisLengthSquared);
                if (axis.dot(capsule.center - hullCenterWorld) < 0.0f) axis = -axis;

                const Eigen::Vector3f capsuleSupport = SupportSegment(capsule, -axis);
                const double separation = static_cast<double>(axis.dot(capsuleSupport - edgeStart));
                if (separation > query.separation)
                {
                    query.separation = separation;
                    query.edgeIndex = static_cast<uint16_t>(edgeIndex);
                }
            }
            return query;
        }

        //================================//
        static void BuildCapsuleFaceContacts(const Mesh* hullMesh, const Mesh* capsuleMesh, uint16_t faceIndex, const Eigen::Vector3f& referenceNormal, const CapsuleSegment& capsule, bool deepContact, CollisionResult& result)
        {
            result.numContacts = 0;
            const Transform& hullTransform = hullMesh->transform;
            const ConvexHull& hull = hullMesh->solver->GetModelConvexHull(hullMesh->modelType);
            if (faceIndex >= hull.faceCount()) return;

            const HullFace& referenceFace = hull.faces[faceIndex];
            std::vector<Eigen::Vector3f> referenceVertices(referenceFace.vertexIndices.size());
            if (referenceVertices.size() < 3) return;
            for (size_t i = 0; i < referenceFace.vertexIndices.size(); ++i)
            {
                if (referenceFace.vertexIndices[i] >= hull.vertices.size()) return;
                referenceVertices[i] = hullTransform.TransformPoint(hull.vertices[referenceFace.vertexIndices[i]].position);
            }

            const std::vector<Eigen::Vector3f> clippedPoints = ClipSegmentAgainstFace(referenceVertices, referenceNormal, capsule);
            if (clippedPoints.empty()) return;

            const float planeOffset = referenceNormal.dot(referenceVertices[0]);
            for (const Eigen::Vector3f& point : clippedPoints)
            {
                const float separation = referenceNormal.dot(point) - planeOffset;
                if (deepContact)
                {
                    if (separation > 1e-5f) continue;
                }
                else
                {
                    if (separation < -1e-5f || separation > capsule.radius) continue;
                }

                const Eigen::Vector3f projectedPoint = point - separation * referenceNormal;
                Eigen::Vector3f pointOnCapsule = Eigen::Vector3f::Zero();
                if (deepContact)
                    pointOnCapsule = point + referenceNormal * capsule.radius;
                else
                    pointOnCapsule = point - referenceNormal * capsule.radius;
                const Eigen::Vector3f manifoldPoint = 0.5f * (projectedPoint + pointOnCapsule);

                bool duplicate = false;
                for (int i = 0; i < result.numContacts; ++i)
                {
                    if ((result.contactPoints[i].position - manifoldPoint).squaredNorm() <= 1e-10f)
                    {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate || result.numContacts >= 8) continue;

                ContactPoint& contact = result.contactPoints[result.numContacts];
                contact.position = manifoldPoint;
                contact.normal = referenceNormal;
                contact.penetration = capsule.radius - separation;
                contact.rA = ToLocalOffset(hullTransform, projectedPoint);
                contact.rB = ToLocalOffset(capsuleMesh->transform, pointOnCapsule);
                contact.id = (static_cast<uint32_t>(faceIndex) << 8) | static_cast<uint32_t>(result.numContacts);
                ++result.numContacts;
            }
        }

        //================================//
        static void BuildCapsuleEdgeContact(const Mesh* hullMesh, const Mesh* capsuleMesh, uint16_t edgeIndex, const Eigen::Vector3f& normalHullToCapsule, const CapsuleSegment& capsule, CollisionResult& result)
        {
            result.numContacts = 0;
            const Transform& hullTransform = hullMesh->transform;
            const ConvexHull& hull = hullMesh->solver->GetModelConvexHull(hullMesh->modelType);
            if (edgeIndex >= hull.edgeCount()) return;

            const HullEdge& edge = hull.edges[edgeIndex];
            if (edge.vertexIndices[0] >= hull.vertices.size() || edge.vertexIndices[1] >= hull.vertices.size()) return;

            const Eigen::Vector3f edgeStart = hullTransform.TransformPoint(hull.vertices[edge.vertexIndices[0]].position);
            const Eigen::Vector3f edgeEnd = hullTransform.TransformPoint(hull.vertices[edge.vertexIndices[1]].position);
            Eigen::Vector3f pointOnHullEdge = Eigen::Vector3f::Zero();
            Eigen::Vector3f pointOnSegment = Eigen::Vector3f::Zero();
            ClosestPointsOnSegments(edgeStart, edgeEnd, capsule.start, capsule.end, pointOnHullEdge, pointOnSegment);

            const float signedDistance = normalHullToCapsule.dot(pointOnSegment - pointOnHullEdge);
            const float penetration = capsule.radius - signedDistance;
            if (penetration <= 0.0f) return;

            const Eigen::Vector3f pointOnCapsule = pointOnSegment - normalHullToCapsule * capsule.radius;
            ContactPoint& contact = result.contactPoints[0];
            contact.position = 0.5f * (pointOnHullEdge + pointOnCapsule);
            contact.normal = normalHullToCapsule;
            contact.penetration = penetration;
            contact.rA = ToLocalOffset(hullTransform, pointOnHullEdge);
            contact.rB = ToLocalOffset(capsuleMesh->transform, pointOnCapsule);
            contact.id = (1u << 31) | static_cast<uint32_t>(edgeIndex);
            result.numContacts = 1;
        }

        //================================//
        static CollisionResult CollideHullCapsuleOrdered(const Mesh* hullMesh, const Mesh* capsuleMesh)
        {
            CollisionResult result;
            const ConvexHull& hull = hullMesh->solver->GetModelConvexHull(hullMesh->modelType);
            if (hull.vertexCount() == 0 || hull.faceCount() == 0) return result;

            const Transform& hullTransform = hullMesh->transform;
            const CapsuleSegment capsule = GetCapsuleSegment(capsuleMesh);
            const SegmentHullGJKResult gjkResult = ComputeSegmentHullDistance(hullTransform, hull, capsule);

            if (!gjkResult.containsOrigin && gjkResult.distanceSquared > 1e-12f)
            {
                const float distance = std::sqrt(gjkResult.distanceSquared);
                if (distance > capsule.radius) return result;

                const Eigen::Vector3f normalHullToCapsule = (gjkResult.closestSegmentPoint - gjkResult.closestHullPoint) / distance;
                const int shallowReferenceFace = FindReferenceFaceForNormal(hullTransform, hull, normalHullToCapsule);
                if (shallowReferenceFace >= 0)
                {
                    BuildCapsuleFaceContacts(hullMesh, capsuleMesh, static_cast<uint16_t>(shallowReferenceFace), normalHullToCapsule, capsule, false, result);
                    if (result.numContacts > 0) return result;
                }

                const Eigen::Vector3f pointOnCapsule = gjkResult.closestSegmentPoint - normalHullToCapsule * capsule.radius;
                ContactPoint& contact = result.contactPoints[0];
                contact.position = 0.5f * (gjkResult.closestHullPoint + pointOnCapsule);
                contact.normal = normalHullToCapsule;
                contact.penetration = capsule.radius - distance;
                contact.rA = ToLocalOffset(hullTransform, gjkResult.closestHullPoint);
                contact.rB = ToLocalOffset(capsuleMesh->transform, pointOnCapsule);
                contact.id = 0xFFF40000u;
                result.numContacts = 1;
                return result;
            }

            const CapsuleFaceQuery faceQuery = QueryCapsuleFaceDirections(hullTransform, hull, capsule);
            if (faceQuery.separation > 0.0) return result;

            const CapsuleEdgeQuery edgeQuery = QueryCapsuleEdgeDirections(hullTransform, hull, capsule);
            if (edgeQuery.separation > 0.0) return result;

            if (faceQuery.separation >= edgeQuery.separation - FACE_CONTACT_EDGE_BIAS)
            {
                const Eigen::Vector3f referenceNormal = TransformNormalToWorld(hullTransform, hull.faces[faceQuery.faceIndex].normal);
                BuildCapsuleFaceContacts(hullMesh, capsuleMesh, faceQuery.faceIndex, referenceNormal, capsule, true, result);
            }
            else
            {
                const HullEdge& edge = hull.edges[edgeQuery.edgeIndex];
                const Eigen::Vector3f edgeStart = hullTransform.TransformPoint(hull.vertices[edge.vertexIndices[0]].position);
                const Eigen::Vector3f edgeEnd = hullTransform.TransformPoint(hull.vertices[edge.vertexIndices[1]].position);
                Eigen::Vector3f normalHullToCapsule = (edgeEnd - edgeStart).cross(capsule.end - capsule.start);
                const float normalLengthSquared = normalHullToCapsule.squaredNorm();
                if (normalLengthSquared <= 1e-12f) normalHullToCapsule = AnyPerpendicular(capsule.axis);
                else normalHullToCapsule /= std::sqrt(normalLengthSquared);
                if (normalHullToCapsule.dot(capsule.center - hullTransform.TransformPoint(hull.centroid)) < 0.0f) normalHullToCapsule = -normalHullToCapsule;

                BuildCapsuleEdgeContact(hullMesh, capsuleMesh, edgeQuery.edgeIndex, normalHullToCapsule, capsule, result);
            }

            return result;
        }
    }

    //================================//
    CollisionResult CollisionHullCapsule(const Mesh* meshA, const Mesh* meshB)
    {
        const bool hullIsA = (meshA->modelType != ModelType_Capsule);
        const Mesh* hullMesh = hullIsA ? meshA : meshB;
        const Mesh* capsuleMesh = hullIsA ? meshB : meshA;

        CollisionResult result = CollideHullCapsuleOrdered(hullMesh, capsuleMesh);
        if (!hullIsA)
        {
            for (int i = 0; i < result.numContacts; ++i)
            {
                std::swap(result.contactPoints[i].rA, result.contactPoints[i].rB);
                result.contactPoints[i].normal = -result.contactPoints[i].normal;
            }
        }

        return result;
    }
}
