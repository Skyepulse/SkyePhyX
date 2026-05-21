#include <gtest/gtest.h>

#include "physics/collision.hpp"
#include "physics/solver.hpp"

#include <array>
#include <cmath>
#include <string>
#include <vector>

//================================//
namespace
{
    static constexpr float POINT_TOLERANCE = 0.085f;
    static constexpr float NORMAL_TOLERANCE = 0.06f;
    static constexpr float PENETRATION_TOLERANCE = 0.045f;

    //================================//
    struct CollisionCase
    {
        std::string name;
        ModelType typeA = ModelType_Cube;
        ModelType typeB = ModelType_Cube;
        Eigen::Vector3f positionA = Eigen::Vector3f::Zero();
        Eigen::Vector3f positionB = Eigen::Vector3f::Zero();
        Eigen::Vector3f scaleA = Eigen::Vector3f::Ones();
        Eigen::Vector3f scaleB = Eigen::Vector3f::Ones();
        Quaternionf rotationA = Quaternionf::Identity();
        Quaternionf rotationB = Quaternionf::Identity();
        Eigen::Vector3f expectedNormal = Eigen::Vector3f::UnitY();
        Eigen::Vector3f expectedPointA = Eigen::Vector3f::Zero();
        Eigen::Vector3f expectedPointB = Eigen::Vector3f::Zero();
        float expectedPenetration = 0.0f;
    };

    //================================//
    struct InterpenetrationCase
    {
        std::string name;
        ModelType typeA = ModelType_TestConvexMesh;
        ModelType typeB = ModelType_TestConvexMesh;
        Eigen::Vector3f positionA = Eigen::Vector3f::Zero();
        Eigen::Vector3f positionB = Eigen::Vector3f::Zero();
        Eigen::Vector3f scaleA = Eigen::Vector3f::Ones();
        Eigen::Vector3f scaleB = Eigen::Vector3f::Ones();
        Quaternionf rotationA = Quaternionf::Identity();
        Quaternionf rotationB = Quaternionf::Identity();
    };

    //================================//
    static Quaternionf RotationAroundZ(float radians)
    {
        return Quaternionf(Eigen::AngleAxisf(radians, Eigen::Vector3f::UnitZ()));
    }

    //================================//
    static Quaternionf RotationXYZ(float radiansX, float radiansY, float radiansZ)
    {
        return Quaternionf(Eigen::AngleAxisf(radiansZ, Eigen::Vector3f::UnitZ()) *
                           Eigen::AngleAxisf(radiansY, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(radiansX, Eigen::Vector3f::UnitX()));
    }

    //================================//
    static Eigen::Vector3f Normalized(const Eigen::Vector3f& vector)
    {
        return vector / vector.norm();
    }

    //================================//
    static Eigen::Vector3f SignedZ(bool positive)
    {
        return positive ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    }

    //================================//
    static Eigen::Vector3f WorldAnchor(const Mesh* mesh, const Eigen::Vector3f& localOffset)
    {
        if (mesh->modelType == ModelType_Sphere)
            return mesh->transform.GetPosition() + localOffset;

        return mesh->transform.GetPosition() + mesh->transform.GetRotation().toRotationMatrix() * localOffset;
    }

    //================================//
    static Eigen::Vector3f ExpectedLocalOffset(const Mesh* mesh, const Eigen::Vector3f& worldPoint)
    {
        if (mesh->modelType == ModelType_Sphere)
            return worldPoint - mesh->transform.GetPosition();

        return mesh->transform.GetRotation().toRotationMatrix().transpose() * (worldPoint - mesh->transform.GetPosition());
    }

    //================================//
    static void ExpectVectorNear(const Eigen::Vector3f& actual, const Eigen::Vector3f& expected, float tolerance)
    {
        EXPECT_NEAR(actual.x(), expected.x(), tolerance);
        EXPECT_NEAR(actual.y(), expected.y(), tolerance);
        EXPECT_NEAR(actual.z(), expected.z(), tolerance);
    }

    //================================//
    static void ExpectFiniteVector(const Eigen::Vector3f& vector)
    {
        EXPECT_TRUE(std::isfinite(vector.x()));
        EXPECT_TRUE(std::isfinite(vector.y()));
        EXPECT_TRUE(std::isfinite(vector.z()));
    }

    //================================//
    static void RunCollisionCase(Solver& solver, const CollisionCase& testCase)
    {
        SCOPED_TRACE(testCase.name);

        Mesh* meshA = solver.AddBody(testCase.typeA, 1.0f, 0.5f,
                                     testCase.positionA, testCase.scaleA,
                                     Eigen::Vector3f::Zero(), testCase.rotationA,
                                     Eigen::Vector3f::Zero(), false);
        Mesh* meshB = solver.AddBody(testCase.typeB, 1.0f, 0.5f,
                                     testCase.positionB, testCase.scaleB,
                                     Eigen::Vector3f::Zero(), testCase.rotationB,
                                     Eigen::Vector3f::Zero(), false);

        const CollisionResult result = CollisionSpace::CollisionMeshMesh(meshA, meshB);
        ASSERT_GT(result.numContacts, 0);

        Eigen::Vector3f averagePointA = Eigen::Vector3f::Zero();
        Eigen::Vector3f averagePointB = Eigen::Vector3f::Zero();
        float averagePenetration = 0.0f;

        for (int contactIndex = 0; contactIndex < result.numContacts; ++contactIndex)
        {
            const ContactPoint& contact = result.contactPoints[contactIndex];
            EXPECT_GT(contact.penetration, 0.0f);
            EXPECT_GT(contact.normal.dot(testCase.expectedNormal), 1.0f - NORMAL_TOLERANCE);

            averagePointA += WorldAnchor(meshA, contact.rA);
            averagePointB += WorldAnchor(meshB, contact.rB);
            averagePenetration += contact.penetration;
        }

        const float contactCount = static_cast<float>(result.numContacts);
        averagePointA /= contactCount;
        averagePointB /= contactCount;
        averagePenetration /= contactCount;

        ExpectVectorNear(averagePointA, testCase.expectedPointA, POINT_TOLERANCE);
        ExpectVectorNear(averagePointB, testCase.expectedPointB, POINT_TOLERANCE);

        const Eigen::Vector3f expectedLocalA = ExpectedLocalOffset(meshA, testCase.expectedPointA);
        const Eigen::Vector3f expectedLocalB = ExpectedLocalOffset(meshB, testCase.expectedPointB);
        Eigen::Vector3f averageLocalA = Eigen::Vector3f::Zero();
        Eigen::Vector3f averageLocalB = Eigen::Vector3f::Zero();

        for (int contactIndex = 0; contactIndex < result.numContacts; ++contactIndex)
        {
            averageLocalA += result.contactPoints[contactIndex].rA;
            averageLocalB += result.contactPoints[contactIndex].rB;
        }

        averageLocalA /= contactCount;
        averageLocalB /= contactCount;

        ExpectVectorNear(averageLocalA, expectedLocalA, POINT_TOLERANCE);
        ExpectVectorNear(averageLocalB, expectedLocalB, POINT_TOLERANCE);
        EXPECT_NEAR(averagePenetration, testCase.expectedPenetration, PENETRATION_TOLERANCE);
    }

    //================================//
    static void RunCollisionCases(const std::vector<CollisionCase>& cases)
    {
        ASSERT_EQ(cases.size(), 10u);

        Solver solver;
        for (int i = 0; i < static_cast<int>(cases.size()); ++i)
            RunCollisionCase(solver, cases[i]);
    }

    //================================//
    static void RunInterpenetrationCase(Solver& solver, const InterpenetrationCase& testCase)
    {
        SCOPED_TRACE(testCase.name);

        Mesh* meshA = solver.AddBody(testCase.typeA, 1.0f, 0.5f,
                                     testCase.positionA, testCase.scaleA,
                                     Eigen::Vector3f::Zero(), testCase.rotationA,
                                     Eigen::Vector3f::Zero(), false);
        Mesh* meshB = solver.AddBody(testCase.typeB, 1.0f, 0.5f,
                                     testCase.positionB, testCase.scaleB,
                                     Eigen::Vector3f::Zero(), testCase.rotationB,
                                     Eigen::Vector3f::Zero(), false);

        const CollisionResult result = CollisionSpace::CollisionMeshMesh(meshA, meshB);
        ASSERT_GT(result.numContacts, 0);

        for (int contactIndex = 0; contactIndex < result.numContacts; ++contactIndex)
        {
            const ContactPoint& contact = result.contactPoints[contactIndex];
            const Eigen::Vector3f worldA = WorldAnchor(meshA, contact.rA);
            const Eigen::Vector3f worldB = WorldAnchor(meshB, contact.rB);
            const Eigen::Vector3f expectedMidPoint = 0.5f * (worldA + worldB);

            ExpectFiniteVector(contact.normal);
            ExpectFiniteVector(worldA);
            ExpectFiniteVector(worldB);
            EXPECT_NEAR(contact.normal.norm(), 1.0f, NORMAL_TOLERANCE);
            EXPECT_GT(contact.penetration, COLLISION_MARGIN);
            EXPECT_LE(contact.normal.dot(worldB - worldA), POINT_TOLERANCE);
            ExpectVectorNear(contact.position, expectedMidPoint, POINT_TOLERANCE);
        }

        Manifold manifold(&solver, meshA, meshB);
        ASSERT_TRUE(manifold.Initialize());
        EXPECT_GT(manifold.numContactPoints, 0);
        EXPECT_LE(manifold.numContactPoints, 4);

        for (int contactIndex = 0; contactIndex < manifold.numContactPoints; ++contactIndex)
        {
            EXPECT_LT(manifold.contactInfos[contactIndex].C0[0], 0.0f);
            EXPECT_GT(manifold.contactPoints[contactIndex].penetration, COLLISION_MARGIN);
            EXPECT_GT(manifold.constraintPoints[contactIndex * 3].penalty, 1.0f);
        }
    }

    //================================//
    static void RunInterpenetrationCases(const std::vector<InterpenetrationCase>& cases)
    {
        ASSERT_EQ(cases.size(), 10u);

        Solver solver;
        for (int i = 0; i < static_cast<int>(cases.size()); ++i)
            RunInterpenetrationCase(solver, cases[i]);
    }

    //================================//
    static void AppendSphereSphereCases(std::vector<CollisionCase>& outCases)
    {
        const std::array<Eigen::Vector3f, 10> directions =
        {{
            Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX(),
            Eigen::Vector3f::UnitY(), -Eigen::Vector3f::UnitY(),
            Eigen::Vector3f::UnitZ(), -Eigen::Vector3f::UnitZ(),
            Normalized(Eigen::Vector3f(1.0f, 1.0f, 0.0f)),
            Normalized(Eigen::Vector3f(-1.0f, 1.0f, 0.0f)),
            Normalized(Eigen::Vector3f(1.0f, 0.0f, 1.0f)),
            Normalized(Eigen::Vector3f(0.0f, -1.0f, 1.0f))
        }};

        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.04f + 0.01f * static_cast<float>(i % 4);
            const Eigen::Vector3f normal = directions[i];
            CollisionCase testCase;
            testCase.name = "sphere-sphere-" + std::to_string(i);
            testCase.typeA = ModelType_Sphere;
            testCase.typeB = ModelType_Sphere;
            testCase.positionB = normal * (1.0f - depth);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = normal * 0.5f;
            testCase.expectedPointB = testCase.positionB - normal * 0.5f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendSphereBoxCases(std::vector<CollisionCase>& outCases)
    {
        const std::array<Eigen::Vector3f, 10> directions =
        {{
            Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX(),
            Eigen::Vector3f::UnitY(), -Eigen::Vector3f::UnitY(),
            Eigen::Vector3f::UnitZ(), -Eigen::Vector3f::UnitZ(),
            Normalized(Eigen::Vector3f(1.0f, 1.0f, 1.0f)),
            Normalized(Eigen::Vector3f(-1.0f, 1.0f, 1.0f)),
            Normalized(Eigen::Vector3f(1.0f, -1.0f, 1.0f)),
            Normalized(Eigen::Vector3f(1.0f, 1.0f, -1.0f))
        }};

        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.035f + 0.008f * static_cast<float>(i % 5);
            const Eigen::Vector3f normal = directions[i];
            const Eigen::Vector3f boxPoint =
                (std::abs(normal.x()) > 0.7f || std::abs(normal.y()) > 0.7f || std::abs(normal.z()) > 0.7f)
                ? normal * 0.5f
                : Eigen::Vector3f(0.5f * (normal.x() > 0.0f ? 1.0f : -1.0f),
                                  0.5f * (normal.y() > 0.0f ? 1.0f : -1.0f),
                                  0.5f * (normal.z() > 0.0f ? 1.0f : -1.0f));

            CollisionCase testCase;
            testCase.name = "box-sphere-" + std::to_string(i);
            testCase.typeA = ModelType_Cube;
            testCase.typeB = ModelType_Sphere;
            testCase.positionB = boxPoint + normal * (0.5f - depth);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = boxPoint;
            testCase.expectedPointB = testCase.positionB - normal * 0.5f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendSphereCapsuleCases(std::vector<CollisionCase>& outCases)
    {
        const std::array<Eigen::Vector3f, 10> normals =
        {{
            Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX(),
            Eigen::Vector3f::UnitZ(), -Eigen::Vector3f::UnitZ(),
            Eigen::Vector3f::UnitY(), -Eigen::Vector3f::UnitY(),
            Normalized(Eigen::Vector3f(1.0f, 0.0f, 1.0f)),
            Normalized(Eigen::Vector3f(-1.0f, 0.0f, 1.0f)),
            Normalized(Eigen::Vector3f(1.0f, 0.0f, -1.0f)),
            Normalized(Eigen::Vector3f(-1.0f, 0.0f, -1.0f))
        }};
        const std::array<float, 10> segmentY = {{ -0.2f, 0.0f, 0.18f, -0.1f, 0.25f, -0.25f, 0.1f, -0.12f, 0.2f, -0.2f }};

        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.03f + 0.007f * static_cast<float>(i % 5);
            const Eigen::Vector3f normal = normals[i];
            const Eigen::Vector3f segmentPoint(0.0f, segmentY[i], 0.0f);

            CollisionCase testCase;
            testCase.name = "capsule-sphere-" + std::to_string(i);
            testCase.typeA = ModelType_Capsule;
            testCase.typeB = ModelType_Sphere;
            testCase.positionB = segmentPoint + normal * (0.75f - depth);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = segmentPoint + normal * 0.25f;
            testCase.expectedPointB = testCase.positionB - normal * 0.5f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendSphereHullCases(std::vector<CollisionCase>& outCases)
    {
        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.03f + 0.006f * static_cast<float>(i % 5);
            const float x = -0.18f + 0.04f * static_cast<float>(i);
            const float y = -0.16f + 0.03f * static_cast<float>(i % 6);
            const Eigen::Vector3f normal = SignedZ(i < 5);
            const float hullZ = (i < 5) ? 0.35f : -0.35f;
            const Eigen::Vector3f hullPoint(x, y, hullZ);

            CollisionCase testCase;
            testCase.name = "hull-sphere-" + std::to_string(i);
            testCase.typeA = ModelType_TestConvexMesh;
            testCase.typeB = ModelType_Sphere;
            testCase.positionB = hullPoint + normal * (0.5f - depth);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = hullPoint;
            testCase.expectedPointB = testCase.positionB - normal * 0.5f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendBoxBoxCases(std::vector<CollisionCase>& outCases)
    {
        const std::array<Eigen::Vector3f, 6> axes =
        {{
            Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX(),
            Eigen::Vector3f::UnitY(), -Eigen::Vector3f::UnitY(),
            Eigen::Vector3f::UnitZ(), -Eigen::Vector3f::UnitZ()
        }};

        for (int i = 0; i < 6; ++i)
        {
            const float depth = 0.035f + 0.01f * static_cast<float>(i % 3);
            const Eigen::Vector3f normal = axes[i];

            CollisionCase testCase;
            testCase.name = "box-box-face-" + std::to_string(i);
            testCase.typeA = ModelType_Cube;
            testCase.typeB = ModelType_Cube;
            testCase.positionB = normal * (1.0f - depth);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = normal * 0.5f;
            testCase.expectedPointB = testCase.positionB - normal * 0.5f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }

        for (int i = 0; i < 4; ++i)
        {
            const float depth = 0.025f + 0.01f * static_cast<float>(i);
            const float offsetX = -0.12f + 0.08f * static_cast<float>(i);
            const float angle = (i % 2 == 0) ? 0.78539816339f : -0.78539816339f;
            const float rotatedBottomExtent = std::sqrt(0.5f);

            CollisionCase testCase;
            testCase.name = "box-box-edge-into-face-" + std::to_string(i);
            testCase.typeA = ModelType_Cube;
            testCase.typeB = ModelType_Cube;
            testCase.rotationB = RotationAroundZ(angle);
            testCase.positionB = Eigen::Vector3f(offsetX, 0.5f + rotatedBottomExtent - depth, 0.0f);
            testCase.expectedNormal = Eigen::Vector3f::UnitY();
            testCase.expectedPointA = Eigen::Vector3f(offsetX, 0.5f, 0.0f);
            testCase.expectedPointB = Eigen::Vector3f(offsetX, 0.5f - depth, 0.0f);
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendBoxCapsuleCases(std::vector<CollisionCase>& outCases)
    {
        const std::array<Eigen::Vector3f, 6> normals =
        {{
            Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX(),
            Eigen::Vector3f::UnitZ(), -Eigen::Vector3f::UnitZ(),
            Eigen::Vector3f::UnitY(), -Eigen::Vector3f::UnitY()
        }};

        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.03f + 0.007f * static_cast<float>(i % 5);
            const Eigen::Vector3f normal = normals[i % 6];
            const float yOnSegment = (std::abs(normal.y()) > 0.5f) ? (normal.y() > 0.0f ? -0.25f : 0.25f) : 0.0f;
            const Eigen::Vector3f boxPoint = normal * 0.5f + Eigen::Vector3f(0.0f, std::abs(normal.y()) > 0.5f ? 0.0f : yOnSegment, 0.0f);
            const Eigen::Vector3f segmentPoint = boxPoint + normal * (0.25f - depth);

            CollisionCase testCase;
            testCase.name = "box-capsule-" + std::to_string(i);
            testCase.typeA = ModelType_Cube;
            testCase.typeB = ModelType_Capsule;
            testCase.positionB = segmentPoint - Eigen::Vector3f(0.0f, yOnSegment, 0.0f);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = boxPoint;
            testCase.expectedPointB = segmentPoint - normal * 0.25f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendBoxHullCases(std::vector<CollisionCase>& outCases)
    {
        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.03f + 0.006f * static_cast<float>(i % 5);
            const Eigen::Vector3f normal = SignedZ(i < 5);
            const Eigen::Vector3f boxPoint(0.0f, 0.0f, normal.z() * 0.5f);

            CollisionCase testCase;
            testCase.name = "box-hull-" + std::to_string(i);
            testCase.typeA = ModelType_Cube;
            testCase.typeB = ModelType_TestConvexMesh;
            testCase.positionB = normal * (0.85f - depth);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = boxPoint;
            testCase.expectedPointB = boxPoint - normal * depth;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendCapsuleCapsuleCases(std::vector<CollisionCase>& outCases)
    {
        const std::array<Eigen::Vector3f, 10> normals =
        {{
            Eigen::Vector3f::UnitX(),
            Normalized(Eigen::Vector3f(1.0f, 0.0f, 1.0f)),
            Eigen::Vector3f::UnitZ(),
            Normalized(Eigen::Vector3f(-1.0f, 0.0f, 1.0f)),
            -Eigen::Vector3f::UnitX(),
            Normalized(Eigen::Vector3f(-1.0f, 0.0f, -1.0f)),
            -Eigen::Vector3f::UnitZ(),
            Normalized(Eigen::Vector3f(1.0f, 0.0f, -1.0f)),
            Normalized(Eigen::Vector3f(1.0f, 0.0f, 1.0f)),
            Normalized(Eigen::Vector3f(-1.0f, 0.0f, -1.0f))
        }};

        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.03f + 0.007f * static_cast<float>(i % 5);
            const Eigen::Vector3f normal = normals[i];
            const Eigen::Vector3f pointAOnSegment = Eigen::Vector3f::Zero();
            const Eigen::Vector3f pointBOnSegment = normal * (0.5f - depth);

            CollisionCase testCase;
            testCase.name = "capsule-capsule-" + std::to_string(i);
            testCase.typeA = ModelType_Capsule;
            testCase.typeB = ModelType_Capsule;
            testCase.positionB = pointBOnSegment;
            testCase.expectedNormal = normal;
            testCase.expectedPointA = pointAOnSegment + normal * 0.25f;
            testCase.expectedPointB = pointBOnSegment - normal * 0.25f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendCapsuleHullCases(std::vector<CollisionCase>& outCases)
    {
        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.03f + 0.006f * static_cast<float>(i % 5);
            const Eigen::Vector3f normal = SignedZ(i < 5);
            const float y = 0.0f;
            const Eigen::Vector3f hullPoint(0.0f, y, normal.z() * 0.35f);
            const Eigen::Vector3f segmentPoint = hullPoint + normal * (0.25f - depth);

            CollisionCase testCase;
            testCase.name = "hull-capsule-" + std::to_string(i);
            testCase.typeA = ModelType_TestConvexMesh;
            testCase.typeB = ModelType_Capsule;
            testCase.positionB = segmentPoint - Eigen::Vector3f(0.0f, y, 0.0f);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = hullPoint;
            testCase.expectedPointB = segmentPoint - normal * 0.25f;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendHullHullCases(std::vector<CollisionCase>& outCases)
    {
        for (int i = 0; i < 10; ++i)
        {
            const float depth = 0.03f + 0.006f * static_cast<float>(i % 5);
            const Eigen::Vector3f normal = SignedZ(i < 5);
            const Eigen::Vector3f pointA(0.0f, 0.0f, normal.z() * 0.35f);

            CollisionCase testCase;
            testCase.name = "hull-hull-" + std::to_string(i);
            testCase.typeA = ModelType_TestConvexMesh;
            testCase.typeB = ModelType_TestConvexMesh;
            testCase.positionB = normal * (0.70f - depth);
            testCase.expectedNormal = normal;
            testCase.expectedPointA = pointA;
            testCase.expectedPointB = pointA - normal * depth;
            testCase.expectedPenetration = depth;
            outCases.push_back(testCase);
        }
    }

    //================================//
    static void AppendDeepHullHullCases(std::vector<InterpenetrationCase>& outCases)
    {
        const std::array<Eigen::Vector3f, 10> positions =
        {{
            Eigen::Vector3f(0.00f, 0.00f, 0.00f),
            Eigen::Vector3f(0.06f, -0.04f, 0.03f),
            Eigen::Vector3f(-0.08f, 0.05f, -0.02f),
            Eigen::Vector3f(0.12f, 0.02f, 0.08f),
            Eigen::Vector3f(-0.11f, -0.06f, 0.07f),
            Eigen::Vector3f(0.04f, 0.13f, -0.05f),
            Eigen::Vector3f(-0.03f, -0.12f, -0.08f),
            Eigen::Vector3f(0.14f, -0.10f, 0.01f),
            Eigen::Vector3f(-0.15f, 0.04f, 0.05f),
            Eigen::Vector3f(0.02f, -0.03f, -0.14f)
        }};
        const std::array<Quaternionf, 10> rotations =
        {{
            RotationXYZ(0.00f, 0.00f, 0.00f),
            RotationXYZ(0.24f, 0.00f, 0.49f),
            RotationXYZ(-0.33f, 0.27f, -0.18f),
            RotationXYZ(0.52f, -0.31f, 0.10f),
            RotationXYZ(-0.44f, 0.38f, 0.63f),
            RotationXYZ(0.12f, 0.57f, -0.46f),
            RotationXYZ(-0.61f, -0.21f, 0.35f),
            RotationXYZ(0.37f, -0.53f, -0.28f),
            RotationXYZ(-0.18f, 0.41f, 0.72f),
            RotationXYZ(0.66f, 0.19f, -0.39f)
        }};

        for (int i = 0; i < 10; ++i)
        {
            InterpenetrationCase testCase;
            testCase.name = "deep-hull-hull-" + std::to_string(i);
            testCase.positionB = positions[i];
            testCase.rotationB = rotations[i];
            outCases.push_back(testCase);
        }
    }
}

//================================//
TEST(CollisionNarrowphase, SphereSphereContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendSphereSphereCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, SphereBoxContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendSphereBoxCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, SphereCapsuleContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendSphereCapsuleCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, SphereHullContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendSphereHullCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, BoxBoxContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendBoxBoxCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, BoxCapsuleContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendBoxCapsuleCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, BoxHullContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendBoxHullCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, CapsuleCapsuleContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendCapsuleCapsuleCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, CapsuleHullContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendCapsuleHullCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, HullHullContactAnchors)
{
    std::vector<CollisionCase> cases;
    AppendHullHullCases(cases);
    RunCollisionCases(cases);
}

//================================//
TEST(CollisionNarrowphase, HullHullDeepInterpenetrations)
{
    std::vector<InterpenetrationCase> cases;
    AppendDeepHullHullCases(cases);
    RunInterpenetrationCases(cases);
}

//================================//
TEST(CollisionManifold, HullHullLocalPatchPersistsThroughSmallMotion)
{
    Solver solver;
    Mesh* meshA = solver.AddBody(ModelType_TestConvexMesh, 1.0f, 0.5f,
                                 Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones(),
                                 Eigen::Vector3f::Zero(), RotationXYZ(0.19f, -0.08f, 0.31f),
                                 Eigen::Vector3f::Zero(), false);
    Mesh* meshB = solver.AddBody(ModelType_TestConvexMesh, 1.0f, 0.5f,
                                 Eigen::Vector3f(0.05f, -0.03f, 0.04f), Eigen::Vector3f::Ones(),
                                 Eigen::Vector3f::Zero(), RotationXYZ(-0.22f, 0.35f, -0.17f),
                                 Eigen::Vector3f::Zero(), false);

    Manifold manifold(&solver, meshA, meshB);
    ASSERT_TRUE(manifold.Initialize());
    ASSERT_GT(manifold.numContactPoints, 0);
    ASSERT_LE(manifold.numContactPoints, 4);

    for (int contactIndex = 0; contactIndex < manifold.numContactPoints; ++contactIndex)
    {
        manifold.constraintPoints[contactIndex * 3 + 0].penalty = 40.0f + static_cast<float>(contactIndex);
        manifold.constraintPoints[contactIndex * 3 + 0].lambda = -2.0f;
    }

    meshB->transform.SetPosition(Eigen::Vector3f(0.051f, -0.029f, 0.039f));
    meshB->transform.SetRotation(RotationXYZ(-0.219f, 0.351f, -0.169f));

    ASSERT_TRUE(manifold.Initialize());
    EXPECT_GT(manifold.numContactPoints, 0);
    EXPECT_LE(manifold.numContactPoints, 4);

    int persistentContactCount = 0;
    for (int contactIndex = 0; contactIndex < manifold.numContactPoints; ++contactIndex)
    {
        if (manifold.constraintPoints[contactIndex * 3 + 0].penalty > 20.0f)
            persistentContactCount++;
    }

    EXPECT_GT(persistentContactCount, 0);
}

//================================//
TEST(CollisionManifold, CapsuleContactsStartWithNormalSupport)
{
    Solver solver;
    Mesh* capsuleA = solver.AddBody(ModelType_Capsule, 1.0f, 0.5f,
                                    Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones(),
                                    Eigen::Vector3f::Zero(), Quaternionf::Identity(),
                                    Eigen::Vector3f::Zero(), false);
    Mesh* capsuleB = solver.AddBody(ModelType_Capsule, 1.0f, 0.5f,
                                    Eigen::Vector3f(0.45f, 0.0f, 0.0f), Eigen::Vector3f::Ones(),
                                    Eigen::Vector3f::Zero(), Quaternionf::Identity(),
                                    Eigen::Vector3f::Zero(), false);
    Mesh* box = solver.AddBody(ModelType_Cube, 1.0f, 0.5f,
                               Eigen::Vector3f(2.0f, 0.0f, 0.0f), Eigen::Vector3f::Ones(),
                               Eigen::Vector3f::Zero(), Quaternionf::Identity(),
                               Eigen::Vector3f::Zero(), false);
    Mesh* capsuleOnBox = solver.AddBody(ModelType_Capsule, 1.0f, 0.5f,
                                        Eigen::Vector3f(2.72f, 0.0f, 0.0f), Eigen::Vector3f::Ones(),
                                        Eigen::Vector3f::Zero(), Quaternionf::Identity(),
                                        Eigen::Vector3f::Zero(), false);

    Manifold capsuleManifold(&solver, capsuleA, capsuleB);
    Manifold boxManifold(&solver, box, capsuleOnBox);

    ASSERT_TRUE(capsuleManifold.Initialize());
    ASSERT_TRUE(boxManifold.Initialize());

    for (Manifold* manifold : { &capsuleManifold, &boxManifold })
    {
        SCOPED_TRACE(manifold == &capsuleManifold ? "capsule-capsule" : "box-capsule");
        ASSERT_GT(manifold->numContactPoints, 0);
        for (int contactIndex = 0; contactIndex < manifold->numContactPoints; ++contactIndex)
        {
            EXPECT_LT(manifold->contactInfos[contactIndex].C0[0], 0.0f);
            EXPECT_GT(manifold->constraintPoints[contactIndex * 3].penalty, 1.0f);
        }
    }
}

//================================//
TEST(CollisionManifold, ShallowPersistentContactsKeepFrictionWarmStart)
{
    Solver solver;
    const float penetration = 0.5f * COLLISION_MARGIN;
    Mesh* sphereA = solver.AddBody(ModelType_Sphere, 1.0f, 1.0f,
                                   Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones(),
                                   Eigen::Vector3f::Zero(), Quaternionf::Identity(),
                                   Eigen::Vector3f::Zero(), false);
    Mesh* sphereB = solver.AddBody(ModelType_Sphere, 1.0f, 1.0f,
                                   Eigen::Vector3f(1.0f - penetration, 0.0f, 0.0f), Eigen::Vector3f::Ones(),
                                   Eigen::Vector3f::Zero(), Quaternionf::Identity(),
                                   Eigen::Vector3f::Zero(), false);

    Manifold manifold(&solver, sphereA, sphereB);
    ASSERT_TRUE(manifold.Initialize());
    ASSERT_EQ(manifold.numContactPoints, 1);
    EXPECT_LT(manifold.contactPoints[0].penetration, COLLISION_MARGIN);

    manifold.constraintPoints[0].penalty = 25.0f;
    manifold.constraintPoints[0].lambda = -3.0f;
    manifold.constraintPoints[1].penalty = 12.0f;
    manifold.constraintPoints[1].lambda = 0.4f;
    manifold.constraintPoints[2].penalty = 14.0f;
    manifold.constraintPoints[2].lambda = -0.3f;
    manifold.contactInfos[0].stick = true;

    ASSERT_TRUE(manifold.Initialize());
    ASSERT_EQ(manifold.numContactPoints, 1);

    EXPECT_FLOAT_EQ(manifold.constraintPoints[0].penalty, 25.0f);
    EXPECT_FLOAT_EQ(manifold.constraintPoints[0].lambda, -3.0f);
    EXPECT_FLOAT_EQ(manifold.constraintPoints[1].penalty, 12.0f);
    EXPECT_FLOAT_EQ(manifold.constraintPoints[1].lambda, 0.4f);
    EXPECT_FLOAT_EQ(manifold.constraintPoints[2].penalty, 14.0f);
    EXPECT_FLOAT_EQ(manifold.constraintPoints[2].lambda, -0.3f);
    EXPECT_TRUE(manifold.contactInfos[0].stick);
}
