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
    static Quaternionf RotationAroundZ(float radians)
    {
        return Quaternionf(Eigen::AngleAxisf(radians, Eigen::Vector3f::UnitZ()));
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
