#include "force.hpp"
#include "solver.hpp"
#include "collisionAlgorithms/sat.hpp"

#include <algorithm>
#include <limits>

constexpr float PENALTY_MIN = 1.0f;
constexpr float CONTACT_MATCH_DISTANCE = 0.02f;
constexpr float CONTACT_MATCH_DISTANCE_SQUARED = CONTACT_MATCH_DISTANCE * CONTACT_MATCH_DISTANCE;
constexpr float CONTACT_MATCH_NORMAL_DOT = 0.95f;
constexpr float DUPLICATE_CONTACT_DISTANCE = 0.001f;
constexpr float DUPLICATE_CONTACT_DISTANCE_SQUARED = DUPLICATE_CONTACT_DISTANCE * DUPLICATE_CONTACT_DISTANCE;
constexpr int MAX_SOLVER_MANIFOLD_CONTACTS = 4;

//================================//
static std::vector<Mesh*> FilterNulls(std::initializer_list<Mesh*> bodies)
{
    std::vector<Mesh*> result;
    for (Mesh* m : bodies)
        if (m) result.push_back(m);
    return result;
}

//================================//
static Eigen::Vector3f GetWorldContactOffset(const Mesh* body, const Eigen::Matrix3f& rotation, const Eigen::Vector3f& storedOffset)
{
    if (body->modelType == ModelType_Sphere)
        return storedOffset;

    return rotation * storedOffset;
}

//================================//
static bool IsConvexPolyhedronMesh(const Mesh* body)
{
    return body &&
           body->modelType != ModelType_Sphere &&
           body->modelType != ModelType_Capsule;
}

//================================//
static float GetNewNormalContactPenalty(const Solver* solver, float normalGap)
{
    // New contacts need enough normal support on their first primal solve.
    // The dual loop will adapt the penalty afterwards.  Using beta keeps the
    // bootstrap tied to the existing AVBD constraint update instead of adding
    // a separate penetration tuning knob.
    const float violation = std::max(0.0f, -normalGap);
    return std::max(PENALTY_MIN, solver->beta * violation);
}

//================================//
static void RemoveCollisionContact(CollisionResult& collision, int contactIndex)
{
    for (int i = contactIndex; i + 1 < collision.numContacts; ++i)
        collision.contactPoints[i] = collision.contactPoints[i + 1];

    collision.numContacts--;
}

//================================//
static void RemoveDuplicatedCollisionContacts(CollisionResult& collision)
{
    for (int i = 0; i < collision.numContacts; ++i)
    {
        for (int j = i + 1; j < collision.numContacts; ++j)
        {
            const bool similarNormal =
                collision.contactPoints[i].normal.dot(collision.contactPoints[j].normal) >= CONTACT_MATCH_NORMAL_DOT;
            const bool sameLocalPoint =
                (collision.contactPoints[i].rA - collision.contactPoints[j].rA).squaredNorm() <=
                DUPLICATE_CONTACT_DISTANCE_SQUARED;

            if (!similarNormal || !sameLocalPoint)
                continue;

            // Keep the deeper candidate when clipping returns the same local patch twice.
            if (collision.contactPoints[j].penetration > collision.contactPoints[i].penetration)
                collision.contactPoints[i] = collision.contactPoints[j];

            RemoveCollisionContact(collision, j);
            j--;
        }
    }
}

//================================//
static int FindSearchDirectionContact(const CollisionResult& collision, const bool selected[8])
{
    const Eigen::Vector3f searchDirection(1.0f, 1.0f, 1.0f);
    int bestContactIndex = -1;
    float bestDot = -std::numeric_limits<float>::max();

    for (int i = 0; i < collision.numContacts; ++i)
    {
        if (selected[i])
            continue;

        const float dot = searchDirection.dot(collision.contactPoints[i].rA);
        if (dot > bestDot)
        {
            bestDot = dot;
            bestContactIndex = i;
        }
    }

    return bestContactIndex;
}

//================================//
static int FindFarthestContact(const CollisionResult& collision, const bool selected[8], int pointIndex)
{
    int bestContactIndex = -1;
    float bestDistance = -1.0f;

    for (int i = 0; i < collision.numContacts; ++i)
    {
        if (selected[i])
            continue;

        const float distance =
            (collision.contactPoints[pointIndex].rA - collision.contactPoints[i].rA).squaredNorm();
        if (distance >= bestDistance)
        {
            bestDistance = distance;
            bestContactIndex = i;
        }
    }

    return bestContactIndex;
}

//================================//
static float ComputeContactArea(const ContactPoint& pointA,
                                const ContactPoint& pointB,
                                const ContactPoint& pointC,
                                const Eigen::Vector3f& localNormal)
{
    const Eigen::Vector3f fromCToA = pointA.rA - pointC.rA;
    const Eigen::Vector3f fromCToB = pointB.rA - pointC.rA;
    return fromCToA.cross(fromCToB).dot(localNormal);
}

//================================//
static int FindLargestAreaContact(const CollisionResult& collision,
                                  const bool selected[8],
                                  int pointIndexA,
                                  int pointIndexB,
                                  const Eigen::Vector3f& localNormal,
                                  float& selectedArea)
{
    int positiveAreaIndex = -1;
    int negativeAreaIndex = -1;
    float largestArea = 0.0f;
    float smallestArea = 0.0f;

    for (int i = 0; i < collision.numContacts; ++i)
    {
        if (selected[i])
            continue;

        const float area = ComputeContactArea(collision.contactPoints[pointIndexA],
                                              collision.contactPoints[pointIndexB],
                                              collision.contactPoints[i],
                                              localNormal);
        if (area >= largestArea)
        {
            largestArea = area;
            positiveAreaIndex = i;
        }
        if (area <= smallestArea)
        {
            smallestArea = area;
            negativeAreaIndex = i;
        }
    }

    if (largestArea > -smallestArea)
    {
        selectedArea = largestArea;
        return positiveAreaIndex;
    }

    selectedArea = smallestArea;
    return negativeAreaIndex;
}

//================================//
static int FindOppositeAreaContact(const CollisionResult& collision,
                                   const bool selected[8],
                                   const int selectedIndices[4],
                                   const Eigen::Vector3f& localNormal,
                                   bool previousAreaPositive)
{
    int bestContactIndex = -1;
    float bestArea = 0.0f;

    for (int i = 0; i < collision.numContacts; ++i)
    {
        if (selected[i])
            continue;

        if (bestContactIndex < 0)
            bestContactIndex = i;

        for (int edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            const int edgePointA = selectedIndices[edgeIndex];
            const int edgePointB = selectedIndices[(edgeIndex + 1) % 3];
            const float area = ComputeContactArea(collision.contactPoints[edgePointA],
                                                  collision.contactPoints[edgePointB],
                                                  collision.contactPoints[i],
                                                  localNormal);

            if (previousAreaPositive && area <= bestArea)
            {
                bestArea = area;
                bestContactIndex = i;
            }
            else if (!previousAreaPositive && area >= bestArea)
            {
                bestArea = area;
                bestContactIndex = i;
            }
        }
    }

    return bestContactIndex;
}

//================================//
static void ReduceCollisionContacts(const Mesh* bodyA, CollisionResult& collision)
{
    RemoveDuplicatedCollisionContacts(collision);
    if (collision.numContacts <= MAX_SOLVER_MANIFOLD_CONTACTS)
        return;

    bool selected[8] = {};
    int selectedIndices[4] = {};

    selectedIndices[0] = FindSearchDirectionContact(collision, selected);
    selected[selectedIndices[0]] = true;

    selectedIndices[1] = FindFarthestContact(collision, selected, selectedIndices[0]);
    selected[selectedIndices[1]] = true;

    Eigen::Vector3f localNormal =
        bodyA->transform.GetRotation().toRotationMatrix().transpose() *
        collision.contactPoints[selectedIndices[0]].normal;
    if (localNormal.squaredNorm() <= 1e-12f)
        localNormal = Eigen::Vector3f::UnitY();
    else
        localNormal.normalize();

    float thirdArea = 0.0f;
    selectedIndices[2] = FindLargestAreaContact(collision, selected,
                                                selectedIndices[0], selectedIndices[1],
                                                localNormal, thirdArea);
    selected[selectedIndices[2]] = true;

    selectedIndices[3] = FindOppositeAreaContact(collision, selected, selectedIndices,
                                                 localNormal, thirdArea >= 0.0f);

    ContactPoint reducedContacts[MAX_SOLVER_MANIFOLD_CONTACTS];
    for (int i = 0; i < MAX_SOLVER_MANIFOLD_CONTACTS; ++i)
        reducedContacts[i] = collision.contactPoints[selectedIndices[i]];

    for (int i = 0; i < MAX_SOLVER_MANIFOLD_CONTACTS; ++i)
        collision.contactPoints[i] = reducedContacts[i];

    collision.numContacts = MAX_SOLVER_MANIFOLD_CONTACTS;
}

//================================//
Manifold::Manifold(Solver* solver, Mesh* bodyA, Mesh* bodyB)
    : Force(solver, FilterNulls({bodyA, bodyB}), NUM_CONSTRAINTS), bodyA(bodyA), bodyB(bodyB)
{
    assert((bodyA != nullptr) && (bodyB != nullptr) && "A manifold must be connected to two bodies.");

    for (int i = 0; i < NUM_CONSTRAINTS; i += 3)
    {
        constraintPoints[i].fmaxMagnitude = 0.f; // max friction force for normal, which is compressive only
        constraintPoints[i].fminMagnitude = -INFINITY;
    }

    isManifold = true;
    includeHessian = false;
}

//================================//
bool Manifold::Initialize()
{
    if (!bodyA || !bodyB) return false;

    friction = std::sqrt(bodyA->friction * bodyB->friction);

    // ---- Save old state for warmstarting ----
    int oldNumContacts = numContactPoints;
    ContactPoint        oldContacts[MAX_CONTACT_POINTS];
    ManifoldContactInfo oldInfo[MAX_CONTACT_POINTS];
    float               oldPenalty[NUM_CONSTRAINTS];
    float               oldLambda[NUM_CONSTRAINTS];
    bool                oldContactUsed[MAX_CONTACT_POINTS] = {};
    bool                contactWasMatched[MAX_CONTACT_POINTS] = {};

    for (int i = 0; i < oldNumContacts; i++)
    {
        oldContacts[i] = contactPoints[i];
        oldInfo[i]     = contactInfos[i];
    }
    for (int i = 0; i < oldNumContacts * 3; i++)
    {
        oldPenalty[i] = constraintPoints[i].penalty;
        oldLambda[i]  = constraintPoints[i].lambda;
    }

    // Collision detection
    CollisionResult collision;
    if (IsConvexPolyhedronMesh(bodyA) && IsConvexPolyhedronMesh(bodyB))
    {
        collision = CollisionSpace::CollideHullHullSAT(bodyA, bodyB, &convexSATCache);
    }
    else
    {
        convexSATCache = ConvexSATCache{};
        collision = CollisionSpace::CollisionMeshMesh(bodyA, bodyB);
    }
    ReduceCollisionContacts(bodyA, collision);
    numContactPoints = collision.numContacts;
    if (numContactPoints == 0)
        return false;

    for (int i = 0; i < numContactPoints; i++)
        contactPoints[i] = collision.contactPoints[i];

    // We have to reset states for the new contacts
    for (int i = 0; i < numContactPoints * 3; i++)
    {
        constraintPoints[i].penalty = 0.f;
        constraintPoints[i].lambda  = 0.f;
    }
    for (int i = 0; i < numContactPoints; i++)
        contactInfos[i] = ManifoldContactInfo{};

    // Matching
    for (int i = 0; i < numContactPoints; i++)
    {
        int matchingContactIndex = -1;
        float bestMatchDistance = std::numeric_limits<float>::max();

        for (int j = 0; j < oldNumContacts; j++)
        {
            if (oldContactUsed[j])
                continue;

            const bool similarNormal = oldContacts[j].normal.dot(contactPoints[i].normal) >= CONTACT_MATCH_NORMAL_DOT;
            const float localDistanceSquared = (oldContacts[j].rA - contactPoints[i].rA).squaredNorm();
            const bool similarAnchor = localDistanceSquared <= CONTACT_MATCH_DISTANCE_SQUARED;

            if (similarNormal && similarAnchor && localDistanceSquared < bestMatchDistance)
            {
                bestMatchDistance = localDistanceSquared;
                matchingContactIndex = j;
            }
        }

        if (matchingContactIndex < 0)
            continue;

        oldContactUsed[matchingContactIndex] = true;
        contactWasMatched[i] = true;

        for (int k = 0; k < 3; k++)
        {
            constraintPoints[i * 3 + k].penalty = oldPenalty[matchingContactIndex * 3 + k];
            constraintPoints[i * 3 + k].lambda  = oldLambda[matchingContactIndex * 3 + k];
        }
        contactInfos[i].stick = oldInfo[matchingContactIndex].stick;

        // If sticking, reuse old local offsets for stable static friction
        const bool nearbyAnchors =
            (oldContacts[matchingContactIndex].rA - contactPoints[i].rA).squaredNorm() <=
            CONTACT_MATCH_DISTANCE_SQUARED &&
            (oldContacts[matchingContactIndex].rB - contactPoints[i].rB).squaredNorm() <=
            CONTACT_MATCH_DISTANCE_SQUARED;
        if (contactPoints[i].penetration <= COLLISION_MARGIN)
        {
            for (int k = 0; k < 3; ++k)
                constraintPoints[i * 3 + k].lambda = 0.0f;
            contactInfos[i].stick = false;
        }
        else if (contactInfos[i].stick && nearbyAnchors)
        {
            contactPoints[i].rA = oldContacts[matchingContactIndex].rA;
            contactPoints[i].rB = oldContacts[matchingContactIndex].rB;
        }
    }

    // Precomputed Jacobian factors for normal and both tangent directions
    Eigen::Vector3f posA = bodyA->transform.GetPosition();
    Eigen::Vector3f posB = bodyB->transform.GetPosition();

    Quaternionf qA = bodyA->transform.GetRotation();
    Quaternionf qB = bodyB->transform.GetRotation();

    Eigen::Matrix3f rotA = qA.toRotationMatrix();
    Eigen::Matrix3f rotB = qB.toRotationMatrix();

    for (int i = 0; i < numContactPoints; i++)
    {
        Eigen::Vector3f n = -contactPoints[i].normal;

        Eigen::Vector3f t1;
        if (std::abs(n.x()) > 0.9f)
            t1 = n.cross(Eigen::Vector3f(0.f, 1.f, 0.f));
        else
            t1 = n.cross(Eigen::Vector3f(1.f, 0.f, 0.f));
        t1.normalize();
        Eigen::Vector3f t2 = n.cross(t1);


        Eigen::Vector3f rAw = GetWorldContactOffset(bodyA, rotA, contactPoints[i].rA);
        Eigen::Vector3f rBw = GetWorldContactOffset(bodyB, rotB, contactPoints[i].rB);

        // ---- Jacobians ----
        //
        //  Each constraint row C_k has a 6-DOF Jacobian per body:
        //    J_A = [ ∂C/∂posA ,  ∂C/∂θA ] = [ d ,  rA_world × d ]
        //    J_B = [ ∂C/∂posB ,  ∂C/∂θB ] = [-d , -(rB_world × d)]
        //
        //  where d is the constraint direction (n, t1, or t2).
        //
        //  2D analogue (vec3 = [tx, ty, ω]):
        //    JacNormA = [n_x, n_y, cross2(rA, n)]
        //
        //  3D version (vec6 = [tx, ty, tz, ωx, ωy, ωz]):
        //    JacNormA = [n, rAw × n]

        // Normal
        contactInfos[i].JacNormA.head<3>() =  n;
        contactInfos[i].JacNormA.tail<3>() =  rAw.cross(n);
        contactInfos[i].JacNormB.head<3>() = -n;
        contactInfos[i].JacNormB.tail<3>() = -(rBw.cross(n));

        // Tangent 1
        contactInfos[i].JacTang1A.head<3>() =  t1;
        contactInfos[i].JacTang1A.tail<3>() =  rAw.cross(t1);
        contactInfos[i].JacTang1B.head<3>() = -t1;
        contactInfos[i].JacTang1B.tail<3>() = -(rBw.cross(t1));

        // Tangent 2
        contactInfos[i].JacTang2A.head<3>() =  t2;
        contactInfos[i].JacTang2A.tail<3>() =  rAw.cross(t2);
        contactInfos[i].JacTang2B.head<3>() = -t2;
        contactInfos[i].JacTang2B.tail<3>() = -(rBw.cross(t2));

        // ---- C0: constraint gap at start of step ----
        // The normal row uses narrowphase penetration below. The local anchor
        // displacement is still needed for both tangential rows.
        //
        //  2D:  C0 = basis * (worldA - worldB) + [margin, 0]
        //  3D:  C0 = [n · rDiff, t1 · rDiff, t2 · rDiff] + [margin, 0, 0]
        //
        //  At detection time worldA ≈ worldB (same contact point),
        //  so C0 ≈ [margin, 0, 0]. On subsequent frames the bodies
        //  have moved and the stored rA/rB track the gap.

        Eigen::Vector3f worldA = posA + rAw;
        Eigen::Vector3f worldB = posB + rBw;
        Eigen::Vector3f rDiff  = worldA - worldB;

        contactInfos[i].C0[0] = COLLISION_MARGIN - contactPoints[i].penetration;
        contactInfos[i].C0[1] = t1.dot(rDiff);
        contactInfos[i].C0[2] = t2.dot(rDiff);

        if (!contactWasMatched[i])
        {
            constraintPoints[i * 3 + 0].penalty =
                GetNewNormalContactPenalty(solver, contactInfos[i].C0[0]);
            constraintPoints[i * 3 + 1].penalty = PENALTY_MIN;
            constraintPoints[i * 3 + 2].penalty = PENALTY_MIN;
        }
    }

    return numContactPoints > 0;
}

//================================//
void Manifold::ComputeConstraints(float alpha)
{
    Eigen::Vector3f posA = bodyA->transform.GetPosition();
    Eigen::Vector3f posB = bodyB->transform.GetPosition();

    Quaternionf qA = bodyA->transform.GetRotation();
    Quaternionf qB = bodyB->transform.GetRotation();

    Vector6f diffA, diffB;
    diffA.head<3>() = posA - bodyA->lastPosition;
    diffA.tail<3>() = RotationDifference(qA, bodyA->lastRotation);
    diffB.head<3>() = posB - bodyB->lastPosition;
    diffB.tail<3>() = RotationDifference(qB, bodyB->lastRotation);
        
    for (int i = 0; i < numContactPoints; i++)
    {
        // taylor series: C(x) ≈ (1-α) * C0 + J * diff(x)
        Eigen::Vector3f alphaC0 = (1.f - alpha) * contactInfos[i].C0;

        constraintPoints[i * 3 + 0].C = alphaC0[0] + contactInfos[i].JacNormA.dot(diffA) + contactInfos[i].JacNormB.dot(diffB);
        constraintPoints[i * 3 + 1].C = alphaC0[1] + contactInfos[i].JacTang1A.dot(diffA) + contactInfos[i].JacTang1B.dot(diffB);
        constraintPoints[i * 3 + 2].C = alphaC0[2] + contactInfos[i].JacTang2A.dot(diffA) + contactInfos[i].JacTang2B.dot(diffB);

        if (constraintPoints[i * 3 + 0].C >= 0.0f)
        {
            constraintPoints[i * 3 + 0].lambda = 0.0f;
            constraintPoints[i * 3 + 1].lambda = 0.0f;
            constraintPoints[i * 3 + 2].lambda = 0.0f;
            contactInfos[i].stick = false;
        }

        // Now the coulomb friction bounds, we normaly did set up the normal to be compressive only
        float normalLambda = std::max(0.0f, -constraintPoints[i * 3 + 0].lambda);
        float bounds = normalLambda * friction;

        constraintPoints[i * 3 + 1].fmaxMagnitude = bounds;
        constraintPoints[i * 3 + 1].fminMagnitude = -bounds;
        constraintPoints[i * 3 + 2].fmaxMagnitude = bounds;
        constraintPoints[i * 3 + 2].fminMagnitude = -bounds;

        // Stickiness check: if inside the bounds, we stick to the contact point below threshold
        float tangentLambda = std::sqrt(constraintPoints[i * 3 + 1].lambda * constraintPoints[i * 3 + 1].lambda +
                                        constraintPoints[i * 3 + 2].lambda * constraintPoints[i * 3 + 2].lambda);
        float tangentC0 = std::sqrt(contactInfos[i].C0[1] * contactInfos[i].C0[1] + contactInfos[i].C0[2] * contactInfos[i].C0[2]);
            
        contactInfos[i].stick = (tangentLambda < bounds) && (tangentC0 < STICK_THRESHOLD);
    }
}

//================================//
void Manifold::ComputeDerivatives(Mesh* mesh)
{
    for (int i = 0; i < numContactPoints; i++)
    {
        if (mesh == bodyA)
        {
            constraintPoints[i * 3 + 0].J = contactInfos[i].JacNormA;
            constraintPoints[i * 3 + 1].J = contactInfos[i].JacTang1A;
            constraintPoints[i * 3 + 2].J = contactInfos[i].JacTang2A;
        }
        else if (mesh == bodyB)
        {
            constraintPoints[i * 3 + 0].J = contactInfos[i].JacNormB;
            constraintPoints[i * 3 + 1].J = contactInfos[i].JacTang1B;
            constraintPoints[i * 3 + 2].J = contactInfos[i].JacTang2B;
        }
    }
}

//================================//
void Manifold::AddLineData(std::vector<GPULineData>& data) const
{
    for (int i = 0; i < numContactPoints; i++)
    {
        if (contactPoints[i].penetration <= 0.f) continue;

        Eigen::Vector3f start = contactPoints[i].position;
        Eigen::Vector3f end   = start + contactPoints[i].normal * contactPoints[i].penetration;

        GPULineData line;
        Eigen::Map<Eigen::Vector3f>(line.start) = start;
        Eigen::Map<Eigen::Vector3f>(line.end)   = end;
        line.color[0] = 1.f;
        line.color[1] = 0.f;
        line.color[2] = 0.f;
        line.color[3] = 1.f;
        data.push_back(line);
    }
}

//================================//
void Manifold::AddDebugPointData(std::vector<GPUDebugPointData>& data) const
{
    Eigen::Vector3f  posA = bodyA->transform.GetPosition();
    Eigen::Vector3f  posB = bodyB->transform.GetPosition();
    Quaternionf qA = bodyA->transform.GetRotation();
    Quaternionf qB = bodyB->transform.GetRotation();

    for (int i = 0; i < numContactPoints; i++)
    {
        GPUDebugPointData point;
        Eigen::Vector3f worldA = posA + GetWorldContactOffset(bodyA, qA.toRotationMatrix(), contactPoints[i].rA);
        Eigen::Map<Eigen::Vector3f>(point.position) = worldA;
        point.color[0] = 1.f; point.color[1] = 1.f;
        point.color[2] = 0.f; point.color[3] = 1.f;
        data.push_back(point);

        GPUDebugPointData pointB;
        Eigen::Vector3f worldB = posB + GetWorldContactOffset(bodyB, qB.toRotationMatrix(), contactPoints[i].rB);
        Eigen::Map<Eigen::Vector3f>(pointB.position) = worldB;
        pointB.color[0] = 0.f; pointB.color[1] = 1.f;
        pointB.color[2] = 1.f; pointB.color[3] = 1.f;
        data.push_back(pointB);
    }
}
