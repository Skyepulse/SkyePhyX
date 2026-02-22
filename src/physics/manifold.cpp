#include "force.hpp"
#include "solver.hpp"

constexpr float PENALTY_MIN = 1.0f;

//================================//
static std::vector<Mesh*> FilterNulls(std::initializer_list<Mesh*> bodies)
{
    std::vector<Mesh*> result;
    for (Mesh* m : bodies)
        if (m) result.push_back(m);
    return result;
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
    ContactPoint        oldContacts[8];
    ManifoldContactInfo oldInfo[8];
    float               oldPenalty[24];
    float               oldLambda[24];

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
    CollisionResult collision = CollisionSpace::CollisionBoxBox(bodyA, bodyB);
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
        uint32_t id = contactPoints[i].id;
        for (int j = 0; j < oldNumContacts; j++)
        {
            if (oldContacts[j].id == id)
            {
                // Restore accumulated solver state
                for (int k = 0; k < 3; k++)
                {
                    constraintPoints[i * 3 + k].penalty = oldPenalty[j * 3 + k];
                    constraintPoints[i * 3 + k].lambda  = oldLambda[j * 3 + k];
                }
                contactInfos[i].stick = oldInfo[j].stick;

                // If sticking, reuse old local offsets for stable static friction
                if (contactInfos[i].stick)
                {
                    contactPoints[i].rA = oldContacts[j].rA;
                    contactPoints[i].rB = oldContacts[j].rB;
                }
                break;
            }
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


        Eigen::Vector3f rAw = rotA * contactPoints[i].rA;
        Eigen::Vector3f rBw = rotB * contactPoints[i].rB;

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

        contactInfos[i].C0[0] = n.dot(rDiff)  + COLLISION_MARGIN;
        contactInfos[i].C0[1] = t1.dot(rDiff);
        contactInfos[i].C0[2] = t2.dot(rDiff);

        // Better penalty init
        bool isNew = true;
        for (int j = 0; j < oldNumContacts; j++)
        {
            if (oldContacts[j].id == contactPoints[i].id) { isNew = false; break; }
        }

        if (isNew)
        {
            float depth = contactPoints[i].penetration;
            float seedPenalty = std::clamp(depth * solver->onPenetrationPenalty, PENALTY_MIN, 100000.0f);
            constraintPoints[i * 3 + 0].penalty = seedPenalty;  // normal
            constraintPoints[i * 3 + 1].penalty = PENALTY_MIN;  // tangent1
            constraintPoints[i * 3 + 2].penalty = PENALTY_MIN;  // tangent2
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

        // Now the coulomb friction bounds, we normaly did set up the normal to be compressive only
        float normalLambda = std::abs(constraintPoints[i * 3 + 0].lambda);
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
    // Line from center of bodyA to center of bodyB
    Eigen::Vector3f posA = bodyA->transform.GetPosition();
    Eigen::Vector3f posB = bodyB->transform.GetPosition();

    GPULineData link;
    Eigen::Map<Eigen::Vector3f>(link.start) = posA;
    Eigen::Map<Eigen::Vector3f>(link.end)   = posB;
    link.color[0] = 1.f;
    link.color[1] = 0.f;
    link.color[2] = 0.f;
    link.color[3] = 1.f;
    data.push_back(link);
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
        Eigen::Vector3f worldA = posA + qA.toRotationMatrix() * contactPoints[i].rA;
        Eigen::Map<Eigen::Vector3f>(point.position) = worldA;
        point.color[0] = 1.f; point.color[1] = 1.f;
        point.color[2] = 0.f; point.color[3] = 1.f;
        data.push_back(point);

        GPUDebugPointData pointB;
        Eigen::Vector3f worldB = posB + qB.toRotationMatrix() * contactPoints[i].rB;
        Eigen::Map<Eigen::Vector3f>(pointB.position) = worldB;
        pointB.color[0] = 0.f; pointB.color[1] = 1.f;
        pointB.color[2] = 1.f; pointB.color[3] = 1.f;
        data.push_back(pointB);
    }
}