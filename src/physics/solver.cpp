#include "solver.hpp"
#include <chrono>
#include <corecrt_math_defines.h>
#include <iostream>

const Eigen::Vector3f GRAVITY(0.0f, -9.81f, 0.0f);

const float PENALTY_MIN = 1;
const float PENALTY_MAX = 1000000000;
const float ENERGY_STIFFNESS_MIN = 0.01;

const float MAX_ROTATION_VELOCITY = 50.0f;

//================================//
Solver::Solver(): solverBodies(nullptr)
{
}

//================================//
Solver::~Solver()
{
    while(solverBodies)
        delete solverBodies;
}

//================================//
void Solver::Start()
{
    this->Clear();
    //Mesh* cube = AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(4.0f, 5.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), false);
    //cube->name = "Cube";
    //Mesh* sphere = AddBody(ModelType_Sphere, 1.0f, 0.5f, Eigen::Vector3f(0.0f, 5.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    //sphere->name = "Sphere";
    //Force* spring = new Spring(this, cube, Eigen::Vector3f(0.5f, 0.5f, 0.5f), sphere, Eigen::Vector3f(0.0f, 0.0f, 0.0f), INFINITY, 10.0f, true);
    Mesh* ground = AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, -10.0f, 0.0f), Eigen::Vector3f(20.0f, 1.0f, 20.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    ground->name = "Ground";

    //Mesh* cube1 = AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(-4.0f, 5.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::UnitRandom(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), false);
    //cube1->name = "Cube1";

    Quaternionf rot = Eigen::AngleAxisf(M_PI / 4.0f, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(M_PI / 6.0f, Eigen::Vector3f::UnitX());
    Mesh* cube2 = AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, 10.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), rot, Eigen::Vector3f(0.0f, 0.0f, 0.0f), false);
    cube2->name = "Cube2";
}

//================================//
void Solver::Clear()
{
    while(solverBodies)
        delete solverBodies;
    solverBodies = nullptr;

    while(solverForces)
        delete solverForces;
    solverForces = nullptr;
}

//================================//
Mesh* Solver::AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color)
{
    new Mesh(this, modelType, color);

    Mesh* newMesh = solverBodies;

    newMesh->density            = density;
    newMesh->friction           = friction;
    newMesh->velocity           = velocity;
    newMesh->angularVelocity    = angularVelocity;
    newMesh->isStatic           = isStatic;

    newMesh->transform.SetPosition(position);
    newMesh->transform.SetRotation(rotation);
    newMesh->transform.SetScale(scale);

    newMesh->detectionRadius = scale.norm() * 0.5f;

    if (isStatic)
    {
        newMesh->inertiaTensorBody    = Eigen::Matrix3f::Zero();
        newMesh->inertiaTensorBodyInv = Eigen::Matrix3f::Zero();
        return newMesh;
    }

    float sx2 = scale.x() * scale.x(); float sy2 = scale.y() * scale.y(); float sz2 = scale.z() * scale.z();
    float r = scale.x() * 0.5f;

    Eigen::Vector3f I;
    switch(modelType)
    {
        case ModelType_Cube:
            newMesh->mass = density * scale.x() * scale.y() * scale.z();
            I = (newMesh->mass / 12.0f) * Eigen::Vector3f(sy2 + sz2, sx2 + sz2, sx2 + sy2);
            break;
        case ModelType_Sphere:
            newMesh->mass = density * (4.0f / 3.0f) * M_PI * r * r * r;
            I = Eigen::Vector3f::Constant((2.0f / 5.0f) * newMesh->mass * r * r);
            break;
        default:
            newMesh->mass = density * scale.x() * scale.y() * scale.z();
    }

    newMesh->inertiaTensorBody = I.asDiagonal();
    newMesh->inertiaTensorBodyInv = I.cwiseInverse().asDiagonal();

    return newMesh;
}

//================================//
void Solver::Step()
{
    SolverTimings& out = this->timings;

    using Clock = std::chrono::high_resolution_clock;
    auto elapsed = [](Clock::time_point start) 
    {
        return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
    };
    auto stepStart = Clock::now();

    // 1. Broad phase detection
    auto phaseStart = Clock::now();
    for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
    {
        for (Mesh* other = mesh->next; other; other = other->next)
        {
            Eigen::Vector3f pos1, pos2;
            mesh->transform.GetPosition(pos1);
            other->transform.GetPosition(pos2);

            float distance = (pos1 - pos2).norm();
            if (distance < mesh->detectionRadius + other->detectionRadius)
            {
                if (!isConstrainedTo(mesh, other))
                {
                    Manifold* manifold = new Manifold(this, mesh, other);
                }
            }
        }
    }
    out.broadPhaseMs = elapsed(phaseStart);

    this->lineData.clear();
    this->debugPointData.clear();

    // 2. Forces Warmstarting
    phaseStart = Clock::now();
    for (Force* force = solverForces; force != nullptr;)
    {
        const bool isUsed = force->Initialize();

        if (!isUsed)
        {
            Manifold* manifold = dynamic_cast<Manifold*>(force);
            if (manifold)
            {
                Eigen::Vector3f p1, p2;
                manifold->bodyA->transform.GetPosition(p1);
                manifold->bodyB->transform.GetPosition(p2);

                float dist = (p1 - p2).norm();
                float threshold = manifold->bodyA->detectionRadius + manifold->bodyB->detectionRadius;

                if (dist < threshold * 1.1f)
                {
                    force->AddLineData(this->lineData);
                    force = force->next;
                    continue;
                }
            }

            Force* next = force->next;
            delete force;
            force = next;
            continue;
        }

        force->AddLineData(this->lineData);
        force->AddDebugPointData(this->debugPointData);
        const int numConstraints = force->numConstraints();
        for (int i = 0; i < numConstraints; ++i)
        {
            if (!this->postStabilization)
                force->constraintPoints[i].lambda *= alpha * gamma;

            float newPenalty = force->constraintPoints[i].penalty * gamma;
            if (newPenalty < PENALTY_MIN) newPenalty = PENALTY_MIN;
            if (newPenalty > PENALTY_MAX) newPenalty = PENALTY_MAX;
            force->constraintPoints[i].penalty = newPenalty;

            force->constraintPoints[i].penalty = std::min(force->constraintPoints[i].penalty, force->constraintPoints[i].stiffness);
        }

        force = force->next;
    }
    out.warmstartMs = elapsed(phaseStart);

    // 3. Energies Warmstarting
    // TODO

    // 4. Bodies Warmstarting
    phaseStart = Clock::now();
    for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
    {
        mesh->angularVelocity = mesh->angularVelocity.cwiseMin(Eigen::Vector3f::Constant(MAX_ROTATION_VELOCITY)).cwiseMax(Eigen::Vector3f::Constant(-MAX_ROTATION_VELOCITY));

        Eigen::Vector3f pos;
        mesh->transform.GetPosition(pos);
        mesh->inertialPosition = pos + mesh->velocity * stepValue;
        if (!mesh->isStatic)
        {
            mesh->inertialPosition += GRAVITY * stepValue * stepValue;
        }

        Quaternionf rot;
        mesh->transform.GetRotation(rot);
        mesh->inertialRotation = rot;
        if (!mesh->isStatic && !mesh->isParticle)
        {
            Quaternionf q = mesh->inertialRotation;
            Quaternionf w(0.f, mesh->angularVelocity.x(), mesh->angularVelocity.y(), mesh->angularVelocity.z());

            Quaternionf wq;
            wq.w() = w.w()*q.w() - w.vec().dot(q.vec());
            wq.vec() = w.w()*q.vec() + q.w()*w.vec() + w.vec().cross(q.vec());

            q.w() += 0.5f * wq.w() * stepValue;
            q.vec() += 0.5f * wq.vec() * stepValue;
            mesh->inertialRotation = q.normalized();
        }

        Eigen::Vector3f linearAcceleration = (mesh->velocity - mesh->prevVelocity) / stepValue;
        float gravityMagnitude = GRAVITY.norm();
        float accelWeight = 0.0f;

        if (gravityMagnitude > 1e-6f)
        {
            Eigen::Vector3f gravityDir = GRAVITY / gravityMagnitude;
            float accelAlongGravity = linearAcceleration.dot(gravityDir);
            accelWeight = accelAlongGravity / gravityMagnitude;
            accelWeight = std::clamp(accelWeight, 0.f, 1.f);
        }

        mesh->lastPosition = pos;
        mesh->lastRotation = rot;

        Eigen::Vector3f newPos = pos + mesh->velocity * stepValue + GRAVITY * stepValue * stepValue * accelWeight;
        mesh->transform.SetPosition(newPos);

        if(!mesh->isStatic && !mesh->isParticle)
        {
            Quaternionf q = rot;
            Quaternionf w(0.f, mesh->angularVelocity.x(), mesh->angularVelocity.y(), mesh->angularVelocity.z());

            Quaternionf wq;
            wq.w() = w.w()*q.w() - w.vec().dot(q.vec());
            wq.vec() = w.w()*q.vec() + q.w()*w.vec() + w.vec().cross(q.vec());

            q.w() += 0.5f * wq.w() * stepValue;
            q.vec() += 0.5f * wq.vec() * stepValue;
            mesh->transform.SetRotation(q.normalized());
        }
    }
    out.predictionMs = elapsed(phaseStart);

    // 5. Main Iteration Loop
    phaseStart = Clock::now();
    for (int iter = 0; iter < this->numIterations; ++iter)
    {
        // 5.1 Primal
        for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
        {
            if (mesh->isStatic) continue;

            Matrix6f lhs = mesh->GetGeneralizedMass() / (stepValue * stepValue);
            Vector6f rhs = lhs * mesh->GetDisplacementFromInertial();

            for (Force* force : mesh->forces)
            {
                force->ComputeConstraints(alpha);
                force->ComputeDerivatives(mesh);

                const int numConstraints = force->numConstraints();
                for (int i = 0; i < numConstraints; ++i)
                {
                    float lambda = isinf(force->constraintPoints[i].stiffness) ? force->constraintPoints[i].lambda : 0.f;
                    float f = std::clamp(force->constraintPoints[i].penalty * force->constraintPoints[i].C + lambda, force->constraintPoints[i].fminMagnitude, force->constraintPoints[i].fmaxMagnitude);

                    Matrix6f H = force->constraintPoints[i].H;
                    Matrix6f G = Matrix6f::Zero();
                    for (int j = 0; j < 6; ++j)
                    {
                        G(j, j) = H.col(j).norm() * std::abs(f);
                    }

                    const Vector6f J = force->constraintPoints[i].J;
                    rhs += J * f;
                    lhs += J * J.transpose() * force->constraintPoints[i].penalty + G;
                }
            }

            Vector6f dx = lhs.ldlt().solve(rhs);
            Eigen::Vector3f dx_lin = dx.head<3>();
            Eigen::Vector3f dx_ang = dx.tail<3>();

            Eigen::Vector3f pos;
            mesh->transform.GetPosition(pos);
            mesh->transform.SetPosition(pos - dx_lin);

            Quaternionf dq = QuaternionFromDifference(dx_ang, -1.f);
            Quaternionf rot;
            mesh->transform.GetRotation(rot);
            mesh->transform.SetRotation((dq * rot).normalized());
        }

        // 5.5 Dual Update
        for (Force* force = solverForces; force != nullptr; force = force->next)
        {
            force->ComputeConstraints(alpha);

            const int numConstraints = force->numConstraints();
            for (int i = 0; i < numConstraints; ++i)
            {
                float lambda = isinf(force->constraintPoints[i].stiffness) ? force->constraintPoints[i].lambda : 0.f;

                force->constraintPoints[i].lambda = lambda + force->constraintPoints[i].penalty * force->constraintPoints[i].C;
                if (force->constraintPoints[i].lambda < force->constraintPoints[i].fminMagnitude)
                    force->constraintPoints[i].lambda = force->constraintPoints[i].fminMagnitude;
                else if (force->constraintPoints[i].lambda > force->constraintPoints[i].fmaxMagnitude)
                    force->constraintPoints[i].lambda = force->constraintPoints[i].fmaxMagnitude;

                if (std::abs(force->constraintPoints[i].lambda) >= force->constraintPoints[i].fracture)
                    force->Disable();

                if (force->constraintPoints[i].lambda > force->constraintPoints[i].fminMagnitude
                    && force->constraintPoints[i].lambda < force->constraintPoints[i].fmaxMagnitude)
                {
                    force->constraintPoints[i].penalty = std::min(force->constraintPoints[i].penalty + beta * std::abs(force->constraintPoints[i].C), std::min(force->constraintPoints[i].stiffness, PENALTY_MAX));
                }
            }
        }
    }
    out.primalDualMs = elapsed(phaseStart);

    // 6. Velocity update
    phaseStart = Clock::now();
    for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
    {
        if (mesh->isStatic) continue;

        mesh->prevVelocity = mesh->velocity;
        mesh->prevAngularVelocity = mesh->angularVelocity;

        Eigen::Vector3f pos;
        mesh->transform.GetPosition(pos);
        Eigen::Vector3f newVelocity = (pos - mesh->lastPosition) / stepValue;

        if (mesh->isDragged)
        {
            newVelocity += mesh->addedDragVelocity;
            mesh->addedDragVelocity.setZero();
        }

        mesh->velocity = newVelocity;

        if (!mesh->isParticle)
        {
            Quaternionf rot;
            mesh->transform.GetRotation(rot);
            mesh->angularVelocity = RotationDifference(rot, mesh->lastRotation) / stepValue;
        }
    }
    out.velocityUpdateMs = elapsed(phaseStart);

    // 7. Post-stabilization: alpha = 0.0f
    phaseStart = Clock::now();
    if (postStabilization)
    {
        for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
        {
            if (mesh->isStatic) continue;

            Matrix6f lhs = mesh->GetGeneralizedMass() / (stepValue * stepValue);
            Vector6f rhs = lhs * mesh->GetDisplacementFromInertial();

            for (Force* force : mesh->forces)
            {
                force->ComputeConstraints(0.0f);
                force->ComputeDerivatives(mesh);

                const int numConstraints = force->numConstraints();
                for (int i = 0; i < numConstraints; ++i)
                {
                    float lambda = isinf(force->constraintPoints[i].stiffness) ? force->constraintPoints[i].lambda : 0.f;
                    float f = std::clamp(force->constraintPoints[i].penalty * force->constraintPoints[i].C + lambda, force->constraintPoints[i].fminMagnitude, force->constraintPoints[i].fmaxMagnitude);

                    Matrix6f H = force->constraintPoints[i].H;
                    Matrix6f G = Matrix6f::Zero();
                    for (int j = 0; j < 6; ++j)
                    {
                        G(j, j) = H.col(j).norm() * std::abs(f);
                    }

                    const Vector6f J = force->constraintPoints[i].J;
                    rhs += J * f;
                    lhs += J * J.transpose() * force->constraintPoints[i].penalty + G;
                }
            }

            Vector6f dx = lhs.ldlt().solve(rhs);
            Eigen::Vector3f dx_lin = dx.head<3>();
            Eigen::Vector3f dx_ang = dx.tail<3>();

            Eigen::Vector3f pos;
            mesh->transform.GetPosition(pos);
            mesh->transform.SetPosition(pos - dx_lin);

            Quaternionf dq = QuaternionFromDifference(dx_ang, -1.f);
            Quaternionf rot;
            mesh->transform.GetRotation(rot);
            mesh->transform.SetRotation((dq * rot).normalized());
        }
    }
    out.postStabMs = elapsed(phaseStart);

    out.totalSubstepMs = elapsed(stepStart);

    // Rolling average
    timingAccumulator.push_back(out);
    if (timingAccumulator.size() > TIMING_WINDOW)
        timingAccumulator.erase(timingAccumulator.begin());

    SolverTimings avg{};
    for (const auto& t : timingAccumulator)
    {
        avg.broadPhaseMs     += t.broadPhaseMs;
        avg.warmstartMs      += t.warmstartMs;
        avg.predictionMs     += t.predictionMs;
        avg.primalDualMs     += t.primalDualMs;
        avg.velocityUpdateMs += t.velocityUpdateMs;
        avg.postStabMs       += t.postStabMs;
        avg.totalSubstepMs   += t.totalSubstepMs;
    }
    float n = static_cast<float>(timingAccumulator.size());
    this->timings.broadPhaseMs     = avg.broadPhaseMs / n;
    this->timings.warmstartMs      = avg.warmstartMs / n;
    this->timings.predictionMs     = avg.predictionMs / n;
    this->timings.primalDualMs     = avg.primalDualMs / n;
    this->timings.velocityUpdateMs = avg.velocityUpdateMs / n;
    this->timings.postStabMs       = avg.postStabMs / n;
    this->timings.totalSubstepMs   = avg.totalSubstepMs / n;

    averageStepTime = this->timings.totalSubstepMs;
}