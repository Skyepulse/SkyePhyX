#include "solver.hpp"
#include <chrono>
#include <corecrt_math_defines.h>
#include <iostream>
#include "../constants.hpp"

const Eigen::Vector3f GRAVITY(0.0f, -9.81f, 0.0f);

const float PENALTY_MIN = 1;
const float PENALTY_MAX = 1000000000;
const float ENERGY_STIFFNESS_MIN = 0.01;

const float MAX_ROTATION_VELOCITY = 50.0f;

//================================//
Solver::Solver()
{
}

//================================//
Solver::~Solver()
{
    Clear();
}

//================================//
void Solver::Start()
{
    this->Clear();
}

//================================//
void Solver::Clear()
{
    solverForces.clear();
    solverBodies.clear();
    forcePtrs.clear();
    bodyPtrs.clear();
}

//================================//
Mesh* Solver::AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color)
{
    std::unique_ptr<Mesh> newMesh = std::make_unique<Mesh>(this, modelType, color);
    Mesh* raw = newMesh.get();

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
    }
    else
    {
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
    }

    raw->solverIndex = static_cast<int>(solverBodies.size());
    solverBodies.push_back(std::move(newMesh));

    this->RebuildPtrCaches();
    return raw;
}

//================================//
void Solver::RemoveBody(Mesh* body)
{
    // forces removed in destructor
    int idx = body->solverIndex;
    int last = static_cast<int>(solverBodies.size()) - 1;
    if (idx != last)
    {
        std::swap(solverBodies[idx], solverBodies[last]);
        solverBodies[idx]->solverIndex = idx;
    }
    solverBodies.pop_back();
    RebuildPtrCaches();
}

//================================//
Force* Solver::AddForce(std::unique_ptr<Force> force)
{
    Force* raw = force.get();
    raw->solverIndex = static_cast<int>(solverForces.size());
    solverForces.push_back(std::move(force));
    return raw;
}

//================================//
void Solver::RemoveForce(Force* force)
{
    int idx = force->solverIndex;
    if (idx < 0) return;

    int last = static_cast<int>(solverForces.size()) - 1;
    if (idx != last)
    {
        std::swap(solverForces[idx], solverForces[last]);
        solverForces[idx]->solverIndex = idx;
    }
    solverForces.pop_back(); // Memo for me: this triggers ~Force()
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

    this->RebuildPtrCaches();

    // 0. Cache Generalized Mass and Rotation Matrices
    for (Mesh* mesh : bodyPtrs)
    {
        if (mesh->isStatic) continue;

        Quaternionf q = mesh->transform.GetRotation();
        mesh->cachedRotationMatrix = q.toRotationMatrix();

        if (!mesh->isStatic && mesh->mass > 0.f)
        {
            Matrix6f& M = mesh->cachedGeneralizedMass;
            M.setZero();
            M.block<3,3>(0,0) = Eigen::Matrix3f::Identity() * mesh->mass;
            M.block<3,3>(3,3) = mesh->cachedRotationMatrix 
                            * mesh->inertiaTensorBody 
                            * mesh->cachedRotationMatrix.transpose();
        }
    }

    // 1. Broad phase detection
    auto phaseStart = Clock::now();
    const int N = static_cast<int>(bodyPtrs.size());
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            Mesh* mesh = bodyPtrs[i];
            Mesh* other = bodyPtrs[j];

            Eigen::Vector3f pos1 = mesh->transform.GetPosition();
            Eigen::Vector3f pos2 = other->transform.GetPosition();

            float distance = (pos1 - pos2).norm();
            if (distance < mesh->detectionRadius + other->detectionRadius)
            {
                if (!isConstrainedTo(mesh, other))
                {
                    this->AddForce(std::make_unique<Manifold>(this, mesh, other));
                }
            }
        }
    }
    out.broadPhaseMs = elapsed(phaseStart);

    // 1.5 Rebuild force cache after broad phase
    phaseStart = Clock::now();

    forcePtrs.clear();
    forcePtrs.reserve(solverForces.size());
    for (auto& f : solverForces) forcePtrs.push_back(f.get());

    // 2. Forces Warmstartin
    this->lineData.clear();
    this->debugPointData.clear();
    std::vector<Force*> toRemove;
    for (Force* force : forcePtrs)
    {

        if (!force->Initialize())
        {
            toRemove.push_back(force);
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
    }
    for (Force* f : toRemove)
        RemoveForce(f);
    forcePtrs.clear();
    for (auto& f : solverForces) forcePtrs.push_back(f.get());

    out.warmstartMs = elapsed(phaseStart);

    // 3. Energies Warmstarting
    // TODO

    // 4. Bodies Warmstarting
    phaseStart = Clock::now();
    for (Mesh* mesh : bodyPtrs)
    {
        mesh->angularVelocity = mesh->angularVelocity.cwiseMin(Eigen::Vector3f::Constant(MAX_ROTATION_VELOCITY)).cwiseMax(Eigen::Vector3f::Constant(-MAX_ROTATION_VELOCITY));

        Eigen::Vector3f pos = mesh->transform.GetPosition();
        mesh->inertialPosition = pos + mesh->velocity * stepValue;
        if (!mesh->isStatic)
        {
            mesh->inertialPosition += GRAVITY * stepValue * stepValue;
        }

        Quaternionf rot = mesh->transform.GetRotation();
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
        for (Mesh* mesh : bodyPtrs)
        {
            if (mesh->isStatic) continue;

            Matrix6f lhs = mesh->cachedGeneralizedMass / (stepValue * stepValue);
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
                    lhs.noalias() += J * J.transpose() * force->constraintPoints[i].penalty;

                    if (force->includeHessian)
                        lhs.noalias() += G;
                }
            }

            ldlt.compute(lhs);
            Vector6f dx = ldlt.solve(rhs);
            Eigen::Vector3f dx_lin = dx.head<3>();
            Eigen::Vector3f dx_ang = dx.tail<3>();

            Eigen::Vector3f pos = mesh->transform.GetPosition();
            mesh->transform.SetPosition(pos - dx_lin);

            Quaternionf dq = QuaternionFromDifference(dx_ang, -1.f);
            Quaternionf rot = mesh->transform.GetRotation();
            mesh->transform.SetRotation((dq * rot).normalized());
        }

        // 5.5 Dual Update
        for (Force* force : forcePtrs)
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
    std::vector<Mesh*> oob;
    for (Mesh* mesh : bodyPtrs)
    {
        if (mesh->isStatic) continue;

        mesh->prevVelocity = mesh->velocity;
        mesh->prevAngularVelocity = mesh->angularVelocity;

        Eigen::Vector3f pos = mesh->transform.GetPosition();
        Eigen::Vector3f newVelocity = (pos - mesh->lastPosition) / stepValue;

        if (mesh->isDragged)
        {
            newVelocity += mesh->addedDragVelocity;
            mesh->addedDragVelocity.setZero();
        }

        mesh->velocity = newVelocity;

        if (!mesh->isParticle)
        {
            Quaternionf rot = mesh->transform.GetRotation();
            mesh->angularVelocity = RotationDifference(rot, mesh->lastRotation) / stepValue;
        }

        Eigen::Vector3f p = mesh->transform.GetPosition();
        if (p.x() < MINBOUNDS[0] || p.x() > MAXBOUNDS[0] ||
            p.y() < MINBOUNDS[1] || p.y() > MAXBOUNDS[1] ||
            p.z() < MINBOUNDS[2] || p.z() > MAXBOUNDS[2])
        {
            oob.push_back(mesh);
        }
    }
    out.velocityUpdateMs = elapsed(phaseStart);

    // 6.5 Remove out of bound bodies
    for (Mesh* m : oob)
        RemoveBody(m);

    // 7. Post-stabilization: alpha = 0.0f
    phaseStart = Clock::now();
    if (postStabilization)
    {
        for (Mesh* mesh : bodyPtrs)
        {
            if (mesh->isStatic) continue;

            Matrix6f lhs = mesh->cachedGeneralizedMass / (stepValue * stepValue);
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
                    lhs.noalias() += J * J.transpose() * force->constraintPoints[i].penalty;

                    if (force->includeHessian)
                        lhs.noalias() += G;
                }
            }

            Vector6f dx = lhs.ldlt().solve(rhs);
            Eigen::Vector3f dx_lin = dx.head<3>();
            Eigen::Vector3f dx_ang = dx.tail<3>();

            Eigen::Vector3f pos = mesh->transform.GetPosition();
            mesh->transform.SetPosition(pos - dx_lin);

            Quaternionf dq = QuaternionFromDifference(dx_ang, -1.f);
            Quaternionf rot = mesh->transform.GetRotation();
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