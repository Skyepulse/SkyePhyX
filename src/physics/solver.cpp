#include "solver.hpp"
#include <chrono>
#include <corecrt_math_defines.h>

const Eigen::Vector3f GRAVITY(0.0f, -9.81f, 0.0f);

const float PENALTY_MIN = 1;
const float PENALTY_MAX = 1000000000;
const float ENERGY_STIFFNESS_MIN = 0.01;

const float MAX_ROTATION_VELOCITY = 50.0f;

//================================//
Solver::Solver(): solverBodies(nullptr)
{
    Mesh* cube = AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), false);
    Mesh* sphere = AddBody(ModelType_Sphere, 1.0f, 0.5f, Eigen::Vector3f(0.0f, 5.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
    Force* spring = new Spring(this, cube, Eigen::Vector3f(0.5f, 0.5f, 0.5f), sphere, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 100.0f, 10.0f, true);

    Mesh* ground = AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, -10.0f, 0.0f), Eigen::Vector3f(20.0f, 1.0f, 20.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), true);
}

//================================//
Solver::~Solver()
{
    while(solverBodies)
        delete solverBodies;
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
    auto startTime = std::chrono::high_resolution_clock::now();

    // 1. Broad phase detection
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
                // ADD MANIFOLD IF NOT ALREADY CONSTRAINED
                // TODO
            }
        }
    }

    // 2. Forces Warmstarting
    for (Force* force = solverForces; force != nullptr;)
    {
        const bool isUsed = force->Initialize();

        if (!isUsed)
        {
            Force* next = force->next;
            delete force; 
            force = next;
            continue;   
        }

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

    // 3. Energies Warmstarting
    // TODO

    // 4. Bodies Warmstarting
    for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
    {
        // 4.1 Constrain rotation velocity ...?
        mesh->angularVelocity = mesh->angularVelocity.cwiseMin(Eigen::Vector3f::Constant(MAX_ROTATION_VELOCITY)).cwiseMax(Eigen::Vector3f::Constant(-MAX_ROTATION_VELOCITY));

        // 4.2 Prediction linear
        Eigen::Vector3f pos;
        mesh->transform.GetPosition(pos);
        mesh->inertialPosition = pos + mesh->velocity * stepValue;
        if (!mesh->isStatic)
        {
            mesh->inertialPosition += GRAVITY * stepValue * stepValue;
        }

        // 4.3 Prediction angular
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

        // 4.4 Adaptive warmstarting
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

    // 5. Main Iteration Loop
    for (int iter = 0; iter < this->numIterations; ++iter)
    {
        // 5.1 Primal
        for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
        {
            if (mesh->isStatic) continue;

            Matrix6f lhs = mesh->GetGeneralizedMass() / (stepValue * stepValue);
            Vector6f rhs = lhs * mesh->GetDisplacementFromInertial();

            // 5.2 Loop over forces
            for (Force* force : mesh->forces)
            {
                force->ComputeConstraints(alpha);
                force->ComputeDerivatives(mesh);

                const int numConstraints = force->numConstraints();
                for (int i = 0; i < numConstraints; ++i)
                {
                    float lambda = isinf(force->constraintPoints[i].stiffness) ?force->constraintPoints[i].lambda: 0.f;
                    float f = std::clamp(force->constraintPoints[i].penalty * force->constraintPoints[i].C + lambda, force->constraintPoints[i].fminMagnitude, force->constraintPoints[i].fmaxMagnitude);

                    // Diagonal matrix, of sum of columns of hessian
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

            // 5.3 Loop over energies

            // 5.4 solve LDLT
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
                float lambda = isinf(force->constraintPoints[i].stiffness) ?force->constraintPoints[i].lambda: 0.f;

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

    // 6. Velocity update
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

    // 7. Post-stabilization: alpha = 0.0f
    if (postStabilization)
    {
        for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
        {
            if (mesh->isStatic) continue;

            Matrix6f lhs = mesh->GetGeneralizedMass() / (stepValue * stepValue);
            Vector6f rhs = lhs * mesh->GetDisplacementFromInertial();

            // 5.2 Loop over forces
            for (Force* force : mesh->forces)
            {
                force->ComputeConstraints(0.0f);
                force->ComputeDerivatives(mesh);

                const int numConstraints = force->numConstraints();
                for (int i = 0; i < numConstraints; ++i)
                {
                    float lambda = isinf(force->constraintPoints[i].stiffness) ?force->constraintPoints[i].lambda: 0.f;
                    float f = std::clamp(force->constraintPoints[i].penalty * force->constraintPoints[i].C + lambda, force->constraintPoints[i].fminMagnitude, force->constraintPoints[i].fmaxMagnitude);

                    // Diagonal matrix, of sum of columns of hessian
                    Matrix6f H = force->constraintPoints[i].H;
                    Matrix6f G;
                    for (int j = 0; j < 6; ++j)
                    {
                        G(j, j) = H.col(j).norm() * std::abs(f);
                    }

                    const Vector6f J = force->constraintPoints[i].J;
                    rhs += J * f;
                    lhs += J * J.transpose() * force->constraintPoints[i].penalty + G;
                }
            }

            // 5.3 Loop over energies

            // 5.4 solve LDLT
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

    // Time accumulation
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> stepDuration = endTime - startTime;
    stepTimeAccumulator.push_back(stepDuration.count());
    if (stepTimeAccumulator.size() > 100)
        stepTimeAccumulator.erase(stepTimeAccumulator.begin());
    averageStepTime = 0.0f;
    for (float t : stepTimeAccumulator) averageStepTime += t;
        averageStepTime /= stepTimeAccumulator.size();
}