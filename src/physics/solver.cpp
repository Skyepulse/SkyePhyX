    #include "solver.hpp"
    #include <chrono>

    const Eigen::Vector3f GRAVITY(0.0f, -9.81f, 0.0f);

    //================================//
    Solver::Solver(): solverBodies(nullptr)
    {
        AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), false);
    }

    //================================//
    Solver::~Solver()
    {
        while(solverBodies)
            delete solverBodies;
    }

    //================================//
    void Solver::AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color)
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

        newMesh->mass = isStatic ? 0.0f : density * scale.x() * scale.y() * scale.z();
        newMesh->detectionRadius = scale.norm() * 0.5f;

        if (isStatic || newMesh->mass <= 0.f)
        {
            newMesh->inertiaTensorBody    = Eigen::Matrix3f::Zero();
            newMesh->inertiaTensorBodyInv = Eigen::Matrix3f::Zero();
            return;
        }

        Eigen::Vector3f I;
        float sx2 = scale.x() * scale.x(); float sy2 = scale.y() * scale.y(); float sz2 = scale.z() * scale.z();
        I = (newMesh->mass / 12.0f) * Eigen::Vector3f(sy2 + sz2, sx2 + sz2, sx2 + sy2);

        newMesh->inertiaTensorBody = I.asDiagonal();
        newMesh->inertiaTensorBodyInv = I.cwiseInverse().asDiagonal();
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
        // TODO

        // 3. Energies Warmstarting
        // TODO

        // 4. Bodies Warmstarting
        for (Mesh* mesh = solverBodies; mesh; mesh = mesh->next)
        {
            // 4.1 Constrain rotation velocity ...?
            // TODO...?

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

                // 5.3 Loop over energies

                // 5.4 solve LDLT
                Vector6f dx = lhs.ldlt().solve(rhs);
                Eigen::Vector3f dx_lin = dx.head<3>();
                Eigen::Vector3f dx_ang = dx.tail<3>();

                Eigen::Vector3f pos;
                mesh->transform.GetPosition(pos);
                mesh->transform.SetPosition(pos - dx_lin);

                float angle = dx_ang.norm();
                Quaternionf dq;
                if (angle < 1e-6f)
                {
                    dq.w() = 1.f;
                    dq.vec() = -0.5f * dx_ang;
                }
                else
                {
                    float halfAngle = 0.5f * angle;
                    dq.w() = std::cos(halfAngle);
                    dq.vec() = -std::sin(halfAngle) * (dx_ang / angle);
                }
                dq.normalize();

                Quaternionf rot;
                mesh->transform.GetRotation(rot);
                mesh->transform.SetRotation((dq * rot).normalized());
            }

            // 5.5 Dual Update
            // TODO
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
                Quaternionf deltaRot = rot * mesh->lastRotation.conjugate();

                if (deltaRot.w() < 0.f) deltaRot.coeffs() = -deltaRot.coeffs();

                float sinHalf = deltaRot.vec().norm();
                if (sinHalf < 1e-6f)
                {
                    mesh->angularVelocity = 2.f * deltaRot.vec() / stepValue;
                }
                else
                {
                    float halfAngle = std::atan2(sinHalf, deltaRot.w());
                    mesh->angularVelocity = 2.f * halfAngle * (deltaRot.vec() / sinHalf) / stepValue;
                }
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

                // 5.3 Loop over energies

                // 5.4 solve LDLT
                Vector6f dx = lhs.ldlt().solve(rhs);
                Eigen::Vector3f dx_lin = dx.head<3>();
                Eigen::Vector3f dx_ang = dx.tail<3>();

                Eigen::Vector3f pos;
                mesh->transform.GetPosition(pos);
                mesh->transform.SetPosition(pos - dx_lin);

                float angle = dx_ang.norm();
                Quaternionf dq;
                if (angle < 1e-6f)
                {
                    dq.w() = 1.f;
                    dq.vec() = -0.5f * dx_ang;
                }
                else
                {
                    float halfAngle = 0.5f * angle;
                    dq.w() = std::cos(halfAngle);
                    dq.vec() = -std::sin(halfAngle) * (dx_ang / angle);
                }
                dq.normalize();

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
        for (float t : stepTimeAccumulator)        averageStepTime += t;
        averageStepTime /= stepTimeAccumulator.size();
    }