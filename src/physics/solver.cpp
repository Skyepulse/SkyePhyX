#include "solver.hpp"

//================================//
Solver::Solver(): solverBodies(nullptr)
{
    AddBody(ModelType_Cube, 1.0f, 0.5f, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), Quaternionf::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f), false);
}

//================================//
Solver::~Solver()
{
    while(solverBodies)
        delete solverBodies;
}

//================================//
void Solver::AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color)
{
    new Mesh(this, modelType, color);

    Mesh* newMesh = solverBodies;
    newMesh->density = density;
    newMesh->friction = friction;
    newMesh->velocity = velocity;
    newMesh->angularVelocity = angularVelocity;
    newMesh->isStatic = isStatic;

    newMesh->transform.SetPosition(position);
    newMesh->transform.SetRotation(rotation);
}

//================================//
void Solver::Step()
{
    return;
}