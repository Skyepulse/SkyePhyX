#include "geometry.hpp"
#include "../physics/solver.hpp"

//================================//
Mesh::Mesh(Solver* solver, ModelType modelType, const Eigen::Vector3f& color) : modelType(modelType), color(color), solver(solver)
{
    return;
}

//================================//
Mesh::~Mesh()   
{
    auto forcesCopy = forces;
    for (Force* f : forcesCopy)
    {
        solver->RemoveForce(f);
    }
};