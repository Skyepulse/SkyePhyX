#include "geometry.hpp"
#include "../physics/solver.hpp"

//================================//
Mesh::Mesh(Solver* solver, ModelType modelType, const Eigen::Vector3f& color) : modelType(modelType), color(color), solver(solver)
{
    next = solver->solverBodies;
    solver->solverBodies = this;
}

//================================//
Mesh::~Mesh()
{
    Mesh** p = &solver->solverBodies;
    while (*p != this)
        p = &((*p)->next);
    *p = next;
};