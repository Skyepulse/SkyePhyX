#include "force.hpp"
#include "solver.hpp"

//================================//
Force::Force(Solver* solver, std::vector<Mesh*> linkedBodies, int numConstraintPoints): solver(solver), linkedBodies(linkedBodies)
{
    next = solver->solverForces;
    solver->solverForces = this;

    constraintPoints.clear();
    for (int i = 0; i < numConstraintPoints; ++i)
        constraintPoints.emplace_back(ConstraintPointProperties{});

    for (Mesh* mesh : linkedBodies)
        mesh->forces.push_back(this);
}

//================================//
Force::~Force()
{
    Force** p = &solver->solverForces;
    while (*p != this)
        p = &((*p)->next);
    *p = next;

    for (Mesh* mesh : linkedBodies)
    {
        auto& f = mesh->forces;
        f.erase(std::remove(f.begin(), f.end(), this), f.end());
    }
};