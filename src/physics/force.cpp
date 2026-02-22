#include "force.hpp"
#include "solver.hpp"

//================================//
Force::Force(Solver* solver, std::vector<Mesh*> linkedBodies, int numConstraintPoints): solver(solver), linkedBodies(linkedBodies)
{
    for (Mesh* mesh : linkedBodies)
        mesh->forces.push_back(this);
}

//================================//
Force::~Force()
{
    for (Mesh* mesh : linkedBodies)
    {
        auto& f = mesh->forces;
        f.erase(std::remove(f.begin(), f.end(), this), f.end());
    }
};