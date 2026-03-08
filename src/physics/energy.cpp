#include "energy.hpp"
#include "solver.hpp"

//================================//
Energy::Energy(Solver* solver, std::vector<Mesh*> linkedBodies): solver(solver), linkedBodies(linkedBodies)
{
    for (Mesh* mesh : linkedBodies)
        mesh->energies.push_back(this);
}

//================================//
Energy::~Energy()
{
    for (Mesh* mesh : linkedBodies)
    {
        auto& e = mesh->energies;
        e.erase(std::remove(e.begin(), e.end(), this), e.end());
    }
};