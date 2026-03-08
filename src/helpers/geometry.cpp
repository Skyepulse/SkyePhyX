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

    auto energiesCopy = energies;
    for (Energy* e : energiesCopy)
    {
        solver->RemoveEnergy(e);
    }
};

//================================//
namespace GeometryHelpers
{
    //================================//
    void makeTet(Solver* solver, float cx, float cy, float cz, float edge, float E, float nu, const Eigen::Vector3f& color, float mass, float friction)
    {
        float a = edge;
        float inv_sqrt3  = 1.f / std::sqrt(3.f);
        float inv_sqrt6  = 1.f / std::sqrt(6.f);
        float sqrt_2_3   = std::sqrt(2.f / 3.f);

        Eigen::Vector3f pyramidVertices[4] = 
        {
            { 0.f,            0.f,              sqrt_2_3 * a },
            { -0.5f * a,     -0.5f * inv_sqrt3 * a,  -inv_sqrt6 * a },
            {  0.5f * a,     -0.5f * inv_sqrt3 * a,  -inv_sqrt6 * a },
            {  0.f,           inv_sqrt3 * a,          -inv_sqrt6 * a },
        };

        Eigen::Vector3f centroid = (pyramidVertices[0] + pyramidVertices[1] + pyramidVertices[2] + pyramidVertices[3]) / 4.f;
        Eigen::Vector3f offset(cx, cy, cz);
        for (int i = 0; i < 4; ++i)
            pyramidVertices[i] += offset - centroid;

        Mesh* particles[4];
        for (int i = 0; i < 4; ++i)
        {
            particles[i] = solver->AddParticle(
                mass, friction,
                pyramidVertices[i], Eigen::Vector3f::Zero(),
                false, color
            );
            particles[i]->isParticle = true;
            particles[i]->name = "Tet_" + std::to_string((int)E) + "_" + std::to_string(i);
        }

        solver->AddEnergy(std::make_unique<NeoHookeanFEM>(
            solver,
            particles[0], particles[1], particles[2], particles[3],
            E, nu
        ));
    };
}