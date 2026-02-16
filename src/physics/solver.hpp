#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "../helpers/geometry.hpp"

//================================//
class Solver
{
public:
    Solver();
    ~Solver();

    Mesh* solverBodies = nullptr;

    void Step();
    void AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 1.0f, 1.0f));

private:
    float stepValue = 16.0f;
};

#endif // SOLVER_HPP