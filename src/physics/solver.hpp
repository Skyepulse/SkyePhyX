#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "../helpers/geometry.hpp"
#include "force.hpp"

//================================//
class Solver
{
public:
    Solver();
    ~Solver();

    Mesh* solverBodies = nullptr;
    Force* solverForces = nullptr;

    void Step();
    Mesh* AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 1.0f, 1.0f));

    float stepValue = 1.0f / 60.0f;;
    float averageStepTime = 0.0f;

private:

    bool postStabilization = true;
    int numIterations = 10;
    float alpha = 0.99;
    float beta = 100000.0f;
    float gamma = 0.99f;

    std::vector<float> stepTimeAccumulator;
};

#endif // SOLVER_HPP