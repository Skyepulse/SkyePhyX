#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "../helpers/geometry.hpp"
#include "force.hpp"
#include <vector>

//================================//
class Solver
{
public:
    Solver();
    ~Solver();

    Mesh* solverBodies = nullptr;
    Force* solverForces = nullptr;

    std::vector<GPULineData> lineData;
    std::vector<GPUDebugPointData> debugPointData;

    void Start();
    void Clear();

    void Step();
    Mesh* AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 1.0f, 1.0f));

    float averageStepTime = 0.0f;

    // changing Parameters
    bool postStabilization = true;
    int numIterations = 3;
    float alpha = 0.95f;
    float beta = 100'000.0f;
    float gamma = 0.99f;

    float onPenetrationPenalty = 1000.0f;

    float stepValue = 1.0f / 60.0f;
   // int numSubsteps = 4;

private:
    std::vector<float> stepTimeAccumulator;
    void Substep(float dt);
};

#endif // SOLVER_HPP