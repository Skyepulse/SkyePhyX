#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "../helpers/geometry.hpp"
#include "force.hpp"
#include <vector>

//================================//
struct SolverTimings
{
    float broadPhaseMs      = 0.0f;
    float warmstartMs       = 0.0f;
    float predictionMs      = 0.0f;
    float primalDualMs      = 0.0f;
    float velocityUpdateMs  = 0.0f;
    float postStabMs        = 0.0f;
    float totalSubstepMs    = 0.0f;
};

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
    SolverTimings timings;

    // Changing Parameters
    bool postStabilization = true;
    //int numSubsteps = 4;
    int numIterations = 3;
    float alpha = 0.95f;
    float beta = 100'000.0f;
    float gamma = 0.99f;
    float onPenetrationPenalty = 10000.0f;

    float stepValue = 1.0f / 60.0f;

private:
    std::vector<float> stepTimeAccumulator;
    static constexpr int TIMING_WINDOW = 60;
    std::vector<SolverTimings> timingAccumulator;
};


#endif // SOLVER_HPP