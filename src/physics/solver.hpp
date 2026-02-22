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

    std::vector<std::unique_ptr<Mesh>> solverBodies;
    std::vector<std::unique_ptr<Force>> solverForces;

    std::vector<Mesh*> bodyPtrs;
    std::vector<Force*> forcePtrs;

    std::vector<GPULineData> lineData;
    std::vector<GPUDebugPointData> debugPointData;

    void Start();
    void Clear();
    void Step();

    Mesh* AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    Force* AddForce(std::unique_ptr<Force> force);
    void RemoveForce(Force* force);

    void RebuildPtrCaches()
    {
        bodyPtrs.clear();
        bodyPtrs.reserve(solverBodies.size());
        for (auto& b : solverBodies) bodyPtrs.push_back(b.get());

        forcePtrs.clear();
        forcePtrs.reserve(solverForces.size());
        for (auto& f : solverForces) forcePtrs.push_back(f.get());
    }

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

    Eigen::LDLT<Matrix6f> ldlt;
};


#endif // SOLVER_HPP