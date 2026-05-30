#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "../helpers/geometry.hpp"
#include "force.hpp"
#include "energy.hpp"
#include <vector>
#include <array>
#include <unordered_map>
#include <utility>

//================================//
struct SolverTimings
{
    float broadPhaseMs      = 0.0f;
    float warmstartMs       = 0.0f;
    float coloringMs        = 0.0f;
    float predictionMs      = 0.0f;
    float primalDualMs      = 0.0f;
    float solveConstraintsMs = 0.0f;
    float solveEnergiesMs   = 0.0f;
    float solveLDLTMs       = 0.0f;
    float velocityUpdateMs  = 0.0f;
    float postStabMs        = 0.0f;
    float totalSubstepMs    = 0.0f;
};

//================================//
struct BroadPhaseSweepPair
{
    Mesh* bodyA = nullptr;
    Mesh* bodyB = nullptr;
};

using MeshPairKey = std::pair<Mesh*, Mesh*>;
struct MeshPairKeyHash
{
    std::size_t operator()(const MeshPairKey& key) const
    {
        const std::size_t hashA = std::hash<Mesh*>{}(key.first);
        const std::size_t hashB = std::hash<Mesh*>{}(key.second);
        return hashA ^ (hashB << 1);
    }
};

struct BroadPhaseSweepEntry
{
    Mesh* mesh = nullptr;
    AABB aabb;
    float minX = 0.0f;
    float maxX = 0.0f;
    Eigen::Vector3f cachedPosition = Eigen::Vector3f::Zero();
    Quaternionf cachedRotation = Quaternionf::Identity();
    Eigen::Vector3f cachedScale = Eigen::Vector3f::Ones();
    bool hasCachedAABB = false;
};

// This struct stores all information of the current coloring
// of the energy // constraint graph.
struct SolverPrimalColoring
{
    std::vector<int> dynamicBodyIndices;
    std::vector<int> bodyColors;
    std::vector<int> colorOffsets;
    std::vector<int> colorBodyIndices;
    int numColors = 0;

    void Clear()
    {
        dynamicBodyIndices.clear();
        bodyColors.clear();
        colorOffsets.clear();
        colorBodyIndices.clear();
        numColors = 0;
    }
};

//================================//
class Solver
{
public:
    Solver();
    ~Solver();

    std::vector<std::unique_ptr<Mesh>> solverBodies;
    std::vector<std::unique_ptr<Force>> solverForces;
    std::vector<std::unique_ptr<Energy>> solverEnergies;

    std::vector<Mesh*> bodyPtrs;
    std::vector<Force*> forcePtrs;
    std::vector<Energy*> energyPtrs;

    std::vector<GPULineData> lineData;
    std::vector<GPUDebugPointData> debugPointData;
    std::vector<std::array<Mesh*, 3>> surfaceFaces;
    std::vector<GPUSoftBodyVertex> softBodySurfaceData;
    SolverPrimalColoring primalColoring;
    bool surfaceDirty = true;

    void Start();
    void Clear();
    void Step();

    Mesh* AddBody(ModelType modelType, float density, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& scale, const Eigen::Vector3f& velocity, const Quaternionf rotation, const Eigen::Vector3f& angularVelocity, bool isStatic, const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    Force* AddForce(std::unique_ptr<Force> force);
    Energy* AddEnergy(std::unique_ptr<Energy> energy);
    Mesh* AddParticle(float mass, float friction, const Eigen::Vector3f& position, const Eigen::Vector3f& velocity, bool isStatic, const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    void RemoveForce(Force* force);
    void RemoveEnergy(Energy* energy);
    void RemoveBody(Mesh* body);

    void RebuildPtrCaches()
    {
        bodyPtrs.clear();
        bodyPtrs.reserve(solverBodies.size());
        for (auto& b : solverBodies) bodyPtrs.push_back(b.get());

        forcePtrs.clear();
        forcePtrs.reserve(solverForces.size());
        for (auto& f : solverForces) forcePtrs.push_back(f.get());

        energyPtrs.clear();
        energyPtrs.reserve(solverEnergies.size());
        for (auto& e : solverEnergies) energyPtrs.push_back(e.get());
    }

    float averageStepTime = 0.0f;
    SolverTimings timings;

    // Changing Parameters
    bool postStabilization = true;
    //int numSubsteps = 4;
    int numIterations = 20;
    float alpha = 0.95f;
    float beta = 100'000.0f;
    float gamma = 0.99f;
    float stepValue = 1.0f / 60.0f;

    EigenProjectionMode projectionMode = EigenProjectionMode::ABSOLUTE;
    float trustRegionRho = 0.0f;
    float prevTotalEnergy = 0.0f;
    float trustRegionThreshold = 0.01f;

    bool emergencyStop = false;
    bool exactHessian  = true;

    //================================//
    AABB GetModelAABB(ModelType modelType) const
    {
        // Fallout to empty AABB if model type is not found
        auto it = modelGeometry.perModelLocalAABBs.find(modelType);
        if (it != modelGeometry.perModelLocalAABBs.end())
        {
            return it->second;
        }
        return AABB{};
    }

    //================================//
    const ConvexHull& GetModelConvexHull(ModelType modelType) const
    {
        // Fallout to empty ConvexHull if model type is not found
        auto it = modelGeometry.perModelConvexHulls.find(modelType);
        if (it != modelGeometry.perModelConvexHulls.end())
        {
            return it->second;
        }

        static const ConvexHull emptyHull{};
        return emptyHull;
    }

    //================================//
    MeshData GetModelMeshData(ModelType modelType) const
    {
        // Fallout to empty MeshData if model type is not found
        auto it = modelGeometry.perModelMeshData.find(modelType);
        if (it != modelGeometry.perModelMeshData.end())
        {
            return it->second;
        }
        return MeshData{};
    }

private:
    ModelGeometry modelGeometry{};

    std::vector<float> stepTimeAccumulator;
    static constexpr int TIMING_WINDOW = 60;
    std::vector<SolverTimings> timingAccumulator;
    std::vector<BroadPhaseSweepEntry> broadPhaseEntries;
    std::unordered_map<MeshPairKey, int, MeshPairKeyHash> constrainedPairCounts;
    bool broadPhaseEntriesDirty = true;

    Eigen::LDLT<Matrix6f> ldlt;

    bool CheckExplosion();
    void BuildSoftBodySurface();
    void UpdateSoftBodySurfaceData();
    void RefreshBroadPhaseEntries();
    void EnsureBroadPhaseOrder();
    std::vector<BroadPhaseSweepPair> broadPhaseSweep();
    void RebuildPrimalColoring();
    void RegisterForcePairs(Force* force, int delta);
    bool HasConstraintPair(Mesh* meshA, Mesh* meshB) const;
    static MeshPairKey MakeMeshPairKey(Mesh* meshA, Mesh* meshB);
};


#endif // SOLVER_HPP
