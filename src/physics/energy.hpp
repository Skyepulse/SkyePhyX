#ifndef ENERGY_HPP
#define ENERGY_HPP

#include "../helpers/geometry.hpp"
#include "../rendering/RenderEngine.hpp"
#include "../helpers/math.hpp"

// MEMO NEOHOOKEAN:
// E (Stiffness)                ν (Incompressibility)
// ─────────────                ─────────────────────
// 50   → jelly                 0.10 → sponge/foam
// 200   → soft rubber          0.25 → cork-like
// 500   → rubber               0.30 → typical solid
// 1000   → firm rubber         0.40 → soft rubber
// 3000   → stiff               0.45 → rubber
// 8000   → very stiff          0.48 → nearly incompressible
// 20000   → nearly rigid       0.495 → highly incompressible

//================================//
struct Energy
{
    Energy(Solver* solver, std::vector<Mesh*> linkedBodies);
    ~Energy();

    int solverIndex = -1;

    std::vector<Mesh*> linkedBodies;
    Solver* solver;

    // Properties
    Vector6f grad = Vector6f::Zero();
    Matrix6f hess = Matrix6f::Zero();

    float fmin = -INFINITY;
    float fmax = INFINITY;

    float cachedEnergy = 0.0f;
    float GetCachedEnergy() const { return cachedEnergy; }

    virtual bool Initialize() = 0;
    virtual void ComputeEnergyTerms(Mesh* mesh, EigenProjectionMode projectionMode, float trustRegionRho) = 0;
    virtual int numBodies() const = 0;
    virtual void AddLineData(std::vector<GPULineData>& data) const = 0;
    virtual void AddDebugPointData(std::vector<GPUDebugPointData>& data) const = 0;
};

//================================//
struct NeoHookeanFEM: Energy
{
    NeoHookeanFEM(Solver* solver,
                  Mesh* body0, Mesh* body1, Mesh* body2, Mesh* body3,
                  float youngsModulus, float poissonRatio);

    Mesh* body0;
    Mesh* body1;
    Mesh* body2;
    Mesh* body3;

    float restVolume;

    float lameMu;
    float lameLambda;

    float poissonRatio;
    float youngsModulus;

    Eigen::Matrix3f Dm = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f DmInv = Eigen::Matrix3f::Zero();

    Eigen::Vector3f gradN0 = Eigen::Vector3f::Zero();
    Eigen::Vector3f gradN1 = Eigen::Vector3f::Zero();
    Eigen::Vector3f gradN2 = Eigen::Vector3f::Zero();
    Eigen::Vector3f gradN3 = Eigen::Vector3f::Zero();

    float trustRegionThreshold = 0.01f;

    virtual bool Initialize() override { return true; };
    virtual void ComputeEnergyTerms(Mesh* mesh, EigenProjectionMode projectionMode, float trustRegionRho) override;
    virtual int  numBodies() const override { return 4; }
    virtual void AddLineData(std::vector<GPULineData>& data) const override;
    virtual void AddDebugPointData(std::vector<GPUDebugPointData>& data) const override {}

private:
    void HandleInvertedElement(Mesh* mesh, const Eigen::Matrix3f& F, float J, const Eigen::Vector3f& gradNi);
};

//================================//
struct STVKFEM: Energy
{
    STVKFEM(Solver* solver,
            Mesh* body0, Mesh* body1, Mesh* body2,
            float youngsModulus, float poissonRatio);

    Mesh* body0;
    Mesh* body1;
    Mesh* body2;

    float restArea;

    float lameMu;
    float lameLambda;
    float a;

    float poissonRatio;
    float youngsModulus;

    Eigen::Matrix2f DmInv2D;

    Eigen::Vector2f gradN0, gradN1, gradN2;

    float trustRegionThreshold = 0.01f;

    virtual bool Initialize() override { return true; };
    virtual void ComputeEnergyTerms(Mesh* mesh, EigenProjectionMode projectionMode, float trustRegionRho) override;
    virtual int  numBodies() const override { return 3; }
    virtual void AddLineData(std::vector<GPULineData>& data) const override;
    virtual void AddDebugPointData(std::vector<GPUDebugPointData>& data) const override {}

private:
    void HandleInvertedElement(Mesh* mesh, const STVKMath::F32& F, const Eigen::Vector2f& gradN);
};

#endif // ENERGY_HPP