#ifndef FORCE_HPP
#define FORCE_HPP

#include "../helpers/geometry.hpp"
#include "../rendering/RenderEngine.hpp"
#include "collision.hpp"

//================================//
struct ConstraintPointProperties
{
    // ---- Force properties ---
    float fminMagnitude = -INFINITY;
    float fmaxMagnitude = INFINITY;
    float stiffness = INFINITY;
    float fracture = INFINITY;

    Vector6f J = Vector6f::Zero();
    Matrix6f H = Matrix6f::Zero();

    // ---- AVBD specifics ---
    float penalty = 0.0f;
    float lambda = 0.0f;
    float C = 0.0f;
};

//================================//
struct Force
{
    Force(Solver* solver, std::vector<Mesh*> linkedBodies, int numConstraintPoints);
    ~Force();

    Force* next = nullptr;

    std::vector<Mesh*> linkedBodies;
    Solver* solver;
    ConstraintPointProperties constraintPoints[24];

    bool isManifold = false;
    bool includeHessian = true;

    // HELPERS //
    //================================//
    void Disable()
    {
        for (int i = 0; i < 24; ++i)
        {
            constraintPoints[i].stiffness = 0.0f;
            constraintPoints[i].penalty = 0.0f;
            constraintPoints[i].lambda = 0.0f;
        }
    }

    virtual bool Initialize() = 0;
    virtual void ComputeConstraints(float alpha) = 0;
    virtual void ComputeDerivatives(Mesh* mesh) = 0;
    virtual int numConstraints() const = 0;
    virtual int numBodies() const = 0;
    virtual void AddLineData(std::vector<GPULineData>& data) const = 0;
    virtual void AddDebugPointData(std::vector<GPUDebugPointData>& data) const = 0;
};

//================================//
struct Spring: Force
{
    static constexpr int NUM_CONSTRAINTS = 1;
    Spring(Solver* solver, Mesh* bodyA, const Eigen::Vector3f& rA, Mesh* bodyB, const Eigen::Vector3f& rB, float stiffness, float restLength = -1.0f, bool isRope = false);

    Eigen::Vector3f rA;
    Eigen::Vector3f rB;

    Mesh* bodyA = nullptr;
    Mesh* bodyB = nullptr;

    float restLength;
    bool isRope;

    virtual bool Initialize() override { return true; }
    virtual void ComputeConstraints(float alpha) override;
    virtual void ComputeDerivatives(Mesh* mesh) override;
    virtual int numConstraints() const override { return 1; }
    virtual int numBodies() const override { return 2; }
    virtual void AddLineData(std::vector<GPULineData>& data) const override;
    virtual void AddDebugPointData(std::vector<GPUDebugPointData>& data) const override {};
};

static constexpr float COLLISION_MARGIN = 0.0005f;
static constexpr float STICK_THRESHOLD  = 0.01f;


//================================//
struct ManifoldContactInfo
{
    Vector6f JacNormA   = Vector6f::Zero();   // [n,   rA×n  ]
    Vector6f JacNormB   = Vector6f::Zero();   // [-n, -rB×n  ]
    Vector6f JacTang1A  = Vector6f::Zero();   // [t1,  rA×t1 ]
    Vector6f JacTang1B  = Vector6f::Zero();   // [-t1,-rB×t1 ]
    Vector6f JacTang2A  = Vector6f::Zero();   // [t2,  rA×t2 ]
    Vector6f JacTang2B  = Vector6f::Zero();   // [-t2,-rB×t2 ]

    Eigen::Vector3f C0  = Eigen::Vector3f::Zero();
    bool stick = false;
};

//================================//
inline bool isConstrainedTo(Mesh* mesh, Mesh* other)
{
    for (Force* f : mesh->forces)
    {
        if (std::find(f->linkedBodies.begin(), f->linkedBodies.end(), other) != f->linkedBodies.end())
            return true;
    }
    return false;
}

//================================//
struct Manifold: Force
{
    static constexpr int NUM_CONSTRAINTS = 24;
    Manifold(Solver* solver, Mesh* bodyA, Mesh* bodyB);

    ContactPoint contactPoints[8]; // Max 8 contact points
    ContactPoint oldContactPoints[8];
    ManifoldContactInfo contactInfos[8];
    int numContactPoints = 0;

    float friction = 0.f;

    Mesh* bodyA = nullptr;
    Mesh* bodyB = nullptr;

    virtual bool Initialize() override;
    virtual void ComputeConstraints(float alpha) override;
    virtual void ComputeDerivatives(Mesh* mesh) override;
    virtual int numConstraints() const override { return numContactPoints * 3; }
    virtual int numBodies() const override { return 2; }
    virtual void AddLineData(std::vector<GPULineData>& data) const override;
    virtual void AddDebugPointData(std::vector<GPUDebugPointData>& data) const override;

    void Reset(Solver* solver, Mesh* bodyA, Mesh* bodyB)
    {
        this->solver = solver;
        this->bodyA = bodyA;
        this->bodyB = bodyB;
        this->linkedBodies = {bodyA, bodyB};
        this->numContactPoints = 0;
    }
};

#endif // FORCE_HPP