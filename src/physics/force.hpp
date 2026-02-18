#ifndef FORCE_HPP
#define FORCE_HPP

#include "../helpers/geometry.hpp"

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

    std::vector<ConstraintPointProperties> constraintPoints;

    // HELPERS //
    //================================//
    void Disable()
    {
        for (ConstraintPointProperties& cp : constraintPoints)
        {
            cp.stiffness = 0.0f;
            cp.penalty = 0.0f;
            cp.lambda = 0.0f;
        }
    }

    virtual bool Initialize() = 0;
    virtual void ComputeConstraints(float alpha) = 0;
    virtual void ComputeDerivatives(Mesh* mesh) = 0;
    virtual int numConstraints() const = 0;
    virtual int numBodies() const = 0;
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
};

#endif // FORCE_HPP