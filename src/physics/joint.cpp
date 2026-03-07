#include "force.hpp"

//================================//
static std::vector<Mesh*> FilterNulls(std::initializer_list<Mesh*> bodies)
{
    std::vector<Mesh*> result;
    for (Mesh* m : bodies)
        if (m) result.push_back(m);
    return result;
}

//================================/
static Eigen::Vector3f AngularError(
    const Quaternionf& qA,
    const Quaternionf& qB,
    const Quaternionf& restRot,
    float torqueArm)
{
    Quaternionf qErr = qA.conjugate() * qB * restRot.conjugate();
    return QuaternionToRotVec(qErr) * torqueArm;
}

//================================//
Joint::Joint(
    Solver* solver,
    Mesh* bodyA,
    const Eigen::Vector3f& rA,
    Mesh* bodyB,
    const Eigen::Vector3f& rB,
    const Vector6f& stiffness,
    const Vector6f& fractures)
    : Force(solver, FilterNulls({ bodyA, bodyB }), NUM_CONSTRAINTS),
      rA(rA), rB(rB), bodyA(bodyA), bodyB(bodyB)
{
    assert(bodyB && "Joint must connect to at least one body");

    includeHessian = true;

    for (int i = 0; i < NUM_CONSTRAINTS; i++)
    {
        constraintPoints[i].stiffness = stiffness[i];
        constraintPoints[i].fracture  = fractures[i];
        constraintPoints[i].fminMagnitude = -fractures[i];
        constraintPoints[i].fmaxMagnitude =  fractures[i];
    }

    for (int i = 3; i < NUM_CONSTRAINTS; i++)
    {
        constraintPoints[i].fracture = fractures[i];
        constraintPoints[i].fminMagnitude = -fractures[i];
        constraintPoints[i].fmaxMagnitude =  fractures[i];
    }

    Quaternionf qA = bodyA ? bodyA->transform.GetRotation() : Quaternionf::Identity();
    Quaternionf qB = bodyB->transform.GetRotation();

    restRotation = qA.conjugate() * qB;

    Eigen::Vector3f scaleA = bodyA ? bodyA->transform.GetScale() : Eigen::Vector3f::Zero();
    Eigen::Vector3f scaleB = bodyB->transform.GetScale();

    torqueArm = (scaleA + scaleB).squaredNorm();

    if (torqueArm < 1e-6f)
        torqueArm = 1.0f;
}

//================================//
bool Joint::Initialize()
{
    if (disabled) return false;
    
    Eigen::Vector3f worldA = bodyA ? bodyA->transform.TransformPoint(rA) : rA;
    Eigen::Vector3f worldB = bodyB->transform.TransformPoint(rB);

    C0_pos = worldA - worldB;

    Quaternionf qA = bodyA ? bodyA->transform.GetRotation() : Quaternionf::Identity();
    Quaternionf qB = bodyB->transform.GetRotation();

    C0_ang = AngularError(qA, qB, restRotation, torqueArm);

    return true;
}

//================================//
void Joint::ComputeConstraints(float alpha)
{
    Eigen::Vector3f worldA = bodyA ? bodyA->transform.TransformPoint(rA) : rA;
    Eigen::Vector3f worldB = bodyB->transform.TransformPoint(rB);

    Eigen::Vector3f Cn_pos = worldA - worldB;

    Quaternionf qA = bodyA ? bodyA->transform.GetRotation() : Quaternionf::Identity();
    Quaternionf qB = bodyB->transform.GetRotation();

    Eigen::Vector3f Cn_ang = AngularError(qA, qB, restRotation, torqueArm);

    for (int i = 0; i < 3; i++)
    {
        if (std::isinf(constraintPoints[i].stiffness))
            constraintPoints[i].C = Cn_pos[i] - C0_pos[i] * alpha;
        else
            constraintPoints[i].C = Cn_pos[i];
    }

    for (int i = 0; i < 3; i++)
    {
        if (std::isinf(constraintPoints[3 + i].stiffness))
            constraintPoints[3 + i].C = Cn_ang[i] - C0_ang[i] * alpha;
        else
            constraintPoints[3 + i].C = Cn_ang[i];
    }
}

//================================//
void Joint::ComputeDerivatives(Mesh* mesh)
{
    if (mesh == bodyA)
    {
        Quaternionf qA = bodyA->transform.GetRotation();
        Eigen::Vector3f r = qA * (bodyA->transform.GetScale().asDiagonal() * rA);  // FIX: include scale

        for (int i = 0; i < 3; i++)
        {
            Eigen::Vector3f ei = Eigen::Vector3f::Zero();
            ei[i] = 1.0f;

            constraintPoints[i].J.head<3>() = ei;
            constraintPoints[i].J.tail<3>() = r.cross(ei);

            constraintPoints[i].H.setZero();
            constraintPoints[i].H(5,5) = -r[i];

            constraintPoints[3+i].J.head<3>() = Eigen::Vector3f::Zero();
            constraintPoints[3+i].J.tail<3>() = -torqueArm * ei;  // FIX: was positive

            constraintPoints[3+i].H.setZero();
        }
    }
    else
    {
        Quaternionf qB = bodyB->transform.GetRotation();
        Eigen::Vector3f r = qB * (bodyB->transform.GetScale().asDiagonal() * rB);  // FIX: include scale

        for (int i = 0; i < 3; i++)
        {
            Eigen::Vector3f ei = Eigen::Vector3f::Zero();
            ei[i] = 1.0f;

            constraintPoints[i].J.head<3>() = -ei;
            constraintPoints[i].J.tail<3>() = -r.cross(ei);

            constraintPoints[i].H.setZero();
            constraintPoints[i].H(5,5) = r[i];

            constraintPoints[3+i].J.head<3>() = Eigen::Vector3f::Zero();
            constraintPoints[3+i].J.tail<3>() = torqueArm * ei;   // FIX: was negative

            constraintPoints[3+i].H.setZero();
        }
    }
}

//================================//
void Joint::AddLineData(std::vector<GPULineData>& data) const
{
    Eigen::Vector3f worldA = bodyA ? bodyA->transform.TransformPoint(rA) : rA;
    Eigen::Vector3f worldB = bodyB->transform.TransformPoint(rB);

    Eigen::Vector4f color;
    color << 0.75f, 0.0f, 0.0f, 1.0f;

    GPULineData line{};

    Eigen::Map<Eigen::Vector3f>(line.start) = worldA;
    Eigen::Map<Eigen::Vector3f>(line.end) = worldB;
    Eigen::Map<Eigen::Vector4f>(line.color) = color;

    data.push_back(line);
}