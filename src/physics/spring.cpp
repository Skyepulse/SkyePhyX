#include "force.hpp"

//================================//
static std::vector<Mesh*> FilterNulls(std::initializer_list<Mesh*> bodies)
{
    std::vector<Mesh*> result;
    for (Mesh* m : bodies)
        if (m) result.push_back(m);
    return result;
}

//================================//
Spring::Spring(Solver* solver, Mesh* bodyA, const Eigen::Vector3f& rA, Mesh* bodyB, const Eigen::Vector3f& rB, float stiffness, float restLength, bool isRope)
    : Force(solver, FilterNulls({bodyA, bodyB}), NUM_CONSTRAINTS), rA(rA), rB(rB), bodyA(bodyA), bodyB(bodyB), isRope(isRope)
{
    assert((bodyB != nullptr) && "A spring must be connected to at least one body.");

    constraintPoints[0].stiffness = stiffness;
    this->restLength = restLength;

    if (isRope)
    {
        constraintPoints[0].fminMagnitude = 0.0f;
        constraintPoints[0].fmaxMagnitude = INFINITY;
    }
    else
    {
        constraintPoints[0].fminMagnitude = -INFINITY;
        constraintPoints[0].fmaxMagnitude = INFINITY;
    }

    if (this->restLength < 0.0f)
    {
        Eigen::Vector3f worldPosA = bodyA ? bodyA->transform.TransformPoint(rA) : rA;
        Eigen::Vector3f worldPosB = bodyB->transform.TransformPoint(rB);
        this->restLength = (worldPosB - worldPosA).norm();
    }
}

//================================//
void Spring::ComputeConstraints(float alpha)
{
    const Eigen::Vector3f posA = bodyA ? bodyA->transform.TransformPoint(rA) : rA;
    const Eigen::Vector3f posB = bodyB->transform.TransformPoint(rB);

    const float d = (posB - posA).norm();
    constraintPoints[0].C = d - restLength;
}

//================================//
void Spring::ComputeDerivatives(Mesh* mesh)
{
    const Eigen::Vector3f posA = bodyA ? bodyA->transform.TransformPoint(rA) : rA;
    const Eigen::Vector3f posB = bodyB->transform.TransformPoint(rB);

    const Eigen::Vector3f d = (posB - posA);

    const float L = d.norm();
    const float L2 = d.squaredNorm();

    if (L2 < 1e-6f)
    {
        constraintPoints[0].J.setZero();
        constraintPoints[0].H.setZero();
        return;
    }

    const Eigen::Vector3f n = d / L;
    const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f outernn = n * n.transpose();
    const Eigen::Matrix3f dxx = (I - outernn) / L;

    if (mesh == bodyA && bodyA != nullptr)
    {
        Quaternionf rotA;
        bodyA->transform.GetRotation(rotA);
        const Eigen::Vector3f r = rotA * rA;

        const Eigen::Matrix3f Sr = -skew(r);

        const Eigen::Matrix3f dxr = dxx * Sr.transpose();
        const Eigen::Matrix3f drr = Sr * dxx * Sr.transpose() - skew(n) * skew(r);

        constraintPoints[0].J.head<3>() = -n;
        constraintPoints[0].J.tail<3>() = n.cross(r);

        constraintPoints[0].H.block<3,3>(0,0) = dxx;
        constraintPoints[0].H.block<3,3>(0,3) = dxr;
        constraintPoints[0].H.block<3,3>(3,0) = dxr.transpose();
        constraintPoints[0].H.block<3,3>(3,3) = drr;
    }
    else
    {
        Quaternionf rotB;
        bodyB->transform.GetRotation(rotB);
        const Eigen::Vector3f r = rotB * rB;

        const Eigen::Matrix3f minusSr = skew(r);

        const Eigen::Matrix3f dxr = dxx * minusSr.transpose();
        const Eigen::Matrix3f drr = minusSr * dxx * minusSr.transpose() + skew(n) * skew(r);

        constraintPoints[0].J.head<3>() = n;
        constraintPoints[0].J.tail<3>() = r.cross(n);

        constraintPoints[0].H.block<3,3>(0,0) = dxx;
        constraintPoints[0].H.block<3,3>(0,3) = dxr;
        constraintPoints[0].H.block<3,3>(3,0) = dxr.transpose();
        constraintPoints[0].H.block<3,3>(3,3) = drr;
    }
}

//================================//
void Spring::AddLineData(std::vector<GPULineData>& data) const
{
    // Transform the points rA and rB into world position
    Eigen::Vector3f rAWorld = this->bodyA ? this->bodyA->transform.TransformPoint(this->rA) : this->rA;
    Eigen::Vector3f rBWorld = this->bodyB->transform.TransformPoint(this->rB);
    Eigen::Vector4f color;
    color << 0.0f, 1.0f, 0.0f, 1.0f;

    GPULineData lineData{};
    Eigen::Map<Eigen::Vector3f>(lineData.start) = rAWorld;
    Eigen::Map<Eigen::Vector3f>(lineData.end) = rBWorld;
    Eigen::Map<Eigen::Vector4f>(lineData.color) = color;

    data.push_back(lineData);
}